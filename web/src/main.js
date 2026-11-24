// 페이지 엔트리포인트: 씬 구성, VR 진입/종료, 포즈 수집·업링크, 로봇·사람·재구성 메시 렌더링.
//
// - VR (Quest 3): 시점을 로봇 머리 카메라 위치에 고정한 로봇 시점으로 재구성 메시·로봇 몸(머리 제외)·HUD를 그린다.
// - PC 브라우저: 로봇 전체·조작자 아바타·재구성 메시를 관찰하고, 모드 전환·dev 수동 조작 패널을 제공한다.
// 통신: RtcLink 하나의 state 채널(포즈·상태·명령)과 mesh 채널(MSH1 바이너리) (net/rtc_link.js, net/packets.js).
// 입력: 손등 컨트롤러 포즈를 공통 샘플로 변환해 보낸다 (input/controller_input.js). 손가락은 서버의 Manus 장갑이 측정한다.
// 명령: VR 진입 2초 후 start, VR 종료·세션 숨김 시 home(ready 복귀 후 해제).

import { createScene } from "./render/scene.js";
import { Hud, formatSample } from "./render/hud.js";
import { HumanView } from "./render/human_view.js";
import { RobotView } from "./render/robot_view.js";
import { MeshView } from "./render/mesh_view.js";
import { ControllerInput } from "./input/controller_input.js";
import { createXR, enterVR, isVRSupported } from "./xr/xr_session.js";
import { RtcLink } from "./net/rtc_link.js";
import { decodeMeshBlocks } from "./net/packets.js";
import { ManualControl } from "./manual_control.js";

// HUD 갱신 주기(ms)
const HUD_INTERVAL_MS = 100;
// 시점 목표 지수 평활 시정수(s)
const VIEW_SMOOTHING_TAU_S = 0.05;
// 시점 오프셋 원점 변화가 이 값(m) 미만이면 기존 기준 공간을 그대로 쓴다
const VIEW_OFFSET_EPSILON_M = 0.001;
// 로봇 머리 위치를 받기 전 시점 목표 (바닥 아래, WebXR 좌표)
const VIEW_HIDDEN_GOAL = { x: 0, y: -100, z: 0 };
// 시점 오프셋 기준 공간의 회전 (회전 없음)
const VIEW_NO_ROTATION = { x: 0, y: 0, z: 0, w: 1 };
// 데스크톱 HUD에서 중계 샘플을 유효하게 보는 최대 경과 시간(ms)
const RELAY_SAMPLE_STALE_MS = 1000;
// 사람 아바타 에셋 경로와 파일명
const AVATAR_BASE = "assets/";
const AVATAR_FILE = "avatar.glb";
// 모드 전환 대상 목록
const MODE_TARGETS = ["dev", "deploy"];
// HUD 제어 상태 안내 기본값 (데스크톱 관찰 뷰)
const DEFAULT_CONTROL_HINT = {
    idle: "제어 대기 — VR 진입 또는 start 명령으로 시작",
    armed: "제어 중 (VR 종료=홈 복귀 후 해제, stop=해제)",
};

/**
 * 전체 초기화 흐름을 실행한다. DOM 로드 후 1회 호출.
 */
async function boot() {
    const canvas = document.getElementById("renderCanvas");
    const overlayEl = document.getElementById("hudOverlay");
    const statusEl = document.getElementById("status");
    const enterBtn = document.getElementById("enterVrBtn");

    // 씬과 공용 표시 모듈 생성
    const { engine, scene } = createScene(canvas);
    const hud = new Hud(scene, overlayEl);
    const robotView = new RobotView(scene);
    const meshView = new MeshView(scene);
    const robotStatus = { source: "?", live: false, armed: null };

    // immersive-vr 지원 기기에서만 XR 경험을 만든다
    const vrSupported = await isVRSupported();
    const xr = vrSupported ? await createXR(scene) : null;
    let human = null;

    if (xr && xr.baseExperience) {
        // VR 기기: 컨트롤러 입력·연결·세션 수명·프레임 루프 구성
        const input = new ControllerInput();
        robotView.setVisibleParts("all");
        const rtc = new RtcLink(
            (msg) => _routeRobotMessage(msg, robotView, robotStatus),
            (buf) => _routeMeshPacket(buf, meshView),
        );
        rtc.connect();

        enterBtn.hidden = false;
        statusEl.textContent = "VR 사용 가능 - Enter VR을 누르세요";
        enterBtn.addEventListener("click", () => _handleEnterVrClick(xr, input, statusEl));

        _wireSessionLifecycle(xr, hud, statusEl, enterBtn, robotView, input, rtc);
        _wireFrameLoop(xr, hud, input, rtc, meshView, robotStatus);
    } else {
        // PC: 로봇 전체 + 조작자 아바타 + 재구성 메시 관찰
        robotView.setVisibleParts("all");
        statusEl.textContent = "데스크톱 모니터링 뷰 (VR 포즈 수신 대기)";

        human = new HumanView(scene);
        human.load(AVATAR_BASE, AVATAR_FILE);

        // 데스크톱 메시지 처리에 필요한 상태 묶음
        const desktop = {
            robotView,
            robotStatus,
            human,
            statusEl,
            modeButtons: {
                dev: document.getElementById("modeDevBtn"),
                deploy: document.getElementById("modeDeployBtn"),
            },
            // requestPending: 이 화면에서 보낸 전환 요청의 결과 대기 여부
            modeUi: { mode: null, switching: false, requestPending: false },
            // 마지막 중계 포즈 샘플과 수신 시각(ms)
            relayState: { sample: null, receivedAtMs: 0 },
            manual: new ManualControl(),
        };

        // 연결 생성
        const rtc = new RtcLink(
            (msg) => _routeDesktopMessage(msg, desktop),
            (buf) => _routeMeshPacket(buf, meshView),
        );
        rtc.connect();

        // 데스크톱 HUD 주기 갱신
        setInterval(() => _updateDesktopHud(hud, desktop.relayState, robotStatus, meshView), HUD_INTERVAL_MS);

        // 수동 조작·모드 버튼 연결
        desktop.manual.setLink(rtc);
        _wireModeButtons(desktop.modeButtons, desktop.modeUi, rtc);
        window.__teleop_manual = desktop.manual;
    }

    // 렌더 루프·창 크기 대응
    engine.runRenderLoop(() => scene.render());
    window.addEventListener("resize", () => engine.resize());

    // 원격 디버깅용 훅
    window.__teleop = { engine, scene, robotView, meshView, human, robotStatus };
}

/**
 * Enter VR 버튼 클릭 시 immersive-vr 세션에 진입한다. 반환값 없음.
 */
async function _handleEnterVrClick(xr, input, statusEl) {
    const isEntered = await enterVR(xr, input.xrFeatures);
    if (!isEntered) statusEl.textContent = "VR 진입 실패 - 콘솔 확인";
}

/**
 * 서버 발신 로봇 메시지를 RobotView로 라우팅한다. 반환: 로봇 메시지였으면 true.
 * status: 마지막 공급자·측정값 수신 여부·제어 상태를 담을 객체 (HUD 표시용).
 */
function _routeRobotMessage(msg, robotView, status) {
    // 로봇 구성: 링크·메시 구축
    if (msg.type === "robot_config") {
        robotView.applyConfig(msg).catch((e) => console.error("[main] robot_config 처리 실패:", e));
        return true;
    }

    // 로봇 상태: 링크 자세·HUD 상태 갱신
    if (msg.type === "robot_state") {
        robotView.applyState(msg);
        status.source = msg.source ?? "?";
        status.live = Boolean(msg.live);
        status.armed = msg.armed ?? null;
        return true;
    }
    return false;
}

/**
 * 데스크톱 state 채널 메시지를 모드 상태·로봇·사람 아바타로 라우팅한다. 반환값 없음.
 */
function _routeDesktopMessage(msg, desktop) {
    // 모드 상태
    if (msg.type === "mode_status") {
        _applyModeStatus(msg, desktop.modeUi, desktop.modeButtons, desktop.manual, desktop.statusEl);
        return;
    }

    // 로봇 메시지
    if (_routeRobotMessage(msg, desktop.robotView, desktop.robotStatus)) return;

    // 중계된 포즈 샘플: 아바타 자세 갱신·HUD용 보관
    desktop.human.update(msg);
    desktop.relayState.sample = msg;
    desktop.relayState.receivedAtMs = performance.now();
}

/**
 * mesh DataChannel로 받은 바이너리 패킷을 디코드해 메시 뷰에 적용한다. 반환값 없음.
 */
function _routeMeshPacket(buf, meshView) {
    const mesh = decodeMeshBlocks(buf);
    if (mesh !== null) meshView.apply(mesh);
}

/**
 * 데스크톱 HUD를 마지막 로봇 상태·중계 샘플로 갱신한다. 반환값 없음.
 * relayState: {sample, receivedAtMs} — RELAY_SAMPLE_STALE_MS가 지난 샘플은 없는 것으로 표시한다.
 */
function _updateDesktopHud(hud, relayState, robotStatus, meshView) {
    const isFresh = performance.now() - relayState.receivedAtMs < RELAY_SAMPLE_STALE_MS;
    hud.update(formatSample(isFresh ? relayState.sample : null, [
        `로봇: ${_robotStatus(robotStatus)}`,
        _meshStatus(meshView),
    ]));
}

/**
 * 서버 mode_status 메시지를 모드 버튼·수동 조작 패널·상태 문구에 반영한다. 반환값 없음.
 * ui: {mode, switching, requestPending} — 버튼 클릭 로직이 참조하는 최신 상태.
 * 버튼은 전환 동작만 표시한다: 현재 모드가 아닌 대상 버튼을 보이고, 모드가 없으면(null) 둘 다 보인다.
 */
function _applyModeStatus(msg, ui, modeButtons, manual, statusEl) {
    ui.mode = msg.mode ?? null;
    ui.switching = Boolean(msg.switching);

    // 전환 대상 버튼 표시·활성 상태 갱신
    for (const target of MODE_TARGETS) {
        const button = modeButtons[target];
        button.hidden = target === ui.mode;
        button.disabled = ui.switching;
        button.textContent = ui.switching ? "전환 중…" : `${target}로 전환`;
    }

    // 현재 모드가 없으면 선택을 안내한다
    if (ui.mode === null && !ui.switching) {
        console.warn("[main] 현재 모드 없음 — dev/deploy 선택 대기");
        statusEl.textContent = "현재 모드 없음 - dev 또는 deploy를 선택하세요";
    }

    // 수동 조작 패널은 dev 모드에서만 활성
    manual.setDevMode(ui.mode === "dev" && !ui.switching);

    // 실패 표시: 직접 요청한 결과면 대화상자, 아니면 상태 문구
    if (!msg.ok) {
        console.warn(`[main] 모드 전환 실패: ${msg.message}`);
        statusEl.textContent = `모드 전환 실패: ${msg.message}`;
        if (ui.requestPending && !ui.switching) alert(`모드 전환 실패: ${msg.message}`);
    }

    // 전환이 끝난 결과를 받으면 요청 대기 해제
    if (!ui.switching) ui.requestPending = false;
}

/**
 * 모드 전환 버튼 클릭 → set_mode 요청 전송을 연결한다. 반환값 없음.
 */
function _wireModeButtons(modeButtons, ui, rtc) {
    for (const target of MODE_TARGETS) {
        modeButtons[target].addEventListener("click", () => _requestModeSwitch(target, ui, rtc));
    }
}

/**
 * target 모드로 전환을 요청한다 (deploy는 확인 대화상자를 거친다). 반환값 없음.
 */
function _requestModeSwitch(target, ui, rtc) {
    if (ui.switching || target === ui.mode) return;

    // 실로봇 연결 전 안전 확인
    if (target === "deploy" &&
        !confirm("deploy 모드로 전환합니다.\n실로봇(RBY1)에 연결됩니다 — 로봇 주변 안전을 확인했습니까?")) {
        return;
    }

    // 전환 요청 전송
    if (!rtc.send({ type: "set_mode", mode: target })) {
        alert("서버 연결이 끊겨 있어 전환 요청을 보낼 수 없습니다.");
        return;
    }
    ui.requestPending = true;
    console.log(`[main] 모드 전환 요청 전송: ${target}`);
}

/**
 * HUD용 재구성 메시 수신 상태 문자열을 반환한다.
 */
function _meshStatus(meshView) {
    return `mesh: 블록 ${meshView.blockCount}, 삼각형 ${meshView.triangleCount}, 수신 ${meshView.rxHz}Hz`;
}

/**
 * HUD용 로봇 상태 문자열(공급자, 측정값 유무, 제어 상태 안내)을 반환한다.
 * hint: 기기별 조작 안내 {idle, armed} (없으면 기본 안내).
 */
function _robotStatus(status, hint = DEFAULT_CONTROL_HINT) {
    const sourceLabel = status.source === "robot" ? "실로봇" : "시뮬레이션";
    const armedLabel = status.armed === null ? "" :
        status.armed ? ` | ${hint.armed}` : ` | ${hint.idle}`;
    return `${sourceLabel}${status.live ? "" : " (측정값 없음)"}${armedLabel}`;
}

/**
 * XR 세션 시작/종료에 맞춰 HUD 패널·시점 고정·입력 모듈·상태 표시를 전환한다. 반환값 없음.
 */
function _wireSessionLifecycle(xr, hud, statusEl, enterBtn, robotView, input, rtc) {
    // 세션 상태 전환에 필요한 상태 묶음 (viewObserver: 시점 고정 옵저버)
    const lifecycle = { xr, hud, statusEl, enterBtn, robotView, input, rtc, viewObserver: null };
    xr.baseExperience.onStateChangedObservable.add((state) => _handleXRStateChange(state, lifecycle));
}

/**
 * XR 상태 변화를 처리한다. 반환값 없음.
 * IN_XR: HUD·시점 고정·입력 연결·자동 start 예약. NOT_IN_XR: home 전송 후 모두 해제.
 */
function _handleXRStateChange(state, lifecycle) {
    const { xr, hud, statusEl, enterBtn, robotView, input, rtc } = lifecycle;

    if (state === BABYLON.WebXRState.IN_XR) {
        // 로봇 시점 구성: HUD·시점 고정·머리 메시 숨김·입력 연결
        hud.attachToCamera(xr.baseExperience.camera);
        lifecycle.viewObserver = _attachViewToRobotHead(xr, robotView);
        robotView.setVisibleParts("body");
        input.attachSession(xr);

        // 세션 가시성 변화 처리 연결
        const session = xr.baseExperience.sessionManager.session;
        session.addEventListener("visibilitychange", () => _handleXRVisibilityChange(session, input, rtc));

        statusEl.textContent = "VR 세션 진행 중 (로봇 시점)";
        enterBtn.hidden = true;
    } else if (state === BABYLON.WebXRState.NOT_IN_XR) {
        // VR 종료: 로봇을 ready(홈) pose로 복귀시킨 뒤 제어 해제
        _sendHome(input, rtc, "VR 종료");

        // 표시·입력·시점 고정 해제
        hud.detach();
        input.detachSession();
        robotView.setVisibleParts("all");
        if (lifecycle.viewObserver !== null) {
            xr.baseExperience.sessionManager.onXRFrameObservable.remove(lifecycle.viewObserver);
            lifecycle.viewObserver = null;
        }

        statusEl.textContent = "VR 세션 종료됨 - 재진입 가능";
        enterBtn.hidden = false;
    }
}

/**
 * XR 세션 가시성 변화를 처리한다. 반환값 없음.
 * hidden: 자동 start를 취소하고 home 명령 샘플을 즉시 보낸다. 제어 재시작은 VR 재진입으로만 한다.
 */
function _handleXRVisibilityChange(session, input, rtc) {
    const visibilityState = session.visibilityState;
    console.log(`[main] XR 세션 가시성: ${visibilityState}`);
    if (visibilityState !== "hidden") return;

    // 숨김: 자동 start 취소 후 ready 복귀·제어 해제
    input.cancelAutoStart("세션 숨김 → home, VR 재진입 시 제어 시작");
    _sendHome(input, rtc, "XR 세션 숨김");
}

/**
 * home(ready 복귀 후 제어 해제) 명령 샘플을 보내고 결과를 로그로 남긴다. 반환값 없음.
 * reason: 로그용 전송 사유.
 */
function _sendHome(input, rtc, reason) {
    if (rtc.sendPose(input.exitSample())) {
        console.log(`[main] ${reason} → home 명령 전송 (ready 복귀 후 제어 해제)`);
    } else {
        console.warn(`[main] ${reason} → home 명령 유실: 서버 연결 끊김 (서버가 업링크 두절로 제자리 해제)`);
    }
}

/**
 * XR 시점을 로봇 머리 장착 카메라 위치에 고정하는 프레임 옵저버를 등록한다. 반환: 옵저버 (해제용).
 *
 * 매 XR 프레임 기본 기준 공간에 위치 오프셋(원점 t = 머리 위치 h - 목표)을 건 기준 공간을 세션 기준 공간으로 쓴다.
 * 머리 위치는 로봇 머리 카메라 위치에 고정되고 고개 회전은 유지된다.
 */
function _attachViewToRobotHead(xr, robotView) {
    const sessionManager = xr.baseExperience.sessionManager;
    // smoothedGoal: 평활된 목표(WebXR 좌표, 첫 유효 목표 전 null), appliedOrigin: 마지막 적용 오프셋 원점
    const viewLock = { smoothedGoal: null, lastFrameMs: 0, appliedOrigin: null };

    // XR 카메라 갱신보다 먼저 실행되도록 맨 앞에 등록
    return sessionManager.onXRFrameObservable.add(
        (frame) => _lockViewToRobotHead(frame, sessionManager, robotView, viewLock), undefined, true,
    );
}

/**
 * XR 프레임 1개에 대해 시점 목표를 평활하고 세션 기준 공간 오프셋을 갱신한다. 반환값 없음.
 */
function _lockViewToRobotHead(frame, sessionManager, robotView, viewLock) {
    const baseSpace = sessionManager.baseReferenceSpace;
    const viewer = baseSpace ? frame.getViewerPose(baseSpace) : null;
    if (!viewer) return;

    // 목표 계산 (Babylon 왼손 좌표 → WebXR 오른손 좌표, 로봇 머리 위치 미수신이면 바닥 아래)
    const target = robotView.cameraAnchorPosition();
    const now = performance.now();
    let goal;
    if (!target) {
        goal = VIEW_HIDDEN_GOAL;
        viewLock.smoothedGoal = null;
    } else if (viewLock.smoothedGoal === null) {
        // 첫 유효 목표는 즉시 적용
        viewLock.smoothedGoal = { x: target.x, y: target.y, z: -target.z };
        goal = viewLock.smoothedGoal;
        console.log(`[view] 로봇 머리 시점 고정 시작: (${goal.x.toFixed(2)}, ${goal.y.toFixed(2)}, ${goal.z.toFixed(2)})`);
    } else {
        // 프레임 시간 기반 지수 평활
        const dtSec = Math.max(0, (now - viewLock.lastFrameMs) / 1000);
        const alpha = 1 - Math.exp(-dtSec / VIEW_SMOOTHING_TAU_S);
        goal = viewLock.smoothedGoal;
        goal.x += alpha * (target.x - goal.x);
        goal.y += alpha * (target.y - goal.y);
        goal.z += alpha * (-target.z - goal.z);
    }
    viewLock.lastFrameMs = now;

    // 오프셋 원점 = 머리 위치 - 목표
    const headPosition = viewer.transform.position;
    const originX = headPosition.x - goal.x;
    const originY = headPosition.y - goal.y;
    const originZ = headPosition.z - goal.z;

    // 원점 변화가 임계값 미만이면 기존 기준 공간 유지
    const applied = viewLock.appliedOrigin;
    if (applied !== null &&
        Math.abs(originX - applied.x) < VIEW_OFFSET_EPSILON_M &&
        Math.abs(originY - applied.y) < VIEW_OFFSET_EPSILON_M &&
        Math.abs(originZ - applied.z) < VIEW_OFFSET_EPSILON_M) {
        return;
    }

    // 새 오프셋 기준 공간 적용
    viewLock.appliedOrigin = { x: originX, y: originY, z: originZ, w: 1 };
    sessionManager.referenceSpace = baseSpace.getOffsetReferenceSpace(
        new XRRigidTransform(viewLock.appliedOrigin, VIEW_NO_ROTATION),
    );
}

/**
 * XR 프레임마다 업링크 전송과 HUD 갱신을 수행하는 옵저버를 등록한다. 반환값 없음.
 */
function _wireFrameLoop(xr, hud, input, rtc, meshView, robotStatus) {
    const frameLoop = { xr, hud, input, rtc, meshView, robotStatus, lastHudUpdateMs: 0 };
    xr.baseExperience.sessionManager.onXRFrameObservable.add((frame) => _handleXRFrame(frame, frameLoop));
}

/**
 * XR 프레임 1개를 처리한다: 전송 차례면 샘플을 만들어 보내고, HUD_INTERVAL_MS마다 HUD를 갱신한다. 반환값 없음.
 */
function _handleXRFrame(frame, frameLoop) {
    const { xr, hud, input, rtc, meshView, robotStatus } = frameLoop;
    const sessionManager = xr.baseExperience.sessionManager;

    // 입력은 시점 오프셋이 없는 기본 기준 공간(실제 방 좌표)에서 읽는다
    const refSpace = sessionManager.baseReferenceSpace;
    const session = sessionManager.session;
    if (!frame || !refSpace || !session) return;

    // 전송 차례일 때만 샘플 생성·전송 (명령은 샘플 생성 시 실린다)
    input.setRobotArmed(robotStatus.armed);
    if (rtc.isPoseDue()) {
        const sample = input.read(frame, refSpace, session);
        const isSent = rtc.sendPose(sample);

        // 명령이 실린 샘플을 못 보냈으면 경고
        if (sample.command != null && !isSent) {
            console.warn(`[main] 명령 '${sample.command}' 유실: 서버 연결 끊김`);
        }
    }

    // HUD 갱신 (마지막 샘플 기준, body는 조인트 수만)
    const now = performance.now();
    if (now - frameLoop.lastHudUpdateMs >= HUD_INTERVAL_MS) {
        frameLoop.lastHudUpdateMs = now;
        hud.update(formatSample(input.lastSample, [
            `${input.trackingStatus}` +
            ` | uplink: ${rtc.connected ? `${rtc.txHz}Hz` : "끊김"}`,
            `로봇: ${_robotStatus(robotStatus, input.controlHint)}`,
            _meshStatus(meshView),
        ], false));
    }
}

/**
 * 초기화 실패를 로그와 상태 문구로 알린다. 반환값 없음.
 */
function _handleBootFailure(error) {
    console.error("초기화 실패:", error);
    document.getElementById("status").textContent = `초기화 실패: ${error.message}`;
}

boot().catch(_handleBootFailure);
