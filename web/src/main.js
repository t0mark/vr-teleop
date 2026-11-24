// 페이지 엔트리포인트: 씬 구성, VR 진입/종료, 포즈 수집·업링크, 로봇·사람·재구성 메시 렌더링.
//
// 두 가지 시점을 한 페이지에서 제공한다.
// - VR (Quest 3): **로봇 시점**. 사용자 시점 위치를 로봇 머리 장착 카메라 위치에 고정하고 회전은 자유롭게 둔다.
//   재구성 메시(환경·작업 영역)와 로봇 몸(머리 제외 — 시점이 머리 카메라라 시야를 가린다), UI(HUD)를 그린다.
// - PC 브라우저: **관찰 시점**. 로봇 전체 + 조작자 아바타를 나란히 놓고 재구성 메시까지 함께 본다.
//
// 장면은 전부 시뮬레이션처럼 메시로 표현한다: 주변·작업 영역은 서버 TSDF 재구성 메시, 로봇은 관절 측정값으로
// 움직이는 로봇 모델. 둘 다 로봇 base 기준(씬 원점)에 놓인다.
// 통신은 전부 WebRTC 하나(RtcLink)로 이뤄진다 — state 채널(포즈·상태·명령)과 mesh 채널(MSH1 바이너리)
// (rtc_link.js, net/packets.js).
//
// 입력은 Quest 3 컨트롤러 모듈(web/src/input/controller_input.js)이 손등에 부착한 컨트롤러 포즈를 공통 샘플
// 형식으로 바꾼다. 손가락은 서버에 붙은 Manus 장갑이 직접 측정하므로 웹은 손 위치·명령·body만 보낸다.
// 명령은 세션 수명으로 자동 생성한다: VR 진입 2초 후 start, VR 종료 시 home(ready 복귀 후 해제).

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

// HUD 갱신 주기(ms): 매 XR 프레임(72-120Hz) 갱신은 텍스처 낭비라 10Hz로 제한
const HUD_INTERVAL_MS = 100;
// 시점 목표 지수 평활 시정수(s): 30Hz 로봇 상태 갱신을 XR 프레임 단위로 부드럽게 보간
const VIEW_SMOOTHING_TAU_S = 0.05;
// 사람 아바타 에셋 (web/assets/, 서버가 /assets/로 정적 서빙)
const AVATAR_BASE = "/assets/";
const AVATAR_FILE = "avatar.glb";
// HUD 제어 상태 안내 기본값 (데스크톱 관찰 뷰: VR 진입 시 자동 시작, 수동 조작은 A/B)
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

    const { engine, scene } = createScene(canvas);
    const hud = new Hud(scene, overlayEl);
    const robotView = new RobotView(scene);
    const meshView = new MeshView(scene);
    // 로봇 상태 공급자(robot/sim)를 HUD에 표시하기 위해 마지막 값을 들고 있는다
    const robotStatus = { source: "?", live: false, armed: null };
    const onMeshPacket = (buf) => _routeMeshPacket(buf, meshView);

    // immersive-vr 미지원(PC)에서는 XR 경험 자체를 만들지 않는다
    // (유휴 XR 카메라가 씬에 남으면 데스크톱 렌더링을 오염시킬 수 있음)
    const vrSupported = await isVRSupported();
    const xr = vrSupported ? await createXR(scene) : null;
    let human = null;

    if (xr && xr.baseExperience) {
        // VR 기기: 컨트롤러 입력 모듈 생성
        const input = new ControllerInput();
        robotView.setVisibleParts("all");
        const rtc = new RtcLink(
            (msg) => _routeRobotMessage(msg, robotView, robotStatus), onMeshPacket,
        );
        rtc.connect();

        enterBtn.hidden = false;
        statusEl.textContent = "VR 사용 가능 - Enter VR을 누르세요";

        enterBtn.addEventListener("click", async () => {
            const ok = await enterVR(xr, input.xrFeatures);
            if (!ok) statusEl.textContent = "VR 진입 실패 - 콘솔 확인";
        });

        _wireSessionLifecycle(xr, hud, statusEl, enterBtn, robotView, input, rtc);
        _wireFrameLoop(xr, hud, input, rtc, meshView, robotStatus);
    } else {
        // PC: 로봇 전체 + 조작자 아바타 + 재구성 메시를 관찰한다
        robotView.setVisibleParts("all");
        statusEl.textContent = "데스크톱 모니터링 뷰 (VR 포즈 수신 대기)";

        human = new HumanView(scene);
        // 아바타 로드는 비동기지만 수신 루프를 막을 이유가 없다 (준비 전 샘플은 무시된다)
        human.load(AVATAR_BASE, AVATAR_FILE);

        const modeBtn = document.getElementById("modeBtn");
        const modeUi = { mode: null, switching: false };
        // 수동 조작 패널: VR 대신 같은 형식의 샘플을 같은 연결로 보낸다 (dev 모드에서만 표시)
        const manual = new ManualControl();

        const rtc = new RtcLink((msg) => {
            if (msg.type === "mode_status") {
                _applyModeStatus(msg, modeUi, modeBtn, manual);
                return;
            }
            if (_routeRobotMessage(msg, robotView, robotStatus)) return;
            human.update(msg);
            hud.update(formatSample(msg, [
                `로봇: ${_robotStatus(robotStatus)}`,
                _meshStatus(meshView),
            ]));
        }, onMeshPacket);
        rtc.connect();

        manual.setLink(rtc);
        _wireModeButton(modeBtn, modeUi, rtc);
        window.__teleop_manual = manual;
    }

    engine.runRenderLoop(() => scene.render());
    window.addEventListener("resize", () => engine.resize());

    // 원격 디버깅용 훅 (콘솔·CDP에서 씬 상태 점검)
    window.__teleop = { engine, scene, robotView, meshView, human, robotStatus };
}

/**
 * 서버 발신 로봇 메시지를 RobotView로 라우팅한다. 반환: 로봇 메시지였으면 true.
 * status: 마지막 공급자·측정값 수신 여부를 담을 객체 (HUD 표시용).
 */
function _routeRobotMessage(msg, robotView, status) {
    if (msg.type === "robot_config") {
        robotView.applyConfig(msg);
        return true;
    }
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
 * mesh DataChannel로 받은 바이너리 패킷을 디코드해 메시 뷰에 적용한다. 반환값 없음.
 */
function _routeMeshPacket(buf, meshView) {
    const mesh = decodeMeshBlocks(buf);
    if (mesh !== null) meshView.apply(mesh);
}

/**
 * 서버 mode_status 메시지를 모드 버튼·수동 조작 패널에 반영한다. 반환값 없음.
 * ui: {mode, switching} — 버튼 클릭 로직이 참조하는 최신 상태.
 * 버튼은 현재 상태가 아니라 **전환 동작**만 표시한다. 현재 모드는 HUD의 로봇 항목으로 보인다.
 */
function _applyModeStatus(msg, ui, modeBtn, manual) {
    ui.mode = msg.mode;
    ui.switching = Boolean(msg.switching);

    modeBtn.hidden = false;
    modeBtn.disabled = ui.switching;
    const target = ui.mode === "deploy" ? "dev" : "deploy";
    modeBtn.classList.toggle("to-deploy", target === "deploy");
    modeBtn.textContent = ui.switching ? "전환 중…" : `${target}로 전환`;

    // 수동 조작은 dev에서만 — deploy로 바뀌면 패널을 숨기고 송신도 멈춘다
    manual.setDevMode(ui.mode === "dev" && !ui.switching);

    // 실패(전환 거부·연결 실패)만 알린다 — 서버는 기존 모드를 유지한다
    if (!msg.ok) alert(`모드 전환 실패: ${msg.message}`);
}

/**
 * 모드 버튼 클릭 → set_mode 명령 전송을 연결한다. 반환값 없음.
 * 실로봇에 연결하는 deploy 전환만 확인 대화상자를 거친다 (dev 전환은 항상 안전).
 */
function _wireModeButton(modeBtn, ui, rtc) {
    modeBtn.addEventListener("click", () => {
        if (ui.switching || ui.mode === null) return;
        const target = ui.mode === "deploy" ? "dev" : "deploy";
        if (target === "deploy" &&
            !confirm("deploy 모드로 전환합니다.\n실로봇(RBY1)에 연결됩니다 — 로봇 주변 안전을 확인했습니까?")) {
            return;
        }
        if (!rtc.send({ type: "set_mode", mode: target })) {
            alert("서버 연결이 끊겨 있어 전환 요청을 보낼 수 없습니다.");
        }
    });
}

/**
 * HUD용 재구성 메시 수신 상태 문자열을 반환한다.
 */
function _meshStatus(meshView) {
    return `mesh: 블록 ${meshView.blockCount}, 삼각형 ${meshView.triangleCount}, 수신 ${meshView.rxHz}Hz`;
}

/**
 * HUD용 로봇 상태 문자열을 반환한다 — 공급자(실로봇/시뮬)와 제어 상태(대기/제어 중)를 한눈에.
 * 제어가 대기면 로봇은 조작을 따라오지 않는다 — 그 이유가 화면에 보여야 한다.
 * hint: 기기별 조작 안내 {idle, armed} (입력 모듈 controlHint, 없으면 기기 무관 기본 안내).
 */
function _robotStatus(status, hint = DEFAULT_CONTROL_HINT) {
    const label = status.source === "robot" ? "실로봇" : "시뮬레이션";
    const armed = status.armed === null ? "" :
        status.armed ? ` | ${hint.armed}` : ` | ${hint.idle}`;
    return `${label}${status.live ? "" : " (측정값 없음)"}${armed}`;
}

/**
 * XR 세션 시작/종료에 맞춰 HUD 패널·시점 고정·입력 모듈·상태 표시를 전환한다. 반환값 없음.
 * 진입 시 입력 모듈이 자동 start를 예약하고, 종료 시 home(ready 복귀 후 제어 해제) 명령을 보낸다.
 */
function _wireSessionLifecycle(xr, hud, statusEl, enterBtn, robotView, input, rtc) {
    let detachView = null;

    xr.baseExperience.onStateChangedObservable.add((state) => {
        if (state === BABYLON.WebXRState.IN_XR) {
            hud.attachToCamera(xr.baseExperience.camera);
            detachView = _attachViewToRobotHead(xr, robotView);
            // 몰입형 로봇 시점: 로봇 몸은 보이되 머리(시점 위치)는 시야를 가리므로 숨긴다
            robotView.setVisibleParts("body");
            input.attachSession(xr);
            statusEl.textContent = "VR 세션 진행 중 (로봇 시점)";
            enterBtn.hidden = true;
        } else if (state === BABYLON.WebXRState.NOT_IN_XR) {
            // VR 종료: 로봇을 ready(홈) pose로 복귀시킨 뒤 제어 해제
            if (rtc.sendPose(input.exitSample())) {
                console.log("[main] VR 종료 → home 명령 전송 (ready 복귀 후 제어 해제)");
            } else {
                console.warn("[main] home 명령 유실: 서버 연결 끊김 (서버가 업링크 두절로 제자리 해제)");
            }
            hud.detach();
            input.detachSession();
            // 몰입형 종료 후 기본 화면에서는 로봇 모델을 다시 보인다
            robotView.setVisibleParts("all");
            if (detachView !== null) {
                detachView();
                detachView = null;
            }
            statusEl.textContent = "VR 세션 종료됨 - 재진입 가능";
            enterBtn.hidden = false;
        }
    });
}

/**
 * XR 시점을 로봇 머리 장착 카메라 위치에 고정한다. 반환: 해제 함수.
 *
 * 매 XR 프레임 기본 기준 공간(baseReferenceSpace)에 위치 오프셋을 걸어 새 기준 공간을 만들고,
 * 세션 기준 공간(referenceSpace)을 그것으로 바꾼다. 머리 위치는 항상 로봇 머리 카메라 위치가 되고
 * 고개 이동은 상쇄, 고개 회전은 유지된다. 진입 시 사용자 정면(WebXR -z)은 Babylon +z = 로봇 정면이다.
 * XR 카메라와 Babylon 손·포인터 메시는 이 기준 공간을 그대로 따르므로 부모 노드가 필요 없다.
 *
 * 오프셋 수식 (WebXR 오른손 좌표): 기본 공간 머리 위치 h = t + 목표  →  오프셋 원점 t = h - 목표
 */
function _attachViewToRobotHead(xr, robotView) {
    const sessionManager = xr.baseExperience.sessionManager;
    const noRotation = { x: 0, y: 0, z: 0, w: 1 };
    // 평활된 목표(WebXR 좌표, 첫 유효 목표 전에는 null)와 직전 프레임 시각(ms)
    let smoothedGoal = null;
    let lastFrameMs = 0;

    // XR 카메라 갱신보다 먼저 실행되도록 맨 앞에 등록한다
    const observer = sessionManager.onXRFrameObservable.add((frame) => {
        const baseSpace = sessionManager.baseReferenceSpace;
        const viewer = baseSpace ? frame.getViewerPose(baseSpace) : null;
        if (!viewer) return;

        // 목표(Babylon 왼손 좌표 → WebXR 오른손 좌표). 로봇 머리 위치를 받기 전에는 바닥 아래에 둬 HUD·버튼 배치를 미룬다
        const target = robotView.cameraAnchorPosition();
        const now = performance.now();
        let goal;
        if (!target) {
            goal = { x: 0, y: -100, z: 0 };
            smoothedGoal = null;
        } else {
            const rawGoal = { x: target.x, y: target.y, z: -target.z };

            // 첫 유효 목표는 즉시 적용, 이후 프레임 시간 기반 지수 평활
            if (smoothedGoal === null) {
                smoothedGoal = rawGoal;
                console.log(`[view] 로봇 머리 시점 고정 시작: (${rawGoal.x.toFixed(2)}, ${rawGoal.y.toFixed(2)}, ${rawGoal.z.toFixed(2)})`);
            } else {
                const dt = Math.max(0, (now - lastFrameMs) / 1000);
                const alpha = 1 - Math.exp(-dt / VIEW_SMOOTHING_TAU_S);
                smoothedGoal = {
                    x: smoothedGoal.x + alpha * (rawGoal.x - smoothedGoal.x),
                    y: smoothedGoal.y + alpha * (rawGoal.y - smoothedGoal.y),
                    z: smoothedGoal.z + alpha * (rawGoal.z - smoothedGoal.z),
                };
            }
            goal = smoothedGoal;
        }
        lastFrameMs = now;

        // 오프셋 원점 = 머리 위치 - 목표
        const head = viewer.transform.position;
        const origin = { x: head.x - goal.x, y: head.y - goal.y, z: head.z - goal.z, w: 1 };
        sessionManager.referenceSpace = baseSpace.getOffsetReferenceSpace(new XRRigidTransform(origin, noRotation));
    }, undefined, true);

    return () => {
        sessionManager.onXRFrameObservable.remove(observer);
    };
}

/**
 * XR 프레임마다 입력 모듈로 공통 샘플을 만들어 업링크 전송하고 일정 주기로 HUD를 갱신한다. 반환값 없음.
 */
function _wireFrameLoop(xr, hud, input, rtc, meshView, robotStatus) {
    let lastHudUpdate = 0;

    xr.baseExperience.sessionManager.onXRFrameObservable.add((frame) => {
        const sessionManager = xr.baseExperience.sessionManager;
        // 입력은 시점 오프셋이 없는 기본 기준 공간(실제 방 좌표)에서 읽는다
        const refSpace = sessionManager.baseReferenceSpace;
        const session = sessionManager.session;
        if (!frame || !refSpace || !session) return;

        const sample = input.read(frame, refSpace, session);
        const sent = rtc.sendPose(sample);

        // 명령이 실린 샘플을 못 보냈으면 경고 (오래된 명령은 위험하므로 재전송하지 않음)
        if (sample.command != null && !sent) {
            console.warn(`[main] 명령 '${sample.command}' 유실: 서버 연결 끊김`);
        }

        // 수집·전송은 매 프레임, 표시는 10Hz
        const now = performance.now();
        if (now - lastHudUpdate >= HUD_INTERVAL_MS) {
            lastHudUpdate = now;
            hud.update(formatSample(sample, [
                `${input.trackingStatus}` +
                ` | uplink: ${rtc.connected ? `${rtc.txHz}Hz` : "끊김"}`,
                `로봇: ${_robotStatus(robotStatus, input.controlHint)}`,
                _meshStatus(meshView),
            ]));
        }
    });
}

boot().catch((e) => {
    console.error("초기화 실패:", e);
    document.getElementById("status").textContent = `초기화 실패: ${e.message}`;
});
