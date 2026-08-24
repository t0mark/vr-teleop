// 페이지 엔트리포인트: 씬 구성, VR 진입/종료, 포즈 수집·업링크, 로봇·사람·포인트클라우드 렌더링.
//
// 두 가지 시점을 한 페이지에서 제공한다.
// - VR (Quest): **로봇 시점**. 사용자 눈을 로봇 머리 장착 카메라 위치에 고정하고 로봇은 팔 계통만
//   그린다 (시점이 머리 안이라 몸통·머리 메시를 그리면 내부 면이 화면을 덮는다).
//   머리 회전은 그대로 살리고 위치만 고정한다 — 로봇 머리가 사용자를 따라 돌지 않는 현 단계에서
//   위치까지 따라 움직이면 로봇 눈과 시점이 어긋나기 때문이다.
// - PC 브라우저: **관찰 시점**. 로봇 전체 + 조작자 아바타를 나란히 놓고 포인트클라우드까지 함께 본다.
//
// 포인트클라우드는 두 시점 모두 로봇 base 기준(씬 원점)에 놓여 로봇 주변에 그려진다.

import { createScene } from "./render/scene.js";
import { Hud, formatSample } from "./render/hud.js";
import { HumanView } from "./render/human_view.js";
import { RobotView } from "./render/robot_view.js";
import { PointCloudView } from "./render/pointcloud_view.js";
import { PoseReader } from "./xr/pose_reader.js";
import { createXR, enterVR, isVRSupported } from "./xr/xr_session.js";
import { PoseUplink } from "./net/uplink.js";
import { MonitorLink } from "./net/monitor.js";
import { decodePointCloud } from "./net/pointcloud.js";

// HUD 갱신 주기(ms): 매 XR 프레임(72-120Hz) 갱신은 텍스처 낭비라 10Hz로 제한
const HUD_INTERVAL_MS = 100;
// 사람 아바타 에셋 (web/assets/, 서버가 /assets/로 정적 서빙)
const AVATAR_BASE = "/assets/";
const AVATAR_FILE = "avatar.glb";

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
    const poseReader = new PoseReader();
    const robotView = new RobotView(scene);
    const pcdView = new PointCloudView(scene);
    // 로봇 상태 공급자(robot/sim)를 HUD에 표시하기 위해 마지막 값을 들고 있는다
    const robotStatus = { source: "?", live: false };
    const onBinary = (buf) => _routeBinary(buf, pcdView);

    // immersive-vr 미지원(PC)에서는 XR 경험 자체를 만들지 않는다
    // (유휴 XR 카메라가 씬에 남으면 데스크톱 렌더링을 오염시킬 수 있음)
    const vrSupported = await isVRSupported();
    const xr = vrSupported ? await createXR(scene) : null;
    let human = null;

    if (xr && xr.baseExperience) {
        // VR 기기: 로봇 시점이므로 팔 계통만 그린다
        robotView.setVisibleParts("arms");
        const uplink = new PoseUplink(
            (msg) => _routeRobotMessage(msg, robotView, robotStatus), onBinary,
        );
        uplink.connect();

        enterBtn.hidden = false;
        statusEl.textContent = "VR 사용 가능 - Enter VR을 누르세요";

        enterBtn.addEventListener("click", async () => {
            const ok = await enterVR(xr);
            if (!ok) statusEl.textContent = "VR 진입 실패 - 콘솔 확인";
        });

        _wireSessionLifecycle(scene, xr, hud, statusEl, enterBtn, robotView);
        _wireFrameLoop(xr, hud, poseReader, uplink, pcdView, robotStatus);
    } else {
        // PC: 로봇 전체 + 조작자 아바타 + 포인트클라우드를 관찰한다
        robotView.setVisibleParts("all");
        statusEl.textContent = "데스크톱 모니터링 뷰 (VR 포즈 수신 대기)";

        human = new HumanView(scene);
        // 아바타 로드는 비동기지만 수신 루프를 막을 이유가 없다 (준비 전 샘플은 무시된다)
        human.load(AVATAR_BASE, AVATAR_FILE);

        const monitor = new MonitorLink((msg) => {
            if (_routeRobotMessage(msg, robotView, robotStatus)) return;
            human.update(msg);
            hud.update(formatSample(msg, [
                `로봇: ${_robotStatus(robotStatus)}`,
                _pcdStatus(pcdView),
            ]));
        }, onBinary);
        monitor.connect();
    }

    engine.runRenderLoop(() => scene.render());
    window.addEventListener("resize", () => engine.resize());

    // 원격 디버깅용 훅 (콘솔·CDP에서 씬 상태 점검)
    window.__teleop = { engine, scene, robotView, pcdView, human, robotStatus };
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
        return true;
    }
    return false;
}

/**
 * 서버 발신 바이너리 패킷을 디코드해 포인트클라우드 뷰로 전달한다. 반환값 없음.
 */
function _routeBinary(buf, pcdView) {
    const pcd = decodePointCloud(buf);
    if (pcd !== null) pcdView.update(pcd);
}

/**
 * HUD용 포인트클라우드 수신 상태 문자열을 반환한다.
 */
function _pcdStatus(pcdView) {
    return `pointcloud: ${pcdView.rxHz}Hz, ${pcdView.lastCount}점`;
}

/**
 * HUD용 로봇 상태 공급자 문자열을 반환한다 (실로봇인지 시뮬레이션인지 한눈에 보이게).
 */
function _robotStatus(status) {
    const label = status.source === "robot" ? "실로봇" : "시뮬레이션";
    return `${label}${status.live ? "" : " (측정값 없음)"}`;
}

/**
 * XR 세션 시작/종료에 맞춰 HUD 패널·시점 고정·상태 표시를 전환한다. 반환값 없음.
 */
function _wireSessionLifecycle(scene, xr, hud, statusEl, enterBtn, robotView) {
    let detachView = null;

    xr.baseExperience.onStateChangedObservable.add((state) => {
        if (state === BABYLON.WebXRState.IN_XR) {
            // EXIT 버튼 클릭(컨트롤러 레이) 시 세션을 종료한다
            hud.attachToCamera(xr.baseExperience.camera, () => xr.baseExperience.exitXRAsync());
            detachView = _attachViewToRobotHead(scene, xr, robotView);
            statusEl.textContent = "VR 세션 진행 중 (로봇 시점)";
            enterBtn.hidden = true;
        } else if (state === BABYLON.WebXRState.NOT_IN_XR) {
            hud.detach();
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
 * XR 카메라를 로봇 머리 장착 카메라 위치에 고정한다. 반환: 해제 함수.
 *
 * XR 카메라의 위치는 헤드셋이 매 프레임 덮어쓰므로 직접 옮길 수 없다. 대신 부모 노드를 두고
 * 부모 위치 = 목표 - 카메라 로컬 위치로 잡아 월드 위치를 목표에 고정한다 (회전은 건드리지 않아
 * 고개 돌리기는 그대로 동작한다). 갱신은 XR 카메라가 갱신된 뒤인 렌더 직전에 한다.
 */
function _attachViewToRobotHead(scene, xr, robotView) {
    const anchor = new BABYLON.TransformNode("xrViewAnchor", scene);
    const camera = xr.baseExperience.camera;
    camera.parent = anchor;

    const observer = scene.onBeforeRenderObservable.add(() => {
        const target = robotView.cameraAnchorPosition();
        if (target === null) return;
        anchor.position.set(
            target.x - camera.position.x,
            target.y - camera.position.y,
            target.z - camera.position.z,
        );
    });

    return () => {
        scene.onBeforeRenderObservable.remove(observer);
        camera.parent = null;
        anchor.dispose();
    };
}

/**
 * XR 프레임마다 포즈를 수집·업링크 전송하고 일정 주기로 HUD를 갱신한다. 반환값 없음.
 */
function _wireFrameLoop(xr, hud, poseReader, uplink, pcdView, robotStatus) {
    let lastHudUpdate = 0;

    xr.baseExperience.sessionManager.onXRFrameObservable.add((frame) => {
        const sessionManager = xr.baseExperience.sessionManager;
        const refSpace = sessionManager.referenceSpace;
        const session = sessionManager.session;
        if (!frame || !refSpace || !session) return;

        const sample = poseReader.read(frame, refSpace, session);
        uplink.send(sample);

        // 수집·전송은 매 프레임, 표시는 10Hz
        const now = performance.now();
        if (now - lastHudUpdate >= HUD_INTERVAL_MS) {
            lastHudUpdate = now;
            hud.update(formatSample(sample, [
                `body-tracking: ${poseReader.bodyJointNames === null ? "대기/미지원" : "활성"}` +
                ` | uplink: ${uplink.connected ? `${uplink.txHz}Hz` : "끊김"}`,
                `로봇: ${_robotStatus(robotStatus)}`,
                _pcdStatus(pcdView),
            ]));
        }
    });
}

boot().catch((e) => {
    console.error("초기화 실패:", e);
    document.getElementById("status").textContent = `초기화 실패: ${e.message}`;
});
