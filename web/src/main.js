// 페이지 엔트리포인트: 씬 구성, VR 진입/종료, 포즈 수집·업링크, 가상 로봇·포인트클라우드 렌더링.
// PC 접속: 데스크톱 뷰 (포즈 마커 + 가상 로봇 + 포인트클라우드 모니터링)
// Quest 접속: Enter VR → 포즈 업링크 + 눈앞 가상 로봇이 IK 결과로 팔을 따라 움직임 + 카메라 포인트클라우드

import { createScene } from "./render/scene.js";
import { Hud, formatSample } from "./render/hud.js";
import { PoseMarkers } from "./render/pose_markers.js";
import { RobotView } from "./render/robot_view.js";
import { PointCloudView } from "./render/pointcloud_view.js";
import { PoseReader } from "./xr/pose_reader.js";
import { createXR, enterVR, isVRSupported } from "./xr/xr_session.js";
import { PoseUplink } from "./net/uplink.js";
import { MonitorLink } from "./net/monitor.js";
import { decodePointCloud } from "./net/pointcloud.js";

// HUD 갱신 주기(ms): 매 XR 프레임(72-120Hz) 갱신은 텍스처 낭비라 10Hz로 제한
const HUD_INTERVAL_MS = 100;

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
    const onBinary = (buf) => _routeBinary(buf, pcdView);

    // immersive-vr 미지원(PC)에서는 XR 경험 자체를 만들지 않는다
    // (유휴 XR 카메라가 씬에 남으면 데스크톱 렌더링을 오염시킬 수 있음)
    const vrSupported = await isVRSupported();
    const xr = vrSupported ? await createXR(scene) : null;
    if (xr && xr.baseExperience) {
        // VR 기기: 업링크로 포즈 송신 + 같은 소켓으로 robot_config/robot_state/포인트클라우드 수신
        const uplink = new PoseUplink((msg) => _routeRobotMessage(msg, robotView), onBinary);
        uplink.connect();

        enterBtn.hidden = false;
        statusEl.textContent = "VR 사용 가능 - Enter VR을 누르세요";

        enterBtn.addEventListener("click", async () => {
            const ok = await enterVR(xr);
            if (!ok) statusEl.textContent = "VR 진입 실패 - 콘솔 확인";
        });

        _wireSessionLifecycle(xr, hud, statusEl, enterBtn);
        _wireFrameLoop(xr, hud, poseReader, uplink, pcdView);
    } else {
        // PC: 모니터 소켓으로 VR 포즈(마커)·로봇 상태·포인트클라우드를 함께 수신
        statusEl.textContent = "데스크톱 모니터링 뷰 (VR 포즈 수신 대기)";
        const markers = new PoseMarkers(scene);
        const monitor = new MonitorLink((msg) => {
            if (!_routeRobotMessage(msg, robotView)) {
                markers.update(msg);
                hud.update(formatSample(msg, ["모니터링: VR 포즈 수신 중", _pcdStatus(pcdView)]));
            }
        }, onBinary);
        monitor.connect();
    }

    engine.runRenderLoop(() => scene.render());
    window.addEventListener("resize", () => engine.resize());

    // 원격 디버깅용 훅 (콘솔·CDP에서 씬 상태 점검)
    window.__teleop = { engine, scene, robotView, pcdView };
}

/**
 * 서버 발신 로봇 메시지를 RobotView로 라우팅한다. 반환: 로봇 메시지였으면 true.
 */
function _routeRobotMessage(msg, robotView) {
    if (msg.type === "robot_config") {
        robotView.applyConfig(msg);
        return true;
    }
    if (msg.type === "robot_state") {
        robotView.applyState(msg);
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
 * XR 세션 시작/종료에 맞춰 HUD 패널(EXIT 버튼 포함)과 상태 표시를 전환한다. 반환값 없음.
 */
function _wireSessionLifecycle(xr, hud, statusEl, enterBtn) {
    xr.baseExperience.onStateChangedObservable.add((state) => {
        if (state === BABYLON.WebXRState.IN_XR) {
            // EXIT 버튼 클릭(컨트롤러 레이) 시 세션을 종료한다
            hud.attachToCamera(xr.baseExperience.camera, () => xr.baseExperience.exitXRAsync());
            statusEl.textContent = "VR 세션 진행 중";
            enterBtn.hidden = true;
        } else if (state === BABYLON.WebXRState.NOT_IN_XR) {
            hud.detach();
            statusEl.textContent = "VR 세션 종료됨 - 재진입 가능";
            enterBtn.hidden = false;
        }
    });
}

/**
 * XR 프레임마다 포즈를 수집·업링크 전송하고 일정 주기로 HUD를 갱신한다. 반환값 없음.
 */
function _wireFrameLoop(xr, hud, poseReader, uplink, pcdView) {
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
                _pcdStatus(pcdView),
            ]));
        }
    });
}

boot().catch((e) => {
    console.error("초기화 실패:", e);
    document.getElementById("status").textContent = `초기화 실패: ${e.message}`;
});
