// 컨트롤러 입력 모듈 (Quest 3): 조작자 손등에 부착한 컨트롤러 포즈를 공통 샘플 형식으로 변환한다.
//
// 공통 샘플 형식 (기기 무관, 서버 teleop/utils/inputs.py가 해석):
//   { t, hands: { left/right: { pos[3], quat[x,y,z,w], follow } | null },
//     head: { pos[3], quat[x,y,z,w] } | null,
//     command: "start" | "home" | null, body: { supported, jointCount, joints } | null }
// head = 조작자 시점(XR viewer) 포즈 — 서버가 로봇 머리를 조작자 시선 방향으로 돌린다.
// 손가락(fingers)은 싣지 않는다 — 서버에 붙은 Manus 장갑(teleop/glove/manus_glove.py)이 직접 측정한다.
//
// 매핑: 손 포즈 = 컨트롤러 grip 포즈 + 장착 오프셋(grip 로컬 → 손바닥 중심), 추적되는 동안 항상 follow
// (재중심(reset) 직후 잠시 follow 해제). 서버는 클러치 델타 매핑이라 장착 각도는 자세 델타에서 상쇄된다.
// 명령: 컨트롤러가 손등에 붙어 있어 레이·버튼 조작이 불가하므로 세션 수명에 맞춰 자동으로 만든다.
//   - VR 진입(attachSession) 후 AUTO_START_DELAY_MS가 지나면 "start" 1회 (제어 시작 → ready 이동 → 추종)
//   - VR 종료 시 main.js가 exitSample()의 "home"(ready 복귀 후 제어 해제)을 보낸다
// body = body-tracking 조인트(body_reader.js, 팔꿈치 유도).
// 좌표계: WebXR reference space(오른손, y-up), 단위 m. 쿼터니언은 (x, y, z, w).

import { BodyReader } from "./body_reader.js";

// 세션에 요청할 WebXR 선택 기능 (손 추적은 요청하지 않는다 — 컨트롤러 포즈만 쓴다)
const XR_FEATURES = ["body-tracking"];
// VR 진입 후 자동 제어 시작까지 대기 시간(ms)
const AUTO_START_DELAY_MS = 2000;
// 재중심(reset) 후 follow를 끊는 시간(ms): 서버가 클러치를 풀고 새 위치에서 다시 기준을 잡게 함
const CLUTCH_BREAK_MS = 200;
// 컨트롤러 grip 로컬 좌표계에서 손바닥 중심까지의 오프셋(m): 장착 위치 실측 후 보정
// (손목 회전 시 장착 지점과 손바닥 사이 레버암만큼 생기는 위치 오차를 없앤다)
const PALM_OFFSET_LOCAL = [0, 0, 0];
// HUD 제어 상태 안내 문구
const CONTROL_HINT = {
    idle: "제어 대기 — VR 진입 2초 후 자동 시작",
    armed: "제어 중 (VR 종료 시 홈 복귀 후 해제)",
};

/**
 * 벡터 v를 쿼터니언 q(x, y, z, w)로 회전한 결과를 반환한다.
 */
function _rotateByQuat(v, q) {
    // t = 2·(q_xyz × v), v' = v + w·t + q_xyz × t
    const [qx, qy, qz, qw] = q;
    const tx = 2 * (qy * v[2] - qz * v[1]);
    const ty = 2 * (qz * v[0] - qx * v[2]);
    const tz = 2 * (qx * v[1] - qy * v[0]);
    return [
        v[0] + qw * tx + (qy * tz - qz * ty),
        v[1] + qw * ty + (qz * tx - qx * tz),
        v[2] + qw * tz + (qx * ty - qy * tx),
    ];
}

export class ControllerInput {
    /**
     * 수집 상태를 초기화한다. 자동 시작 예약·재중심 이벤트는 attachSession()에서 연결한다.
     */
    constructor() {
        this._lastSample = null;
        this._bodyReader = new BodyReader();
        // 손별 추적 여부 (HUD·로그용)
        this._tracked = { left: false, right: false };
        // 자동 start 예정 시각(ms, 예약 없으면 null)
        this._autoStartAtMs = null;
        // 재중심 이벤트 대상 기준 공간과 follow 해제 마감 시각(ms)
        this._resetSpace = null;
        this._clutchBreakUntilMs = 0;
        this._onReferenceReset = () => this._handleReferenceReset();
        console.log(`[controller_input] 컨트롤러 손 위치 입력 (기능 ${XR_FEATURES.join(", ")}, 손가락은 Manus 장갑)`);
    }

    /** 세션에 요청할 WebXR 선택 기능 목록. */
    get xrFeatures() {
        return XR_FEATURES;
    }

    /** 마지막으로 만든 샘플 (없으면 null). */
    get lastSample() {
        return this._lastSample;
    }

    /** 컨트롤러·body-tracking 추적 상태 문자열 (HUD 표시용). */
    get trackingStatus() {
        const label = (hand) => (this._tracked[hand] ? "추적" : "미검출");
        return `controller L/R: ${label("left")}/${label("right")} | ${this._bodyReader.status}`;
    }

    /** HUD 제어 상태 안내 문구 (대기/제어 중). */
    get controlHint() {
        return CONTROL_HINT;
    }

    /**
     * VR 진입 시 자동 start를 예약하고 기준 공간 재중심 이벤트를 연결한다. 반환값 없음.
     */
    attachSession(xrExperience) {
        this._autoStartAtMs = performance.now() + AUTO_START_DELAY_MS;
        console.log(`[controller_input] VR 진입 → ${AUTO_START_DELAY_MS}ms 후 자동 제어 시작`);

        // 기준 공간 재중심(reset) 이벤트 연결
        this._resetSpace = xrExperience.baseExperience.sessionManager.baseReferenceSpace;
        if (this._resetSpace) {
            this._resetSpace.addEventListener("reset", this._onReferenceReset);
            console.log("[controller_input] 기준 공간 reset 이벤트 연결");
        }
    }

    /**
     * VR 종료 시 자동 start 예약과 재중심 이벤트를 해제한다. 반환값 없음.
     */
    detachSession() {
        this._autoStartAtMs = null;
        if (this._resetSpace) {
            this._resetSpace.removeEventListener("reset", this._onReferenceReset);
            this._resetSpace = null;
        }
        this._clutchBreakUntilMs = 0;
    }

    /**
     * VR 종료 시 보낼 "home"(ready 복귀 후 제어 해제) 명령 샘플을 만들어 반환한다.
     */
    exitSample() {
        return { t: performance.now(), hands: { left: null, right: null }, head: null, command: "home", body: null };
    }

    /**
     * XRFrame에서 공통 형식 샘플 1개를 만들어 반환한다.
     * frame: XRFrame, refSpace: XRReferenceSpace, session: XRSession.
     */
    read(frame, refSpace, session) {
        const now = performance.now();
        const hands = { left: null, right: null };

        // 컨트롤러 입력 소스(grip 공간 보유, 손 추적 소스 제외)의 포즈를 읽는다
        for (const source of session.inputSources) {
            if (source.hand || !source.gripSpace) continue;
            if (!hands.hasOwnProperty(source.handedness)) continue;
            const pose = frame.getPose(source.gripSpace, refSpace);
            if (!pose) continue;
            hands[source.handedness] = this._handFromPose(pose);
        }

        // 추적 상태 변화 로그
        for (const handedness of ["left", "right"]) {
            const tracked = hands[handedness] !== null;
            if (tracked !== this._tracked[handedness]) {
                console.log(`[controller_input] ${handedness} 컨트롤러: ${tracked ? "추적 시작" : "추적 끊김"}`);
                this._tracked[handedness] = tracked;
            }
        }

        // 재중심 직후에는 모든 손 follow 해제 (위치 점프가 팔 동작으로 이어지지 않게)
        if (now < this._clutchBreakUntilMs) {
            for (const handedness of ["left", "right"]) {
                if (hands[handedness] !== null) hands[handedness].follow = false;
            }
        }

        // 자동 start: 예약 시각이 지나면 1회 싣는다
        let command = null;
        if (this._autoStartAtMs !== null && now >= this._autoStartAtMs) {
            this._autoStartAtMs = null;
            command = "start";
            console.log("[controller_input] 자동 제어 시작 명령: start");
        }

        // 조작자 시점 포즈 (로봇 머리 시선 추종용)
        const viewer = frame.getViewerPose(refSpace);
        const head = viewer ? {
            pos: [viewer.transform.position.x, viewer.transform.position.y, viewer.transform.position.z],
            quat: [viewer.transform.orientation.x, viewer.transform.orientation.y,
                viewer.transform.orientation.z, viewer.transform.orientation.w],
        } : null;

        const body = this._bodyReader.read(frame, refSpace);
        const sample = { t: now, hands, head, command, body };
        this._lastSample = sample;
        return sample;
    }

    /**
     * 컨트롤러 grip 포즈를 공통 형식 손 항목으로 변환해 반환한다 (위치 = grip + 손바닥 오프셋).
     */
    _handFromPose(pose) {
        const p = pose.transform.position;
        const o = pose.transform.orientation;
        const quat = [o.x, o.y, o.z, o.w];
        const offset = _rotateByQuat(PALM_OFFSET_LOCAL, quat);
        return {
            pos: [p.x + offset[0], p.y + offset[1], p.z + offset[2]],
            quat,
            follow: true,
        };
    }

    /**
     * 기준 공간 재중심 시 follow 해제 마감 시각을 설정한다. 반환값 없음.
     */
    _handleReferenceReset() {
        this._clutchBreakUntilMs = performance.now() + CLUTCH_BREAK_MS;
        console.log(`[controller_input] 기준 공간 reset(재중심) 감지 → ${CLUTCH_BREAK_MS}ms 동안 follow 해제`);
    }
}
