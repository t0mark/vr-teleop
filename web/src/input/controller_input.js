// 컨트롤러 입력 모듈 (Quest 3): 조작자 손등에 부착한 컨트롤러 포즈를 공통 샘플 형식으로 변환한다.
//
// 공통 샘플 형식 (서버 teleop/utils/inputs.py가 해석):
//   { t, hands: { left/right: { pos[3], quat[x,y,z,w], follow } | null },
//     head: { pos[3], quat[x,y,z,w] } | null,
//     command: "start" | "home" | null, body: { supported, jointCount, joints } | null }
// head: 조작자 시점(XR viewer) 포즈. 손가락(fingers)은 싣지 않는다 (서버의 Manus 장갑이 측정).
// 손 포즈: 컨트롤러 grip 포즈 + 손바닥 오프셋. 재중심 직후·추정 위치(emulatedPosition)일 때는 follow 해제.
// 명령: VR 진입 후 AUTO_START_DELAY_MS가 지나면 "start", armed=false가 이어지면 START_RETRY_MAX회까지 재전송.
//   VR 종료·세션 숨김 시 main.js가 exitSample()의 "home"을 보낸다. 숨김 후 제어 재시작은 VR 재진입으로만 한다.
// 좌표계: WebXR reference space(오른손, y-up), 단위 m. 쿼터니언은 (x, y, z, w).

import { BodyReader } from "./body_reader.js";

// 세션에 요청할 WebXR 선택 기능
const XR_FEATURES = ["body-tracking"];
// VR 진입 후 자동 제어 시작까지 대기 시간(ms)
const AUTO_START_DELAY_MS = 2000;
// start 전송 후 제어 상태(armed)가 false로 남아 있을 때 start를 다시 보내는 간격(ms)
const START_RETRY_INTERVAL_MS = 5000;
// start 재전송 최대 횟수
const START_RETRY_MAX = 2;
// 재중심(reset) 후 follow를 끊는 시간(ms)
const CLUTCH_BREAK_MS = 200;
// 컨트롤러 grip 로컬 좌표계에서 손바닥 중심까지의 오프셋(m)
const PALM_OFFSET_LOCAL = [0, 0, 0];
// HUD 제어 상태 안내 문구
const CONTROL_HINT = {
    idle: "제어 대기 — VR (재)진입 2초 후 자동 시작",
    armed: "제어 중 (VR 종료 시 홈 복귀 후 해제)",
};

/**
 * 벡터 vec를 쿼터니언 quat(x, y, z, w)로 회전한 결과를 반환한다.
 */
function _rotateByQuat(vec, quat) {
    // t = 2·(q_xyz × v), v' = v + w·t + q_xyz × t
    const [qx, qy, qz, qw] = quat;
    const tx = 2 * (qy * vec[2] - qz * vec[1]);
    const ty = 2 * (qz * vec[0] - qx * vec[2]);
    const tz = 2 * (qx * vec[1] - qy * vec[0]);
    return [
        vec[0] + qw * tx + (qy * tz - qz * ty),
        vec[1] + qw * ty + (qz * tx - qx * tz),
        vec[2] + qw * tz + (qx * ty - qy * tx),
    ];
}

/**
 * 손 하나의 추적 상태 라벨(미검출/추정/추적)을 반환한다 (HUD 표시용).
 */
function _trackingLabel(isTracked, isEmulated) {
    if (!isTracked) return "미검출";
    return isEmulated ? "추정(follow 해제)" : "추적";
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
        // 손별 추적 상실 중 추정 위치(emulatedPosition) 여부 (HUD·로그용)
        this._emulated = { left: false, right: false };
        // 자동 start 예정 시각(ms, 예약 없으면 null)
        this._autoStartAtMs = null;
        // 마지막 start 전송 시각(ms, 제어 확인·포기 후 null)과 재전송 횟수
        this._startSentAtMs = null;
        this._startRetryCount = 0;
        // start 재시도·유실 안내 문구 (HUD 표시용, 없으면 빈 문자열)
        this._startNotice = "";
        // 서버가 알려 준 제어 상태 (robot_state.armed, 모르면 null)
        this._robotArmed = null;
        // 재중심 이벤트 대상 기준 공간과 follow 해제 마감 시각(ms)
        this._resetSpace = null;
        this._clutchBreakUntilMs = 0;
        this._onReferenceReset = this._handleReferenceReset.bind(this);
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
        const leftLabel = _trackingLabel(this._tracked.left, this._emulated.left);
        const rightLabel = _trackingLabel(this._tracked.right, this._emulated.right);
        const notice = this._startNotice === "" ? "" : ` | ${this._startNotice}`;
        return `controller L/R: ${leftLabel}/${rightLabel} | ${this._bodyReader.status}${notice}`;
    }

    /** HUD 제어 상태 안내 문구 (대기/제어 중). */
    get controlHint() {
        return CONTROL_HINT;
    }

    /**
     * VR 진입 시 자동 start를 예약하고 기준 공간 재중심 이벤트를 연결한다. 반환값 없음.
     */
    attachSession(xrExperience) {
        this._scheduleAutoStart("VR 진입");

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
        this.cancelAutoStart();

        // 재중심 이벤트 해제
        if (this._resetSpace) {
            this._resetSpace.removeEventListener("reset", this._onReferenceReset);
            this._resetSpace = null;
        }
        this._clutchBreakUntilMs = 0;
    }

    /**
     * AUTO_START_DELAY_MS 뒤 자동 start를 예약하고 재시도 상태를 초기화한다. 반환값 없음.
     * reason: 로그용 예약 사유.
     */
    _scheduleAutoStart(reason) {
        this._autoStartAtMs = performance.now() + AUTO_START_DELAY_MS;
        this._startSentAtMs = null;
        this._startRetryCount = 0;
        this._startNotice = "";
        console.log(`[controller_input] ${reason} → ${AUTO_START_DELAY_MS}ms 후 자동 제어 시작`);
    }

    /**
     * 자동 start 예약과 재시도를 취소한다. 반환값 없음.
     * notice: HUD에 남길 안내 문구 (없으면 빈 문자열).
     */
    cancelAutoStart(notice = "") {
        this._autoStartAtMs = null;
        this._startSentAtMs = null;
        this._startRetryCount = 0;
        this._startNotice = notice;
    }

    /**
     * 서버가 알려 준 제어 상태(robot_state.armed)를 반영한다. 반환값 없음.
     * armed가 true가 되면 보낸 start가 처리된 것으로 보고 재시도를 끝낸다.
     */
    setRobotArmed(armed) {
        this._robotArmed = armed;
        if (armed === true && this._startSentAtMs !== null) {
            console.log("[controller_input] 제어 시작 확인 (armed)");
            this._startSentAtMs = null;
            this._startNotice = "";
        }
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
            hands[source.handedness] = this._handFromPose(source.handedness, pose);
        }

        // 추적 상태 변화 로그
        for (const handedness of ["left", "right"]) {
            const tracked = hands[handedness] !== null;
            if (tracked !== this._tracked[handedness]) {
                console.log(`[controller_input] ${handedness} 컨트롤러: ${tracked ? "추적 시작" : "추적 끊김"}`);
                this._tracked[handedness] = tracked;
            }
        }

        // 재중심 직후에는 모든 손 follow 해제
        if (now < this._clutchBreakUntilMs) {
            for (const handedness of ["left", "right"]) {
                if (hands[handedness] !== null) hands[handedness].follow = false;
            }
        }

        // 자동 start 명령
        const command = this._takeStartCommand(now);

        // 조작자 시점 포즈
        const viewer = frame.getViewerPose(refSpace);
        const head = viewer ? {
            pos: [viewer.transform.position.x, viewer.transform.position.y, viewer.transform.position.z],
            quat: [viewer.transform.orientation.x, viewer.transform.orientation.y,
                viewer.transform.orientation.z, viewer.transform.orientation.w],
        } : null;

        // body-tracking 조인트와 샘플 조립
        const body = this._bodyReader.read(frame, refSpace);
        const sample = { t: now, hands, head, command, body };
        this._lastSample = sample;
        return sample;
    }

    /**
     * 이번 샘플에 실을 start 명령을 반환한다 (없으면 null).
     * 예약 시각이 지나면 1회 싣고, 이후 armed가 false로 남아 있으면 START_RETRY_INTERVAL_MS마다
     * START_RETRY_MAX회까지 다시 싣는다. 재시도를 다 쓰면 유실로 표시한다.
     */
    _takeStartCommand(now) {
        // 자동 start: 예약 시각이 지나면 1회 싣는다
        if (this._autoStartAtMs !== null && now >= this._autoStartAtMs) {
            this._autoStartAtMs = null;
            this._startSentAtMs = now;
            console.log("[controller_input] 자동 제어 시작 명령: start");
            return "start";
        }

        // 재시도 대상: start를 보냈고 서버가 아직 대기(armed=false)인 경우
        if (this._startSentAtMs === null || this._robotArmed !== false) return null;
        if (now - this._startSentAtMs < START_RETRY_INTERVAL_MS) return null;

        // 재시도 횟수를 다 쓰면 유실로 표시하고 멈춘다
        if (this._startRetryCount >= START_RETRY_MAX) {
            this._startSentAtMs = null;
            this._startNotice = "start 유실 — VR 재진입 필요";
            console.warn(`[controller_input] start ${START_RETRY_MAX}회 재시도 후에도 제어 대기 — 유실 처리`);
            return null;
        }

        // start 재전송
        this._startRetryCount += 1;
        this._startSentAtMs = now;
        this._startNotice = `start 재시도 ${this._startRetryCount}/${START_RETRY_MAX}`;
        console.warn(`[controller_input] 제어 대기 유지 → start 재전송 (${this._startRetryCount}/${START_RETRY_MAX})`);
        return "start";
    }

    /**
     * 컨트롤러 grip 포즈를 공통 형식 손 항목으로 변환해 반환한다 (위치 = grip + 손바닥 오프셋).
     * 추적을 잃어 위치가 추정값(emulatedPosition)이면 follow를 끈다.
     */
    _handFromPose(handedness, pose) {
        const gripPosition = pose.transform.position;
        const gripOrientation = pose.transform.orientation;
        const quat = [gripOrientation.x, gripOrientation.y, gripOrientation.z, gripOrientation.w];
        const palmOffset = _rotateByQuat(PALM_OFFSET_LOCAL, quat);

        // 추정 위치 상태 변화 로그
        const isEmulated = Boolean(pose.emulatedPosition);
        if (isEmulated !== this._emulated[handedness]) {
            console.log(`[controller_input] ${handedness} 컨트롤러: ${isEmulated ? "추정 위치 → follow 해제" : "위치 추적 복구 → follow 재개"}`);
            this._emulated[handedness] = isEmulated;
        }

        return {
            pos: [gripPosition.x + palmOffset[0], gripPosition.y + palmOffset[1], gripPosition.z + palmOffset[2]],
            quat,
            follow: !isEmulated,
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
