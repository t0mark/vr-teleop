// 수동 조작 (dev 모드 데스크톱 전용): 패널·키보드 입력으로 공통 형식 업링크 샘플을 만들어 보낸다.
//
// 입력 모듈(web/src/input/)과 같은 공통 샘플 형식을 데스크톱 뷰의 RtcLink state 채널로 보낸다.
// 패널 조작: grip = follow(클러치), 트리거 = 손가락 굽힘, A/B = start/stop 명령.
// 손 위치: rest 자세 기준 오프셋 슬라이더 (grip을 켠 채 움직인 거리가 로봇 TCP 이동 거리, 1m=1m).
// body 조인트(어깨·팔꿈치): tools/fake_vr_client.py와 같은 근사로 합성한다.

// 송신 주기(ms)
const SEND_INTERVAL_MS = 33;
// 사람 기하 (WebXR: x=우, y=상, -z=전방, m, fake_vr_client.py와 동일)
const SHOULDER_Y = 1.4;
const HAND_REST = { right: [0.3, 1.0, -0.25], left: [-0.3, 1.0, -0.25] };
// 키보드 홀드 이동 속도: 손 오프셋(m/s), 트리거 램프(1/s)
const KEY_MOVE_RATE = 0.15;
const KEY_TRIGGER_RATE = 2.5;
// 오프셋 한계 (슬라이더 범위와 동일)
const OFFSET_LIMIT = 0.3;
// 트리거 하나로 굽히는 손가락 자유도 수 (little·ring·middle·index·thumb_bend)
const TRIGGER_CURL_DOF = 5;
// 엄지 회전 고정값 (1.0 = 손바닥 쪽 완전 대향)
const THUMB_ROTATE_FIXED = 1.0;
// 이동 키 → [오프셋 축, 부호] (W/S 앞뒤, A/D 좌우, R/F 위아래, z는 +가 앞)
const MOVE_KEYS = {
    KeyW: [2, +1],
    KeyS: [2, -1],
    KeyA: [0, -1],
    KeyD: [0, +1],
    KeyR: [1, +1],
    KeyF: [1, -1],
};
// 오프셋 슬라이더 축 이름 → 오프셋 인덱스
const OFFSET_AXES = [["X", 0], ["Y", 1], ["Z", 2]];

/**
 * 트리거 값(0-1)을 공통 형식 손가락 굽힘 6개로 변환해 반환한다.
 */
function _triggerToFingers(trigger) {
    const curl = Math.min(1, Math.max(0, trigger));
    return [...new Array(TRIGGER_CURL_DOF).fill(curl), THUMB_ROTATE_FIXED];
}

export class ManualControl {
    /**
     * 패널 DOM을 연결하고 송신 상태를 초기화한다. 실제 송신은 활성 체크박스를 켠 뒤부터.
     */
    constructor() {
        this._panel = document.getElementById("manualPanel");
        this._enableEl = document.getElementById("mpEnable");
        this._fieldsets = [document.getElementById("mpControls"), document.getElementById("mpControlsL")];
        // 조작 상태: DOM 이벤트와 디버깅 훅(window.__teleop_manual) 양쪽에서 갱신한다
        this._state = {
            grip: { right: false, left: false },
            trigger: { right: 0, left: 0 },
            offset: { right: [0, 0, 0], left: [0, 0, 0] },
            a: false,
            b: false,
        };
        this._link = null;
        this._timer = null;
        this._devMode = false;
        // 키보드 홀드 상태: 이동 {code: side}, 트리거 {side}
        this._heldMoves = new Map();
        this._heldTriggers = new Set();
        // 누른 순간 생긴 명령(start/stop): 다음 송신 샘플 1개에 실어 보내고 비운다
        this._pendingCommand = null;

        // DOM·키보드 이벤트 연결
        this._bindDom();
        this._bindKeyboard();
    }

    /**
     * 공유 RtcLink를 등록한다. 반환값 없음.
     */
    setLink(link) {
        this._link = link;
    }

    /**
     * 모드에 따라 패널을 보이거나 숨긴다 (dev에서만 조작 허용, dev가 아니면 송신 중지). 반환값 없음.
     */
    setDevMode(isDev) {
        this._devMode = isDev;
        this._panel.hidden = !isDev;
        if (!isDev) this.setEnabled(false);
    }

    /**
     * 송신을 켜거나 끈다 (끄면 서버가 입력 두절로 제어를 해제한다). 반환값 없음.
     */
    setEnabled(on) {
        const isEnabled = Boolean(on) && this._devMode;

        // 체크박스·조작 필드 상태 반영
        this._enableEl.checked = isEnabled;
        for (const fieldset of this._fieldsets) fieldset.disabled = !isEnabled;

        // 송신 타이머 시작·정지
        if (isEnabled && this._timer === null) {
            this._timer = setInterval(() => this._sendTick(), SEND_INTERVAL_MS);
        } else if (!isEnabled && this._timer !== null) {
            clearInterval(this._timer);
            this._timer = null;
        }
    }

    /**
     * 송신 주기마다 홀드 키를 반영하고 샘플 1개를 보낸다. 반환값 없음.
     */
    _sendTick() {
        this._applyHeldKeys(SEND_INTERVAL_MS / 1000);
        const sample = this._sample();
        const isSent = this._link !== null && this._link.sendPose(sample);

        // 명령이 실린 샘플을 못 보냈으면 유실 경고
        if (sample.command != null && !isSent) {
            console.warn(`[manual] 명령 '${sample.command}' 유실: 서버 연결 끊김`);
        }
    }

    // ── 조작 상태 설정 (DOM 이벤트·디버깅 훅 공용) ──────────────────────────

    /** grip(클러치)을 설정한다. side: left/right. 반환값 없음. */
    setGrip(side, on) {
        this._state.grip[side] = Boolean(on);
        document.getElementById(`mpGrip-${side}`).checked = this._state.grip[side];
    }

    /** 트리거(0-1, 손가락 굽힘)를 설정한다. 반환값 없음. */
    setTrigger(side, value) {
        this._state.trigger[side] = Math.min(1, Math.max(0, value));
        document.getElementById(`mpTrig-${side}`).value = this._state.trigger[side];
    }

    /** rest 기준 손 오프셋(m)을 설정한다. dz는 +가 앞(전방). 반환값 없음. */
    setOffset(side, dx, dy, dz) {
        this._state.offset[side] = [dx, dy, dz];
        document.getElementById(`mpX-${side}`).value = dx;
        document.getElementById(`mpY-${side}`).value = dy;
        document.getElementById(`mpZ-${side}`).value = dz;
    }

    /** A/B 버튼 눌림을 설정한다. 누르는 순간(상승 에지) A=start, B=stop 명령을 1회 예약한다. 반환값 없음. */
    setButton(name, pressed) {
        const wasPressed = this._state[name];
        this._state[name] = Boolean(pressed);
        if (this._state[name] && !wasPressed) {
            this._pendingCommand = name === "a" ? "start" : "stop";
            console.log(`[manual] 명령 예약: ${this._pendingCommand}`);
        }
    }

    // ── 샘플 생성 (입력 모듈과 동일한 공통 형식) ────────────────────────────

    /**
     * 현재 조작 상태로 공통 형식 업링크 샘플 1개를 만들어 반환한다.
     */
    _sample() {
        // 예약된 명령은 이 샘플 1개에만 싣고 비운다
        const command = this._pendingCommand;
        this._pendingCommand = null;

        // 손: rest + 오프셋 (슬라이더 dz(+앞)를 WebXR 전방(-z)으로 변환)
        const hands = {};
        for (const side of ["left", "right"]) {
            const rest = HAND_REST[side];
            const [dx, dy, dz] = this._state.offset[side];
            hands[side] = {
                pos: [rest[0] + dx, rest[1] + dy, rest[2] - dz],
                quat: [0, 0, 0, 1],
                follow: this._state.grip[side],
                fingers: _triggerToFingers(this._state.trigger[side]),
            };
        }

        // body 조인트 합성과 샘플 조립
        const joints = this._bodyJoints(hands);
        return {
            t: performance.now(),
            hands,
            command,
            body: { supported: true, jointCount: Object.keys(joints).length, joints },
        };
    }

    /**
     * 어깨·팔꿈치 등 body 조인트를 손 위치와 정합되게 합성해 반환한다 (fake_vr_client와 동일 근사).
     */
    _bodyJoints(hands) {
        const identity = [0, 0, 0, 1];
        const joints = {
            hips: { pos: [0, 1.0, 0], quat: identity },
            head: { pos: [0, 1.62, 0], quat: identity },
        };

        // 좌우 어깨·위팔·아래팔 (아래팔은 위팔과 손의 중점에서 약간 아래)
        for (const [side, sideSign] of [["right", 1], ["left", -1]]) {
            const upperArm = [sideSign * 0.22, SHOULDER_Y - 0.02, 0];
            const handPosition = hands[side].pos;
            joints[`${side}-shoulder`] = { pos: [sideSign * 0.18, SHOULDER_Y, 0], quat: identity };
            joints[`${side}-arm-upper`] = { pos: upperArm, quat: identity };
            joints[`${side}-arm-lower`] = {
                pos: [
                    (upperArm[0] + handPosition[0]) / 2,
                    (upperArm[1] + handPosition[1]) / 2 - 0.08,
                    (upperArm[2] + handPosition[2]) / 2,
                ],
                quat: identity,
            };
        }
        return joints;
    }

    // ── DOM 배선 ────────────────────────────────────────────────────────────

    /**
     * 패널 입력 요소들을 조작 상태에 연결한다. 반환값 없음.
     */
    _bindDom() {
        // 활성 체크박스
        this._enableEl.addEventListener("change", () => this.setEnabled(this._enableEl.checked));

        // 패널 조작 후 포커스 해제 (키보드 조작 유지)
        this._panel.addEventListener("pointerup", () => this._releasePanelFocus());

        // 손별 grip·트리거·오프셋 슬라이더
        for (const side of ["left", "right"]) {
            document.getElementById(`mpGrip-${side}`).addEventListener("change", (event) => this._handleGripInput(side, event));
            document.getElementById(`mpTrig-${side}`).addEventListener("input", (event) => this._handleTriggerInput(side, event));
            for (const [axisName, axisIndex] of OFFSET_AXES) {
                document.getElementById(`mp${axisName}-${side}`).addEventListener(
                    "input", (event) => this._handleOffsetInput(side, axisIndex, event),
                );
            }
        }

        // A/B 버튼: 누르는 동안만 활성
        this._bindMomentaryButton("mpBtnA", "a");
        this._bindMomentaryButton("mpBtnB", "b");
    }

    /**
     * 패널 안 요소에 남은 포커스를 해제한다. 반환값 없음.
     */
    _releasePanelFocus() {
        if (this._panel.contains(document.activeElement)) document.activeElement.blur();
    }

    /** grip 체크박스 입력을 조작 상태에 반영한다. 반환값 없음. */
    _handleGripInput(side, event) {
        this._state.grip[side] = event.target.checked;
    }

    /** 트리거 슬라이더 입력을 조작 상태에 반영한다. 반환값 없음. */
    _handleTriggerInput(side, event) {
        this._state.trigger[side] = Number(event.target.value);
    }

    /** 오프셋 슬라이더 입력을 조작 상태에 반영한다. 반환값 없음. */
    _handleOffsetInput(side, axisIndex, event) {
        this._state.offset[side][axisIndex] = Number(event.target.value);
    }

    /**
     * 버튼을 누르는 동안만 buttonName 상태를 켠다 (포인터가 벗어나면 해제). 반환값 없음.
     */
    _bindMomentaryButton(elementId, buttonName) {
        const buttonEl = document.getElementById(elementId);
        buttonEl.addEventListener("pointerdown", () => this.setButton(buttonName, true));
        buttonEl.addEventListener("pointerup", () => this.setButton(buttonName, false));
        buttonEl.addEventListener("pointerleave", () => this.setButton(buttonName, false));
    }

    // ── 키보드 조작 ─────────────────────────────────────────────────────────

    /**
     * 키보드 바인딩을 등록한다. 반환값 없음.
     *
     * 배치: W/S 앞뒤, A/D 좌우, R/F 위아래 (홀드 이동, Shift=왼손) / G grip 토글 (Shift+G 왼손) /
     *       T 트리거 홀드 (Shift+T 왼손) / Enter=A, Backspace=B (누르는 동안) / M 활성 토글.
     */
    _bindKeyboard() {
        window.addEventListener("keydown", (event) => this._handleKeyDown(event));
        window.addEventListener("keyup", (event) => this._handleKeyUp(event));
    }

    /**
     * 키 누름을 조작 상태에 반영한다 (입력 요소에 포커스가 있거나 dev 모드가 아니면 무시). 반환값 없음.
     */
    _handleKeyDown(event) {
        if (event.target.tagName === "INPUT" || !this._devMode) return;

        // M: 활성 토글 (비활성 상태에서도 받는다)
        if (event.code === "KeyM" && !event.repeat) {
            this.setEnabled(!this._enableEl.checked);
            return;
        }
        if (this._timer === null) return;
        const side = event.shiftKey ? "left" : "right";

        // 이동·트리거 홀드, grip 토글, A/B 버튼
        if (event.code in MOVE_KEYS) {
            this._heldMoves.set(event.code, side);
        } else if (event.code === "KeyT") {
            this._heldTriggers.add(side);
        } else if (event.code === "KeyG" && !event.repeat) {
            this.setGrip(side, !this._state.grip[side]);
        } else if (event.code === "Enter" && !event.repeat) {
            this.setButton("a", true);
        } else if (event.code === "Backspace" && !event.repeat) {
            this.setButton("b", true);
            event.preventDefault();
        }
    }

    /**
     * 키 놓음을 조작 상태에 반영한다. 반환값 없음.
     */
    _handleKeyUp(event) {
        this._heldMoves.delete(event.code);

        // T: 양손 트리거 홀드 해제
        if (event.code === "KeyT") {
            this._heldTriggers.delete("left");
            this._heldTriggers.delete("right");
        }

        // A/B 버튼 해제
        if (event.code === "Enter") this.setButton("a", false);
        if (event.code === "Backspace") this.setButton("b", false);
    }

    /**
     * 홀드 중인 키를 시간에 비례해 조작 상태에 반영한다 (송신 주기마다 호출). 반환값 없음.
     */
    _applyHeldKeys(dtSec) {
        // 손 이동: 홀드 키마다 해당 손 오프셋을 속도 × dt 만큼 (슬라이더 UI도 함께 갱신)
        for (const [code, side] of this._heldMoves) {
            const [axis, sign] = MOVE_KEYS[code];
            const offset = this._state.offset[side];
            offset[axis] = Math.min(OFFSET_LIMIT, Math.max(-OFFSET_LIMIT, offset[axis] + sign * KEY_MOVE_RATE * dtSec));
            this.setOffset(side, offset[0], offset[1], offset[2]);
        }

        // 트리거: 홀드면 1로, 놓으면 0으로 램프
        for (const side of ["left", "right"]) {
            const goal = this._heldTriggers.has(side) ? 1 : 0;
            const current = this._state.trigger[side];
            if (current !== goal) {
                const step = KEY_TRIGGER_RATE * dtSec;
                this.setTrigger(side, current + Math.min(step, Math.abs(goal - current)) * Math.sign(goal - current));
            }
        }
    }
}
