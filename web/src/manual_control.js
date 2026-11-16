// 수동 조작 (dev 모드 데스크톱 전용): VR 컨트롤러 대신 패널 입력으로 업링크 샘플을 만든다.
//
// 원칙: **입력 값의 출처만 다르고 코드 경로는 VR과 동일하다.**
// PoseReader가 XRFrame에서 만드는 것과 같은 형식의 샘플을 만들어 데스크톱 뷰와 같은 RtcLink의
// state 채널로 보내므로, 서버(rtc_hub → pose_to_target → ik_solver → robot_controller)는 이
// 클라이언트가 Quest인지 패널인지 구분할 수 없다.
//
// 손 위치는 rest 자세 기준 오프셋 슬라이더다 — 클러치 델타 매핑은 grip 체결 순간을 앵커로 잡고
// 이후의 델타만 이식하므로, grip을 켠 채 슬라이더를 움직인 거리가 곧 로봇 TCP의 이동 거리(1m=1m)가 된다.
// body 조인트(어깨·팔꿈치)는 tools/fake_vr_quest3.py와 같은 근사로 합성한다.

// 송신 주기(ms): 서버 IK 30Hz와 같은 급이면 충분
const SEND_INTERVAL_MS = 33;
// 사람 기하 (WebXR: x=우, y=상, -z=전방, m) — fake_vr_quest3.py와 동일한 값
const SHOULDER_Y = 1.4;
const HAND_REST = { right: [0.3, 1.0, -0.25], left: [-0.3, 1.0, -0.25] };
// 키보드 홀드 이동 속도: 손 오프셋(m/s), 트리거 램프(1/s)
const KEY_MOVE_RATE = 0.15;
const KEY_TRIGGER_RATE = 2.5;
// 오프셋 한계 (슬라이더 범위와 동일)
const OFFSET_LIMIT = 0.3;
// 이동 키 → 오프셋 축·부호 (Shift 없음 = 오른손, Shift = 왼손. z는 +가 앞)
const MOVE_KEYS = {
    KeyW: [2, +1], KeyS: [2, -1],   // 앞/뒤
    KeyA: [0, -1], KeyD: [0, +1],   // 좌/우
    KeyR: [1, +1], KeyF: [1, -1],   // 위/아래
};

export class ManualControl {
    /**
     * 패널 DOM을 연결하고 송신 상태를 초기화한다. 실제 송신은 활성 체크박스를 켠 뒤부터.
     */
    constructor() {
        this._panel = document.getElementById("manualPanel");
        this._enableEl = document.getElementById("mpEnable");
        this._fieldsets = [document.getElementById("mpControls"), document.getElementById("mpControlsL")];
        // 조작 상태: DOM 이벤트와 디버깅 훅(window.__teleop.manual) 양쪽에서 갱신한다
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
        // 버튼 래치: 순간 클릭이 송신 틱(33ms) 사이에 끼어 눌린 샘플이 0개가 되는 것 방지 —
        // 누르는 순간부터 최소 이 횟수만큼은 눌린 상태로 송신을 보장한다 (서버 에지 감지용)
        this._pressHold = { a: 0, b: 0 };
        this._bindDom();
        this._bindKeyboard();
    }

    /**
     * 공유 RtcLink를 등록한다 (데스크톱 모니터링 뷰와 같은 연결을 재사용). 반환값 없음.
     */
    setLink(link) {
        this._link = link;
    }

    /**
     * 모드에 따라 패널을 보이거나 숨긴다 (dev에서만 조작 허용). 반환값 없음.
     * deploy로 바뀌면 송신도 함께 멈춘다 — 패널 입력이 실로봇으로 가는 일을 막는다.
     */
    setDevMode(isDev) {
        this._devMode = isDev;
        this._panel.hidden = !isDev;
        if (!isDev) this.setEnabled(false);
    }

    /**
     * 송신을 켜거나 끈다. 반환값 없음.
     * 끄면 서버가 입력 두절(3초)로 자동 해제하므로 별도 정지 명령이 필요 없다.
     */
    setEnabled(on) {
        on = Boolean(on) && this._devMode;
        this._enableEl.checked = on;
        this._fieldsets.forEach((f) => (f.disabled = !on));
        if (on && this._timer === null) {
            this._timer = setInterval(() => {
                this._applyHeldKeys(SEND_INTERVAL_MS / 1000);
                this._link?.sendPose(this._sample());
            }, SEND_INTERVAL_MS);
        } else if (!on && this._timer !== null) {
            clearInterval(this._timer);
            this._timer = null;
        }
    }

    // ── 조작 상태 설정 (DOM 이벤트·디버깅 훅 공용) ──────────────────────────

    /** grip(클러치)을 설정한다. side: left/right. 반환값 없음. */
    setGrip(side, on) {
        this._state.grip[side] = Boolean(on);
        document.getElementById(`mpGrip-${side}`).checked = this._state.grip[side];
    }

    /** 트리거(0-1, 그리퍼 개폐)를 설정한다. 반환값 없음. */
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

    /** A/B 버튼 눌림을 설정한다 (누르는 동안 true — 서버가 상승 에지를 감지한다). 반환값 없음.

     * 순간 클릭도 서버에 반드시 도달하도록, 누름 시작 시 최소 6샘플(약 0.2초)의 눌린 송신을 보장한다.
     */
    setButton(name, pressed) {
        this._state[name] = Boolean(pressed);
        if (pressed) this._pressHold[name] = 6;
    }

    // ── 샘플 생성 (PoseReader.read()와 동일 형식) ───────────────────────────

    /**
     * 현재 조작 상태로 업링크 샘플 1개를 만들어 반환한다.
     */
    _sample() {
        // 버튼 래치 소모: 실제 눌림 또는 잔여 래치가 있으면 눌린 것으로 송신
        const aActive = this._state.a || this._pressHold.a > 0;
        const bActive = this._state.b || this._pressHold.b > 0;
        this._pressHold.a = Math.max(0, this._pressHold.a - 1);
        this._pressHold.b = Math.max(0, this._pressHold.b - 1);

        const hands = {};
        for (const side of ["left", "right"]) {
            const rest = HAND_REST[side];
            const [dx, dy, dz] = this._state.offset[side];
            const trigger = this._state.trigger[side];
            const grip = this._state.grip[side];
            hands[side] = {
                // 슬라이더 dz(+앞)를 WebXR 전방(-z)으로 변환
                pos: [rest[0] + dx, rest[1] + dy, rest[2] - dz],
                quat: [0, 0, 0, 1],
                trigger,
                squeeze: grip ? 1 : 0,
                // xr-standard 버튼 순서: 0=trigger, 1=squeeze, 4=A/X, 5=B/Y
                buttons: [
                    trigger > 0.5 ? 1 : 0, grip ? 1 : 0, 0, 0,
                    side === "right" && aActive ? 1 : 0,
                    side === "right" && bActive ? 1 : 0,
                ],
            };
        }
        return { t: performance.now(), body: { joints: this._bodyJoints(hands) }, hands };
    }

    /**
     * 어깨·팔꿈치 등 body 조인트를 손 위치와 정합되게 합성해 반환한다 (fake_vr_quest3와 동일 근사).
     */
    _bodyJoints(hands) {
        const identity = [0, 0, 0, 1];
        const joints = {
            hips: { pos: [0, 1.0, 0], quat: identity },
            head: { pos: [0, 1.62, 0], quat: identity },
        };
        for (const [side, sx] of [["right", 1], ["left", -1]]) {
            const upper = [sx * 0.22, SHOULDER_Y - 0.02, 0];
            const hand = hands[side].pos;
            joints[`${side}-shoulder`] = { pos: [sx * 0.18, SHOULDER_Y, 0], quat: identity };
            joints[`${side}-arm-upper`] = { pos: upper, quat: identity };
            joints[`${side}-arm-lower`] = {
                pos: [(upper[0] + hand[0]) / 2, (upper[1] + hand[1]) / 2 - 0.08, (upper[2] + hand[2]) / 2],
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
        this._enableEl.addEventListener("change", () => this.setEnabled(this._enableEl.checked));

        // 패널 조작 후 포커스를 해제한다 — 체크박스·슬라이더에 포커스가 남으면
        // 이후의 키보드 입력이 전부 그 요소로 가서 키 조작이 죽는다
        this._panel.addEventListener("pointerup", () => {
            if (this._panel.contains(document.activeElement)) document.activeElement.blur();
        });

        for (const side of ["left", "right"]) {
            document.getElementById(`mpGrip-${side}`).addEventListener("change", (e) => {
                this._state.grip[side] = e.target.checked;
            });
            document.getElementById(`mpTrig-${side}`).addEventListener("input", (e) => {
                this._state.trigger[side] = Number(e.target.value);
            });
            for (const [axis, idx] of [["X", 0], ["Y", 1], ["Z", 2]]) {
                document.getElementById(`mp${axis}-${side}`).addEventListener("input", (e) => {
                    this._state.offset[side][idx] = Number(e.target.value);
                });
            }
        }

        // A/B는 누르는 동안만 활성 (momentary) — 마우스가 벗어나도 풀리게 pointerleave까지 처리
        const hold = (id, down, up) => {
            const el = document.getElementById(id);
            el.addEventListener("pointerdown", down);
            el.addEventListener("pointerup", up);
            el.addEventListener("pointerleave", up);
        };
        hold("mpBtnA", () => this.setButton("a", true), () => this.setButton("a", false));
        hold("mpBtnB", () => this.setButton("b", true), () => this.setButton("b", false));
    }

    // ── 키보드 조작 ─────────────────────────────────────────────────────────

    /**
     * 키보드 바인딩을 등록한다. 반환값 없음.
     *
     * 배치: W/S 앞뒤, A/D 좌우, R/F 위아래 (홀드 이동, Shift=왼손) / G grip 토글 (Shift+G 왼손) /
     *       T 트리거 홀드 (Shift+T 왼손) / Enter=A, Backspace=B (누르는 동안) / M 활성 토글.
     * 입력 요소에 포커스가 있으면 무시한다 (슬라이더 조작과 충돌 방지).
     */
    _bindKeyboard() {
        window.addEventListener("keydown", (e) => {
            if (e.target.tagName === "INPUT" || !this._devMode) return;
            // M은 활성 토글이므로 비활성 상태에서도 받는다
            if (e.code === "KeyM" && !e.repeat) {
                this.setEnabled(!this._enableEl.checked);
                return;
            }
            if (this._timer === null) return;
            const side = e.shiftKey ? "left" : "right";

            if (e.code in MOVE_KEYS) {
                this._heldMoves.set(e.code, side);
            } else if (e.code === "KeyT") {
                this._heldTriggers.add(side);
            } else if (e.code === "KeyG" && !e.repeat) {
                this.setGrip(side, !this._state.grip[side]);
            } else if (e.code === "Enter" && !e.repeat) {
                this.setButton("a", true);
            } else if (e.code === "Backspace" && !e.repeat) {
                this.setButton("b", true);
                e.preventDefault();
            }
        });
        window.addEventListener("keyup", (e) => {
            this._heldMoves.delete(e.code);
            if (e.code === "KeyT") {
                // Shift를 먼저 놓는 경우를 위해 양쪽 다 해제
                this._heldTriggers.delete("left");
                this._heldTriggers.delete("right");
            }
            if (e.code === "Enter") this.setButton("a", false);
            if (e.code === "Backspace") this.setButton("b", false);
        });
    }

    /**
     * 홀드 중인 키를 시간에 비례해 조작 상태에 반영한다 (송신 주기마다 호출). 반환값 없음.
     */
    _applyHeldKeys(dt) {
        // 손 이동: 홀드 키마다 해당 손 오프셋을 속도 × dt 만큼 (슬라이더 UI도 함께 갱신)
        for (const [code, side] of this._heldMoves) {
            const [axis, sign] = MOVE_KEYS[code];
            const offset = this._state.offset[side];
            offset[axis] = Math.min(OFFSET_LIMIT, Math.max(-OFFSET_LIMIT, offset[axis] + sign * KEY_MOVE_RATE * dt));
            this.setOffset(side, offset[0], offset[1], offset[2]);
        }

        // 트리거: 홀드면 1로, 놓으면 0으로 램프 (그리퍼가 서서히 여닫힘)
        for (const side of ["left", "right"]) {
            const goal = this._heldTriggers.has(side) ? 1 : 0;
            const cur = this._state.trigger[side];
            if (cur !== goal) {
                const step = KEY_TRIGGER_RATE * dt;
                this.setTrigger(side, cur + Math.min(step, Math.abs(goal - cur)) * Math.sign(goal - cur));
            }
        }
    }
}
