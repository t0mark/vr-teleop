// 단일 WebRTC 연결: RTCPeerConnection 하나에 DataChannel 2개를 열어 서버와 통신한다.
//   - "state" (ordered+reliable): 포즈 업링크, robot_config/robot_state/mode_status, set_mode 명령
//   - "mesh"  (ordered+reliable): 재구성 메시 블록 바이너리(MSH1) — 스냅샷·블록 갱신 순서가 바뀌면 맵이 틀어지므로
//     순서를 보장한다 (teleop/net/rtc_hub.py 참고)
//
// 시그널링은 지속 연결이 아니라 HTTPS POST 1회(/rtc/offer)로 끝난다 — WebRTC 협상 자체에는
// 상시 연결이 필요 없다 (서버 쪽 이유는 teleop/net/rtc_hub.py 참고). STUN도 끈다 — Quest-PC는
// 유선 직결(같은 서브넷)이라 NAT 통과가 필요 없고, 기본 STUN은 공인 IP hairpin 시도로 협상을 지연시킨다.

const RECONNECT_MS = 2000;
const OFFER_URL = "/rtc/offer";
// 포즈 전송 주기 상한(Hz): XR 프레임(72-120Hz)보다 낮춰 대역폭·서버 부하를 제어
const POSE_SEND_HZ = 60;

export class RtcLink {
    /**
     * onStateMessage: state 채널 JSON 메시지 콜백. onMeshPacket: mesh 채널 바이너리(ArrayBuffer) 콜백.
     */
    constructor(onStateMessage, onMeshPacket) {
        this._onStateMessage = onStateMessage;
        this._onMeshPacket = onMeshPacket;
        this._pc = null;
        this._stateChannel = null;
        this.connected = false;
        // 대기 중인 재접속 타이머 (없으면 null)
        this._reconnectTimer = null;
        this._lastPoseSendMs = 0;
        this._txCount = 0;
        // 최근 1초 포즈 전송률 (HUD 표시용, 1초마다 갱신)
        this.txHz = 0;
        this._startTxRateTimer();
    }

    /**
     * RTCPeerConnection·두 DataChannel을 만들고 /rtc/offer로 협상한다. 반환값 없음.
     * 끊기면(연결 끊김·실패·종료, state 채널 닫힘, 협상 실패) 자동 재접속한다.
     */
    async connect() {
        if (this._pc) {
            this._pc.close();
            this._pc = null;
        }
        this.connected = false;
        const pc = new RTCPeerConnection({ iceServers: [] });
        this._pc = pc;
        console.log("[rtc] 연결 시도");

        const stateChannel = pc.createDataChannel("state", { ordered: true });
        stateChannel.onopen = () => {
            if (pc !== this._pc) return;
            this.connected = true;
            console.log("[rtc] state 채널 열림");
        };
        stateChannel.onclose = () => {
            if (pc !== this._pc) return;
            this.connected = false;
            console.warn("[rtc] state 채널 닫힘");
            this._scheduleReconnect("state 채널 닫힘");
        };
        stateChannel.onmessage = (ev) => {
            try {
                this._onStateMessage(JSON.parse(ev.data));
            } catch (e) {
                console.warn("[rtc] state 메시지 파싱 실패:", e);
            }
        };
        this._stateChannel = stateChannel;

        const meshChannel = pc.createDataChannel("mesh", { ordered: true });
        meshChannel.binaryType = "arraybuffer";
        meshChannel.onmessage = (ev) => this._onMeshPacket(ev.data);

        // 연결 끊김·실패·종료 시 재접속 예약 (이전 연결의 이벤트는 무시)
        pc.onconnectionstatechange = () => {
            if (pc !== this._pc) return;
            const state = pc.connectionState;
            console.log(`[rtc] 연결 상태: ${state}`);
            if (state === "disconnected" || state === "failed" || state === "closed") {
                this.connected = false;
                this._scheduleReconnect(`연결 상태 ${state}`);
            }
        };

        // SDP offer/answer 협상 (실패 시 재접속 예약)
        try {
            const offer = await pc.createOffer();
            await pc.setLocalDescription(offer);
            await this._waitIceGatheringComplete(pc);

            const resp = await fetch(OFFER_URL, {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ sdp: pc.localDescription.sdp, type: pc.localDescription.type }),
            });
            if (!resp.ok) throw new Error(`${OFFER_URL} 응답 ${resp.status}`);
            const answer = await resp.json();
            if (pc !== this._pc) return;
            await pc.setRemoteDescription(answer);
            console.log("[rtc] 협상 완료");
        } catch (e) {
            if (pc !== this._pc) return;
            console.warn("[rtc] 협상 실패:", e);
            this._scheduleReconnect("협상 실패");
        }
    }

    /**
     * RECONNECT_MS 뒤 재접속을 예약한다 (이미 예약돼 있으면 무시). 반환값 없음.
     */
    _scheduleReconnect(reason) {
        if (this._reconnectTimer !== null) return;
        console.log(`[rtc] ${RECONNECT_MS}ms 후 재접속 예약 (${reason})`);
        this._reconnectTimer = setTimeout(() => {
            this._reconnectTimer = null;
            this.connect();
        }, RECONNECT_MS);
    }

    /**
     * state 채널이 열려 있는지 반환한다.
     */
    _isStateChannelOpen() {
        return Boolean(this._stateChannel && this._stateChannel.readyState === "open");
    }

    /**
     * JSON 메시지를 state 채널로 즉시 보낸다 (명령 등). 반환: 전송 여부 (채널 미개방이면 false).
     */
    send(obj) {
        if (!this._isStateChannelOpen()) return false;
        this._stateChannel.send(JSON.stringify(obj));
        return true;
    }

    /**
     * 포즈 샘플 1개를 전송한다(주기 상한 초과분은 버림). 반환: 전송 여부 (버리면 false).
     * 명령(command)이 실린 샘플은 누른 순간 한 번뿐이므로 주기 상한과 무관하게 항상 보낸다.
     */
    sendPose(sample) {
        if (!this._isStateChannelOpen()) return false;
        const now = performance.now();
        const hasCommand = sample.command != null;
        if (!hasCommand && now - this._lastPoseSendMs < 1000 / POSE_SEND_HZ) return false;
        this._lastPoseSendMs = now;
        this._stateChannel.send(JSON.stringify(sample));
        this._txCount += 1;
        return true;
    }

    /**
     * 1초마다 전송률(txHz)을 갱신하는 타이머를 시작한다. 반환값 없음.
     */
    _startTxRateTimer() {
        setInterval(() => {
            this.txHz = this._txCount;
            this._txCount = 0;
        }, 1000);
    }

    /**
     * ICE gathering이 끝날 때까지 기다린다 (이미 끝났으면 즉시 반환). 반환: Promise.
     */
    _waitIceGatheringComplete(pc) {
        if (pc.iceGatheringState === "complete") return Promise.resolve();
        return new Promise((resolve) => {
            const check = () => {
                if (pc.iceGatheringState === "complete") {
                    pc.removeEventListener("icegatheringstatechange", check);
                    resolve();
                }
            };
            pc.addEventListener("icegatheringstatechange", check);
        });
    }
}
