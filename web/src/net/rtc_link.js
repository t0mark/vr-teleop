// 단일 WebRTC 연결: RTCPeerConnection 하나에 DataChannel 2개를 열어 서버와 통신한다.
//   - "state" (ordered+reliable): 포즈 업링크, robot_config/robot_state/mode_status, set_mode 명령
//   - "pcd"   (unordered+unreliable): 포인트클라우드 바이너리 — 한 프레임 스냅샷이라 유실되면
//     버리는 게 맞다 (재전송이 최신 데이터를 지연시키는 TCP류 head-of-line blocking 회피)
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
     * onStateMessage: state 채널 JSON 메시지 콜백. onPcdPacket: pcd 채널 바이너리(ArrayBuffer) 콜백.
     */
    constructor(onStateMessage, onPcdPacket) {
        this._onStateMessage = onStateMessage;
        this._onPcdPacket = onPcdPacket;
        this._pc = null;
        this._stateChannel = null;
        this.connected = false;
        this._lastPoseSendMs = 0;
        this._txCount = 0;
        // 최근 1초 포즈 전송률 (HUD 표시용, 1초마다 갱신)
        this.txHz = 0;
        this._startTxRateTimer();
    }

    /**
     * RTCPeerConnection·두 DataChannel을 만들고 /rtc/offer로 협상한다. 반환값 없음.
     * 끊기면(연결 실패·종료) 자동 재접속한다.
     */
    async connect() {
        if (this._pc) {
            this._pc.close();
            this._pc = null;
        }
        const pc = new RTCPeerConnection({ iceServers: [] });
        this._pc = pc;

        const stateChannel = pc.createDataChannel("state", { ordered: true });
        stateChannel.onopen = () => {
            this.connected = true;
            console.log("[rtc] state 채널 열림");
        };
        stateChannel.onmessage = (ev) => {
            try {
                this._onStateMessage(JSON.parse(ev.data));
            } catch (e) {
                console.warn("[rtc] state 메시지 파싱 실패:", e);
            }
        };
        this._stateChannel = stateChannel;

        const pcdChannel = pc.createDataChannel("pcd", { ordered: false, maxRetransmits: 0 });
        pcdChannel.binaryType = "arraybuffer";
        pcdChannel.onmessage = (ev) => this._onPcdPacket(ev.data);

        pc.onconnectionstatechange = () => {
            if (pc.connectionState === "failed" || pc.connectionState === "closed") {
                this.connected = false;
                setTimeout(() => this.connect(), RECONNECT_MS);
            }
        };

        const offer = await pc.createOffer();
        await pc.setLocalDescription(offer);
        await this._waitIceGatheringComplete(pc);

        const resp = await fetch(OFFER_URL, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ sdp: pc.localDescription.sdp, type: pc.localDescription.type }),
        });
        const answer = await resp.json();
        await pc.setRemoteDescription(answer);
    }

    /**
     * JSON 메시지를 state 채널로 즉시 보낸다 (명령 등). 반환: 전송 여부 (미접속이면 false).
     */
    send(obj) {
        if (!this.connected) return false;
        this._stateChannel.send(JSON.stringify(obj));
        return true;
    }

    /**
     * 포즈 샘플 1개를 전송한다(주기 상한 초과분은 버림). 반환값 없음.
     */
    sendPose(sample) {
        if (!this.connected) return;
        const now = performance.now();
        if (now - this._lastPoseSendMs < 1000 / POSE_SEND_HZ) return;
        this._lastPoseSendMs = now;
        this._stateChannel.send(JSON.stringify(sample));
        this._txCount += 1;
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
