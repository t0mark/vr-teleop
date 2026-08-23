// 포즈 업링크: PoseReader 샘플을 WSS(/ws/pose)로 고정 주기 전송하고,
// 같은 소켓으로 내려오는 서버 메시지를 콜백에 전달한다.
//   - 텍스트 프레임(robot_config/robot_state): JSON 파싱 후 onServerMessage
//   - 바이너리 프레임(포인트클라우드 패킷): ArrayBuffer 그대로 onBinary
// 접속 호스트를 그대로 사용하므로 443/8443 어느 포트로 접속했든 동작한다.

// 전송 주기 상한(Hz): XR 프레임(72-120Hz)보다 낮춰 대역폭·서버 부하를 제어
const SEND_HZ = 60;
// 끊김 시 재접속 대기(ms)
const RECONNECT_MS = 2000;

export class PoseUplink {
    /**
     * 업링크 상태를 초기화한다. onServerMessage: JSON 메시지 콜백, onBinary: 바이너리 패킷 콜백.
     * 실제 접속은 connect()에서 수행.
     */
    constructor(onServerMessage, onBinary) {
        this._onServerMessage = onServerMessage;
        this._onBinary = onBinary;
        this._ws = null;
        this._lastSendMs = 0;
        this._txCount = 0;
        this.connected = false;
        // 최근 1초 전송률 (HUD 표시용, 1초마다 갱신)
        this.txHz = 0;
        this._startTxRateTimer();
    }

    /**
     * WSS 접속을 시작한다. 끊기면 자동 재접속한다. 반환값 없음.
     */
    connect() {
        const url = `wss://${location.host}/ws/pose`;
        this._ws = new WebSocket(url);
        // Blob 변환 비용을 피하기 위해 바이너리는 ArrayBuffer로 받는다
        this._ws.binaryType = "arraybuffer";

        this._ws.onopen = () => {
            this.connected = true;
            console.log(`[uplink] 접속: ${url}`);
        };
        this._ws.onmessage = (ev) => {
            if (ev.data instanceof ArrayBuffer) {
                this._onBinary?.(ev.data);
                return;
            }
            try {
                this._onServerMessage?.(JSON.parse(ev.data));
            } catch (e) {
                console.warn("[uplink] 서버 메시지 파싱 실패:", e);
            }
        };
        this._ws.onclose = () => {
            this.connected = false;
            // 서버 재시작·일시 단선에 대비해 상시 재접속 (VR 세션은 유지)
            setTimeout(() => this.connect(), RECONNECT_MS);
        };
        this._ws.onerror = () => this._ws.close();
    }

    /**
     * 샘플 1개를 전송한다(주기 상한 초과분은 버림). 반환값 없음.
     */
    send(sample) {
        if (!this.connected) return;
        const now = performance.now();
        if (now - this._lastSendMs < 1000 / SEND_HZ) return;
        this._lastSendMs = now;
        this._ws.send(JSON.stringify(sample));
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
}
