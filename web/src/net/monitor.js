// 모니터 다운링크: 데스크톱 뷰에서 WSS(/ws/monitor)로 서버 메시지를 수신한다.
//   - 텍스트 프레임: VR 포즈 샘플 중계 + robot_config/robot_state → JSON 파싱 후 onMessage
//   - 바이너리 프레임: 포인트클라우드 패킷 → ArrayBuffer 그대로 onBinary
// 라우팅은 호출측 콜백에서 처리한다.

// 끊김 시 재접속 대기(ms)
const RECONNECT_MS = 2000;

export class MonitorLink {
    /**
     * 다운링크 상태를 초기화한다. onMessage: JSON 메시지 콜백, onBinary: 바이너리 패킷 콜백.
     */
    constructor(onMessage, onBinary) {
        this._onMessage = onMessage;
        this._onBinary = onBinary;
        this._ws = null;
        this.connected = false;
    }

    /**
     * WSS 접속을 시작한다. 끊기면 자동 재접속한다. 반환값 없음.
     */
    connect() {
        const url = `wss://${location.host}/ws/monitor`;
        this._ws = new WebSocket(url);
        // Blob 변환 비용을 피하기 위해 바이너리는 ArrayBuffer로 받는다
        this._ws.binaryType = "arraybuffer";

        this._ws.onopen = () => {
            this.connected = true;
            console.log(`[monitor] 접속: ${url}`);
        };
        this._ws.onmessage = (ev) => {
            if (ev.data instanceof ArrayBuffer) {
                this._onBinary?.(ev.data);
                return;
            }
            try {
                this._onMessage(JSON.parse(ev.data));
            } catch (e) {
                console.warn("[monitor] 메시지 파싱 실패:", e);
            }
        };
        this._ws.onclose = () => {
            this.connected = false;
            setTimeout(() => this.connect(), RECONNECT_MS);
        };
        this._ws.onerror = () => this._ws.close();
    }

    /**
     * JSON 명령 메시지를 서버로 보낸다 (모드 전환 등). 반환: 전송 여부 (미접속이면 false).
     */
    send(obj) {
        if (!this.connected || this._ws?.readyState !== WebSocket.OPEN) return false;
        this._ws.send(JSON.stringify(obj));
        return true;
    }
}
