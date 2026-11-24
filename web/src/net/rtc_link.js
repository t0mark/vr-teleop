// 단일 WebRTC 연결: RTCPeerConnection 하나에 DataChannel 2개를 열어 서버와 통신한다.
//   - "state" (ordered+reliable): 포즈 업링크, robot_config/robot_state/mode_status, set_mode 명령
//   - "mesh"  (ordered+reliable): 재구성 메시 블록 바이너리(MSH1)
// 시그널링: HTTPS POST 1회(rtc/offer)로 offer/answer를 교환한다. ICE 서버(STUN)는 쓰지 않는다.
// 서버 쪽 규약: teleop/net/rtc_hub.py.

// 재접속 대기 시간(ms)
const RECONNECT_MS = 2000;
// 시그널링 주소 (페이지 기준 상대 경로)
const OFFER_URL = "rtc/offer";
// 시그널링 POST 응답 대기 한도(ms)
const OFFER_TIMEOUT_MS = 5000;
// ICE gathering 완료 대기 한도(ms)와 상태 확인 간격(ms)
const ICE_GATHER_TIMEOUT_MS = 3000;
const ICE_GATHER_POLL_MS = 10;
// "disconnected" 상태에서 복구를 기다리는 유예 시간(ms)
const DISCONNECT_GRACE_MS = 3000;
// 포즈 전송 주기 상한(Hz)
const POSE_SEND_HZ = 60;
// 전송률 갱신 주기(ms)
const TX_RATE_INTERVAL_MS = 1000;

/**
 * ms 동안 기다리는 Promise를 반환한다.
 */
function _sleep(ms) {
    return new Promise((resolve) => setTimeout(resolve, ms));
}

export class RtcLink {
    /**
     * onStateMessage: state 채널 JSON 메시지 콜백. onMeshPacket: mesh 채널 바이너리(ArrayBuffer) 콜백.
     */
    constructor(onStateMessage, onMeshPacket) {
        this._onStateMessage = onStateMessage;
        this._onMeshPacket = onMeshPacket;
        this._pc = null;
        this._stateChannel = null;
        // state 채널 연결 여부
        this._connected = false;
        // 대기 중인 재접속 타이머 (없으면 null)
        this._reconnectTimer = null;
        // "disconnected" 유예 타이머 (없으면 null)
        this._disconnectTimer = null;
        // 다음 포즈 전송 마감 시각(ms)
        this._nextPoseSendMs = 0;
        // 전송 횟수와 최근 1초 포즈 전송률
        this._txCount = 0;
        this._txHz = 0;
        setInterval(() => this._updateTxRate(), TX_RATE_INTERVAL_MS);
    }

    /** state 채널 연결 여부. */
    get connected() {
        return this._connected;
    }

    /** 최근 1초 포즈 전송률(Hz). */
    get txHz() {
        return this._txHz;
    }

    /**
     * RTCPeerConnection·두 DataChannel을 만들고 OFFER_URL로 협상한다. 반환값 없음.
     * 끊기면(연결 끊김·실패·종료, state 채널 닫힘, 협상 실패) 자동 재접속한다.
     */
    async connect() {
        // 이전 연결 정리
        if (this._pc) {
            this._pc.close();
            this._pc = null;
        }
        this._clearDisconnectTimer();
        this._connected = false;

        // 새 연결 생성
        const pc = new RTCPeerConnection({ iceServers: [] });
        this._pc = pc;
        console.log("[rtc] 연결 시도");

        // state 채널: 열림·닫힘·메시지 처리 연결
        const stateChannel = pc.createDataChannel("state", { ordered: true });
        stateChannel.onopen = () => this._handleStateOpen(pc);
        stateChannel.onclose = () => this._handleStateClose(pc);
        stateChannel.onmessage = (event) => this._handleStateMessage(event);
        this._stateChannel = stateChannel;

        // mesh 채널: 바이너리 패킷 전달
        const meshChannel = pc.createDataChannel("mesh", { ordered: true });
        meshChannel.binaryType = "arraybuffer";
        meshChannel.onmessage = (event) => this._onMeshPacket(event.data);

        // 연결 상태 변화 처리
        pc.onconnectionstatechange = () => this._handleConnectionState(pc);

        // SDP offer/answer 협상 (실패 시 재접속 예약)
        try {
            const offer = await pc.createOffer();
            await pc.setLocalDescription(offer);
            await this._waitIceGatheringComplete(pc);

            const answer = await this._postOffer(pc.localDescription);
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
     * state 채널 열림을 반영한다 (이전 연결의 이벤트는 무시). 반환값 없음.
     */
    _handleStateOpen(pc) {
        if (pc !== this._pc) return;
        this._connected = true;
        console.log("[rtc] state 채널 열림");
    }

    /**
     * state 채널 닫힘 시 재접속을 예약한다 (이전 연결의 이벤트는 무시). 반환값 없음.
     */
    _handleStateClose(pc) {
        if (pc !== this._pc) return;
        this._connected = false;
        console.warn("[rtc] state 채널 닫힘");
        this._scheduleReconnect("state 채널 닫힘");
    }

    /**
     * state 채널 JSON 메시지를 파싱해 콜백으로 넘긴다. 반환값 없음.
     */
    _handleStateMessage(event) {
        try {
            this._onStateMessage(JSON.parse(event.data));
        } catch (e) {
            console.warn("[rtc] state 메시지 파싱 실패:", e);
        }
    }

    /**
     * 연결 상태를 처리한다 (이전 연결의 이벤트는 무시). 반환값 없음.
     * failed·closed는 즉시, disconnected는 DISCONNECT_GRACE_MS 동안 복구되지 않으면 재접속을 예약한다.
     */
    _handleConnectionState(pc) {
        if (pc !== this._pc) return;
        const state = pc.connectionState;
        console.log(`[rtc] 연결 상태: ${state}`);

        // 연결 복구: 유예 타이머 해제
        if (state === "connected") {
            this._clearDisconnectTimer();
            return;
        }

        // 실패·종료: 즉시 재접속 예약
        if (state === "failed" || state === "closed") {
            this._clearDisconnectTimer();
            this._connected = false;
            this._scheduleReconnect(`연결 상태 ${state}`);
            return;
        }

        // 일시 끊김: 유예 시간 뒤에도 그대로면 재접속 예약
        if (state === "disconnected" && this._disconnectTimer === null) {
            console.warn(`[rtc] 연결 끊김 — ${DISCONNECT_GRACE_MS}ms 동안 복구 대기`);
            this._disconnectTimer = setTimeout(() => this._handleDisconnectGraceEnd(pc), DISCONNECT_GRACE_MS);
        }
    }

    /**
     * disconnected 유예가 끝났을 때 아직 끊겨 있으면 재접속을 예약한다. 반환값 없음.
     */
    _handleDisconnectGraceEnd(pc) {
        this._disconnectTimer = null;
        if (pc !== this._pc || pc.connectionState !== "disconnected") return;
        this._connected = false;
        this._scheduleReconnect(`연결 끊김 ${DISCONNECT_GRACE_MS}ms 지속`);
    }

    /**
     * disconnected 유예 타이머를 해제한다. 반환값 없음.
     */
    _clearDisconnectTimer() {
        if (this._disconnectTimer === null) return;
        clearTimeout(this._disconnectTimer);
        this._disconnectTimer = null;
    }

    /**
     * 로컬 SDP를 OFFER_URL로 보내고 answer를 반환한다 (OFFER_TIMEOUT_MS 초과 시 중단·예외). 반환: Promise.
     */
    async _postOffer(localDescription) {
        const abortController = new AbortController();
        const abortTimer = setTimeout(() => abortController.abort(), OFFER_TIMEOUT_MS);
        try {
            const response = await fetch(OFFER_URL, {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ sdp: localDescription.sdp, type: localDescription.type }),
                signal: abortController.signal,
            });
            if (!response.ok) throw new Error(`${OFFER_URL} 응답 ${response.status}`);
            return await response.json();
        } catch (e) {
            if (e.name === "AbortError") throw new Error(`${OFFER_URL} 응답 시간 초과 (${OFFER_TIMEOUT_MS}ms)`);
            throw e;
        } finally {
            clearTimeout(abortTimer);
        }
    }

    /**
     * RECONNECT_MS 뒤 재접속을 예약한다 (이미 예약돼 있으면 무시). 반환값 없음.
     */
    _scheduleReconnect(reason) {
        if (this._reconnectTimer !== null) return;
        console.log(`[rtc] ${RECONNECT_MS}ms 후 재접속 예약 (${reason})`);
        this._reconnectTimer = setTimeout(() => this._handleReconnectTimer(), RECONNECT_MS);
    }

    /**
     * 재접속 타이머 만료 시 재접속한다. 반환값 없음.
     */
    _handleReconnectTimer() {
        this._reconnectTimer = null;
        this.connect();
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
    send(message) {
        if (!this._isStateChannelOpen()) return false;
        this._stateChannel.send(JSON.stringify(message));
        return true;
    }

    /**
     * 지금 포즈를 보낼 차례인지 반환한다 (채널이 열려 있고 전송 마감 시각이 지났으면 true).
     */
    isPoseDue() {
        return this._isStateChannelOpen() && performance.now() >= this._nextPoseSendMs;
    }

    /**
     * 포즈 샘플 1개를 전송한다(주기 상한 초과분은 버림). 반환: 전송 여부 (버리면 false).
     * 명령(command)이 실린 샘플은 주기 상한과 무관하게 항상 보낸다.
     */
    sendPose(sample) {
        if (!this._isStateChannelOpen()) return false;
        const now = performance.now();
        const isDue = now >= this._nextPoseSendMs;
        if (!isDue && sample.command == null) return false;

        // 마감 시각을 한 주기씩 누적 (밀렸으면 현재 시각부터 다시 센다)
        if (isDue) {
            this._nextPoseSendMs = Math.max(this._nextPoseSendMs + 1000 / POSE_SEND_HZ, now);
        }

        // 전송
        this._stateChannel.send(JSON.stringify(sample));
        this._txCount += 1;
        return true;
    }

    /**
     * 최근 주기 전송 횟수로 전송률(txHz)을 갱신한다. 반환값 없음.
     */
    _updateTxRate() {
        this._txHz = this._txCount;
        this._txCount = 0;
    }

    /**
     * ICE gathering이 끝날 때까지 기다린다 (ICE_GATHER_TIMEOUT_MS 초과 시 예외). 반환: Promise.
     */
    async _waitIceGatheringComplete(pc) {
        const deadlineMs = performance.now() + ICE_GATHER_TIMEOUT_MS;
        while (pc.iceGatheringState !== "complete") {
            if (performance.now() >= deadlineMs) {
                throw new Error(`ICE gathering 시간 초과 (${ICE_GATHER_TIMEOUT_MS}ms)`);
            }
            await _sleep(ICE_GATHER_POLL_MS);
        }
    }
}
