"""WebRTC 허브: 클라이언트별 RTCPeerConnection과 두 DataChannel(state·mesh)을 관리한다.

시그널링은 지속 연결이 아니라 HTTP POST 1회(web_server.py의 /rtc/offer)로 끝난다 — WebRTC 스펙은
시그널링 방식을 정하지 않으므로, 매번 재사용할 상시 연결(웹소켓 등)을 따로 둘 필요가 없다.

채널 2개, 성격이 다르다.
- "state" (ordered+reliable): 포즈 업링크(브라우저→서버), robot_config/robot_state/mode_status
  (서버→브라우저), set_mode 명령(브라우저→서버). 순서·신뢰성이 중요한 소량 JSON 텍스트.
- "mesh" (ordered+reliable): 재구성 메시 블록 바이너리(MSH1, 서버→브라우저). 스냅샷(전체 삭제)과 블록 갱신의
  순서가 바뀌면 맵이 틀어지고, 버리면 구멍이 남으므로 순서·신뢰성을 모두 보장한다.
브라우저가 offer에 두 채널을 모두 실어 보내므로 서버는 "datachannel" 이벤트에서 label로 구분해 받는다.

STUN 서버 없음: Quest-PC는 유선 직결(같은 서브넷)이라 NAT 통과가 필요 없다. 기본 STUN을 켜 두면
이 PC의 공인 IP로 hairpin 접속을 시도하다 실패해 협상이 지연되므로 명시적으로 끈다.
"""

import asyncio
import functools
import json
import logging
import time

from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription

from teleop.utils import config
from teleop.utils.inputs import sample_command
from teleop.utils.transforms import webxr_sample_to_robot

logger = logging.getLogger("rtc_hub")

_RTC_CONFIG = RTCConfiguration(iceServers=[])


class _Peer:
    """클라이언트 1개의 RTCPeerConnection과 두 DataChannel을 묶는다 (허브 내부용)."""

    def __init__(self, pc: RTCPeerConnection, peer_id: int):
        self.pc = pc
        self.peer_id = peer_id
        self.state_channel = None
        self.mesh_channel = None


class RtcHub:
    """WebRTC 협상·클라이언트 집합·최신 포즈 상태를 관리한다 (단일 asyncio 루프 전제)."""

    def __init__(self):
        self._peers: list[_Peer] = []
        self._latest_raw: dict | None = None
        self._latest_robot: dict | None = None
        self._latest_rx_mono: float = 0.0
        # 수신한 샘플 명령(start/ready/stop) 래치: 제어 루프가 take_command()로 한 번만 소비한다
        self._pending_command: str | None = None
        self._rx_count = 0
        # 포즈 소유 피어: 이 피어의 샘플만 IK 입력·명령으로 받는다 (소유 피어 샘플이 stale이면 이양)
        self._pose_owner: _Peer | None = None
        # 소유 피어가 아니라 샘플을 무시당한 피어 (무시 경고를 피어당 1회만 남기기 위함)
        self._ignored_pose_peers: set[int] = set()
        self._next_peer_id = 1
        self._stats_task: asyncio.Task | None = None
        # RobotStateBroadcaster가 시작 시 채우는 로봇 메시 구성 (신규 접속마다 1회 전송)
        self.robot_config: dict | None = None
        # 모드 매니저가 채우는 현재 모드 상태 (신규 접속마다 1회 전송)
        self.mode_status: dict | None = None
        # 클라이언트 set_mode 요청 처리 콜백 (async, 모드 매니저가 등록)
        self.on_set_mode = None
        # mesh 채널이 새로 열릴 때 호출할 콜백 (channel 인자, 인식 서비스가 메시 스냅샷 전송용으로 등록)
        self.on_mesh_open = None

    @property
    def latest_robot(self) -> dict | None:
        """로봇 좌표계로 변환된 최신 샘플 (IK 입력)."""
        return self._latest_robot

    def take_command(self) -> str | None:
        """래치된 최신 샘플 명령을 꺼내 반환하고 비운다 (없으면 None)."""
        command = self._pending_command
        self._pending_command = None
        return command

    def sample_age_s(self) -> float:
        """최신 샘플 수신 후 경과 시간(초)을 반환한다 (신선도 판단용)."""
        if self._latest_rx_mono == 0.0:
            return float("inf")
        return time.monotonic() - self._latest_rx_mono

    def start(self):
        """수신률 로그 태스크를 시작한다. 반환값 없음."""
        self._stats_task = asyncio.create_task(self._log_stats())

    async def stop(self):
        """로그 태스크를 정리한다. 반환값 없음."""
        if self._stats_task is not None:
            self._stats_task.cancel()
            self._stats_task = None

    async def handle_offer(self, sdp: str, sdp_type: str) -> dict:
        """offer(SDP)를 받아 RTCPeerConnection을 만들고 answer를 {sdp, type}으로 반환한다.

        두 DataChannel은 브라우저가 offer에 실어 만들므로 서버는 "datachannel" 이벤트로 받는다.
        """
        pc = RTCPeerConnection(configuration=_RTC_CONFIG)
        peer = _Peer(pc, self._next_peer_id)
        self._next_peer_id += 1
        self._peers.append(peer)

        @pc.on("datachannel")
        def _on_datachannel(channel):
            if channel.label == "state":
                peer.state_channel = channel
                self._send_initial_state(channel)
                channel.on("message", functools.partial(self._handle_state_message, peer))
            elif channel.label == "mesh":
                peer.mesh_channel = channel
                if self.on_mesh_open is not None:
                    self.on_mesh_open(channel)
            logger.info(
                f"데이터 채널 열림: {channel.label} (피어 {peer.peer_id}, 클라이언트 {len(self._peers)}명)"
            )

        @pc.on("connectionstatechange")
        async def _on_state_change():
            if pc.connectionState in ("failed", "closed"):
                self._remove_peer(peer)

        # SDP 협상: setLocalDescription은 ICE gathering 완료까지 대기 (aiortc는 trickle 미지원)
        try:
            await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type=sdp_type))
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)
        except Exception as e:
            logger.error(f"WebRTC 협상 실패 (피어 {peer.peer_id}): {type(e).__name__}: {e}")
            self._remove_peer(peer)
            await pc.close()
            raise
        return {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}

    def _remove_peer(self, peer: _Peer):
        """접속 종료된 피어를 목록에서 뺀다. 반환값 없음."""
        if peer in self._peers:
            self._peers.remove(peer)
        self._ignored_pose_peers.discard(peer.peer_id)

        # 포즈 소유 피어가 나가면 소유권을 비운다 (다음 샘플 송신 피어가 소유)
        if self._pose_owner is peer:
            self._pose_owner = None
            logger.warning(f"포즈 소유 피어 {peer.peer_id} 연결 종료 — 소유권 해제")
        logger.info(f"클라이언트 연결 종료 (남은 {len(self._peers)}명)")

    def _send_initial_state(self, channel):
        """신규 state 채널에 로봇 메시 구성·현재 모드 상태를 1회 보낸다. 반환값 없음."""
        if self.robot_config is not None:
            channel.send(json.dumps(self.robot_config))
        if self.mode_status is not None:
            channel.send(json.dumps(self.mode_status))

    def _handle_state_message(self, peer: _Peer, data):
        """peer의 state 채널 수신 메시지를 처리한다 (포즈 샘플 또는 명령). 반환값 없음."""
        try:
            msg = json.loads(data)
        except (json.JSONDecodeError, TypeError) as e:
            logger.warning(f"state 채널 메시지 파싱 오류: {e}")
            return

        if isinstance(msg, dict) and "type" in msg:
            self._handle_command(msg)
            return

        # 포즈 소유 피어가 아닌 샘플은 IK 입력·명령·중계 모두에서 제외한다
        if not self._accept_pose_sender(peer):
            return

        # 포즈 샘플: 원본은 다른 클라이언트 중계용(데스크톱 사람 아바타 표시), 변환본은 IK 입력용
        self._latest_raw = msg
        self._latest_robot = webxr_sample_to_robot(msg)
        self._latest_rx_mono = time.monotonic()
        self._rx_count += 1

        # 명령은 누른 순간의 샘플 하나에만 실리므로, 제어 루프 주기 사이에 덮이지 않게 래치한다
        command = sample_command(self._latest_robot)
        if command is not None:
            if self._pending_command is not None:
                logger.warning(f"미처리 명령 {self._pending_command} 를 새 명령 {command} 로 대체")
            self._pending_command = command
            logger.info(f"샘플 명령 수신: {command}")
        self._relay_pose(peer, data)

    def _accept_pose_sender(self, peer: _Peer) -> bool:
        """peer의 포즈 샘플을 받아들일지 판단하고 필요하면 소유권을 넘긴다. 반환: 수락 여부."""
        if self._pose_owner is peer:
            return True

        # 소유 피어가 없으면 첫 송신 피어가 소유
        if self._pose_owner is None:
            self._pose_owner = peer
            self._ignored_pose_peers.clear()
            logger.info(f"포즈 소유 피어 지정: 피어 {peer.peer_id}")
            return True

        # 소유 피어 샘플이 stale이면 새 송신 피어로 소유권 이양
        if self.sample_age_s() > config.POSE_STALE_S:
            logger.warning(
                f"포즈 소유권 이양: 피어 {self._pose_owner.peer_id} → 피어 {peer.peer_id} "
                f"(소유 피어 샘플 {self.sample_age_s():.2f}초 경과)"
            )
            self._pose_owner = peer
            self._ignored_pose_peers.clear()
            return True

        # 소유 피어가 살아 있으면 다른 피어 샘플은 무시 (경고는 피어당 1회)
        if peer.peer_id not in self._ignored_pose_peers:
            self._ignored_pose_peers.add(peer.peer_id)
            logger.warning(
                f"피어 {peer.peer_id} 포즈 샘플 무시 — 현재 소유 피어 {self._pose_owner.peer_id}"
            )
        return False

    def _relay_pose(self, sender: _Peer, raw_text: str):
        """수신한 포즈 샘플 원본을 보낸 피어를 뺀 클라이언트의 state 채널로 중계한다. 반환값 없음."""
        for peer in self._peers:
            if peer is sender:
                continue
            ch = peer.state_channel
            if ch is not None and ch.readyState == "open":
                ch.send(raw_text)

    def _handle_command(self, msg: dict):
        """클라이언트 명령 메시지를 처리한다 (현재는 set_mode뿐). 반환값 없음."""
        if msg.get("type") != "set_mode":
            logger.warning(f"알 수 없는 명령 무시: {msg.get('type')!r}")
            return
        if self.on_set_mode is None:
            logger.warning("set_mode 처리자 미등록 — 무시")
            return
        mode = str(msg.get("mode", ""))
        logger.warning(f"모드 전환 요청 수신: {mode}")
        asyncio.create_task(self.on_set_mode(mode))

    async def broadcast_state(self, state: dict):
        """robot_state 등 서버 발신 JSON 메시지를 모든 클라이언트에 방송한다. 반환값 없음."""
        data = json.dumps(state)
        for peer in self._peers:
            ch = peer.state_channel
            if ch is not None and ch.readyState == "open":
                ch.send(data)

    def broadcast_mesh(self, payload: bytes):
        """메시 블록 패킷을 모든 클라이언트에 방송한다 (버리면 맵에 구멍이 남으므로 항상 보낸다). 반환값 없음."""
        for peer in self._peers:
            ch = peer.mesh_channel
            if ch is None or ch.readyState != "open":
                continue
            try:
                ch.send(payload)
            except Exception as e:
                logger.warning(f"메시 블록 송신 실패: {type(e).__name__}: {e}")

    async def close_all(self):
        """전체 연결을 정리한다 (서버 종료 경로). 반환값 없음."""
        for peer in list(self._peers):
            await peer.pc.close()
        self._peers.clear()

    async def _log_stats(self):
        """주기적으로 수신률·변환된 손목 위치·접속 클라이언트 수를 로그로 남긴다."""
        while True:
            await asyncio.sleep(config.POSE_STATS_INTERVAL_S)
            hz = self._rx_count / config.POSE_STATS_INTERVAL_S
            self._rx_count = 0

            wrist_info = "-"
            if self._latest_robot is not None:
                right = self._latest_robot["hands"].get("right")
                if right is not None:
                    p = right["pos"]
                    wrist_info = f"오른손목(로봇) [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}]"

            logger.info(
                f"포즈 수신 {hz:.1f} Hz, 클라이언트 {len(self._peers)}명, {wrist_info}"
            )
