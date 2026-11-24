"""WebRTC 허브: 클라이언트별 RTCPeerConnection과 두 DataChannel(state·mesh)을 관리한다.

시그널링: HTTP POST 1회(web_server.py의 /rtc/offer)로 offer/answer를 교환한다.

채널 2개 (브라우저가 offer에 실어 만들고, 서버는 "datachannel" 이벤트에서 label로 구분한다).
- "state" (ordered+reliable): 포즈 업링크(브라우저→서버), robot_config/robot_state/mode_status
  (서버→브라우저), set_mode 명령(브라우저→서버) JSON 텍스트.
- "mesh" (ordered+reliable): 재구성 메시 블록 바이너리(MSH1, 서버→브라우저).

ICE: STUN 서버 없이 호스트 후보만 쓴다. offer의 mDNS(<uuid>.local) 후보 주소는 HTTP 요청 원격 주소로 치환한다.
"""

import asyncio
import functools
import json
import logging
import re
import time

from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription

from teleop.utils import config
from teleop.utils.inputs import sample_command
from teleop.utils.transforms import webxr_sample_to_robot

logger = logging.getLogger("rtc_hub")

# STUN 서버 없는 RTCPeerConnection 설정
_RTC_CONFIG = RTCConfiguration(iceServers=[])
# SDP 후보 줄의 mDNS 감춤 주소 ("a=candidate:... <uuid>.local <port> typ host")
_MDNS_CANDIDATE = re.compile(r"(a=candidate:\S+ \d+ \S+ \d+ )([0-9a-fA-F-]+\.local)( \d+ typ )")
# 명령 래치 우선순위: 대기 중인 명령보다 낮은 순위의 새 명령은 대기 명령을 덮지 못한다
_COMMAND_PRIORITY = {"start": 0, "ready": 0, "home": 1, "stop": 2}
# 포즈 소유 피어가 아니어도 받아들이는 정지 계열 명령
_STOP_COMMANDS = ("stop", "home")


class _Peer:
    """클라이언트 1개의 RTCPeerConnection과 두 DataChannel을 묶는다 (허브 내부용)."""

    def __init__(self, peer_connection: RTCPeerConnection, peer_id: int):
        self._peer_connection = peer_connection
        self._peer_id = peer_id
        self._state_channel = None
        self._mesh_channel = None
        # 채널별 송신 대기 초과 상태 (상태가 바뀔 때만 로그를 남긴다)
        self._state_congested = False
        self._mesh_congested = False

    @property
    def peer_connection(self) -> RTCPeerConnection:
        """이 피어의 RTCPeerConnection."""
        return self._peer_connection

    @property
    def peer_id(self) -> int:
        """허브가 부여한 피어 번호."""
        return self._peer_id

    @property
    def state_channel(self):
        """state DataChannel (열리기 전이면 None)."""
        return self._state_channel

    @state_channel.setter
    def state_channel(self, channel):
        self._state_channel = channel

    @property
    def mesh_channel(self):
        """mesh DataChannel (열리기 전이면 None)."""
        return self._mesh_channel

    @mesh_channel.setter
    def mesh_channel(self, channel):
        self._mesh_channel = channel

    @property
    def state_congested(self) -> bool:
        """state 채널 송신 대기 상한 초과 여부."""
        return self._state_congested

    @state_congested.setter
    def state_congested(self, congested: bool):
        self._state_congested = congested

    @property
    def mesh_congested(self) -> bool:
        """mesh 채널 송신 대기 경고 기준 초과 여부."""
        return self._mesh_congested

    @mesh_congested.setter
    def mesh_congested(self, congested: bool):
        self._mesh_congested = congested


class RtcHub:
    """WebRTC 협상·클라이언트 집합·최신 포즈 상태를 관리한다 (단일 asyncio 루프 전제)."""

    def __init__(self):
        self._peers: list[_Peer] = []
        self._latest_robot: dict | None = None
        self._latest_rx_mono: float = 0.0
        # 수신한 샘플 명령(start/ready/stop/home) 래치: 제어 루프가 take_command()로 한 번만 소비한다
        self._pending_command: str | None = None
        self._rx_count = 0
        # 로그 주기 동안 형식 오류·변환 실패로 버린 샘플 수와 마지막 사유
        self._invalid_sample_count = 0
        self._last_invalid_reason = ""
        # 포즈 소유 피어: 이 피어의 샘플만 IK 입력·명령으로 받는다
        self._pose_owner: _Peer | None = None
        # 샘플을 무시당한 피어 번호 (무시 경고를 피어당 1회만 남긴다)
        self._ignored_pose_peers: set[int] = set()
        self._next_peer_id = 1
        self._stats_task: asyncio.Task | None = None
        # 실행 중인 set_mode 처리 태스크 (완료까지 참조 보관)
        self._mode_tasks: set[asyncio.Task] = set()
        # 로봇 제어 중(armed) 여부 조회 콜백 (bool 반환, None이면 대기로 간주)
        self._armed_provider = None
        # 신규 접속마다 1회 전송하는 로봇 메시 구성 (RobotStateBroadcaster가 등록)
        self._robot_config: dict | None = None
        # 신규 접속마다 1회 전송하는 현재 모드 상태 (모드 매니저가 등록)
        self._mode_status: dict | None = None
        # 클라이언트 set_mode 요청 처리 콜백 (async, 모드 매니저가 등록)
        self._on_set_mode = None
        # mesh 채널이 새로 열릴 때 호출할 콜백 (channel 인자, 인식 서비스가 등록)
        self._on_mesh_open = None

    @property
    def latest_robot(self) -> dict | None:
        """로봇 좌표계로 변환된 최신 샘플 (IK 입력)."""
        return self._latest_robot

    @property
    def robot_config(self) -> dict | None:
        """신규 접속에 1회 보내는 로봇 메시 구성 메시지."""
        return self._robot_config

    @robot_config.setter
    def robot_config(self, robot_config: dict | None):
        self._robot_config = robot_config

    @property
    def mode_status(self) -> dict | None:
        """신규 접속에 1회 보내는 현재 모드 상태 메시지."""
        return self._mode_status

    @mode_status.setter
    def mode_status(self, mode_status: dict | None):
        self._mode_status = mode_status

    @property
    def on_set_mode(self):
        """클라이언트 set_mode 요청 처리 콜백 (async, mode 문자열 인자)."""
        return self._on_set_mode

    @on_set_mode.setter
    def on_set_mode(self, handler):
        self._on_set_mode = handler
        logger.info(f"set_mode 처리 콜백 {'등록' if handler is not None else '해제'}")

    @property
    def on_mesh_open(self):
        """mesh 채널이 새로 열릴 때 호출할 콜백 (channel 인자)."""
        return self._on_mesh_open

    @on_mesh_open.setter
    def on_mesh_open(self, handler):
        self._on_mesh_open = handler
        logger.info(f"mesh 채널 열림 콜백 {'등록' if handler is not None else '해제'}")

    @property
    def armed_provider(self):
        """로봇 제어 중(armed) 여부를 반환하는 콜백 (None이면 대기로 간주)."""
        return self._armed_provider

    @armed_provider.setter
    def armed_provider(self, provider):
        self._armed_provider = provider
        logger.info(f"제어 상태 조회 콜백 {'등록' if provider is not None else '해제'}")

    def _is_armed(self) -> bool:
        """armed_provider로 로봇 제어 중 여부를 조회한다. 반환: 제어 중 여부 (조회 실패 시 제어 중으로 간주)."""
        if self._armed_provider is None:
            return False

        try:
            return bool(self._armed_provider())
        except Exception as e:
            logger.error(f"제어 상태 조회 실패, 제어 중으로 간주: {type(e).__name__}: {e}")
            return True

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
        """로그 태스크를 취소하고 종료를 기다린다. 반환값 없음."""
        if self._stats_task is None:
            return

        self._stats_task.cancel()
        try:
            await self._stats_task
        except asyncio.CancelledError:
            pass
        self._stats_task = None
        logger.info("수신률 로그 태스크 종료")

    async def handle_offer(self, sdp: str, sdp_type: str, remote_ip: str | None = None) -> dict:
        """offer(SDP)를 받아 RTCPeerConnection을 만들고 answer를 {sdp, type}으로 반환한다.

        remote_ip: 시그널링 요청을 보낸 클라이언트 IP — offer의 mDNS(.local) 후보 주소를 이 IP로 바꾼다.
        """
        # offer의 mDNS 후보 주소를 요청 주소로 치환
        if remote_ip:
            sdp, replaced = _MDNS_CANDIDATE.subn(rf"\g<1>{remote_ip}\g<3>", sdp)
            if replaced:
                logger.info(f"mDNS 후보 {replaced}개를 요청 주소 {remote_ip}로 치환")

        # 피어 생성·등록
        peer_connection = RTCPeerConnection(configuration=_RTC_CONFIG)
        peer = _Peer(peer_connection, self._next_peer_id)
        self._next_peer_id += 1
        self._peers.append(peer)

        # 채널 열림·연결 상태 이벤트 처리자 등록
        peer_connection.on("datachannel", functools.partial(self._on_datachannel, peer))
        peer_connection.on("connectionstatechange", functools.partial(self._on_connection_state_change, peer))

        # SDP 협상 (setLocalDescription은 ICE gathering 완료까지 대기)
        try:
            await peer_connection.setRemoteDescription(RTCSessionDescription(sdp=sdp, type=sdp_type))
            answer = await peer_connection.createAnswer()
            await peer_connection.setLocalDescription(answer)
        except Exception as e:
            logger.error(f"WebRTC 협상 실패 (피어 {peer.peer_id}): {type(e).__name__}: {e}")
            self._remove_peer(peer)
            await peer_connection.close()
            raise
        return {"sdp": peer_connection.localDescription.sdp, "type": peer_connection.localDescription.type}

    def _on_datachannel(self, peer: _Peer, channel):
        """peer의 DataChannel 열림을 label별로 처리한다. 반환값 없음."""
        # state 채널: 초기 상태 전송 후 수신 처리자 등록
        if channel.label == "state":
            peer.state_channel = channel
            self._send_initial_state(channel)
            channel.on("message", functools.partial(self._handle_state_message, peer))

        # mesh 채널: 열림 콜백(스냅샷 전송) 호출
        elif channel.label == "mesh":
            peer.mesh_channel = channel
            self._notify_mesh_open(peer, channel)

        logger.info(
            f"데이터 채널 열림: {channel.label} (피어 {peer.peer_id}, 클라이언트 {len(self._peers)}명)"
        )

    async def _on_connection_state_change(self, peer: _Peer):
        """peer의 연결 상태 변화를 처리한다 (실패·종료 시 정리). 반환값 없음."""
        connection_state = peer.peer_connection.connectionState
        logger.info(f"피어 {peer.peer_id} 연결 상태: {connection_state}")

        # 실패한 연결은 목록에서 빼고 트랜스포트까지 닫는다
        if connection_state == "failed":
            self._remove_peer(peer)
            await peer.peer_connection.close()

        # 종료된 연결은 목록에서 뺀다
        elif connection_state == "closed":
            self._remove_peer(peer)

    def _remove_peer(self, peer: _Peer):
        """접속 종료된 피어를 목록에서 뺀다. 반환값 없음."""
        if peer not in self._peers:
            return
        self._peers.remove(peer)
        self._ignored_pose_peers.discard(peer.peer_id)

        # 포즈 소유 피어가 나가면 소유권을 비운다 (제어 대기 중 다음 샘플 송신 피어가 소유)
        if self._pose_owner is peer:
            self._pose_owner = None
            logger.warning(f"포즈 소유 피어 {peer.peer_id} 연결 종료 — 소유권 해제")
        logger.info(f"클라이언트 연결 종료 (남은 {len(self._peers)}명)")

    def _notify_mesh_open(self, peer: _Peer, channel):
        """mesh 채널 열림을 등록된 콜백(스냅샷 전송)에 알린다. 반환값 없음."""
        if self._on_mesh_open is None:
            return

        try:
            self._on_mesh_open(channel)
        except Exception as e:
            logger.error(f"메시 스냅샷 전송 실패 (피어 {peer.peer_id}): {type(e).__name__}: {e}")

    def _send_initial_state(self, channel):
        """신규 state 채널에 로봇 메시 구성·현재 모드 상태를 1회 보낸다. 반환값 없음."""
        if channel.readyState != "open":
            logger.warning(f"state 채널 미개방({channel.readyState}) — 초기 상태 전송 생략")
            return

        # 보관된 초기 상태 메시지를 순서대로 전송
        for initial_message in (self._robot_config, self._mode_status):
            if initial_message is None:
                continue
            try:
                channel.send(json.dumps(initial_message))
            except Exception as e:
                logger.warning(f"초기 상태 전송 실패: {type(e).__name__}: {e}")

    def _handle_state_message(self, peer: _Peer, data):
        """peer의 state 채널 수신 메시지를 처리한다 (포즈 샘플 또는 명령). 반환값 없음."""
        # JSON 파싱
        try:
            message = json.loads(data)
        except (json.JSONDecodeError, TypeError) as e:
            logger.warning(f"state 채널 메시지 파싱 오류: {e}")
            return

        # type 필드가 있으면 명령 메시지로 처리
        if isinstance(message, dict) and "type" in message:
            self._handle_command(message)
            return

        # 포즈 샘플은 JSON 객체만 받는다
        if not isinstance(message, dict):
            self._count_invalid_sample(f"피어 {peer.peer_id} 형식 오류: {type(message).__name__}")
            return
        command = sample_command(message)

        # 포즈 소유 피어가 아닌 샘플은 정지 계열 명령만 받고 IK 입력·중계에서 제외한다
        if not self._accept_pose_sender(peer):
            if command in _STOP_COMMANDS:
                self._latch_command(command, peer)
            return

        # 포즈 샘플을 로봇 좌표계로 변환 (형식 오류 샘플은 버림)
        try:
            robot_sample = webxr_sample_to_robot(message)
        except (KeyError, TypeError, ValueError, AttributeError, IndexError) as e:
            self._count_invalid_sample(f"피어 {peer.peer_id} 변환 실패: {type(e).__name__}: {e}")
            return

        # 변환 성공 샘플만 IK 입력으로 갱신
        self._latest_robot = robot_sample
        self._latest_rx_mono = time.monotonic()
        self._rx_count += 1

        # 샘플 명령 래치 후 다른 클라이언트로 원본 중계
        if command is not None:
            self._latch_command(command, peer)
        self._relay_pose(peer, data)

    def _count_invalid_sample(self, reason: str):
        """버린 샘플 수와 마지막 사유를 기록한다 (로그는 수신률 로그 주기에 모아 남김). 반환값 없음."""
        self._invalid_sample_count += 1
        self._last_invalid_reason = reason

    def _latch_command(self, command: str, sender: _Peer):
        """샘플 명령을 우선순위에 따라 래치한다 (정지 계열 대기 명령은 낮은 순위 명령에 덮이지 않음). 반환값 없음."""
        pending_command = self._pending_command

        # 대기 명령보다 우선순위가 낮은 새 명령은 버린다
        if pending_command is not None and _COMMAND_PRIORITY[command] < _COMMAND_PRIORITY[pending_command]:
            logger.warning(f"미처리 명령 {pending_command} 유지 — 피어 {sender.peer_id}의 명령 {command} 무시")
            return

        # 새 명령으로 래치 갱신
        if pending_command is not None:
            logger.warning(f"미처리 명령 {pending_command} 를 새 명령 {command} 로 대체")
        self._pending_command = command
        logger.info(f"샘플 명령 수신: {command} (피어 {sender.peer_id})")

    def _accept_pose_sender(self, peer: _Peer) -> bool:
        """peer의 포즈 샘플을 받아들일지 판단하고 필요하면 소유권을 넘긴다. 반환: 수락 여부."""
        if self._pose_owner is peer:
            return True

        # 로봇 제어 중에는 소유권 지정·이양을 하지 않는다 (경고는 피어당 1회)
        if self._is_armed():
            if peer.peer_id not in self._ignored_pose_peers:
                self._ignored_pose_peers.add(peer.peer_id)
                owner_text = f"피어 {self._pose_owner.peer_id}" if self._pose_owner is not None else "없음"
                logger.warning(
                    f"피어 {peer.peer_id} 포즈 샘플 무시 — 제어 중 소유권 변경 금지 (현재 소유 {owner_text})"
                )
            return False

        # 소유 피어가 없으면 첫 송신 피어가 소유
        if self._pose_owner is None:
            self._pose_owner = peer
            self._ignored_pose_peers.clear()
            logger.info(f"포즈 소유 피어 지정: 피어 {peer.peer_id}")
            return True

        # 대기 중 소유 피어 샘플이 stale이면 새 송신 피어로 소유권 이양
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
            self._send_state(peer, raw_text)

    def _handle_command(self, message: dict):
        """클라이언트 명령 메시지를 처리한다 (현재는 set_mode뿐). 반환값 없음."""
        # 지원 명령·처리자 등록 여부 확인
        if message.get("type") != "set_mode":
            logger.warning(f"알 수 없는 명령 무시: {message.get('type')!r}")
            return
        if self._on_set_mode is None:
            logger.warning("set_mode 처리자 미등록 — 무시")
            return

        mode = str(message.get("mode", ""))
        logger.warning(f"모드 전환 요청 수신: {mode}")

        # 처리 태스크는 완료까지 참조를 보관하고, 끝나면 예외를 로그로 남긴다
        mode_task = asyncio.create_task(self._on_set_mode(mode))
        self._mode_tasks.add(mode_task)
        mode_task.add_done_callback(self._on_mode_task_done)

    def _on_mode_task_done(self, mode_task: asyncio.Task):
        """set_mode 처리 태스크 완료 시 참조를 정리하고 예외를 로그로 남긴다. 반환값 없음."""
        self._mode_tasks.discard(mode_task)
        if mode_task.cancelled():
            logger.warning("모드 전환 처리 태스크 취소됨")
            return

        # 처리 중 발생한 예외를 traceback과 함께 기록
        error = mode_task.exception()
        if error is not None:
            logger.error(
                f"모드 전환 처리 중 예외: {type(error).__name__}: {error}",
                exc_info=(type(error), error, error.__traceback__),
            )

    def _send_state(self, peer: _Peer, text: str):
        """peer의 state 채널로 텍스트를 보낸다 (미개방·송신 대기 초과면 건너뜀). 반환값 없음."""
        state_channel = peer.state_channel
        if state_channel is None or state_channel.readyState != "open":
            return

        # 송신 대기 바이트가 상한을 넘은 피어는 이번 메시지를 건너뛴다 (상태 변화만 로그)
        congested = state_channel.bufferedAmount > config.RTC_STATE_BUFFER_LIMIT_BYTES
        if congested != peer.state_congested:
            peer.state_congested = congested
            if congested:
                logger.warning(
                    f"피어 {peer.peer_id} state 채널 송신 대기 {state_channel.bufferedAmount} B 초과 — 방송 건너뜀"
                )
            else:
                logger.info(f"피어 {peer.peer_id} state 채널 송신 대기 해소 — 방송 재개")
        if congested:
            return

        # 텍스트 송신
        try:
            state_channel.send(text)
        except Exception as e:
            logger.warning(f"피어 {peer.peer_id} state 송신 실패: {type(e).__name__}: {e}")

    async def broadcast_state(self, state: dict):
        """robot_state 등 서버 발신 JSON 메시지를 모든 클라이언트에 방송한다. 반환값 없음."""
        state_text = json.dumps(state)
        for peer in self._peers:
            self._send_state(peer, state_text)

    def broadcast_mesh(self, payload: bytes):
        """메시 블록 패킷을 모든 클라이언트에 방송한다 (드롭 없이 항상 보낸다). 반환값 없음."""
        for peer in self._peers:
            mesh_channel = peer.mesh_channel
            if mesh_channel is None or mesh_channel.readyState != "open":
                continue

            # 송신 대기 바이트가 경고 기준을 넘으면 상태 변화만 로그로 남긴다
            congested = mesh_channel.bufferedAmount > config.RTC_MESH_BUFFER_WARN_BYTES
            if congested != peer.mesh_congested:
                peer.mesh_congested = congested
                if congested:
                    logger.warning(f"피어 {peer.peer_id} mesh 채널 송신 대기 {mesh_channel.bufferedAmount} B 초과")
                else:
                    logger.info(f"피어 {peer.peer_id} mesh 채널 송신 대기 해소")

            # 패킷 송신
            try:
                mesh_channel.send(payload)
            except Exception as e:
                logger.warning(f"메시 블록 송신 실패: {type(e).__name__}: {e}")

    async def close_all(self):
        """전체 연결을 정리한다 (서버 종료 경로). 반환값 없음."""
        for peer in list(self._peers):
            await peer.peer_connection.close()
        self._peers.clear()
        logger.info("전체 WebRTC 연결 종료")

    async def _log_stats(self):
        """주기적으로 수신률·변환된 손목 위치·접속 클라이언트 수·버린 샘플 수를 로그로 남긴다."""
        while True:
            await asyncio.sleep(config.POSE_STATS_INTERVAL_S)
            rx_hz = self._rx_count / config.POSE_STATS_INTERVAL_S
            self._rx_count = 0

            # 최신 샘플의 오른손목 위치를 로그 문자열로 만든다 (형식 오류는 사유만 표시)
            wrist_info = "-"
            try:
                if self._latest_robot is not None:
                    right_hand = self._latest_robot["hands"].get("right")
                    if right_hand is not None:
                        wrist_pos = right_hand["pos"]
                        wrist_info = f"오른손목(로봇) [{wrist_pos[0]:.3f}, {wrist_pos[1]:.3f}, {wrist_pos[2]:.3f}]"
            except (KeyError, TypeError, ValueError, IndexError) as e:
                wrist_info = f"손목 정보 오류({type(e).__name__})"

            logger.info(
                f"포즈 수신 {rx_hz:.1f} Hz, 클라이언트 {len(self._peers)}명, {wrist_info}"
            )

            # 주기 동안 버린 샘플 수를 모아 경고
            if self._invalid_sample_count > 0:
                logger.warning(
                    f"포즈 샘플 {self._invalid_sample_count}개 버림 (최근 {config.POSE_STATS_INTERVAL_S:.0f}초, "
                    f"마지막 사유: {self._last_invalid_reason})"
                )
                self._invalid_sample_count = 0
