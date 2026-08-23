"""포즈 허브: VR 업링크 샘플을 수신·변환·보관하고, 서버 발신 메시지를 전체 클라이언트에 방송한다.

데이터 흐름:
- 업링크: /ws/pose 수신(WebXR 좌표) → 로봇 좌표 변환 → 최신 상태 보관(IK 입력)
          → /ws/monitor 구독자에게 원본 샘플 중계(데스크톱 포즈 마커용)
- 다운링크(텍스트): IkService가 broadcast_state()로 robot_state를 모든 클라이언트(VR 포함)에 방송
- 다운링크(바이너리): PointCloudService가 broadcast_bytes()로 포인트클라우드 패킷을 방송
- 신규 접속 시 robot_config(로봇 메시 구성) 1회 전송

송신 격리: 방송은 클라이언트별 큐에 넣기만 하고(논블로킹), 실제 전송은 클라이언트마다 전용 writer
태스크가 수행한다. 느린 클라이언트(슬립한 Quest, 백그라운드 탭 등)는 큐가 차면 가장 오래된 메시지를
버린다(최신 우선). 한 소켓의 배압이 IK·포인트클라우드 루프 전체를 멈추게 했던 장애의 재발 방지.
"""

import asyncio
import json
import logging
import time

from aiohttp import web

from teleop.utils import config
from teleop.utils.transforms import webxr_sample_to_robot

logger = logging.getLogger("pose_hub")

# 클라이언트별 송신 큐 깊이: robot_state(30Hz)·포인트클라우드(10Hz)가 섞여 들어오므로
# 수 프레임 분량만 보관하고 넘치면 오래된 것부터 버린다
_SEND_QUEUE_DEPTH = 8


class _Client:
    """접속 1개의 소켓·송신 큐·writer 태스크를 묶는다 (허브 내부용)."""

    def __init__(self, ws: web.WebSocketResponse, name: str):
        self.ws = ws
        self.name = name
        self.queue: asyncio.Queue = asyncio.Queue(maxsize=_SEND_QUEUE_DEPTH)
        self.dropped = 0
        self.writer: asyncio.Task = asyncio.create_task(self._write_loop())

    def enqueue(self, data):
        """메시지를 큐에 넣는다. 가득 차면 가장 오래된 것을 버려 최신을 보장한다. 반환값 없음."""
        if self.queue.full():
            self.queue.get_nowait()
            self.dropped += 1
        self.queue.put_nowait(data)

    async def _write_loop(self):
        """큐를 비우며 소켓에 쓴다. 소켓 오류 시 종료한다 (연결 정리는 핸들러가 담당)."""
        try:
            while True:
                data = await self.queue.get()
                if isinstance(data, (bytes, bytearray)):
                    await self.ws.send_bytes(data)
                else:
                    await self.ws.send_str(data)
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.warning(f"{self.name} 송신 실패, writer 종료: {type(e).__name__}: {e}")

    async def close(self):
        """writer를 취소한다. 반환값 없음."""
        self.writer.cancel()
        try:
            await self.writer
        except asyncio.CancelledError:
            pass


class PoseHub:
    """포즈 스트림의 수신 상태·구독자 집합을 관리한다 (단일 asyncio 루프 전제)."""

    def __init__(self):
        self._latest_raw: dict | None = None
        self._latest_robot: dict | None = None
        self._latest_rx_mono: float = 0.0
        self._monitors: set[_Client] = set()
        self._pose_clients: set[_Client] = set()
        self._rx_count = 0
        self._stats_task: asyncio.Task | None = None
        # IkService가 시작 시 채우는 로봇 메시 구성 (신규 접속마다 1회 전송)
        self.robot_config: dict | None = None

    @property
    def latest_robot(self) -> dict | None:
        """로봇 좌표계로 변환된 최신 샘플 (IK 입력)."""
        return self._latest_robot

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

    async def handle_pose_ws(self, request: web.Request) -> web.WebSocketResponse:
        """/ws/pose 핸들러: VR 클라이언트의 포즈 수신 + 로봇 상태 하향 전송 대상 등록."""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        client = _Client(ws, f"VR({request.remote})")
        self._pose_clients.add(client)
        logger.info(f"포즈 업링크 접속: {request.remote}")
        self._send_config(client)

        try:
            async for msg in ws:
                if msg.type != web.WSMsgType.TEXT:
                    continue
                try:
                    sample = json.loads(msg.data)
                except json.JSONDecodeError as e:
                    logger.warning(f"포즈 샘플 JSON 오류: {e}")
                    continue

                # 최신 샘플 갱신: 원본은 모니터 중계용, 변환본은 IK 입력용
                self._latest_raw = sample
                self._latest_robot = webxr_sample_to_robot(sample)
                self._latest_rx_mono = time.monotonic()
                self._rx_count += 1
                self._fanout(self._monitors, msg.data)
        finally:
            self._pose_clients.discard(client)
            await client.close()
            logger.info(f"포즈 업링크 종료: {request.remote} (드롭 {client.dropped})")
        return ws

    async def handle_monitor_ws(self, request: web.Request) -> web.WebSocketResponse:
        """/ws/monitor 핸들러: 데스크톱 뷰 구독을 등록하고 연결 종료까지 유지한다."""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        client = _Client(ws, f"모니터({request.remote})")
        self._monitors.add(client)
        logger.info(f"모니터 접속: {request.remote} (총 {len(self._monitors)}명)")
        self._send_config(client)

        try:
            # 모니터는 수신 전용이지만, 연결 유지를 위해 수신 루프를 돌린다
            async for _ in ws:
                pass
        finally:
            self._monitors.discard(client)
            await client.close()
            logger.info(f"모니터 종료: {request.remote} (총 {len(self._monitors)}명, 드롭 {client.dropped})")
        return ws

    async def broadcast_state(self, state: dict):
        """robot_state 등 서버 발신 JSON 메시지를 모든 클라이언트(VR+모니터)에 방송한다. 반환값 없음."""
        self._fanout(self._monitors | self._pose_clients, json.dumps(state))

    async def broadcast_bytes(self, payload: bytes):
        """바이너리 패킷(포인트클라우드 등)을 모든 클라이언트에 방송한다. 반환값 없음."""
        self._fanout(self._monitors | self._pose_clients, payload)

    def _send_config(self, client: _Client):
        """신규 접속에 로봇 메시 구성을 1회 큐잉한다. 반환값 없음."""
        if self.robot_config is not None:
            client.enqueue(json.dumps(self.robot_config))

    def _fanout(self, targets, data):
        """대상 클라이언트들의 큐에 메시지를 넣는다 (블로킹 없음). 반환값 없음."""
        for client in targets:
            client.enqueue(data)

    async def _log_stats(self):
        """주기적으로 수신률·변환된 손목 위치·송신 드롭 수를 로그로 남긴다 (파이프라인 상태 모니터링)."""
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

            # 드롭이 누적되는 클라이언트 = 수신이 느린 클라이언트 (Quest 슬립·백그라운드 탭 진단)
            dropped = sum(c.dropped for c in self._monitors | self._pose_clients)
            logger.info(
                f"포즈 수신 {hz:.1f} Hz, 모니터 {len(self._monitors)}명, VR {len(self._pose_clients)}명, "
                f"송신 드롭 누적 {dropped}, {wrist_info}"
            )
