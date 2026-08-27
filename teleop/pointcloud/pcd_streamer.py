"""포인트클라우드 스트리머: 카메라 최신 프레임을 주기적으로 변환·패킹해 전 클라이언트에 방송한다.

데이터 흐름: FrameSource.latest() → PointCloudConverter → pack_pointcloud → WebSocketHub.broadcast_bytes

패킷 하나 = **그 프레임에서 본 것만**이다. 시간 누적을 하지 않으므로 로봇·머리가 움직여도 잔상이
남지 않고, 보이는 것이 곧 지금 카메라에 잡힌 표면이다 (대역폭도 누적 방식의 1/5 수준).
변환(numpy, 약 1 ms)은 executor에서 수행해 asyncio 루프를 막지 않는다.
같은 프레임(seq 동일)은 다시 보내지 않는다.

카메라 외부 파라미터는 머리·토르소 관절에 딸린 값이라 프레임마다 다시 계산한다. FK는 반드시
이벤트 루프 쪽(비동기 본체)에서 수행한다 — RobotModel의 pinocchio Data를 다른 서비스와 공유하므로
executor 스레드에서 부르면 경합이 난다.
"""

import asyncio
import logging
import time

import numpy as np

from teleop.camera.camera_interface import FrameSource
from teleop.pointcloud.depth_to_pcd import PointCloudConverter, pack_pointcloud
from teleop.utils import config

logger = logging.getLogger("pointcloud")


def _log_task_exception(task: asyncio.Task):
    """루프 태스크가 예외로 끝나면 로그에 남긴다 (조용한 정지 방지). 반환값 없음."""
    if task.cancelled() or task.exception() is None:
        return
    logger.error(f"포인트클라우드 루프 종료: {task.exception()!r}", exc_info=task.exception())


class PointCloudStreamer:
    """방송 루프 태스크와 처리 통계를 소유한다."""

    def __init__(self, camera: FrameSource, ws_hub, robot_model, joint_source,
                 rate_hz: float = config.PCD_RATE_HZ):
        """robot_model: 카메라 외부 파라미터(머리 장착 FK) 제공자.

        joint_source: 현재 관절값 공급자 (머리가 움직이면 외부 파라미터가 따라 바뀐다).
        """
        self._camera = camera
        self._hub = ws_hub
        self._rm = robot_model
        self._source = joint_source
        self._period = 1.0 / rate_hz
        self._converter: PointCloudConverter | None = None
        self._task: asyncio.Task | None = None
        self._last_seq = -1
        self._points = 0
        self._sent = 0
        self._bytes = 0
        self._proc_ms = 0.0
        self._stats_t = 0.0

    def start(self):
        """변환기를 구성하고 방송 루프를 시작한다 (카메라 start 이후 호출). 반환값 없음."""
        cam_pos, cam_rot = self._rm.camera_pose(self._source.joint_positions())
        self._converter = PointCloudConverter(self._camera.intrinsics, cam_pos, cam_rot)
        self._stats_t = time.monotonic()
        logger.info(f"카메라 외부 파라미터(base): pos {np.round(cam_pos, 3).tolist()}")
        self._task = asyncio.create_task(self._loop())
        self._task.add_done_callback(_log_task_exception)
        logger.info(
            f"포인트클라우드 스트리머 시작: {1.0 / self._period:.0f} Hz, "
            f"stride {config.PCD_STRIDE} (최대 {self._converter.max_points}점/프레임), 단일 프레임"
        )

    async def stop(self):
        """방송 루프를 정지한다. 반환값 없음."""
        if self._task is not None:
            self._task.cancel()
            self._task = None

    async def _loop(self):
        """고정 주기로 새 프레임만 변환·방송한다 (루프 본체)."""
        loop = asyncio.get_running_loop()
        while True:
            t0 = time.monotonic()
            frame = self._camera.latest()
            if frame is not None and frame.seq != self._last_seq:
                self._last_seq = frame.seq
                # 머리 자세가 바뀌면 광학 프레임 포즈도 바뀐다 (FK는 루프 스레드에서)
                self._converter.set_extrinsics(*self._rm.camera_pose(self._source.joint_positions()))
                # numpy 처리는 GIL을 놓으므로 executor로 넘겨 루프 지연을 피한다
                payload = await loop.run_in_executor(None, self._process, frame)
                await self._hub.broadcast_bytes(payload)
                self._sent += 1
                self._bytes += len(payload)

            self._log_stats()
            await asyncio.sleep(max(0.0, self._period - (time.monotonic() - t0)))

    def _process(self, frame) -> bytes:
        """프레임 → 점·색 → 패킷 바이트열 (executor에서 실행)."""
        t0 = time.perf_counter()
        pts, cols = self._converter.convert(frame)
        self._points = len(pts)
        payload = pack_pointcloud(pts, cols)
        self._proc_ms += (time.perf_counter() - t0) * 1e3
        return payload

    def _log_stats(self):
        """주기적으로 송신률·점 수·처리 시간을 로그로 남긴다."""
        now = time.monotonic()
        if now - self._stats_t < config.CAMERA_STATS_INTERVAL_S:
            return
        dt = now - self._stats_t
        n = max(self._sent, 1)
        logger.info(
            f"포인트클라우드 송신 {self._sent / dt:.1f} Hz, "
            f"{self._bytes / dt / 1e6:.2f} MB/s, 처리 {self._proc_ms / n:.1f} ms/프레임, "
            f"{self._points}점/프레임"
        )
        self._sent = 0
        self._bytes = 0
        self._proc_ms = 0.0
        self._stats_t = now
