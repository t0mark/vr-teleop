"""RealSense D455 캡처: 장치를 1회 열고 백그라운드 스레드에서 depth·color를 받아 최신 프레임을 보관한다.

pyrealsense2 wait_for_frames는 블로킹이라 asyncio 루프 밖(스레드)에서 돌리고,
소비자(포인트클라우드·영상 스트림)는 latest()로 스냅샷만 가져간다. depth는 color에 정렬해 두므로
같은 픽셀 좌표로 색상·깊이를 함께 쓸 수 있다 (한 번 정렬해 모든 소비자가 공유).
"""

import logging
import threading
import time

import numpy as np
import pyrealsense2 as rs

from teleop.utils import config
from teleop.camera.camera_interface import FrameSource, Intrinsics, RGBDFrame

logger = logging.getLogger("camera")


class D455Camera(FrameSource):
    """D455 파이프라인·캡처 스레드·최신 프레임을 소유한다."""

    def __init__(self, width: int = config.CAMERA_WIDTH, height: int = config.CAMERA_HEIGHT,
                 fps: int = config.CAMERA_FPS):
        self._width = width
        self._height = height
        self._fps = fps
        self._pipeline: rs.pipeline | None = None
        self._align = rs.align(rs.stream.color)
        self._depth_scale = 0.001
        self._intrinsics: Intrinsics | None = None
        self._thread: threading.Thread | None = None
        self._running = False
        self._lock = threading.Lock()
        self._latest: RGBDFrame | None = None
        self._seq = 0
        self._stats_count = 0
        self._stats_t = 0.0

    @property
    def intrinsics(self) -> Intrinsics:
        """color 스트림 내부 파라미터 (start() 이후 유효)."""
        if self._intrinsics is None:
            raise RuntimeError("카메라가 시작되지 않아 내부 파라미터가 없음")
        return self._intrinsics

    def start(self):
        """장치를 열고 캡처 스레드를 시작한다. 반환값 없음."""
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, self._width, self._height, rs.format.z16, self._fps)
        cfg.enable_stream(rs.stream.color, self._width, self._height, rs.format.rgb8, self._fps)
        self._pipeline = rs.pipeline()
        profile = self._pipeline.start(cfg)

        # depth 원시값(uint16)을 m로 바꾸는 스케일과, 정렬 기준(color) 내부 파라미터를 1회 확보
        self._depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self._intrinsics = Intrinsics(intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)

        dev = profile.get_device()
        logger.info(
            f"D455 시작: {dev.get_info(rs.camera_info.serial_number)} "
            f"{self._width}x{self._height}@{self._fps}, fx={intr.fx:.1f} fy={intr.fy:.1f}"
        )

        self._running = True
        self._stats_t = time.monotonic()
        self._thread = threading.Thread(target=self._capture_loop, name="d455-capture", daemon=True)
        self._thread.start()

    def stop(self):
        """캡처 스레드를 멈추고 파이프라인을 닫는다. 반환값 없음."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None
        logger.info("D455 정지")

    def latest(self) -> RGBDFrame | None:
        """최신 정렬 프레임 스냅샷을 반환한다 (없으면 None)."""
        with self._lock:
            return self._latest

    def _capture_loop(self):
        """프레임을 받아 정렬·변환 후 최신 프레임을 교체한다 (스레드 본체)."""
        while self._running:
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=2000)
            except RuntimeError as e:
                logger.warning(f"프레임 대기 실패: {e}")
                continue

            # depth를 color 화소 격자에 정렬 — 소비자들이 같은 (u, v)로 색·깊이를 함께 쓰기 위함
            aligned = self._align.process(frames)
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # SDK 버퍼는 다음 프레임에 재사용되므로 복사본을 만들어 소비자가 안전하게 참조하게 한다
            depth = np.asanyarray(depth_frame.get_data()).astype(np.float32) * self._depth_scale
            color = np.asanyarray(color_frame.get_data()).copy()

            self._seq += 1
            frame = RGBDFrame(depth=depth, color=color, seq=self._seq, t_mono=time.monotonic())
            with self._lock:
                self._latest = frame
            self._log_stats()

    def _log_stats(self):
        """주기적으로 캡처 프레임률을 로그로 남긴다 (USB 대역·드롭 감시용)."""
        self._stats_count += 1
        now = time.monotonic()
        if now - self._stats_t >= config.CAMERA_STATS_INTERVAL_S:
            hz = self._stats_count / (now - self._stats_t)
            logger.info(f"D455 캡처 {hz:.1f} fps")
            self._stats_count = 0
            self._stats_t = now
