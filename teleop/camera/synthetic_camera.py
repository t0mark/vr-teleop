"""합성 RGB-D 소스: D455 없이 FrameSource 계약을 구현한다 (dev 모드용 D455Camera 대체).

고정 장면(벽 + 바닥)에 움직이는 구를 합성해 프레임을 만든다 — 인식(메시 재구성) 파이프라인이
"프레임마다 달라지는 유효 깊이"를 처리하는 것을 눈으로 확인할 수 있게 정적인 장면을 피한다.

좌표계: 광학 프레임 (x=우, y=하, z=전방, 단위 m). depth는 z-깊이(광축 방향 거리)로,
소비자(perception DepthProjector)의 역투영 p = z·((u−ppx)/fx, (v−ppy)/fy, 1) 규약과 일치한다.
내부 파라미터는 D455 640x480 근사값(fx=fy=385)을 쓴다.
"""

import logging
import threading
import time

import numpy as np

from teleop.utils import config
from teleop.camera.camera_interface import FrameSource, Intrinsics, RGBDFrame

logger = logging.getLogger("synthetic_camera")

# 장면 파라미터: 벽 z-깊이, 바닥 높이(광학 중심 아래 m), 구 (공전 반경, 반지름, 중심 깊이, 각속도 rad/s)
_WALL_DEPTH = 3.0
_FLOOR_DROP = 1.2
_SPHERE_ORBIT = 0.45
_SPHERE_RADIUS = 0.25
_SPHERE_DEPTH = 1.5
_SPHERE_OMEGA = 0.5


class SyntheticCamera(FrameSource):
    """합성 장면 생성 스레드와 최신 프레임을 소유한다."""

    def __init__(self, width: int = config.CAMERA_WIDTH, height: int = config.CAMERA_HEIGHT,
                 fps: int = config.CAMERA_FPS):
        self._width = width
        self._height = height
        self._fps = fps
        self._intrinsics = Intrinsics(width, height, fx=385.0, fy=385.0, ppx=width / 2.0, ppy=height / 2.0)
        self._thread: threading.Thread | None = None
        self._running = False
        self._lock = threading.Lock()
        self._latest: RGBDFrame | None = None
        self._seq = 0
        self._stats_count = 0
        self._stats_t = 0.0

        # 역투영용 정규화 좌표 격자와 배경(벽+바닥) 깊이·색은 시간 불변이라 1회만 계산한다
        u = np.arange(width, dtype=np.float32)
        v = np.arange(height, dtype=np.float32)
        self._nx = (u[None, :] - self._intrinsics.ppx) / self._intrinsics.fx
        self._ny = (v[:, None] - self._intrinsics.ppy) / self._intrinsics.fy
        self._base_depth, self._base_color = self._build_background()

    def _build_background(self) -> tuple[np.ndarray, np.ndarray]:
        """벽·바닥의 깊이(z-깊이)와 색을 계산해 (depth, color)로 반환한다.

        바닥: 광학 중심에서 y=+_FLOOR_DROP 평면. 광선 (nx, ny, 1)·z가 평면에 닿는 z = drop/ny (ny>0).
        벽보다 가까운 픽셀만 바닥으로 채우고 나머지는 벽 깊이를 쓴다.
        """
        h, w = self._height, self._width
        depth = np.full((h, w), _WALL_DEPTH, dtype=np.float32)
        # 벽 색: 체커보드 — 깊이가 같아도 픽셀이 구분되어 메시 색 경로 확인이 쉽다
        checker = ((np.arange(w)[None, :] // 40 + np.arange(h)[:, None] // 40) % 2).astype(np.uint8)
        color = np.empty((h, w, 3), dtype=np.uint8)
        color[..., 0] = 90 + checker * 60
        color[..., 1] = 100 + checker * 60
        color[..., 2] = 120 + checker * 60

        # 바닥: ny가 0에 가까우면(수평선 근처) 발산하므로 아래쪽 광선만 유효
        ny = np.broadcast_to(self._ny, (h, w))
        floor_valid = ny > 1e-3
        floor_z = np.where(floor_valid, _FLOOR_DROP / np.where(floor_valid, ny, 1.0), np.inf)
        is_floor = floor_z < depth
        depth[is_floor] = floor_z[is_floor].astype(np.float32)
        color[is_floor] = (130, 95, 60)
        return depth, color

    @property
    def intrinsics(self) -> Intrinsics:
        """합성 color 스트림의 내부 파라미터."""
        return self._intrinsics

    def start(self):
        """생성 스레드를 시작한다. 반환값 없음."""
        self._running = True
        self._stats_t = time.monotonic()
        self._thread = threading.Thread(target=self._render_loop, name="synthetic-camera", daemon=True)
        self._thread.start()
        logger.info(f"합성 카메라 시작 (dev 모드): {self._width}x{self._height}@{self._fps}, 벽 {_WALL_DEPTH}m + 공전 구")

    def stop(self):
        """생성 스레드를 멈춘다. 반환값 없음."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        logger.info("합성 카메라 정지")

    def latest(self) -> RGBDFrame | None:
        """최신 합성 프레임 스냅샷을 반환한다 (없으면 None)."""
        with self._lock:
            return self._latest

    def _render_loop(self):
        """고정 주기로 프레임을 합성해 최신 프레임을 교체한다 (스레드 본체)."""
        period = 1.0 / self._fps
        t0 = time.monotonic()
        while self._running:
            tick = time.monotonic()
            depth, color = self._render(tick - t0)
            self._seq += 1
            frame = RGBDFrame(depth=depth, color=color, seq=self._seq, t_mono=tick)
            with self._lock:
                self._latest = frame
            self._log_stats()
            time.sleep(max(0.0, period - (time.monotonic() - tick)))

    def _render(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        """시각 t(초)의 프레임을 합성해 (depth, color)로 반환한다.

        구는 원궤도 중심을 이미지에 투영해 정규화 좌표 거리로 마스크를 만들고, 마스크 안은
        z = 중심 깊이 − 반지름·√(1 − (거리/각반경)²) 인 근사 돔 깊이로 채운다 (원근 왜곡은
        시각 확인 용도에 영향이 없어 무시한다).
        """
        depth = self._base_depth.copy()
        color = self._base_color.copy()

        # 구 중심의 공전 (광학 프레임): x는 좌우 원운동, y는 두 배 주기의 상하 요동
        cx = _SPHERE_ORBIT * np.cos(_SPHERE_OMEGA * t)
        cy = 0.15 * np.sin(2.0 * _SPHERE_OMEGA * t) + 0.1
        cz = _SPHERE_DEPTH

        # 정규화 좌표에서의 중심·각반경으로 픽셀 마스크 계산
        ncx, ncy = cx / cz, cy / cz
        ang_r = _SPHERE_RADIUS / cz
        d2 = (self._nx - ncx) ** 2 + (self._ny - ncy) ** 2
        mask = d2 < ang_r * ang_r
        if mask.any():
            dome = np.sqrt(1.0 - d2[mask] / (ang_r * ang_r)).astype(np.float32)
            depth[mask] = cz - _SPHERE_RADIUS * dome
            # 돔 값으로 음영을 줘 구가 입체로 보이게 한다
            shade = (0.5 + 0.5 * dome)[:, None]
            color[mask] = (np.array([230, 120, 40], dtype=np.float32)[None, :] * shade).astype(np.uint8)
        return depth, color

    def _log_stats(self):
        """주기적으로 합성 프레임률을 로그로 남긴다 (D455Camera와 동일한 모니터링 포맷)."""
        self._stats_count += 1
        now = time.monotonic()
        if now - self._stats_t >= config.CAMERA_STATS_INTERVAL_S:
            hz = self._stats_count / (now - self._stats_t)
            logger.info(f"합성 카메라 {hz:.1f} fps")
            self._stats_count = 0
            self._stats_t = now
