"""합성 RGB-D 소스: D455 없이 FrameSource 계약을 구현한다 (dev 모드용 D455Camera 대체).

고정 장면(벽 + 바닥)에 움직이는 구를 합성해 프레임마다 유효 깊이가 달라지는 프레임을 만든다.

좌표계: 광학 프레임 (x=우, y=하, z=전방, 단위 m). depth는 z-깊이(광축 방향 거리)로,
소비자(perception DepthProjector)의 역투영 p = z·((u−ppx)/fx, (v−ppy)/fy, 1) 규약과 일치한다.
내부 파라미터는 D455 640x480 근사값(fx=fy=385)을 해상도에 비례해 쓴다.
"""

import logging
import threading
import time

import numpy as np

from teleop.utils import config
from teleop.camera.camera_interface import FrameSource, Intrinsics

logger = logging.getLogger("synthetic_camera")

# 장면 기하: 벽 z-깊이, 바닥 높이(광학 중심 아래 m), 바닥으로 인정하는 광선 최소 ny
_WALL_DEPTH = 3.0
_FLOOR_DROP = 1.2
_FLOOR_MIN_RAY_NY = 1e-3

# 구 운동: 공전 반경, 반지름, 중심 깊이, 각속도(rad/s), 상하 요동 진폭·중심 오프셋(m)
_SPHERE_ORBIT = 0.45
_SPHERE_RADIUS = 0.25
_SPHERE_DEPTH = 1.5
_SPHERE_OMEGA = 0.5
_SPHERE_BOB_AMPLITUDE = 0.15
_SPHERE_BOB_OFFSET = 0.1

# 장면 색(RGB): 벽 체커보드 칸 크기(픽셀)·기본색·밝은 칸 증분, 바닥색, 구 색과 최소 음영 비율
_CHECKER_SIZE_PX = 40
_WALL_COLOR = (90, 100, 120)
_WALL_CHECKER_STEP = 60
_FLOOR_COLOR = (130, 95, 60)
_SPHERE_COLOR = (230, 120, 40)
_SPHERE_MIN_SHADE = 0.5

# 기준 내부 파라미터: 기준 해상도(가로, 세로)와 그 해상도에서의 초점거리(픽셀)
_BASE_WIDTH = 640
_BASE_HEIGHT = 480
_BASE_FOCAL_PX = 385.0

# 생성 스레드 정지 대기 한도(초)
_STOP_TIMEOUT_S = 2.0


class SyntheticCamera(FrameSource):
    """합성 장면 생성 스레드와 최신 프레임을 소유한다."""

    def __init__(self, width: int = config.CAMERA_WIDTH, height: int = config.CAMERA_HEIGHT,
                 fps: int = config.CAMERA_FPS):
        super().__init__()

        # 스트림 설정
        self._width = width
        self._height = height
        self._fps = fps

        # 기준 초점거리를 해상도 비율로 환산한 내부 파라미터
        focal_x = _BASE_FOCAL_PX * width / _BASE_WIDTH
        focal_y = _BASE_FOCAL_PX * height / _BASE_HEIGHT
        self._intrinsics = Intrinsics(width, height, fx=focal_x, fy=focal_y, ppx=width / 2.0, ppy=height / 2.0)

        # 생성 스레드와 실행 플래그
        self._thread: threading.Thread | None = None
        self._running = False

        # 프레임률 통계
        self._stats_count = 0
        self._stats_t = 0.0

        # 역투영용 정규화 좌표 격자와 배경(벽+바닥) 깊이·색을 미리 계산
        pixel_cols = np.arange(width, dtype=np.float32)
        pixel_rows = np.arange(height, dtype=np.float32)
        self._nx = (pixel_cols[None, :] - self._intrinsics.ppx) / self._intrinsics.fx
        self._ny = (pixel_rows[:, None] - self._intrinsics.ppy) / self._intrinsics.fy
        self._base_depth, self._base_color = self._build_background()

    def _build_background(self) -> tuple[np.ndarray, np.ndarray]:
        """벽·바닥의 깊이(z-깊이)와 색을 계산해 (depth, color)로 반환한다.

        바닥: 광학 중심에서 y=+_FLOOR_DROP 평면. 광선 (nx, ny, 1)·z가 평면에 닿는 z = drop/ny (ny>0).
        벽보다 가까운 픽셀만 바닥으로 채우고 나머지는 벽 깊이를 쓴다.
        """
        height, width = self._height, self._width
        depth = np.full((height, width), _WALL_DEPTH, dtype=np.float32)

        # 벽 색: 체커보드 무늬
        checker = (
            (np.arange(width)[None, :] // _CHECKER_SIZE_PX + np.arange(height)[:, None] // _CHECKER_SIZE_PX) % 2
        ).astype(np.uint8)
        color = np.empty((height, width, 3), dtype=np.uint8)
        for channel, wall_base in enumerate(_WALL_COLOR):
            color[..., channel] = wall_base + checker * _WALL_CHECKER_STEP

        # 바닥: 아래쪽 광선의 바닥 평면 교차 깊이가 벽보다 가까운 픽셀을 바닥으로 채운다
        ray_ny = np.broadcast_to(self._ny, (height, width))
        floor_valid = ray_ny > _FLOOR_MIN_RAY_NY
        floor_z = np.where(floor_valid, _FLOOR_DROP / np.where(floor_valid, ray_ny, 1.0), np.inf)
        is_floor = floor_z < depth
        depth[is_floor] = floor_z[is_floor].astype(np.float32)
        color[is_floor] = _FLOOR_COLOR
        return depth, color

    @property
    def intrinsics(self) -> Intrinsics:
        """합성 color 스트림의 내부 파라미터."""
        return self._intrinsics

    def start(self):
        """생성 스레드를 시작한다. 반환값 없음."""
        # 생성 스레드가 이미 돌고 있으면 중복 시작하지 않는다
        if self._thread is not None and self._thread.is_alive():
            logger.warning("합성 카메라 스레드가 이미 실행 중이라 start()를 무시")
            return

        # 생성 스레드 시작
        self._running = True
        self._stats_t = time.monotonic()
        self._thread = threading.Thread(target=self._render_loop, name="synthetic-camera", daemon=True)
        self._thread.start()
        logger.info(f"합성 카메라 시작 (dev 모드): {self._width}x{self._height}@{self._fps}, 벽 {_WALL_DEPTH}m + 공전 구")

    def stop(self):
        """생성 스레드를 멈춘다. 반환값 없음."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=_STOP_TIMEOUT_S)
            self._thread = None
        logger.info("합성 카메라 정지")

    def _render_loop(self):
        """고정 주기로 프레임을 합성해 최신 프레임을 교체한다 (스레드 본체)."""
        period = 1.0 / self._fps
        render_start_t = time.monotonic()
        while self._running:
            # 현재 시각의 프레임 합성 후 최신 프레임 교체
            frame_t = time.monotonic()
            depth, color = self._render(frame_t - render_start_t)
            self._publish_frame(depth, color, frame_t)
            self._log_stats()

            # 다음 주기까지 대기
            time.sleep(max(0.0, period - (time.monotonic() - frame_t)))

    def _render(self, elapsed_s: float) -> tuple[np.ndarray, np.ndarray]:
        """시작 후 elapsed_s(초) 시점의 프레임을 합성해 (depth, color)로 반환한다.

        구는 원궤도 중심을 이미지에 투영해 정규화 좌표 거리로 마스크를 만들고, 마스크 안은
        z = 중심 깊이 − 반지름·√(1 − (거리/각반경)²) 인 근사 돔 깊이로 채운다 (원근 왜곡 무시).
        """
        depth = self._base_depth.copy()
        color = self._base_color.copy()

        # 구 중심의 공전 (광학 프레임): x는 좌우 원운동, y는 두 배 주기의 상하 요동
        sphere_x = _SPHERE_ORBIT * np.cos(_SPHERE_OMEGA * elapsed_s)
        sphere_y = _SPHERE_BOB_AMPLITUDE * np.sin(2.0 * _SPHERE_OMEGA * elapsed_s) + _SPHERE_BOB_OFFSET
        sphere_z = _SPHERE_DEPTH

        # 정규화 좌표에서의 중심·각반경으로 픽셀 마스크 계산
        sphere_nx, sphere_ny = sphere_x / sphere_z, sphere_y / sphere_z
        angular_radius = _SPHERE_RADIUS / sphere_z
        dist_sq = (self._nx - sphere_nx) ** 2 + (self._ny - sphere_ny) ** 2
        mask = dist_sq < angular_radius * angular_radius

        # 마스크 안을 돔 깊이와 음영 색으로 채운다
        if mask.any():
            dome = np.sqrt(1.0 - dist_sq[mask] / (angular_radius * angular_radius)).astype(np.float32)
            depth[mask] = sphere_z - _SPHERE_RADIUS * dome
            shade = (_SPHERE_MIN_SHADE + (1.0 - _SPHERE_MIN_SHADE) * dome)[:, None]
            color[mask] = (np.array(_SPHERE_COLOR, dtype=np.float32)[None, :] * shade).astype(np.uint8)
        return depth, color

    def _log_stats(self):
        """주기적으로 합성 프레임률을 로그로 남긴다."""
        self._stats_count += 1
        now = time.monotonic()
        if now - self._stats_t >= config.CAMERA_STATS_INTERVAL_S:
            render_hz = self._stats_count / (now - self._stats_t)
            logger.info(f"합성 카메라 {render_hz:.1f} fps")
            self._stats_count = 0
            self._stats_t = now
