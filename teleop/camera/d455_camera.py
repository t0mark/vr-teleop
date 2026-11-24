"""RealSense D455 캡처: 장치를 열고 백그라운드 스레드에서 depth·color를 받아 최신 프레임을 보관한다.
장치가 끊기면(USB 재연결 등) 캡처 스레드가 파이프라인을 다시 열 때까지 재시도한다.

pyrealsense2 wait_for_frames는 블로킹이라 asyncio 루프 밖(스레드)에서 돌리고,
소비자(인식 서비스)는 latest()로 스냅샷만 가져간다. depth는 color에 정렬해 두므로
같은 픽셀 좌표로 색상·깊이를 함께 쓸 수 있다 (한 번 정렬해 모든 소비자가 공유).

depth 품질: 장치 프리셋(config.CAMERA_DEPTH_PRESET)으로 신뢰도 낮은 깊이를 거르고, 정렬된 depth에
RealSense 후처리(disparity 변환 → 공간 필터 → 시간 필터 → depth 복원)를 적용해 노이즈를 줄인다.
"""

import logging
import threading
import time

import numpy as np
import pyrealsense2 as rs

from teleop.utils import config
from teleop.camera.camera_interface import FrameSource, Intrinsics

logger = logging.getLogger("d455_camera")


class D455Camera(FrameSource):
    """D455 파이프라인·캡처 스레드·최신 프레임을 소유한다."""

    def __init__(self, width: int = config.CAMERA_WIDTH, height: int = config.CAMERA_HEIGHT,
                 fps: int = config.CAMERA_FPS):
        super().__init__()

        # 스트림 설정
        self._width = width
        self._height = height
        self._fps = fps

        # 파이프라인·정렬기와 장치에서 얻는 depth 스케일·내부 파라미터
        self._pipeline: rs.pipeline | None = None
        self._align = rs.align(rs.stream.color)
        self._depth_scale = 0.001
        self._intrinsics: Intrinsics | None = None

        # depth 후처리 필터 체인 (적용 순서대로)
        self._depth_filters = self._build_depth_filters()

        # 캡처 스레드와 정지 신호
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        # 프레임률·드롭 통계
        self._stats_count = 0
        self._stats_drop_count = 0
        self._stats_t = 0.0

    @property
    def intrinsics(self) -> Intrinsics:
        """color 스트림 내부 파라미터 (start() 이후 유효)."""
        if self._intrinsics is None:
            raise RuntimeError("카메라가 시작되지 않아 내부 파라미터가 없음")
        return self._intrinsics

    def start(self):
        """장치를 열고 캡처 스레드를 시작한다. 반환값 없음 (장치를 열 수 없으면 RuntimeError)."""
        # 캡처 스레드가 이미 돌고 있으면 중복 시작하지 않는다
        if self._thread is not None and self._thread.is_alive():
            logger.warning("D455 캡처 스레드가 이미 실행 중이라 start()를 무시")
            return

        self._open_pipeline()

        # 정지 신호 초기화 후 캡처 스레드 시작
        self._stop_event.clear()
        self._stats_t = time.monotonic()
        self._thread = threading.Thread(target=self._capture_loop, name="d455-capture", daemon=True)
        self._thread.start()

    def stop(self):
        """캡처 스레드에 정지를 알리고 종료(파이프라인 닫기 포함)를 기다린다. 반환값 없음."""
        self._stop_event.set()

        # 캡처 스레드 종료 대기, 시간 안에 끝나지 않으면 경고
        if self._thread is not None:
            join_timeout_s = config.CAMERA_FRAME_TIMEOUT_MS / 1000.0 + config.CAMERA_STOP_MARGIN_S
            self._thread.join(timeout=join_timeout_s)
            if self._thread.is_alive():
                logger.warning(f"D455 캡처 스레드가 {join_timeout_s:.1f}초 안에 종료되지 않음 (장치 해제 지연)")
                return
            self._thread = None
        logger.info("D455 정지")

    def _open_pipeline(self):
        """파이프라인을 열고 depth 스케일·내부 파라미터를 확보한다. 반환값 없음 (실패 시 RuntimeError)."""
        # depth(z16)·color(rgb8) 스트림을 같은 해상도로 열기
        stream_config = rs.config()
        stream_config.enable_stream(rs.stream.depth, self._width, self._height, rs.format.z16, self._fps)
        stream_config.enable_stream(rs.stream.color, self._width, self._height, rs.format.rgb8, self._fps)
        pipeline = rs.pipeline()
        profile = pipeline.start(stream_config)

        # depth 프리셋 적용 (실패하면 장치 기본 설정으로 진행)
        self._apply_depth_preset(profile.get_device().first_depth_sensor())

        # depth 스케일(원시값 → m), color 내부 파라미터, 시리얼 조회 (실패 시 파이프라인 정지)
        try:
            depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            color_intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            serial_number = profile.get_device().get_info(rs.camera_info.serial_number)
        except Exception as e:
            logger.warning(f"D455 파이프라인 시작 후 장치 정보 조회 실패, 파이프라인 정지: {e}")
            self._stop_pipeline(pipeline)
            raise

        # 조회한 값과 파이프라인 보관
        self._pipeline = pipeline
        self._depth_scale = depth_scale
        self._intrinsics = Intrinsics(
            color_intr.width, color_intr.height, color_intr.fx, color_intr.fy, color_intr.ppx, color_intr.ppy,
            distortion_model=str(color_intr.model), distortion_coeffs=tuple(color_intr.coeffs),
        )

        logger.info(
            f"D455 시작: {serial_number} {self._width}x{self._height}@{self._fps}, "
            f"fx={color_intr.fx:.1f} fy={color_intr.fy:.1f}, "
            f"왜곡 {self._intrinsics.distortion_model} {self._intrinsics.distortion_coeffs}"
        )

    @staticmethod
    def _build_depth_filters() -> list:
        """depth 후처리 필터 체인 [disparity 변환, 공간 필터, 시간 필터, depth 복원]을 만들어 반환한다."""
        # 공간 필터: 경계를 보존하며 이웃 화소로 평활
        spatial_filter = rs.spatial_filter()
        spatial_filter.set_option(rs.option.filter_smooth_alpha, config.CAMERA_SPATIAL_ALPHA)
        spatial_filter.set_option(rs.option.filter_smooth_delta, config.CAMERA_SPATIAL_DELTA)
        spatial_filter.set_option(rs.option.filter_magnitude, config.CAMERA_SPATIAL_MAGNITUDE)

        # 시간 필터: 경계를 보존하며 이전 프레임과 평활
        temporal_filter = rs.temporal_filter()
        temporal_filter.set_option(rs.option.filter_smooth_alpha, config.CAMERA_TEMPORAL_ALPHA)
        temporal_filter.set_option(rs.option.filter_smooth_delta, config.CAMERA_TEMPORAL_DELTA)

        # disparity 영역에서 평활 (먼 거리 노이즈를 거리 비례로 다룬다)
        return [rs.disparity_transform(True), spatial_filter, temporal_filter, rs.disparity_transform(False)]

    @staticmethod
    def _apply_depth_preset(depth_sensor):
        """depth 센서에 config.CAMERA_DEPTH_PRESET 프리셋을 적용한다 (실패하면 경고만). 반환값 없음."""
        try:
            preset = getattr(rs.rs400_visual_preset, config.CAMERA_DEPTH_PRESET)
            depth_sensor.set_option(rs.option.visual_preset, float(preset))
            logger.info(f"D455 depth 프리셋: {config.CAMERA_DEPTH_PRESET}")
        except (AttributeError, RuntimeError) as e:
            logger.warning(f"D455 depth 프리셋 {config.CAMERA_DEPTH_PRESET} 적용 실패, 장치 기본 설정 사용: {e}")

    def _close_pipeline(self):
        """보관 중인 파이프라인을 닫는다. 반환값 없음."""
        if self._pipeline is None:
            return
        self._stop_pipeline(self._pipeline)
        self._pipeline = None

    @staticmethod
    def _stop_pipeline(pipeline):
        """파이프라인을 정지한다 (이미 끊긴 장치의 정지 오류는 무시). 반환값 없음."""
        try:
            pipeline.stop()
        except RuntimeError as e:
            logger.debug(f"D455 파이프라인 정지 오류 무시: {e}")

    def _reconnect(self):
        """파이프라인을 닫고 다시 열릴 때까지 CAMERA_RECONNECT_S 간격으로 재시도한다 (정지 요청 시 중단). 반환값 없음.

        재시도 실패 로그는 끊김 1회당 한 번만 남긴다.
        """
        self._close_pipeline()

        # 정지 요청 전까지 주기적으로 재오픈 시도
        failures = 0
        while not self._stop_event.wait(config.CAMERA_RECONNECT_S):
            try:
                self._open_pipeline()
            except Exception as e:
                if failures == 0:
                    logger.warning(f"D455 재연결 실패, {config.CAMERA_RECONNECT_S:.0f}초 간격으로 계속 재시도: {e}")
                failures += 1
                continue

            # 재연결 성공: 통계 초기화
            logger.info(f"D455 재연결 완료 (재시도 {failures + 1}회)")
            self._stats_count = 0
            self._stats_drop_count = 0
            self._stats_t = time.monotonic()
            return

    def _capture_loop(self):
        """프레임 수신·처리를 반복하고, 종료 시 파이프라인을 닫는다 (스레드 본체)."""
        logger.info("D455 캡처 스레드 시작")
        try:
            while not self._stop_event.is_set():
                # 프레임 1장 처리, 실패(장치 끊김·타임아웃·처리 오류)면 파이프라인을 다시 연다
                try:
                    self._capture_frame()
                except Exception as e:
                    logger.warning(f"D455 프레임 처리 실패, 재연결 시작: {e}")
                    self._reconnect()
                    continue

                self._log_stats()
        except Exception:
            logger.exception("D455 캡처 스레드 비정상 종료")
        finally:
            # 캡처 스레드가 소유한 파이프라인을 닫는다
            self._close_pipeline()
            logger.info("D455 캡처 스레드 종료")

    def _capture_frame(self):
        """프레임셋을 받아 정렬·변환 후 최신 프레임을 교체한다. 반환값 없음 (수신·처리 실패 시 예외)."""
        frames = self._pipeline.wait_for_frames(timeout_ms=config.CAMERA_FRAME_TIMEOUT_MS)

        # 프레임셋 수신 직후 시각을 프레임 시각으로 기록
        received_t_mono = time.monotonic()

        # depth를 color 화소 격자에 정렬, 한쪽이 없는 프레임셋은 드롭 수로 집계하고 버린다
        aligned = self._align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            self._stats_drop_count += 1
            return

        # depth 후처리 필터 적용
        filtered_depth_frame = depth_frame
        for depth_filter in self._depth_filters:
            filtered_depth_frame = depth_filter.process(filtered_depth_frame)

        # SDK 버퍼에서 depth(m 단위 float32)·color 복사본 생성
        depth = np.asanyarray(filtered_depth_frame.get_data()).astype(np.float32) * self._depth_scale
        color = np.asanyarray(color_frame.get_data()).copy()

        # 최신 프레임 교체
        self._publish_frame(depth, color, received_t_mono)
        self._stats_count += 1

    def _log_stats(self):
        """주기적으로 캡처 프레임률과 불완전 프레임셋 드롭 수를 로그로 남긴다."""
        now = time.monotonic()
        if now - self._stats_t >= config.CAMERA_STATS_INTERVAL_S:
            capture_hz = self._stats_count / (now - self._stats_t)
            logger.info(f"D455 캡처 {capture_hz:.1f} fps, 불완전 프레임셋 드롭 {self._stats_drop_count}")
            self._stats_count = 0
            self._stats_drop_count = 0
            self._stats_t = now
