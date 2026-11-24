"""RGB-D 프레임 소스 추상 인터페이스: 캡처 구현(D455 등)과 소비자(perception)를 분리한다."""

import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Intrinsics:
    """핀홀 카메라 내부 파라미터 (픽셀 단위). 주점(ppx, ppy)과 초점거리(fx, fy).

    distortion_model·distortion_coeffs: 렌즈 왜곡 모델 이름과 계수 5개 (기본값은 왜곡 없음).
    """

    width: int
    height: int
    fx: float
    fy: float
    ppx: float
    ppy: float
    distortion_model: str = "none"
    distortion_coeffs: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class RGBDFrame:
    """color에 정렬된 RGB-D 프레임 1장.

    depth: (H, W) float32, 단위 m (0 = 무효). color: (H, W, 3) uint8 RGB.
    seq: 캡처 일련번호(소비자가 중복 처리를 피할 때 사용).
    t_mono: 프레임셋 수신 직후 시각(monotonic, 초). 실제 노출 시각보다 장치 내부·USB 전송 지연만큼 늦다.
    """

    depth: np.ndarray
    color: np.ndarray
    seq: int
    t_mono: float


class FrameSource(ABC):
    """RGB-D 프레임 공급자 인터페이스. 구현체는 백그라운드에서 캡처하고 최신 프레임만 보관한다.

    최신 프레임 보관(잠금·일련번호)과 latest()는 공통 구현을 제공한다.
    """

    def __init__(self):
        # 최신 프레임 보관 상태
        self._lock = threading.Lock()
        self._latest: RGBDFrame | None = None
        self._seq = 0

    @property
    @abstractmethod
    def intrinsics(self) -> Intrinsics:
        """color(=정렬 기준) 스트림의 내부 파라미터."""

    @abstractmethod
    def start(self):
        """캡처를 시작한다. 반환값 없음."""

    @abstractmethod
    def stop(self):
        """캡처를 정지하고 장치를 해제한다. 반환값 없음."""

    def latest(self) -> RGBDFrame | None:
        """가장 최근 프레임을 반환한다 (아직 없으면 None). 호출자는 반환 배열을 수정하지 않는다."""
        with self._lock:
            return self._latest

    def _publish_frame(self, depth: np.ndarray, color: np.ndarray, t_mono: float):
        """일련번호를 붙인 새 프레임으로 최신 프레임을 교체한다. 반환값 없음."""
        self._seq += 1
        frame = RGBDFrame(depth=depth, color=color, seq=self._seq, t_mono=t_mono)
        with self._lock:
            self._latest = frame
