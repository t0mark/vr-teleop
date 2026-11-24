"""RGB-D 프레임 소스 추상 인터페이스: 캡처 구현(D455 등)과 소비자(perception)를 분리한다."""

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Intrinsics:
    """핀홀 카메라 내부 파라미터 (픽셀 단위). 주점(ppx, ppy)과 초점거리(fx, fy)."""

    width: int
    height: int
    fx: float
    fy: float
    ppx: float
    ppy: float


@dataclass(frozen=True)
class RGBDFrame:
    """color에 정렬된 RGB-D 프레임 1장.

    depth: (H, W) float32, 단위 m (0 = 무효). color: (H, W, 3) uint8 RGB.
    seq: 캡처 일련번호(소비자가 중복 처리를 피할 때 사용), t_mono: 수신 시각(monotonic, 초).
    """

    depth: np.ndarray
    color: np.ndarray
    seq: int
    t_mono: float


class FrameSource(ABC):
    """RGB-D 프레임 공급자 인터페이스. 구현체는 백그라운드에서 캡처하고 최신 프레임만 보관한다."""

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

    @abstractmethod
    def latest(self) -> RGBDFrame | None:
        """가장 최근 프레임을 반환한다 (아직 없으면 None). 호출자는 반환 배열을 수정하지 않는다."""
