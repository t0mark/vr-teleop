"""시뮬 손: HandBackend 계약을 하드웨어 없이 구현한다 (dev 모드용 다지 손 대체).

가짜 호밍 지연(config.SIM_HAND_HOMING_S) 뒤 ready가 되어, 상위의 "준비 전 상태" 처리
(fingers None → 대체 공급자 폴백, set_fingers 무시)까지 실기기와 같은 순서로 자극한다.
측정값은 목표를 1차 지연(config.SIM_HAND_TAU)으로 따라가 손가락이 서서히 움직이는 모습을 재현한다.
    measured += (target − measured) · (1 − exp(−dt/τ))   (측정값을 읽을 때 경과 시간만큼 진행)
"""

import logging
import math
import threading
import time

import numpy as np

from teleop.robot.robot_interface import HandBackend
from teleop.utils import config
from teleop.utils.inputs import FINGER_COUNT

logger = logging.getLogger("sim_hand")


class SimHand(HandBackend):
    """시뮬 손가락 목표·측정 상태와 가짜 호밍 스레드를 소유한다."""

    def __init__(self, homing_s: float = config.SIM_HAND_HOMING_S, tau: float = config.SIM_HAND_TAU):
        """homing_s: ready가 되기까지의 가짜 호밍 소요 시간(초). tau: 손가락 추종 1차 지연 시간 상수(초)."""
        self._homing_s = homing_s
        self._tau = tau
        self._ready = False

        # 손가락 목표·측정 굽힘과 마지막 갱신 시각 (락으로 보호)
        self._lock = threading.Lock()
        self._target = {side: np.zeros(FINGER_COUNT) for side in ("left", "right")}
        self._measured = {side: np.zeros(FINGER_COUNT) for side in ("left", "right")}
        self._update_t = time.monotonic()

        # 호밍 스레드와 정지 요청 이벤트
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    @property
    def ready(self) -> bool:
        """가짜 호밍이 끝나 목표를 받을 수 있으면 True."""
        return self._ready

    def start(self):
        """가짜 호밍을 백그라운드에서 시작한다. 반환값 없음."""
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._home, name="sim-hand-homing", daemon=True)
        self._thread.start()

    def stop(self):
        """호밍을 중단하고 구동을 멈춘다. 반환값 없음."""
        # 호밍 스레드 중단·종료 대기
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

        self._ready = False
        logger.info("시뮬 손 정지")

    def _home(self):
        """호밍 지연 후 ready로 전환한다 (스레드 본체, 지연 중 정지 요청이면 ready로 바꾸지 않고 종료)."""
        logger.info(f"시뮬 손 호밍 시작 ({self._homing_s:.1f}초)")
        if self._stop_event.wait(self._homing_s):
            logger.info("시뮬 손 호밍 중단 (정지 요청)")
            return

        # 측정 진행 기준 시각을 맞추고 ready로 전환
        with self._lock:
            self._update_t = time.monotonic()
        self._ready = True
        logger.info(f"시뮬 손 준비 완료 (추종 τ={self._tau:.2f}s)")

    def set_fingers(self, fingers: dict[str, list[float]]):
        """목표 손가락 굽힘 {left/right: [6] 0-1}를 설정한다 (준비 전이거나 형식이 틀리거나 유한하지 않으면 무시). 반환값 없음."""
        if not self._ready or fingers is None:
            return
        with self._lock:
            self._advance()
            for side in ("left", "right"):
                # 형식이 틀리거나 유한하지 않은 값은 무시하고 0-1로 잘라 목표로 둔다
                values = fingers.get(side)
                if values is None or len(values) != FINGER_COUNT:
                    continue
                curl = np.asarray(values, dtype=float)
                if not np.all(np.isfinite(curl)):
                    continue
                self._target[side] = np.clip(curl, 0.0, 1.0)

    def fingers(self) -> dict[str, list[float]] | None:
        """측정 손가락 굽힘 스냅샷을 반환한다 (준비 전이면 None)."""
        if not self._ready:
            return None
        with self._lock:
            self._advance()
            return {side: self._measured[side].tolist() for side in ("left", "right")}

    def _advance(self):
        """마지막 갱신 이후 경과 시간만큼 측정값을 목표 쪽으로 1차 지연 진행시킨다 (락 보유 상태에서 호출)."""
        # 경과 시간에 따른 1차 지연 계수
        now = time.monotonic()
        alpha = 1.0 - math.exp(-(now - self._update_t) / self._tau)
        self._update_t = now
        for side in ("left", "right"):
            self._measured[side] += (self._target[side] - self._measured[side]) * alpha
