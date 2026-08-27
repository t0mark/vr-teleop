"""시뮬 그리퍼: GripperBackend 계약을 하드웨어 없이 구현한다 (dev 모드용 Rby1Gripper 대체).

가짜 호밍 지연(config.SIM_GRIPPER_HOMING_S) 뒤 ready가 되어, 상위의 "준비 전 상태" 처리
(openness None → 대체 공급자 폴백, set_openness 무시)까지 실기기와 같은 순서로 자극한다.
버스 동역학은 검증 대상이 아니므로 목표를 즉시 측정값으로 반영한다.
"""

import logging
import threading
import time

import numpy as np

from teleop.robot.robot_interface import GripperBackend
from teleop.utils import config

logger = logging.getLogger("sim_gripper")


class SimGripper(GripperBackend):
    """시뮬 열림 상태와 가짜 호밍 스레드를 소유한다."""

    def __init__(self, homing_s: float = config.SIM_GRIPPER_HOMING_S):
        """homing_s: ready가 되기까지의 가짜 호밍 소요 시간(초)."""
        self._homing_s = homing_s
        self._ready = False
        # set_openness(제어 루프)와 openness(상태 서비스)가 교차 접근하므로 락으로 보호
        self._lock = threading.Lock()
        self._openness = {"left": 1.0, "right": 1.0}
        self._thread: threading.Thread | None = None

    @property
    def ready(self) -> bool:
        """가짜 호밍이 끝나 목표를 받을 수 있으면 True."""
        return self._ready

    def start(self):
        """가짜 호밍을 백그라운드에서 시작한다. 반환값 없음."""
        self._thread = threading.Thread(target=self._home, name="sim-gripper-homing", daemon=True)
        self._thread.start()

    def stop(self):
        """구동을 멈춘다. 반환값 없음."""
        self._ready = False
        logger.info("시뮬 그리퍼 정지")

    def _home(self):
        """호밍 지연 후 ready로 전환한다 (스레드 본체)."""
        logger.info(f"시뮬 그리퍼 호밍 시작 ({self._homing_s:.1f}초)")
        time.sleep(self._homing_s)
        self._ready = True
        logger.info("시뮬 그리퍼 준비 완료")

    def set_openness(self, openness: dict[str, float]):
        """목표 열림 정도 {left/right: 0-1}를 측정값에 즉시 반영한다 (준비 전이면 무시). 반환값 없음."""
        if not self._ready or openness is None:
            return
        with self._lock:
            for side in ("left", "right"):
                if side in openness:
                    self._openness[side] = float(np.clip(openness[side], 0.0, 1.0))

    def openness(self) -> dict[str, float] | None:
        """측정 열림 정도 스냅샷을 반환한다 (준비 전이면 None)."""
        if not self._ready:
            return None
        with self._lock:
            return dict(self._openness)
