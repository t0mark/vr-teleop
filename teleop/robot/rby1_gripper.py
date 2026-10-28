"""RBY1 그리퍼: 본체와 별도의 Dynamixel 버스(UPC 장치 /dev/rby1_gripper)로 양쪽 그리퍼를 구동한다.

custom-teleop의 gripper.py를 인터페이스(GripperBackend)에 맞춰 옮긴 것이다.
- 초기화: 포트 열기 → 모터 ping → 토크 on → **호밍**(약한 전류로 양끝까지 밀어 엔코더 범위 측정)
  호밍은 그리퍼가 실제로 열렸다 닫히는 동작이며 수 초 걸리므로 전용 스레드에서 한다
- 구동: 전류 제한 위치 제어 모드로 10 Hz마다 목표 엔코더를 쓰고 현재 엔코더를 읽는다
- 열림 정도 규약: 0=닫힘, 1=열림. 방향이 반대인 모터(config.GRIPPER_INVERT)는 엔코더 정규화값을
  뒤집어 쓰고 읽는다

버스 접근은 전부 그리퍼 스레드에서만 한다. 외부는 목표·측정값만 락으로 주고받는다.
"""

import logging
import threading
import time

import numpy as np

from teleop.robot.robot_interface import GripperBackend
from teleop.utils import config

logger = logging.getLogger("gripper")


class Rby1Gripper(GripperBackend):
    """Dynamixel 버스, 엔코더 범위, 목표·측정 열림 정도를 소유한다."""

    def __init__(self, device: str = config.GRIPPER_DEVICE):
        self._device = device
        self._bus = None
        self._ids = dict(config.GRIPPER_MOTOR_IDS)
        self._id_list = [self._ids[s] for s in ("right", "left")]
        self._min_q = np.full(2, np.inf)
        self._max_q = np.full(2, -np.inf)
        self._lock = threading.Lock()
        self._target: dict[str, float] = {"left": 1.0, "right": 1.0}
        self._measured: dict[str, float] | None = None
        self._ready = False
        self._running = False
        self._thread: threading.Thread | None = None

    @property
    def ready(self) -> bool:
        """호밍이 끝나 목표를 받을 수 있으면 True."""
        return self._ready

    # ── 외부 인터페이스 ──────────────────────────────────────────────────────

    def start(self):
        """그리퍼 스레드를 시작한다 (초기화·호밍·구동 루프 전부 스레드 안). 반환값 없음."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, name="gripper", daemon=True)
        self._thread.start()

    def stop(self):
        """구동 루프를 멈춘다. 반환값 없음 (토크는 유지 — 물건을 쥔 채 풀리지 않게)."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def set_openness(self, openness: dict[str, float]):
        """목표 열림 정도 {left/right: 0-1}를 설정한다. 반환값 없음 (준비 전이면 무시)."""
        if not self._ready:
            return
        with self._lock:
            for side in ("left", "right"):
                if side in openness:
                    self._target[side] = float(np.clip(openness[side], 0.0, 1.0))

    def openness(self) -> dict[str, float] | None:
        """엔코더로 측정한 열림 정도를 반환한다 (준비 전이면 None)."""
        with self._lock:
            return dict(self._measured) if self._measured is not None else None

    # ── 스레드 본체 ──────────────────────────────────────────────────────────

    def _run(self):
        """초기화 → 호밍 → 구동 루프. 어느 단계든 실패하면 경고 후 그리퍼를 비활성으로 둔다."""
        try:
            if not self._initialize():
                return
            self._homing()
        except Exception as e:
            logger.error(f"그리퍼 초기화 실패, 비활성: {e!r}")
            return

        self._ready = True
        logger.info(f"그리퍼 준비 완료, 엔코더 범위 {self._min_q} - {self._max_q}")
        try:
            self._loop()
        except Exception as e:
            logger.error(f"그리퍼 루프 종료: {e!r}")
        finally:
            self._ready = False

    def _initialize(self) -> bool:
        """버스를 열고 두 모터에 응답·토크를 확인한다. 반환: 성공 여부."""
        import rby1_sdk as rby

        # 툴 플랜지 전원 투입 직후에는 모터가 부팅 중이라 잠시 기다린다
        time.sleep(config.GRIPPER_POWER_SETTLE_S)
        self._bus = rby.DynamixelBus(self._device)
        self._bus.open_port()
        self._bus.set_baud_rate(config.GRIPPER_BAUDRATE)
        self._bus.set_torque_constant([1, 1])

        # 두 모터가 모두 응답해야 한다 — 하나만 살아 있으면 호밍에서 무한 대기한다
        # 전원 직후 모터가 늦게 뜰 수 있어 일정 시간 재시도한다
        deadline = time.monotonic() + config.GRIPPER_PING_TIMEOUT_S
        for side, dev_id in self._ids.items():
            while not self._bus.ping(dev_id):
                if time.monotonic() > deadline:
                    logger.warning(f"그리퍼 모터 {side}(ID {dev_id}) 응답 없음, 그리퍼 비활성")
                    return False
                time.sleep(0.5)
        self._bus.group_sync_write_torque_enable([(i, 1) for i in self._id_list])
        logger.info("그리퍼 모터 2개 응답, 토크 on")
        return True

    def _set_operating_mode(self, mode):
        """동작 모드를 바꾼다 (모드 변경은 토크 off 상태에서만 허용된다). 반환값 없음."""
        self._bus.group_sync_write_torque_enable([(i, 0) for i in self._id_list])
        self._bus.group_sync_write_operating_mode([(i, mode) for i in self._id_list])
        self._bus.group_sync_write_torque_enable([(i, 1) for i in self._id_list])

    def _homing(self):
        """전류 제어로 양끝까지 밀어 엔코더 최소·최대를 잰다. 반환값 없음 (실패 시 RuntimeError).

        엔코더 변화가 EPS 미만인 읽기가 STALL_COUNT회(0.1 s 간격) 이어지면 끝에 닿은 것으로 보고
        방향을 바꾼다. 엔코더를 한 번도 못 읽었거나 한 방향이 TIMEOUT을 넘기면 실패다.
        """
        import rby1_sdk as rby

        logger.info("그리퍼 호밍 시작 (양끝까지 열었다 닫는다)")
        self._set_operating_mode(rby.DynamixelBus.CurrentControlMode)
        q = np.full(2, np.nan)
        prev_q = np.full(2, np.nan)
        direction = 0
        stall = 0
        t_dir = time.monotonic()
        t_log = t_dir
        while direction < 2 and self._running:
            sign = 1.0 if direction == 0 else -1.0
            self._bus.group_sync_write_send_torque(
                [(i, config.GRIPPER_HOMING_CURRENT * sign) for i in self._id_list]
            )
            rv = self._bus.group_fast_sync_read_encoder(self._id_list)
            if rv is not None:
                for dev_id, enc in rv:
                    q[dev_id] = enc

            # 읽기가 한 번이라도 된 뒤부터 범위·정지 판정을 한다
            if np.isfinite(q).all():
                self._min_q = np.minimum(self._min_q, q)
                self._max_q = np.maximum(self._max_q, q)
                still = np.isfinite(prev_q).all() and np.all(np.abs(q - prev_q) < config.GRIPPER_HOMING_STALL_EPS)
                stall = stall + 1 if still else 0
                prev_q = q.copy()

            now = time.monotonic()
            if now - t_log >= 2.0:
                t_log = now
                logger.info(f"호밍 진행: 방향 {direction}, 엔코더 {np.round(q, 3)}, 정지 {stall}회")
            if stall >= config.GRIPPER_HOMING_STALL_COUNT:
                logger.info(f"호밍 방향 {direction} 끝 도달: {np.round(q, 3)}")
                direction += 1
                stall = 0
                t_dir = now
            if now - t_dir > config.GRIPPER_HOMING_TIMEOUT_S:
                self._bus.group_sync_write_send_torque([(i, 0.0) for i in self._id_list])
                raise RuntimeError(f"호밍 방향 {direction} 시간 초과 (엔코더 {np.round(q, 3)})")
            time.sleep(0.1)

        if not np.isfinite(self._min_q).all() or not np.isfinite(self._max_q).all():
            raise RuntimeError("호밍 중 엔코더를 읽지 못함")
        if np.any(self._max_q - self._min_q < 1e-6):
            raise RuntimeError(f"엔코더 범위가 0 (모터가 움직이지 않음): {self._min_q} - {self._max_q}")

    def _loop(self):
        """전류 제한 위치 제어로 목표를 쓰고 엔코더를 읽는 구동 루프."""
        import rby1_sdk as rby

        self._set_operating_mode(rby.DynamixelBus.CurrentBasedPositionControlMode)
        self._bus.group_sync_write_send_torque([(i, config.GRIPPER_HOLD_CURRENT) for i in self._id_list])
        period = 1.0 / config.GRIPPER_RATE_HZ
        while self._running:
            t0 = time.monotonic()
            with self._lock:
                target = dict(self._target)
            self._bus.group_sync_write_send_position(
                [(self._ids[side], self._to_encoder(side, target[side])) for side in ("right", "left")]
            )

            # 측정 열림 정도: 시각화·HUD가 실제 그리퍼 상태를 보게 한다
            rv = self._bus.group_fast_sync_read_encoder(self._id_list)
            if rv is not None:
                measured = {}
                for dev_id, enc in rv:
                    side = "right" if dev_id == self._ids["right"] else "left"
                    measured[side] = self._from_encoder(side, enc)
                with self._lock:
                    self._measured = measured
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def _to_encoder(self, side: str, openness: float) -> float:
        """열림 정도(0-1)를 해당 모터 엔코더 목표로 변환한다 (GRIPPER_INVERT면 방향 반전)."""
        dev_id = self._ids[side]
        ratio = 1.0 - openness if config.GRIPPER_INVERT[side] else openness
        return ratio * (self._max_q[dev_id] - self._min_q[dev_id]) + self._min_q[dev_id]

    def _from_encoder(self, side: str, enc: float) -> float:
        """엔코더값을 열림 정도(0-1)로 변환한다 (_to_encoder의 역)."""
        dev_id = self._ids[side]
        ratio = (enc - self._min_q[dev_id]) / (self._max_q[dev_id] - self._min_q[dev_id])
        ratio = float(np.clip(ratio, 0.0, 1.0))
        return 1.0 - ratio if config.GRIPPER_INVERT[side] else ratio
