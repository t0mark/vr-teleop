"""시뮬 로봇 백엔드: RobotBackend 계약을 하드웨어 없이 구현해 제어 경로 전체를 dev 모드에서 돌린다.

Rby1Backend 자리에 그대로 꽂히는 대체 부품이다. 상위 서비스(RobotController·RobotStateBroadcaster·
PointCloudStreamer)는 어느 쪽인지 모른다 — 이것이 dev/deploy 모드가 같은 코드로 도는 근거다.

명령 추종 모델: 매 send_targets에서 관절 상태를 1차 지연으로 목표에 접근시킨다.
    state += (target − state) · (1 − exp(−dt/τ)),  τ = config.SIM_TRACKING_TAU
지연이 있어야 명령-측정 오차가 실제로 생기고, 추적 오차 해제·속도 클램프 같은 안전 경로가
실기기 없이도 자극된다. 주행(SE2 속도)은 평면 적분으로 가상 베이스 포즈에 누적해 로그로 확인한다.

초기 자세는 전 관절 0 — 부팅 직후의 실로봇처럼 ready와 다른 자세에서 시작해,
제어 시작 시 "측정값에서 연속 출발 → ready 복귀 보간" 경로가 dev에서 그대로 관찰되게 한다.
"""

import logging
import math
import threading
import time

from teleop.robot.robot_interface import GripperBackend, RobotBackend
from teleop.robot.rby1_robot import COMMAND_JOINT_NAMES
from teleop.utils import config

logger = logging.getLogger("sim_robot")


class SimRobotBackend(RobotBackend):
    """시뮬 관절 상태·가상 베이스 포즈·스트림 상태를 소유한다."""

    def __init__(self, tau: float = config.SIM_TRACKING_TAU, gripper: GripperBackend | None = None):
        """tau: 명령 추종 1차 지연 시간 상수(초). gripper: 측정 열림 정도를 위임할 그리퍼 백엔드."""
        self._tau = tau
        self._gripper = gripper
        # send_targets(executor 스레드)와 joint_positions(이벤트 루프)가 교차 접근하므로 락으로 보호
        self._lock = threading.Lock()
        self._positions: dict[str, float] = {name: 0.0 for name in COMMAND_JOINT_NAMES}
        # 가상 베이스 포즈 (x m, y m, yaw rad): 주행 명령 적분 결과, 로그 확인용
        self._base = [0.0, 0.0, 0.0]
        self._connected = False
        self._streaming = False
        self._velocity_limits: dict[str, float] = {}
        self._sent = 0
        self._worst_err = 0.0
        self._stats_t = 0.0

    # ── 속성 ────────────────────────────────────────────────────────────────

    @property
    def connected(self) -> bool:
        """현재 연결 상태를 반환한다."""
        return self._connected

    @property
    def streaming(self) -> bool:
        """명령 스트림이 열려 있으면 True."""
        return self._streaming

    def set_velocity_limits(self, limits: dict[str, float]):
        """관절별 속도 상한을 보관한다 (실 SDK와 인터페이스를 맞추기 위한 것, 동역학에는 미사용)."""
        self._velocity_limits = dict(limits)

    # ── 연결·상태 ────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """시뮬 연결: 항상 성공한다. 반환: True."""
        self._connected = True
        self._stats_t = time.monotonic()
        logger.info(f"시뮬 로봇 연결 (dev 모드): 관절 {len(self._positions)}개, 추종 τ={self._tau:.2f}s, 초기 자세 0")
        return True

    def disconnect(self):
        """스트림을 닫고 연결 상태를 내린다. 반환값 없음."""
        self.stop_stream()
        self._connected = False
        logger.info("시뮬 로봇 연결 종료")

    def joint_positions(self) -> dict[str, float] | None:
        """시뮬 관절 상태 스냅샷 {관절명: rad}을 반환한다."""
        with self._lock:
            return dict(self._positions)

    def base_pose(self) -> tuple[float, float, float] | None:
        """주행 적분으로 누적된 가상 베이스 포즈 (x, y, yaw)를 반환한다 (주행 시각화용)."""
        with self._lock:
            return tuple(self._base)

    def gripper_openness(self) -> dict[str, float] | None:
        """그리퍼 측정 열림 정도를 반환한다 (그리퍼 백엔드가 없거나 준비 전이면 None).

        실로봇(Rby1Backend)과 같은 위임 구조 — 상태 서비스의 대체 공급자 경로까지 동일하게 자극된다.
        """
        if self._gripper is None:
            return None
        return self._gripper.openness()

    def power_tool_flange(self, voltage: int = config.GRIPPER_TOOL_VOLTAGE) -> bool:
        """툴 플랜지 전원 공급 흉내: 항상 성공한다. 반환: True (main.py의 그리퍼 기동 순서 유지용)."""
        logger.info(f"시뮬 툴 플랜지 {voltage}V 공급")
        return True

    # ── 제어 준비·스트림 ─────────────────────────────────────────────────────

    def prepare_control(self) -> bool:
        """전원·서보·Control Manager 준비 흉내. 반환: True (블로킹 시간만 재현).

        executor에서 불리는 블로킹 경로임을 유지하려고 짧게 대기한다 — 준비 중에 제어 루프가
        sync_to를 계속 도는 실기기 타이밍을 dev에서도 재현하기 위함.
        """
        time.sleep(0.2)
        logger.info("시뮬 제어 준비 완료 (전원·서보·CM 흉내)")
        return True

    def start_stream(self) -> bool:
        """명령 스트림 열기 흉내. 반환: True."""
        self._streaming = True
        logger.info("시뮬 명령 스트림 열림")
        return True

    def send_targets(self, named: dict[str, float], mobility: tuple[float, float], dt: float) -> bool:
        """한 주기 목표를 1차 지연으로 반영한다. 반환: 성공 여부 (스트림이 닫혀 있으면 False).

        named: 목표 관절각 {관절명: rad}, mobility: (전진 m/s, yaw rad/s), dt: 명령 주기(초).
        누락 관절은 실 SDK의 KeyError → False 규약과 맞춰 실패로 처리한다.
        """
        if not self._streaming:
            return False
        if any(name not in named for name in self._positions):
            logger.error("목표에 누락 관절이 있어 스트림을 닫음")
            self.stop_stream()
            return False

        # 1차 지연 계수: dt/τ가 커도 1을 넘지 않아 (목표를 지나치지 않아) 수치적으로 안전
        alpha = 1.0 - math.exp(-dt / self._tau)
        with self._lock:
            worst = 0.0
            for name in self._positions:
                err = named[name] - self._positions[name]
                worst = max(worst, abs(err))
                self._positions[name] += err * alpha
            # 주행 적분: 단순 오일러 (yaw 반영 전진) — 값 확인용이라 충분
            v, w = mobility
            self._base[0] += v * math.cos(self._base[2]) * dt
            self._base[1] += v * math.sin(self._base[2]) * dt
            self._base[2] += w * dt
            self._worst_err = max(self._worst_err, worst)
        self._sent += 1
        self._log_stats()
        return True

    def stop_stream(self):
        """명령 스트림을 닫는다 (열려 있지 않으면 무시). 반환값 없음."""
        if not self._streaming:
            return
        self._streaming = False
        logger.info("시뮬 명령 스트림 닫힘 — 현재 자세 유지")

    def _log_stats(self):
        """주기적으로 수신 명령률·최대 추종 오차·가상 베이스 포즈를 로그로 남긴다. 반환값 없음."""
        now = time.monotonic()
        if now - self._stats_t < config.POSE_STATS_INTERVAL_S:
            return
        dt = now - self._stats_t
        with self._lock:
            base = tuple(self._base)
            worst = self._worst_err
            self._worst_err = 0.0
        logger.info(
            f"시뮬 명령 수신 {self._sent / dt:.1f} Hz, 최대 추종 오차 {worst:.3f} rad, "
            f"베이스 x={base[0]:.2f} y={base[1]:.2f} yaw={math.degrees(base[2]):.0f}°"
        )
        self._sent = 0
        self._stats_t = now
