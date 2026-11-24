"""RBY1 백엔드: rby1-sdk gRPC로 상태를 구독하고 관절 목표를 스트리밍한다.

연결(connect)은 읽기 전용이다 — 전원이 꺼져 있을 때만 켜고(인코더 값을 읽으려면 필요) 서보·제어
관리자는 건드리지 않는다. 로봇을 움직이는 준비(서보 투입, Control Manager 활성화)는 조작자가 제어
시작을 명시했을 때 prepare_control()에서만 한다.

명령은 관절 임피던스 스트림이다 (custom-teleop의 Cartesian 임피던스 대신 IK가 낸 관절각을 목표로 보냄):
    body = torso(6) + right_arm(7) + left_arm(7) 각각 JointImpedanceControlCommand,
    head(2) JointPositionCommand.
    임피던스라 접촉하면 stiffness만큼 밀리고 토크 제한에서 포화된다 (위치 명령은 끝까지 위치를 고집한다).
    중력 보상은 SDK 제어기가 피드포워드로 더하므로 낮은 stiffness에서도 팔이 처지지 않는다.
    관절 순서는 SDK 구성요소 순서(torso_0.., right_arm_0.., left_arm_0.., head_0..)로
    이름에서 조립하므로 상위가 주는 사전의 순서에 의존하지 않는다.

상태 콜백은 SDK 내부 스레드에서 호출되므로, 최신 값 한 벌만 락으로 보호해 보관하고
소비자(상태 서비스)는 스냅샷만 가져간다.
"""

import logging
import threading
import time

import numpy as np

from teleop.robot.robot_interface import HandBackend, RobotBackend
from teleop.utils import config

logger = logging.getLogger("rby1")

# 명령 구성요소별 관절명 (SDK 모델 A의 DOF 순서와 일치)
_TORSO = [f"torso_{i}" for i in range(6)]
_RIGHT_ARM = [f"right_arm_{i}" for i in range(7)]
_LEFT_ARM = [f"left_arm_{i}" for i in range(7)]
_HEAD = [f"head_{i}" for i in range(2)]
COMMAND_JOINT_NAMES = _TORSO + _RIGHT_ARM + _LEFT_ARM + _HEAD


class Rby1Backend(RobotBackend):
    """rby1-sdk 연결, 최신 관절 측정값, 명령 스트림을 소유한다."""

    def __init__(self, address: str = config.ROBOT_ADDRESS, rate_hz: float = config.ROBOT_STATE_RATE_HZ,
                 hand: HandBackend | None = None):
        """hand: 측정 손가락 굽힘을 대신 제공할 손 백엔드 (본체 gRPC에는 손 값이 없다)."""
        self._address = address
        self._rate_hz = rate_hz
        self._hand = hand
        self._rby = None
        self._robot = None
        self._stream = None
        self._joint_names: list[str] = []
        self._lock = threading.Lock()
        self._positions: dict[str, float] | None = None
        # 마지막 상태 수신 시각(monotonic, 수신 전 0.0)과 두절 로그 중복 방지 플래그
        self._state_rx_mono = 0.0
        self._state_stale_logged = False
        self._connected = False
        self._velocity_limits: dict[str, float] = {}
        # 머리 PID 게인은 연결당 1회만 적용한다 (적용에 CM 비활성이 필요해 반복하면 제어가 끊긴다)
        self._head_gains_applied = False

    # ── 속성 ────────────────────────────────────────────────────────────────

    @property
    def connected(self) -> bool:
        """현재 연결 상태를 반환한다."""
        return self._connected

    @property
    def streaming(self) -> bool:
        """명령 스트림이 열려 있으면 True."""
        return self._stream is not None

    @property
    def joint_names(self) -> list[str]:
        """로봇이 보고하는 관절 이름 순서를 반환한다 (연결 전에는 빈 목록)."""
        return list(self._joint_names)

    def set_velocity_limits(self, limits: dict[str, float]):
        """명령에 실을 관절별 속도 상한 {관절명: rad/s}을 설정한다. 반환값 없음.

        SDK 쪽 안전 상한이다. 없는 관절은 SDK가 URDF 한계를 쓴다.
        """
        self._velocity_limits = dict(limits)

    # ── 연결·상태 ────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """로봇에 연결하고 상태 구독을 시작한다. 반환: 성공 여부.

        연결 실패는 예외가 아니라 False로 알린다 — 로봇이 없어도 서버는 시뮬레이션으로 떠야 한다.
        """
        # SDK가 없거나 로봇이 꺼진 환경에서도 서버가 기동해야 하므로 import·연결 실패를 삼킨다
        try:
            import rby1_sdk as rby
        except ImportError as e:
            logger.warning(f"rby1-sdk를 불러올 수 없음, 실로봇 비활성: {e}")
            return False

        try:
            robot = rby.create_robot_a(self._address)
            if not robot.connect():
                logger.warning(f"로봇 연결 실패: {self._address}")
                return False
        except Exception as e:
            logger.warning(f"로봇 연결 예외: {e!r}")
            return False

        self._rby = rby
        self._robot = robot
        self._joint_names = list(robot.model().robot_joint_names)
        self._ensure_power()

        # 상태 구독: 콜백이 SDK 스레드에서 호출된다
        robot.start_state_update(self._on_state, self._rate_hz)
        self._connected = True
        logger.info(
            f"로봇 연결: {self._address}, 관절 {len(self._joint_names)}개, "
            f"상태 구독 {self._rate_hz:.0f} Hz"
        )
        return True

    def _ensure_power(self) -> bool:
        """전원이 꺼져 있으면 켠다. 반환: 전원이 켜져 있는지.

        전원이 꺼진 상태에서는 인코더 값이 0으로만 올라와 시각화가 무의미하다.
        """
        try:
            if self._robot.is_power_on(".*"):
                return True
            logger.warning("로봇 전원이 꺼져 있어 투입한다")
            if not self._robot.power_on(".*"):
                logger.error("전원 투입 실패")
                return False
            return True
        except Exception as e:
            logger.warning(f"전원 상태 확인 실패: {e!r}")
            return False

    def _on_state(self, state):
        """상태 구독 콜백(SDK 스레드): 최신 관절값을 이름 사전으로 바꿔 보관한다. 반환값 없음."""
        try:
            positions = {name: float(v) for name, v in zip(self._joint_names, state.position)}
        except Exception as e:
            logger.warning(f"상태 파싱 실패: {e!r}")
            return
        with self._lock:
            self._positions = positions
            self._state_rx_mono = time.monotonic()

    def joint_positions(self) -> dict[str, float] | None:
        """최신 관절값 스냅샷을 반환한다 (수신 전이거나 ROBOT_STATE_STALE_S 넘게 갱신이 없으면 None)."""
        with self._lock:
            if self._positions is None:
                return None
            age = time.monotonic() - self._state_rx_mono
            stale = age > config.ROBOT_STATE_STALE_S

            # 두절·복구 전이를 1회씩 로그로 남긴다
            if stale and not self._state_stale_logged:
                logger.warning(f"로봇 상태 수신 두절: {age:.2f}초 경과 — 측정값 무효")
                self._state_stale_logged = True
            elif not stale and self._state_stale_logged:
                logger.info("로봇 상태 수신 복구")
                self._state_stale_logged = False

            return None if stale else dict(self._positions)

    def hand_fingers(self) -> dict[str, list[float]] | None:
        """손가락 측정 굽힘을 반환한다 (손 백엔드가 없거나 준비 전이면 None).

        손은 본체 gRPC가 아니라 별도 버스에 있어 손 백엔드에 위임한다.
        """
        if self._hand is None:
            return None
        return self._hand.fingers()

    # ── 제어 준비 ────────────────────────────────────────────────────────────

    def prepare_control(self) -> bool:
        """전원·서보·Control Manager를 명령 수신 가능 상태로 만든다. 반환: 성공 여부 (블로킹).

        순서는 custom-teleop의 connect_rby1과 같다: 전원 → 서보 → CM 폴트 리셋 → CM 활성화.
        이미 준비된 단계는 건너뛴다 (반복 호출해도 로봇이 흔들리지 않는다).
        """
        if self._robot is None:
            return False
        rby = self._rby
        robot = self._robot
        try:
            if not self._ensure_power():
                return False

            # 서보 투입: 로봇이 현재 자세를 물리적으로 붙잡기 시작한다
            if robot.is_servo_on(".*"):
                logger.info("서보 이미 켜짐")
            else:
                logger.warning("서보 투입")
                if not robot.servo_on(".*"):
                    logger.error("서보 투입 실패")
                    return False

            # 폴트 상태면 리셋해야 활성화가 된다
            cm = robot.get_control_manager_state()
            faulted = cm.state in (rby.ControlManagerState.State.MajorFault,
                                   rby.ControlManagerState.State.MinorFault)
            if faulted:
                logger.warning(f"Control Manager 폴트({cm.state}) 리셋")
                if not robot.reset_fault_control_manager():
                    logger.error("Control Manager 폴트 리셋 실패")
                    return False

            # 머리 PID 게인은 CM이 꺼진 상태에서만 적용된다 (custom-teleop과 동일 순서, 연결당 1회)
            if not self._head_gains_applied:
                if not self._apply_head_gains():
                    return False
                self._head_gains_applied = True

            # 활성화: 이후부터 명령이 관절로 전달된다
            if robot.get_control_manager_state().state == rby.ControlManagerState.State.Enabled:
                logger.info("Control Manager 이미 활성")
            else:
                if not robot.enable_control_manager(unlimited_mode_enabled=config.CM_UNLIMITED_MODE):
                    logger.error("Control Manager 활성화 실패")
                    self._log_robot_reason()
                    return False
                logger.info(f"Control Manager 활성화 (unlimited: {config.CM_UNLIMITED_MODE})")
            return True
        except Exception as e:
            logger.error(f"제어 준비 예외: {e!r}")
            return False

    def _apply_head_gains(self) -> bool:
        """머리 관절 위치 PID 게인을 적용한다. 반환: 성공 여부.

        게인 적용은 CM이 켜져 있으면 반영되지 않으므로 먼저 끈다 (custom-teleop connect_rby1과 동일).
        큰 D(8000)는 머리 팬틸트의 떨림 감쇠용이다.
        """
        rby = self._rby
        robot = self._robot
        # CM이 켜져 있으면 끄고, Idle로 내려올 때까지 기다린다
        if robot.get_control_manager_state().state == rby.ControlManagerState.State.Enabled:
            logger.info("머리 PID 게인 적용을 위해 Control Manager 일시 비활성")
            robot.disable_control_manager()
            deadline = time.monotonic() + 5.0
            while robot.get_control_manager_state().state == rby.ControlManagerState.State.Enabled:
                if time.monotonic() > deadline:
                    logger.error("Control Manager 비활성 대기 시간 초과")
                    return False
                time.sleep(0.1)
        for joint, (p, i, d) in config.HEAD_PID_GAINS.items():
            if not robot.set_position_pid_gain(joint, p, i, d):
                logger.error(f"머리 PID 게인 적용 실패: {joint}")
                return False
        logger.info(f"머리 PID 게인 적용: {config.HEAD_PID_GAINS}")
        return True

    def _log_robot_reason(self):
        """로봇 측 최근 로그에서 Control Manager 관련 줄을 서버 로그로 옮긴다. 반환값 없음.

        SDK는 실패를 bool로만 알려 주고 이유는 로봇 로그에만 남는다 (예: 관절이 펌웨어 한계 밖이라 활성화 거부 —
        head_1이 위를 본 채로는 CM이 켜지지 않는다). 사유를 바로 보여야 현장에서 조치할 수 있다.
        """
        try:
            for entry in self._robot.get_last_log(30):
                message = getattr(entry, "message", str(entry))
                if "ControlManager" in message and "Loop Stat" not in message:
                    logger.error(f"로봇 로그: {message.strip()}")
        except Exception as e:
            logger.warning(f"로봇 로그 조회 실패: {e!r}")

    # ── 명령 스트림 ──────────────────────────────────────────────────────────

    def start_stream(self) -> bool:
        """연속 명령 스트림을 연다. 반환: 성공 여부 (블로킹, 제어 준비 대기 포함).

        다른 제어가 돌고 있으면 취소하고 Idle이 될 때까지 기다린 뒤 연다.
        """
        if self._robot is None:
            return False
        rby = self._rby
        robot = self._robot
        try:
            if robot.get_control_manager_state().control_state != rby.ControlManagerState.ControlState.Idle:
                logger.warning("진행 중인 제어 취소")
                robot.cancel_control()
            if not robot.wait_for_control_ready(config.CONTROL_READY_TIMEOUT_MS):
                logger.error("제어 준비 대기 시간 초과")
                return False
            self._stream = robot.create_command_stream()
            logger.info("명령 스트림 열림")
            return True
        except Exception as e:
            logger.error(f"명령 스트림 생성 예외: {e!r}")
            self._stream = None
            return False

    def send_targets(self, named: dict[str, float], dt: float) -> bool:
        """한 주기의 관절 목표를 스트림으로 보낸다. 반환: 성공 여부.

        실패(예외·스트림 종료)면 스트림을 닫고 False를 돌려준다 — 상위가 제어 해제로 처리한다.
        """
        if self._stream is None:
            return False
        rby = self._rby
        min_time = dt * config.CONTROL_MIN_TIME_FACTOR
        hold_time = dt * config.CONTROL_HOLD_TIME_FACTOR
        try:
            body = (
                rby.BodyComponentBasedCommandBuilder()
                .set_torso_command(self._impedance_command(named, _TORSO, "torso", min_time, hold_time))
                .set_right_arm_command(self._impedance_command(named, _RIGHT_ARM, "arm", min_time, hold_time))
                .set_left_arm_command(self._impedance_command(named, _LEFT_ARM, "arm", min_time, hold_time))
            )
            head = self._joint_command(named, _HEAD, min_time, hold_time)
            command = rby.RobotCommandBuilder().set_command(
                rby.ComponentBasedCommandBuilder()
                .set_body_command(body)
                .set_head_command(head)
            )
            self._stream.send_command(command, config.CONTROL_SEND_TIMEOUT_MS)
            return True
        except Exception as e:
            logger.error(f"명령 전송 실패, 스트림 닫음: {e!r}")
            self.stop_stream()
            return False

    def _joint_command(self, named: dict, joints: list[str], min_time: float, hold_time: float):
        """구성요소 하나의 JointPositionCommand 빌더를 만들어 반환한다.

        named에서 joints 순서대로 값을 뽑는다 (누락 관절은 KeyError → 호출부가 실패로 처리).
        속도 상한은 set_velocity_limits로 받은 값이 전부 있을 때만 싣는다 (일부만 주면 SDK가 거부).
        """
        rby = self._rby
        builder = (
            rby.JointPositionCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(hold_time))
            .set_position(np.array([named[j] for j in joints], dtype=np.float64))
            .set_minimum_time(min_time)
        )
        if all(j in self._velocity_limits for j in joints):
            builder.set_velocity_limit(np.array([self._velocity_limits[j] for j in joints], dtype=np.float64))
        return builder

    def _impedance_command(self, named: dict, joints: list[str], gain_key: str, min_time: float, hold_time: float):
        """구성요소 하나의 JointImpedanceControlCommand 빌더를 만들어 반환한다.

        gain_key: config.IMPEDANCE_GAINS의 항목("torso"/"arm") — stiffness·토크 제한·감쇠비를 싣는다.
        목표·속도 상한 처리는 _joint_command와 같다.
        """
        rby = self._rby
        gains = config.IMPEDANCE_GAINS[gain_key]
        builder = (
            rby.JointImpedanceControlCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(hold_time))
            .set_position(np.array([named[j] for j in joints], dtype=np.float64))
            .set_stiffness(np.array(gains["stiffness"], dtype=np.float64))
            .set_torque_limit(np.array(gains["torque_limit"], dtype=np.float64))
            .set_damping_ratio(gains["damping_ratio"])
            .set_minimum_time(min_time)
        )
        if all(j in self._velocity_limits for j in joints):
            builder.set_velocity_limit(np.array([self._velocity_limits[j] for j in joints], dtype=np.float64))
        return builder

    def stop_stream(self):
        """명령 스트림을 닫는다 (열려 있지 않으면 무시). 반환값 없음.

        스트림을 닫아도 서보는 켜진 채라 로봇은 현재 자세를 유지한다.
        """
        stream, self._stream = self._stream, None
        if stream is None:
            return
        try:
            stream.cancel()
        except Exception as e:
            logger.warning(f"스트림 취소 중 예외: {e!r}")
        logger.info("명령 스트림 닫힘")

    # ── 종료 ────────────────────────────────────────────────────────────────

    def disconnect(self):
        """스트림·상태 구독을 멈추고 연결을 닫는다. 반환값 없음.

        서보·Control Manager는 끄지 않는다 — 로봇이 자세를 잡은 채 남는 편이 갑자기 풀리는 것보다 안전하다.
        """
        if self._robot is None:
            return
        self.stop_stream()
        # 종료 경로에서 예외가 나도 서버 종료를 막지 않는다
        try:
            self._robot.stop_state_update()
            self._robot.disconnect()
        except Exception as e:
            logger.warning(f"로봇 연결 종료 중 예외: {e!r}")
        self._robot = None
        self._connected = False
        logger.info("로봇 연결 종료")
