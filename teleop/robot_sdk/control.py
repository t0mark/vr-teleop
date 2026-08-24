"""로봇 제어 서비스: IK 결과를 실로봇 명령 스트림으로 내보낸다.

데이터 흐름 (30 Hz):
    IkService.joint_positions()(양팔) + ready 보간(토르소·머리) + 스틱(주행)
        → 안전 검사 → RobotBackend.send_targets()
    IkService.gripper_openness()(트리거) → GripperBackend.set_openness()

제어 상태는 두 가지뿐이다.
- 대기(armed=False): 로봇을 움직이지 않는다. 대신 **IK 상태를 실로봇 측정값에 계속 동기화**해
  시뮬레이션과 실로봇이 같은 자세를 가리키게 한다. 이 덕분에 제어를 시작하는 순간 명령이
  현재 자세에서 연속으로 출발한다 (점프 없음).
- 제어(armed=True): 오른손 A로 진입. 전원·서보·Control Manager 준비 → 스트림 열기 → 주기 전송.
  진입 직후 IkService가 ready 복귀를 시작하므로(같은 A 버튼) 로봇은 ready pose로 먼저 이동한 뒤
  클러치 추종에 들어간다. 토르소·머리는 IK 밖이라 여기서 측정값에서 ready로 속도 제한 보간한다.

해제 조건 (어느 것이든 스트림을 닫고 대기로 돌아간다 — 서보는 켜진 채라 로봇은 그 자리에 멈춘다):
    오른손 B / 업링크 두절(CONTROL_INPUT_TIMEOUT_S) / 명령-측정 추적 오차 초과 / 전송 실패

명령 직전 안전 검사: 이전 명령 대비 관절별 변화량을 속도 상한으로 다시 클램프한다
(IK 솔버의 속도 제한과 이중 방어). 클램프가 걸리면 경고 로그로 남긴다.
"""

import asyncio
import logging
import time

import numpy as np

from teleop.robot_sdk.base import GripperBackend, RobotBackend
from teleop.utils import config
from teleop.utils.inputs import hand_button, hand_thumbstick

logger = logging.getLogger("robot_control")

_TORSO_HEAD = [f"torso_{i}" for i in range(6)] + ["head_0", "head_1"]
_ARMS = [f"right_arm_{i}" for i in range(7)] + [f"left_arm_{i}" for i in range(7)]


def _log_task_exception(task: asyncio.Task):
    """루프 태스크가 예외로 끝나면 로그에 남긴다 (조용한 정지 방지). 반환값 없음."""
    if task.cancelled() or task.exception() is None:
        return
    logger.error(f"로봇 제어 루프 종료: {task.exception()!r}", exc_info=task.exception())


class RobotControlService:
    """제어 상태(대기/제어), 마지막 명령, 주행 속도 필터를 소유한다."""

    def __init__(self, pose_hub, robot_model, ik_service, robot: RobotBackend,
                 gripper: GripperBackend | None = None, rate_hz: float = config.CONTROL_RATE_HZ):
        self._hub = pose_hub
        self._rm = robot_model
        self._ik = ik_service
        self._robot = robot
        self._gripper = gripper
        self._period = 1.0 / rate_hz
        self._task: asyncio.Task | None = None
        self._armed = False
        self._arm_requested = False
        self._prev_a = False
        self._prev_b = False
        # 마지막으로 보낸 명령 (안전 클램프 기준, 제어 시작 시 측정값으로 초기화)
        self._last_cmd: dict[str, float] | None = None
        self._ready = robot_model.ready_positions()
        # 관절별 한 주기 최대 변화량 계산용 속도 상한 (팔: IK와 동일, 토르소·머리: 더 보수적)
        urdf = robot_model.velocity_limits_named()
        self._vel = {
            **{j: min(urdf[j] * config.IK_VEL_FRACTION, config.IK_MAX_JOINT_VEL) for j in _ARMS},
            **{j: urdf[j] * config.CONTROL_BODY_VEL_FRACTION for j in _TORSO_HEAD},
        }
        robot.set_velocity_limits(self._vel)
        self._mobility = np.zeros(2)
        self._clamped = 0
        self._sent = 0
        self._stats_t = 0.0

    @property
    def armed(self) -> bool:
        """제어 중이면 True."""
        return self._armed

    def start(self):
        """제어 루프를 시작한다. 반환값 없음."""
        self._stats_t = time.monotonic()
        self._task = asyncio.create_task(self._loop())
        self._task.add_done_callback(_log_task_exception)
        logger.info(f"로봇 제어 서비스 시작: {1.0 / self._period:.0f} Hz (A=제어 시작·ready, B=해제)")

    async def stop(self):
        """제어를 해제하고 루프를 멈춘다. 반환값 없음."""
        if self._task is not None:
            self._task.cancel()
            self._task = None
        self._disarm("서비스 종료")

    # ── 루프 ────────────────────────────────────────────────────────────────

    async def _loop(self):
        """고정 주기로 버튼 처리 → (대기: 동기화 / 제어: 명령 전송)을 수행한다."""
        loop = asyncio.get_running_loop()
        last_t = time.monotonic()
        while True:
            t0 = time.monotonic()
            dt = min(t0 - last_t, 0.2)
            last_t = t0

            measured = self._robot.joint_positions()
            sample = self._hub.latest_robot
            fresh = sample is not None and self._hub.sample_age_s() < config.POSE_STALE_S
            self._handle_buttons(sample if fresh else None)

            if measured is None:
                pass
            elif not self._armed:
                # 대기: 시뮬레이션(IK)을 실로봇에 맞춰 둔다 — 제어 시작 시 연속 출발 보장
                self._ik.sync_to(measured)
                # 준비 요청은 블로킹(SDK 호출)이라 executor에서
                if self._arm_requested:
                    self._arm_requested = False
                    if await loop.run_in_executor(None, self._arm, measured):
                        self._ik.begin_return_to_ready()
            else:
                await self._control_step(loop, measured, sample if fresh else None, dt)

            self._log_stats()
            await asyncio.sleep(max(0.0, self._period - (time.monotonic() - t0)))

    def _handle_buttons(self, sample: dict | None):
        """A 상승 에지 → 제어 시작 요청, B 상승 에지 → 해제. 반환값 없음."""
        a = hand_button(sample, "right", config.GAMEPAD_BUTTON_A)
        b = hand_button(sample, "right", config.GAMEPAD_BUTTON_B)
        if a and not self._prev_a and not self._armed:
            self._arm_requested = True
        if b and not self._prev_b and self._armed:
            self._disarm("B 버튼")
        self._prev_a = a
        self._prev_b = b

    # ── 제어 시작·해제 ───────────────────────────────────────────────────────

    def _arm(self, measured: dict[str, float]) -> bool:
        """전원·서보·CM 준비 후 스트림을 열고 제어 상태로 들어간다. 반환: 성공 여부 (블로킹)."""
        logger.info("제어 시작 요청: 전원·서보·Control Manager 준비")
        if not self._robot.prepare_control():
            logger.error("제어 준비 실패 — 대기 유지")
            return False
        if not self._robot.start_stream():
            logger.error("스트림 열기 실패 — 대기 유지")
            return False
        # 첫 명령은 측정 자세 그대로 — 모든 변화는 여기서부터 속도 제한 안에서 시작된다
        self._last_cmd = {j: measured[j] for j in _TORSO_HEAD + _ARMS}
        self._mobility[:] = 0.0
        self._clamped = 0
        self._armed = True
        logger.info("제어 시작: ready pose로 이동 후 클러치 추종")
        return True

    def _disarm(self, reason: str):
        """스트림을 닫고 대기 상태로 돌아간다. 반환값 없음 (이미 대기면 무시)."""
        if not self._armed:
            return
        self._armed = False
        self._robot.stop_stream()
        logger.warning(f"제어 해제 ({reason}) — 로봇은 현재 자세 유지")

    # ── 제어 한 주기 ─────────────────────────────────────────────────────────

    async def _control_step(self, loop, measured: dict, sample: dict | None, dt: float):
        """목표 조립 → 안전 검사 → 전송 → 그리퍼. 해제 조건을 만나면 해제한다. 반환값 없음."""
        if self._hub.sample_age_s() > config.CONTROL_INPUT_TIMEOUT_S:
            self._disarm(f"업링크 두절 {config.CONTROL_INPUT_TIMEOUT_S:.0f}초")
            return

        # 추적 오차: 로봇이 명령을 못 따라오면(충돌·끼임) 더 밀어붙이지 않는다
        worst = max(abs(self._last_cmd[j] - measured[j]) for j in _TORSO_HEAD + _ARMS)
        if worst > config.CONTROL_MAX_TRACKING_ERR:
            self._disarm(f"추적 오차 {worst:.2f} rad 초과")
            return

        target = self._compose_target()
        cmd = self._clamp_step(target, dt)
        mobility = self._update_mobility(sample, dt)

        # 전송은 블로킹(gRPC)이라 executor에서
        ok = await loop.run_in_executor(None, self._robot.send_targets, cmd, tuple(mobility), self._period)
        if not ok:
            self._disarm("전송 실패")
            return
        self._last_cmd = cmd
        self._sent += 1

        if self._gripper is not None:
            self._gripper.set_openness(self._ik.gripper_openness())

    def _compose_target(self) -> dict[str, float]:
        """팔은 IK 결과, 토르소·머리는 ready 목표로 한 주기 목표 {관절명: rad}를 만든다."""
        target = dict(self._ik.joint_positions())
        for j in _TORSO_HEAD:
            target[j] = self._ready[j]
        return target

    def _clamp_step(self, target: dict[str, float], dt: float) -> dict[str, float]:
        """이전 명령 대비 변화량을 관절별 속도 상한 × dt로 제한한 명령을 반환한다.

        IK 솔버도 같은 제한을 걸지만 실로봇으로 나가는 마지막 관문에서 한 번 더 보장한다.
        토르소·머리의 ready 보간도 이 클램프가 만든다.
        """
        cmd = {}
        clamped = []
        for j, value in target.items():
            step = self._vel[j] * dt
            delta = value - self._last_cmd[j]
            if abs(delta) > step:
                delta = np.sign(delta) * step
                if j in _ARMS:
                    clamped.append(j)
            cmd[j] = self._last_cmd[j] + delta
        if clamped:
            self._clamped += 1
        return cmd

    def _update_mobility(self, sample: dict | None, dt: float) -> np.ndarray:
        """오른손 스틱을 주행 속도 (전진 m/s, yaw rad/s)로 바꿔 1차 필터를 거친 값을 반환한다.

        입력이 없으면 목표 0 (필터 때문에 부드럽게 멈춘다). 데드존 안은 0.
        """
        sx, sy = hand_thumbstick(sample, "right")
        sx = 0.0 if abs(sx) < config.MOBILE_STICK_DEADZONE else sx
        sy = 0.0 if abs(sy) < config.MOBILE_STICK_DEADZONE else sy
        # 게임패드 규약: 앞으로 밀기 = -y, 오른쪽 = +x → 전진 +x, 왼쪽 밀기 = +yaw(CCW)
        goal = np.array([
            -sy * config.MOBILE_LINEAR_MAX * config.MOBILE_LINEAR_SIGN,
            -sx * config.MOBILE_ANGULAR_MAX * config.MOBILE_ANGULAR_SIGN,
        ])
        alpha = min(1.0, dt / config.MOBILE_SMOOTHING_TAU)
        self._mobility += alpha * (goal - self._mobility)
        return self._mobility

    def _log_stats(self):
        """주기적으로 제어 상태·전송률·클램프 횟수를 로그로 남긴다. 반환값 없음."""
        now = time.monotonic()
        if now - self._stats_t < config.POSE_STATS_INTERVAL_S:
            return
        dt = now - self._stats_t
        if self._armed:
            logger.info(
                f"제어 중: 전송 {self._sent / dt:.1f} Hz, 팔 속도 클램프 {self._clamped}회, "
                f"주행 v={self._mobility[0]:.2f} w={self._mobility[1]:.2f}"
            )
        else:
            logger.info("제어 대기 (IK를 실로봇 측정값에 동기화 중, 오른손 A로 시작)")
        self._sent = 0
        self._clamped = 0
        self._stats_t = now
