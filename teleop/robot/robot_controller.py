"""로봇 제어기: 조작 입력을 해석해 IK로 목표를 계산하고 실로봇 명령 스트림으로 내보낸다.

데이터 흐름 (30 Hz, 이 루프가 IK 호출까지 소유한다 — IK는 계산기(ik_solver·pose_to_target)일 뿐
스스로 돌지 않는다):
    포즈 샘플 → TargetMapper(클러치 델타) → ArmIkSolver.solve(양팔 관절각)
        + ready 보간(토르소·머리) → 안전 검사 → RobotBackend.send_targets()
    트리거 → 그리퍼 열림 정도 → GripperBackend.set_openness()

제어 상태는 두 가지뿐이다.
- 대기(armed=False): 로봇을 움직이지 않는다. 대신 **IK 상태(_last_q·솔버 warm start)를 실로봇
  측정값에 매 주기 시드**해 둔다. 이 덕분에 제어를 시작하는 순간 명령이 현재 자세에서 연속으로
  출발한다 (점프 없음).
- 제어(armed=True): 오른손 A로 진입. 전원·서보·Control Manager 준비 → 스트림 열기 → ready pose로
  속도 제한 보간 이동 → 이후 클러치 추종. 팔은 IK, 토르소·머리는 ready 고정값으로 명령한다.
  제어 중 A를 다시 누르면 언제든 ready로 복귀한 뒤 추종을 재개한다.

해제 조건 (어느 것이든 스트림을 닫고 대기로 돌아간다 — 서보는 켜진 채라 로봇은 그 자리에 멈춘다):
    오른손 B / 업링크 두절(CONTROL_INPUT_TIMEOUT_S) / 명령-측정 추적 오차 초과 / 전송 실패

입력이 stale이면 마지막 자세를 유지하고 클러치를 해제한다 (복귀 시 새 앵커로 재-engagement).
명령 직전 안전 검사: 이전 명령 대비 관절별 변화량을 속도 상한으로 다시 클램프한다
(IK 솔버의 속도 제한과 이중 방어).
"""

import asyncio
import logging
import time

import numpy as np

from teleop.robot.robot_interface import GripperBackend, RobotBackend
from teleop.utils import config
from teleop.utils.inputs import hand_button, hand_trigger

logger = logging.getLogger("robot_controller")

_TORSO_HEAD = [f"torso_{i}" for i in range(6)] + ["head_0", "head_1"]
_ARMS = [f"right_arm_{i}" for i in range(7)] + [f"left_arm_{i}" for i in range(7)]


def _log_task_exception(task: asyncio.Task):
    """루프 태스크가 예외로 끝나면 로그에 남긴다 (조용한 정지 방지). 반환값 없음."""
    if task.cancelled() or task.exception() is None:
        return
    logger.error(f"로봇 제어 루프 종료: {task.exception()!r}", exc_info=task.exception())


class RobotController:
    """제어 상태(대기/제어), IK 상태(_last_q), 마지막 명령을 소유한다."""

    def __init__(self, rtc_hub, robot_model, solver, mapper, robot: RobotBackend,
                 gripper: GripperBackend | None = None, rate_hz: float = config.CONTROL_RATE_HZ):
        """solver·mapper: main이 1회 생성해 넘기는 IK 계산기 (모드 전환 시에도 재구성하지 않는다)."""
        self._hub = rtc_hub
        self._rm = robot_model
        self._solver = solver
        self._mapper = mapper
        self._robot = robot
        self._gripper = gripper
        self._period = 1.0 / rate_hz
        self._task: asyncio.Task | None = None
        self._armed = False
        self._arm_requested = False
        self._prev_a = False
        self._prev_b = False
        # IK 상태: 축소 모델 관절각 (대기 중엔 측정값 시드, 제어 중엔 solve/ready 보간 결과)
        self._last_q = robot_model.q_ready.copy()
        # ready 복귀 모드: A를 누를 때마다(제어 시작 직후 포함) True — 추종 대신 ready로 속도 제한 보간
        self._returning = False
        # 그리퍼 열림 정도(0=닫힘, 1=열림): 트리거에서 갱신, 명령·시각화 폴백이 가져다 쓴다
        self._gripper_open = {"left": 1.0, "right": 1.0}
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
        self._clamped = 0
        self._sent = 0
        self._solve_ms_acc = 0.0
        self._solve_count = 0
        self._stats_t = 0.0

    @property
    def armed(self) -> bool:
        """제어 중이면 True."""
        return self._armed

    def gripper_openness(self) -> dict[str, float]:
        """트리거 입력에서 계산한 그리퍼 열림 정도 {left/right: 0-1}를 반환한다 (시각화 폴백용)."""
        return dict(self._gripper_open)

    def start(self):
        """제어 루프를 시작한다. 반환값 없음."""
        self._stats_t = time.monotonic()
        self._task = asyncio.create_task(self._loop())
        self._task.add_done_callback(_log_task_exception)
        logger.info(
            f"로봇 제어기 시작: {1.0 / self._period:.0f} Hz "
            f"(A=제어 시작·ready 복귀(제어 중에도), B=해제, 클러치: grip > {config.GRIP_ENGAGE_THRESHOLD})"
        )

    async def stop(self):
        """제어를 해제하고 루프를 멈춘다. 반환값 없음."""
        if self._task is not None:
            self._task.cancel()
            self._task = None
        self._disarm("서비스 종료")

    # ── 루프 ────────────────────────────────────────────────────────────────

    async def _loop(self):
        """고정 주기로 입력 처리 → (대기: IK 시드 / 제어: IK 계산·명령 전송)을 수행한다."""
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
            if fresh:
                self._update_gripper_input(sample)

            if measured is None:
                pass
            elif not self._armed:
                # 대기: IK 상태를 실로봇 측정값에 맞춰 둔다 — 제어 시작 시 연속 출발 보장
                self._seed_from_measured(measured)
                # 준비 요청은 블로킹(SDK 호출)이라 executor에서
                if self._arm_requested:
                    self._arm_requested = False
                    if await loop.run_in_executor(None, self._arm, measured):
                        self._returning = True
                        logger.info("ready pose 복귀 시작")
            else:
                await self._control_step(loop, measured, sample if fresh else None, dt)

            self._log_stats(sample)
            await asyncio.sleep(max(0.0, self._period - (time.monotonic() - t0)))

    def _handle_buttons(self, sample: dict | None):
        """A 상승 에지 → 대기 중이면 제어 시작 요청, 제어 중이면 ready 복귀. B 상승 에지 → 해제. 반환값 없음."""
        a = hand_button(sample, "right", config.GAMEPAD_BUTTON_A)
        b = hand_button(sample, "right", config.GAMEPAD_BUTTON_B)
        if a and not self._prev_a:
            if not self._armed:
                self._arm_requested = True
            else:
                # 제어 중 A: 추종을 멈추고 ready로 보간 복귀 (도달 후 클러치 추종 재개)
                self._returning = True
                self._mapper.disengage_all()
                logger.info("A 버튼: ready pose 복귀 시작")
        if b and not self._prev_b and self._armed:
            self._disarm("B 버튼")
        self._prev_a = a
        self._prev_b = b

    def _update_gripper_input(self, sample: dict):
        """트리거 값을 그리퍼 열림 정도로 갱신한다 (당길수록 닫힘). 반환값 없음."""
        for side in ("left", "right"):
            trigger = hand_trigger(sample, side)
            if trigger is not None:
                self._gripper_open[side] = 1.0 - float(np.clip(trigger, 0.0, 1.0))

    # ── IK 상태 관리 ─────────────────────────────────────────────────────────

    def _seed_from_measured(self, measured: dict[str, float]):
        """IK 상태(_last_q·솔버 warm start)를 측정 관절값으로 맞추고 클러치를 푼다. 반환값 없음.

        대기 중 매 주기 호출된다. 양팔 관절이 모두 있어야 하며 없으면 무시한다.
        """
        q = self._last_q.copy()
        for name, idx in self._rm.arm_q_indices().items():
            if name not in measured:
                return
            q[idx] = measured[name]
        self._last_q = q
        self._returning = False
        self._solver.sync(q)
        self._mapper.disengage_all()

    async def _update_arm_targets(self, loop, sample: dict | None, dt: float):
        """팔 IK 상태를 한 주기 진행시킨다 (ready 보간 / 클러치 추종 solve / 유지). 반환값 없음."""
        if self._returning:
            # ready 복귀: 추종을 멈추고 관절 속도 제한으로 ready pose까지 보간
            self._mapper.disengage_all()
            self._step_toward_ready(dt)
        elif sample is not None:
            # 클러치 델타 매핑은 로봇의 현재 자세(FK)를 앵커 기준으로 쓴다
            targets = self._mapper.map(sample, self._rm.fk_frames(self._last_q))
            # solve(3-4ms)는 블로킹이라 executor에서 — WSS 이벤트 루프를 막지 않는다
            t0 = time.monotonic()
            self._last_q = await loop.run_in_executor(None, self._solver.solve, targets, dt)
            self._solve_ms_acc += (time.monotonic() - t0) * 1000
            self._solve_count += 1
        else:
            # 입력 stale: 마지막 자세 유지 + 클러치 해제 (복귀 시 새 앵커로 재-engagement)
            self._mapper.disengage_all()

    def _step_toward_ready(self, dt: float):
        """관절 속도 제한 안에서 ready pose 쪽으로 한 스텝 이동한다. 반환값 없음.

        도달하면 복귀 모드를 끝내고 솔버 warm start를 동기화한다.
        """
        max_step = self._rm.velocity_limits() * config.IK_VEL_FRACTION * dt
        dq = np.clip(self._rm.q_ready - self._last_q, -max_step, max_step)
        self._last_q = self._last_q + dq

        if np.max(np.abs(self._rm.q_ready - self._last_q)) < 0.01:
            self._last_q = self._rm.q_ready.copy()
            self._returning = False
            self._solver.sync(self._last_q)
            logger.info("ready pose 복귀 완료")

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
        """IK 진행 → 목표 조립 → 안전 검사 → 전송 → 그리퍼. 해제 조건을 만나면 해제한다. 반환값 없음."""
        if self._hub.sample_age_s() > config.CONTROL_INPUT_TIMEOUT_S:
            self._disarm(f"업링크 두절 {config.CONTROL_INPUT_TIMEOUT_S:.0f}초")
            return

        # 추적 오차: 로봇이 명령을 못 따라오면(충돌·끼임) 더 밀어붙이지 않는다
        worst = max(abs(self._last_cmd[j] - measured[j]) for j in _TORSO_HEAD + _ARMS)
        if worst > config.CONTROL_MAX_TRACKING_ERR:
            self._disarm(f"추적 오차 {worst:.2f} rad 초과")
            return

        await self._update_arm_targets(loop, sample, dt)
        target = self._compose_target()
        cmd = self._clamp_step(target, dt)

        # 전송은 블로킹(gRPC)이라 executor에서
        ok = await loop.run_in_executor(None, self._robot.send_targets, cmd, self._period)
        if not ok:
            self._disarm("전송 실패")
            return
        self._last_cmd = cmd
        self._sent += 1

        if self._gripper is not None:
            self._gripper.set_openness(self._gripper_open)

    def _compose_target(self) -> dict[str, float]:
        """팔은 IK 상태(_last_q), 토르소·머리는 ready 목표로 한 주기 목표 {관절명: rad}를 만든다."""
        target = self._rm.arm_positions(self._last_q)
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

    # ── 통계·진단 ────────────────────────────────────────────────────────────

    def _log_stats(self, sample: dict | None):
        """주기적으로 제어 상태·전송률·IK 시간·클램프 횟수를 로그로 남긴다. 반환값 없음."""
        now = time.monotonic()
        if now - self._stats_t < config.POSE_STATS_INTERVAL_S:
            return
        dt = now - self._stats_t
        if self._armed:
            solve_info = (
                f", IK 평균 {self._solve_ms_acc / self._solve_count:.1f} ms/solve ({self._solve_count}회)"
                if self._solve_count > 0 else ""
            )
            logger.info(
                f"제어 중: 전송 {self._sent / dt:.1f} Hz, 팔 속도 클램프 {self._clamped}회{solve_info}"
            )
            self._log_diagnostics(sample)
        else:
            logger.info("제어 대기 (IK를 실로봇 측정값에 시드 중, 오른손 A로 시작)")
        self._sent = 0
        self._clamped = 0
        self._solve_ms_acc = 0.0
        self._solve_count = 0
        self._stats_t = now

    def _log_diagnostics(self, sample: dict | None):
        """관절 고착·조인트 의미 검증용 진단을 로그로 남긴다. 반환값 없음.

        - 한계에 박힌 관절: 해당 관절이 안 움직이는 것처럼 보이는 원인 (매핑 편향 신호)
        - 사람 조인트 좌우 간격: shoulder(쇄골)-arm-upper(어깨 볼) 의미 구분의 실측 근거
        """
        lb, ub = self._rm.joint_bounds()
        q = self._last_q
        pinned = [
            self._rm.model.names[jid]
            for jid in range(1, self._rm.model.njoints)
            if min(q[self._rm.model.joints[jid].idx_q] - lb[self._rm.model.joints[jid].idx_q],
                   ub[self._rm.model.joints[jid].idx_q] - q[self._rm.model.joints[jid].idx_q]) < 0.02
        ]
        if pinned:
            logger.info(f"관절 한계 도달: {pinned}")

        joints = (sample or {}).get("body", {}).get("joints", {})
        spacing = []
        for label, keys in (("shoulder", ("left-shoulder", "right-shoulder")),
                            ("arm-upper", ("left-arm-upper", "right-arm-upper"))):
            l, r = joints.get(keys[0]), joints.get(keys[1])
            if l is not None and r is not None:
                d = float(np.linalg.norm(np.asarray(l["pos"]) - np.asarray(r["pos"])))
                spacing.append(f"{label} {d:.2f}m")
        if spacing:
            logger.info(f"사람 조인트 좌우 간격: {', '.join(spacing)}")
