"""로봇 제어기: 조작 입력을 해석해 IK로 목표를 계산하고 실로봇 명령 스트림으로 내보낸다.

데이터 흐름 (30 Hz, 이 루프가 IK 호출까지 소유한다 — IK는 계산기(ik_solver·pose_to_target)일 뿐
스스로 돌지 않는다):
    포즈 샘플 → TargetMapper(클러치 델타) → ArmIkSolver.solve(양팔 관절각)
        + ready 보간(토르소·머리) → 안전 검사 → RobotBackend.send_targets()
    손가락 굽힘(Manus 장갑 우선, 없으면 샘플 fingers) → 손 목표(_hand_targets) → HandBackend.set_fingers()

제어 상태는 두 가지뿐이다.
- 대기(armed=False): 로봇을 움직이지 않는다. 대신 **IK 상태(_last_q·솔버 warm start)를 실로봇
  측정값에 매 주기 시드**해 둔다. 이 덕분에 제어를 시작하는 순간 명령이 현재 자세에서 연속으로
  출발한다 (점프 없음).
- 제어(armed=True): "start" 명령으로 진입. 전원·서보·Control Manager 준비 → 스트림 열기 → ready pose로
  속도 제한 보간 이동 → 이후 클러치(follow) 추종. 팔은 IK, 토르소는 ready 고정값, 머리는 조작자 시선
  (샘플 head의 yaw·pitch)을 로봇 카메라 광축이 따라가도록 명령한다 (head가 없거나 ready 복귀 중이면 ready).
  제어 중 "start" 또는 "ready" 명령이 오면 언제든 ready로 복귀한 뒤 추종을 재개한다.
  "home" 명령이 오면 ready로 복귀한 뒤 제어를 해제한다 (VR 종료 시 웹이 보낸다 — 이후 업링크가 끊기므로
  복귀 동안은 업링크 두절 해제를 적용하지 않는다).

명령은 공통 샘플의 command 필드로 온다 (VR: 진입 2초 후 start, 종료 시 home). 허브가 수신 시 래치해 두고
이 루프가 한 번씩 소비하므로 같은 명령이 두 번 처리되지 않는다.

해제 조건 (어느 것이든 스트림을 닫고 대기로 돌아간다 — 서보는 켜진 채라 로봇은 그 자리에 멈춘다):
    "stop" 명령 / "home" 복귀 완료 / 업링크 두절(CONTROL_INPUT_TIMEOUT_S) / 전송 실패 /
    로봇 상태 수신 두절(ROBOT_STATE_STALE_S)
명령-측정 추적 오차가 CONTROL_MAX_TRACKING_ERR를 넘으면 해제하지 않고 명령·IK 상태를 측정 자세로
재동기화한 뒤 추종을 이어 간다 (밀린 자세에서 새로 출발하므로 튀어 돌아오지 않는다).

입력이 stale이면 마지막 자세를 유지하고 클러치를 해제한다 (복귀 시 새 앵커로 재-engagement).
명령 직전 안전 검사: 이전 명령 대비 관절별 변화량을 속도 상한으로 다시 클램프한다
(IK 솔버의 속도 제한과 이중 방어).
"""

import asyncio
import contextlib
import logging
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

from teleop.robot.robot_interface import HandBackend, RobotBackend
from teleop.utils import config
from teleop.utils.inputs import FINGER_COUNT, hand_fingers, head_pose

logger = logging.getLogger("robot_controller")

_TORSO = [f"torso_{i}" for i in range(6)]
_HEAD = ["head_0", "head_1"]
_TORSO_HEAD = _TORSO + _HEAD
_ARMS = [f"right_arm_{i}" for i in range(7)] + [f"left_arm_{i}" for i in range(7)]
# ready pose 도달 판정 허용 오차 (rad)
_READY_TOL_RAD = 0.01


def _log_task_exception(task: asyncio.Task):
    """루프 태스크가 예외로 끝나면 로그에 남긴다 (조용한 정지 방지). 반환값 없음."""
    if task.cancelled() or task.exception() is None:
        return
    logger.error(f"로봇 제어 루프 종료: {task.exception()!r}", exc_info=task.exception())


class RobotController:
    """제어 상태(대기/제어), IK 상태(_last_q), 마지막 명령을 소유한다."""

    def __init__(self, rtc_hub, robot_model, solver, mapper, robot: RobotBackend,
                 hand: HandBackend | None = None, glove=None, rate_hz: float = config.CONTROL_RATE_HZ):
        """solver·mapper: main이 1회 생성해 넘기는 IK 계산기 (모드 전환 시에도 재구성하지 않는다).

        glove: 조작자 손가락 측정 공급자 — fingers()만 있으면 된다 (ManusGlove). 없으면 샘플 fingers만 쓴다.
        """
        self._hub = rtc_hub
        self._rm = robot_model
        self._solver = solver
        self._mapper = mapper
        self._robot = robot
        self._hand = hand
        self._glove = glove
        self._period = 1.0 / rate_hz
        self._task: asyncio.Task | None = None
        self._armed = False
        self._arm_requested = False
        # IK 상태: 축소 모델 관절각 (대기 중엔 측정값 시드, 제어 중엔 solve/ready 보간 결과)
        self._last_q = robot_model.q_ready.copy()
        # ready 복귀 모드: start/ready/home 명령마다(제어 시작 직후 포함) True — 추종 대신 ready로 속도 제한 보간
        self._returning = False
        # 홈 복귀 후 해제 요청: home 명령으로 True — ready 복귀가 끝나면 제어를 해제한다
        self._home_requested = False
        # 손가락 목표 굽힘(0=펼침, 1=굽힘): 장갑·샘플 fingers에서 갱신, 명령·시각화 폴백이 가져다 쓴다
        self._hand_targets = {side: [0.0] * FINGER_COUNT for side in ("left", "right")}
        # 마지막으로 보낸 명령 (안전 클램프 기준, 제어 시작 시 측정값으로 초기화)
        self._last_cmd: dict[str, float] | None = None
        self._ready = robot_model.ready_positions()
        # 머리 목표 (조작자 시선 추종, 기본 ready)와 머리 (0, 0)일 때 카메라 광축의 (yaw, pitch) 기준값
        self._head_target = {j: self._ready[j] for j in _HEAD}
        self._gaze_zero = self._camera_gaze_at_head_zero()
        # 관절별 한 주기 최대 변화량 계산용 속도 상한 (팔: IK와 동일, 토르소: 보수적, 머리: 시선 추종용)
        urdf = robot_model.velocity_limits_named()
        self._vel = {
            **{j: min(urdf[j] * config.IK_VEL_FRACTION, config.IK_MAX_JOINT_VEL) for j in _ARMS},
            **{j: urdf[j] * config.CONTROL_BODY_VEL_FRACTION for j in _TORSO},
            **{j: urdf[j] * config.CONTROL_HEAD_VEL_FRACTION for j in _HEAD},
        }
        robot.set_velocity_limits(self._vel)
        # 관절별 위치 한계 (로봇이 범위 밖 목표가 섞인 명령을 통째로 거부하므로 전송 전에 자른다)
        self._pos_limits = robot_model.position_limits_named()
        self._clamped = 0
        self._resyncs = 0
        self._sent = 0
        self._solve_ms_acc = 0.0
        self._solve_count = 0
        self._stats_t = 0.0

    @property
    def armed(self) -> bool:
        """제어 중이면 True."""
        return self._armed

    def hand_fingers(self) -> dict[str, list[float]]:
        """조작 입력에서 받은 손가락 목표 굽힘 {left/right: [6] 0-1}를 반환한다 (시각화 폴백용)."""
        return {side: list(values) for side, values in self._hand_targets.items()}

    def start(self):
        """제어 루프를 시작한다. 반환값 없음."""
        self._stats_t = time.monotonic()
        self._task = asyncio.create_task(self._loop())
        self._task.add_done_callback(_log_task_exception)
        logger.info(
            f"로봇 제어기 시작: {1.0 / self._period:.0f} Hz "
            f"(start=제어 시작·ready 복귀(제어 중에도), ready=ready 복귀, home=ready 복귀 후 해제, stop=해제, "
            f"클러치: follow, 손 백엔드 {'있음' if self._hand is not None else '없음'}, "
            f"장갑 {'있음' if self._glove is not None else '없음'})"
        )

    async def stop(self):
        """제어를 해제하고 루프를 멈춘다. 반환값 없음."""
        # 루프 태스크 취소 후 종료까지 대기
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._task
            self._task = None

        # 스트림 닫기는 블로킹(gRPC)이라 executor에서
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._disarm, "서비스 종료")
        logger.info("로봇 제어기 정지")

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
            self._handle_command(self._hub.take_command(), measured)
            self._update_hand_input(sample if fresh else None)

            if measured is None:
                # 측정값 두절: 제어 중이면 오래된 자세로 명령을 이어 가지 않고 해제한다
                if self._armed:
                    self._disarm("로봇 상태 수신 두절")
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

    def _handle_command(self, command: str | None, measured: dict[str, float] | None):
        """명령(start/ready/stop)을 제어 상태에 반영한다. 반환값 없음 (None이면 무시).

        start: 대기면 제어 시작 요청(측정값이 없으면 무시), 제어 중이면 ready 복귀 /
        ready: ready 복귀 / home: ready 복귀 후 해제 (대기 중이면 시작 요청 취소) /
        stop: 대기 중 시작 요청 취소 + 해제.
        """
        if command is None:
            return
        logger.info(f"명령 수신: {command} ({'제어 중' if self._armed else '대기'})")

        # 대기 중 start: 측정값이 있을 때만 이번 주기에 제어 준비 시작
        if command == "start" and not self._armed:
            if measured is None:
                logger.warning("측정값 없음 — start 무시")
                return
            self._arm_requested = True
            return

        # 제어 중 start·ready: 추종을 멈추고 ready로 보간 복귀 (도달 후 클러치 추종 재개, 진행 중인 home 취소)
        if command in ("start", "ready"):
            if not self._armed:
                logger.warning("제어 대기 중 ready 명령 무시 — start로 제어를 시작해야 한다")
                return
            if self._home_requested:
                logger.info(f"{command} 명령: 진행 중이던 home(복귀 후 해제) 취소")
            self._home_requested = False
            self._returning = True
            self._mapper.disengage_all()
            logger.info(f"{command} 명령: ready pose 복귀 시작")
            return

        # home: 제어 중이면 ready로 보간 복귀 후 해제, 대기 중이면 시작 요청만 취소
        if command == "home":
            if self._arm_requested:
                self._arm_requested = False
                logger.warning("home 명령: 대기 중이던 제어 시작 요청 취소")
            if not self._armed:
                return
            self._home_requested = True
            self._returning = True
            self._mapper.disengage_all()
            logger.info("home 명령: ready pose 복귀 후 제어 해제")
            return

        # stop: 대기 중 시작 요청 취소 + 제어 해제
        if command == "stop":
            if self._arm_requested:
                self._arm_requested = False
                logger.warning("stop 명령: 대기 중이던 제어 시작 요청 취소")
            if self._armed:
                self._disarm("stop 명령")

    def _update_hand_input(self, sample: dict | None):
        """손가락 굽힘으로 손 목표를 갱신한다. 반환값 없음.

        손별로 Manus 장갑 측정값을 우선하고, 없으면 샘플 fingers(신선할 때만, sample=None이면 생략)를 쓴다.
        둘 다 없는 손은 이전 값을 유지한다.
        """
        glove_fingers = self._glove.fingers() if self._glove is not None else {}
        for side in ("left", "right"):
            fingers = glove_fingers.get(side)
            if fingers is None and sample is not None:
                fingers = hand_fingers(sample, side)
            if fingers is not None:
                self._hand_targets[side] = fingers

    # ── IK 상태 관리 ─────────────────────────────────────────────────────────

    def _seed_from_measured(self, measured: dict[str, float]):
        """대기 중 IK 상태를 측정 관절값으로 맞추고 ready 복귀 모드를 끈다. 반환값 없음 (매 주기 호출)."""
        if self._sync_ik_to_measured(measured):
            self._returning = False

    def _sync_ik_to_measured(self, measured: dict[str, float]) -> bool:
        """IK 상태(_last_q·솔버 warm start)를 측정 관절값으로 맞추고 클러치를 푼다.

        반환: 동기화 여부 (양팔 관절이 모두 있어야 하며 없으면 무시하고 False).
        클러치를 풀어 두므로 다음 추종 주기에 측정 자세 기준으로 새 앵커를 잡는다.
        """
        q = self._last_q.copy()
        for name, idx in self._rm.arm_q_indices().items():
            if name not in measured:
                return False
            q[idx] = measured[name]
        self._last_q = q
        self._solver.sync(q)
        self._mapper.disengage_all()
        return True

    def _resync_to_measured(self, measured: dict[str, float]):
        """제어 중 명령·IK 상태를 측정 자세로 재동기화한다 (추적 오차 초과 시). 반환값 없음.

        밀린 자세에서 새로 출발하므로 밀림이 풀려도 튀어 돌아오지 않고, 제어는 해제하지 않는다.
        ready 복귀 중이면 측정 자세에서 복귀를 이어 간다.
        """
        self._last_cmd = {j: measured[j] for j in _TORSO_HEAD + _ARMS}
        self._sync_ik_to_measured(measured)
        self._resyncs += 1

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

        IK 상태와 마지막 전송 명령(토르소·머리·팔)이 모두 ready에 도달하면 복귀 모드를 끝내고
        솔버 warm start를 동기화한다.
        """
        # IK 상태를 솔버·전송 클램프와 같은 속도 상한으로 ready 쪽으로 이동
        vel_limits = np.minimum(
            self._rm.velocity_limits() * config.IK_VEL_FRACTION, config.IK_MAX_JOINT_VEL
        )
        max_step = vel_limits * dt
        dq = np.clip(self._rm.q_ready - self._last_q, -max_step, max_step)
        self._last_q = self._last_q + dq

        # IK 상태가 ready에 도달하면 그 자리에 고정
        if np.max(np.abs(self._rm.q_ready - self._last_q)) >= _READY_TOL_RAD:
            return
        self._last_q = self._rm.q_ready.copy()

        # 실제 전송 명령(토르소·머리·팔)까지 ready에 도달해야 복귀 완료
        cmd_ready = all(
            abs(self._last_cmd[j] - self._ready[j]) < _READY_TOL_RAD for j in _TORSO_HEAD + _ARMS
        )
        if not cmd_ready:
            return
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
        # 첫 명령은 스트림을 연 직후의 최신 측정 자세 그대로 — 모든 변화는 여기서부터 속도 제한 안에서 시작된다
        latest = self._robot.joint_positions()
        if latest is not None:
            measured = latest
        self._last_cmd = {j: measured[j] for j in _TORSO_HEAD + _ARMS}
        self._clamped = 0
        self._home_requested = False
        self._armed = True
        logger.info("제어 시작: ready pose로 이동 후 클러치 추종")
        return True

    def _disarm(self, reason: str):
        """스트림을 닫고 대기 상태로 돌아간다. 반환값 없음 (이미 대기면 무시)."""
        if not self._armed:
            return
        self._armed = False
        self._home_requested = False
        self._robot.stop_stream()
        logger.warning(f"제어 해제 ({reason}) — 로봇은 현재 자세 유지")

    # ── 제어 한 주기 ─────────────────────────────────────────────────────────

    async def _control_step(self, loop, measured: dict, sample: dict | None, dt: float):
        """IK 진행 → 목표 조립 → 안전 검사 → 전송 → 손. 해제 조건을 만나면 해제한다. 반환값 없음."""
        # 업링크 두절 해제 (home 복귀 중은 제외 — VR 종료로 업링크가 끊긴 채 복귀해야 한다)
        if not self._home_requested and self._hub.sample_age_s() > config.CONTROL_INPUT_TIMEOUT_S:
            self._disarm(f"업링크 두절 {config.CONTROL_INPUT_TIMEOUT_S:.0f}초")
            return

        # 추적 오차: 로봇이 명령을 못 따라오면(충돌·끼임·토크 포화) 명령을 측정 자세로 재동기화하고 추종을 이어 간다
        worst_joint = max(_TORSO_HEAD + _ARMS, key=lambda j: abs(self._last_cmd[j] - measured[j]))
        worst = abs(self._last_cmd[worst_joint] - measured[worst_joint])
        if worst > config.CONTROL_MAX_TRACKING_ERR:
            logger.warning(
                f"추적 오차 {worst:.2f} rad 초과: {worst_joint} 명령 {self._last_cmd[worst_joint]:.3f}, "
                f"측정 {measured[worst_joint]:.3f} — 측정 자세로 재동기화 후 추종 계속"
            )
            self._resync_to_measured(measured)

        await self._update_arm_targets(loop, sample, dt)

        # 머리: ready 복귀 중이면 ready, 입력이 신선하면 조작자 시선 추종, 아니면 마지막 목표 유지
        if self._returning:
            self._head_target = {j: self._ready[j] for j in _HEAD}
        elif sample is not None:
            self._update_head_target(sample)

        # home 복귀 완료: 마지막 명령이 이미 ready에 도달했으므로 더 보내지 않고 해제
        if self._home_requested and not self._returning:
            self._disarm("home 복귀 완료")
            return
        target = self._compose_target()
        cmd = self._clamp_step(target, dt)

        # 전송은 블로킹(gRPC)이라 executor에서
        ok = await loop.run_in_executor(None, self._robot.send_targets, cmd, self._period)
        if not ok:
            self._disarm("전송 실패")
            return
        self._last_cmd = cmd
        self._sent += 1

        if self._hand is not None:
            self._hand.set_fingers(self._hand_targets)

    def _compose_target(self) -> dict[str, float]:
        """팔은 IK 상태(_last_q), 토르소는 ready, 머리는 시선 목표로 한 주기 목표 {관절명: rad}를 만든다."""
        target = self._rm.arm_positions(self._last_q)
        for j in _TORSO:
            target[j] = self._ready[j]
        for j in _HEAD:
            target[j] = self._head_target[j]
        return target

    # ── 머리 시선 추종 ───────────────────────────────────────────────────────

    def _camera_gaze_at_head_zero(self) -> tuple[float, float]:
        """ready 토르소·머리 (0, 0)에서 카메라 광축의 (yaw, 아래 방향 pitch)(rad)를 base 프레임 기준으로 반환한다."""
        named = dict(self._ready)
        for j in _HEAD:
            named[j] = 0.0
        _, cam_rot = self._rm.camera_pose(named)
        axis = cam_rot[:, 2]
        yaw = float(np.arctan2(axis[1], axis[0]))
        pitch = float(-np.arctan2(axis[2], np.hypot(axis[0], axis[1])))
        logger.info(f"머리 (0, 0) 카메라 광축: yaw {np.degrees(yaw):+.1f}°, pitch(아래+) {np.degrees(pitch):+.1f}°")
        return yaw, pitch

    def _update_head_target(self, sample: dict):
        """샘플 head(조작자 시점) 전방 방향의 yaw·pitch로 머리 목표를 갱신한다. 반환값 없음 (head가 없으면 유지).

        조작자 전방(로봇 좌표에서 시점 로컬 x축)을 카메라 광축이 보도록 기준값 차이만큼 명령한다.
        관절 한계는 전송 직전 _clamp_step이 자른다.
        """
        pose = head_pose(sample)
        if pose is None:
            return
        forward = R.from_quat(pose[1]).as_matrix()[:, 0]
        yaw = float(np.arctan2(forward[1], forward[0]))
        pitch = float(-np.arctan2(forward[2], np.hypot(forward[0], forward[1])))
        self._head_target = {
            "head_0": yaw - self._gaze_zero[0],
            "head_1": pitch - self._gaze_zero[1],
        }

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

            # 관절 위치 한계 안으로 자르기
            lower, upper = self._pos_limits[j]
            cmd[j] = float(np.clip(cmd[j], lower, upper))
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
                f"제어 중: 전송 {self._sent / dt:.1f} Hz, 팔 속도 클램프 {self._clamped}회, "
                f"추적 오차 재동기화 {self._resyncs}회{solve_info}, "
                f"머리 목표 yaw {np.degrees(self._head_target['head_0']):+.1f}° "
                f"pitch {np.degrees(self._head_target['head_1']):+.1f}°"
            )
            self._log_diagnostics(sample)
        else:
            logger.info("제어 대기 (IK를 실로봇 측정값에 시드 중, start 명령(VR 진입 2초 후 자동)으로 시작)")
        self._sent = 0
        self._clamped = 0
        self._resyncs = 0
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

        joints = ((sample or {}).get("body") or {}).get("joints", {})
        spacing = []
        for label, keys in (("shoulder", ("left-shoulder", "right-shoulder")),
                            ("arm-upper", ("left-arm-upper", "right-arm-upper"))):
            l, r = joints.get(keys[0]), joints.get(keys[1])
            if l is not None and r is not None:
                d = float(np.linalg.norm(np.asarray(l["pos"]) - np.asarray(r["pos"])))
                spacing.append(f"{label} {d:.2f}m")
        if spacing:
            logger.info(f"사람 조인트 좌우 간격: {', '.join(spacing)}")
