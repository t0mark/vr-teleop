"""IK 서비스: 포즈 허브의 최신 샘플을 주기적으로 IK에 넣고 결과를 시각화용으로 방송한다.

데이터 흐름: PoseHub.latest_robot → TargetMapper(클러치 델타) → ArmIkSolver → FK(visual 링크)
             → WebXR 좌표 변환 → PoseHub.broadcast_state({type: robot_state})

정지 규약: 입력이 없거나 stale이면 마지막 관절각을 유지한 채 방송한다 (ready로 점프하지 않음 —
실로봇 연결 시 순간 텔레포트 명령이 되는 것을 방지). 트래킹 두절 시 클러치도 해제해
복귀 순간 새 앵커로 다시 engagement 되게 한다.

실로봇 제어 단계에서는 solve 결과 q를 robot_sdk로도 보내게 된다 (현재는 시뮬레이션 시각화만).
q 순서는 pinocchio 축소 모델 순서이므로, 브리지는 robot_config의 joint_names로 이름 매핑할 것.
"""

import asyncio
import logging
import time

import numpy as np

from teleop.utils import config
from teleop.utils.transforms import robot_to_webxr

logger = logging.getLogger("ik_service")


def _log_task_exception(task: asyncio.Task):
    """루프 태스크가 예외로 끝나면 로그에 남긴다 (조용한 정지 방지). 반환값 없음."""
    if task.cancelled() or task.exception() is None:
        return
    logger.error(f"IK 루프 종료: {task.exception()!r}", exc_info=task.exception())


class IkService:
    """IK 루프 태스크와 상태(마지막 관절각·통계)를 소유한다."""

    def __init__(self, pose_hub, robot_model, solver, mapper):
        self._hub = pose_hub
        self._rm = robot_model
        self._solver = solver
        self._mapper = mapper
        self._task: asyncio.Task | None = None
        self._last_q = robot_model.q_ready.copy()
        # 그리퍼 열림 정도(0=닫힘, 1=열림): 트리거에서 갱신, 시각화·상태 방송에 사용
        self._gripper = {"left": 1.0, "right": 1.0}
        # ready 복귀 모드 (오른손 A 버튼): True인 동안 추종을 멈추고 ready로 보간 이동
        self._returning = False
        self._prev_a = False
        self._solve_ms_acc = 0.0
        self._solve_count = 0

    def start(self):
        """IK 루프 태스크를 시작하고 시각화 메타를 허브에 등록한다. 반환값 없음."""
        # 브라우저가 접속 시 1회 받아야 하는 로봇 메시 구성 + 브리지용 q 순서 관절명
        self._hub.robot_config = {
            "type": "robot_config",
            "mesh_base": config.MESH_URL_PREFIX,
            "links": self._rm.visual_config(),
            "joint_names": self._rm.joint_names_in_q_order(),
            "camera": self._rm.camera_visual_config(),
        }
        self._task = asyncio.create_task(self._loop())
        self._task.add_done_callback(_log_task_exception)
        logger.info(f"IK 서비스 시작: {config.IK_RATE_HZ} Hz (클러치: grip > {config.GRIP_ENGAGE_THRESHOLD})")

    async def stop(self):
        """IK 루프를 정지한다. 반환값 없음."""
        if self._task is not None:
            self._task.cancel()
            self._task = None

    async def _loop(self):
        """고정 주기로 IK를 풀고 결과를 방송한다. 입력이 없으면 마지막 자세를 유지 방송한다."""
        loop = asyncio.get_running_loop()
        period = 1.0 / config.IK_RATE_HZ
        last_stats = time.monotonic()
        last_solve = time.monotonic()

        while True:
            t0 = time.monotonic()

            sample = self._hub.latest_robot
            fresh = sample is not None and self._hub.sample_age_s() < config.POSE_STALE_S
            dt = min(t0 - last_solve, 0.2)
            last_solve = t0

            if fresh:
                self._update_inputs(sample)

            if self._returning:
                # ready 복귀: 추종을 멈추고 관절 속도 제한으로 ready pose까지 보간
                self._mapper.disengage_all()
                q = self._step_toward_ready(dt)
            elif fresh:
                # 클러치 델타 매핑은 로봇의 현재 자세(FK)를 앵커 기준으로 쓴다
                targets = self._mapper.map(sample, self._rm.fk_frames(self._last_q))
                # ipopt류 블로킹 호출이므로 executor에서 실행해 WSS 이벤트 루프를 막지 않는다
                q = await loop.run_in_executor(None, self._solver.solve, targets, dt)
                self._last_q = q
                self._solve_ms_acc += (time.monotonic() - t0) * 1000
                self._solve_count += 1
            else:
                # 트래킹 두절: 마지막 자세 유지 + 클러치 해제 (복귀 시 새 앵커로 재-engagement)
                self._mapper.disengage_all()
                q = self._last_q

            await self._hub.broadcast_state(self._make_robot_state(q))

            # 주기 유지 + 5초마다 평균 solve 시간·진단 로그
            now = time.monotonic()
            if now - last_stats >= config.POSE_STATS_INTERVAL_S:
                last_stats = now
                if self._solve_count > 0:
                    avg = self._solve_ms_acc / self._solve_count
                    logger.info(f"IK 평균 {avg:.1f} ms/solve ({self._solve_count}회), 입력 {'유' if fresh else '무'}")
                    self._solve_ms_acc = 0.0
                    self._solve_count = 0
                    self._log_diagnostics(q, sample)
            await asyncio.sleep(max(0.0, period - (time.monotonic() - t0)))

    def _update_inputs(self, sample: dict):
        """샘플에서 그리퍼(트리거)·ready 복귀(A 버튼) 입력을 갱신한다. 반환값 없음."""
        hands = sample.get("hands", {})
        for side in ("left", "right"):
            hand = hands.get(side)
            if hand is not None:
                # 트리거를 당길수록 닫힘 (열림 정도 = 1 - trigger)
                self._gripper[side] = 1.0 - float(np.clip(hand.get("trigger", 0.0), 0.0, 1.0))

        # 오른손 A 버튼 상승 에지에서 ready 복귀 시작 (기존 teleop의 Right-A와 동일)
        right = hands.get("right")
        buttons = right.get("buttons", []) if right else []
        a_pressed = len(buttons) > config.GAMEPAD_BUTTON_A and bool(buttons[config.GAMEPAD_BUTTON_A])
        if a_pressed and not self._prev_a and not self._returning:
            self._returning = True
            logger.info("A 버튼: ready pose 복귀 시작")
        self._prev_a = a_pressed

    def _step_toward_ready(self, dt: float) -> np.ndarray:
        """관절 속도 제한 안에서 ready pose 쪽으로 한 스텝 이동한 q를 반환한다.

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
        return self._last_q

    def _log_diagnostics(self, q, sample):
        """관절 고착·조인트 의미 검증용 진단을 로그로 남긴다. 반환값 없음.

        - 한계에 박힌 관절: 해당 관절이 안 움직이는 것처럼 보이는 원인 (매핑 편향 신호)
        - 사람 조인트 좌우 간격: shoulder(쇄골)-arm-upper(어깨 볼) 의미 구분의 실측 근거
        """
        lb, ub = self._rm.joint_bounds()
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

    def _make_robot_state(self, q) -> dict:
        """관절각 q의 FK 결과를 WebXR 좌표계 링크 포즈 메시지로 변환해 반환한다.

        gripper 필드(열림 정도 0-1)는 시각화와 실로봇 브리지(그리퍼 명령)가 함께 쓴다.
        """
        links = {}
        for name, (pos, quat) in self._rm.fk_visual_links(q, gripper_open=self._gripper).items():
            p, r = robot_to_webxr(pos, quat)
            links[name] = {"pos": p.tolist(), "quat": r.tolist()}
        return {"type": "robot_state", "links": links, "q": q.tolist(), "gripper": dict(self._gripper)}
