"""양팔 IK 솔버: Pink(Pinocchio 기반 미분 IK)로 손 TCP 포즈를 추종하는 관절 속도를 QP로 풀어 한 스텝 적분한다.

정식화 (매 주기 QP 1회):
    min_v  Σ_arm ‖J_tcp·v − gain·e_tcp‖²_W  + w_posture·‖v − (q_ready − q)/dt‖² + damping·‖v‖²
    s.t.   q_lower ≤ q + v·dt ≤ q_upper (URDF ∩ 안전 한계),  |v| ≤ v_max
    q_next = q ⊕ v·dt

- 손 작업(FrameTask): 손 TCP 프레임(ee + HAND_TCP_OFFSET)의 위치·자세 오차. 위치 비용을 가장 크게 둬 손 위치 정확도를 우선한다
- 자세 작업(PostureTask): ready pose 쪽으로 약하게 당겨 남는 자유도(팔꿈치 회전)를 정한다
- 관절 위치·속도 한계는 비용이 아니라 QP 제약이라 해가 한계를 넘지 않는다
- 클러치 미체결 팔: 손 작업을 넣지 않고, 적분 후 해당 팔 관절각을 이전 해로 고정한다
"""

import logging
import time

import numpy as np
import pinocchio as pin
from pink import Configuration, solve_ik
from pink.limits import ConfigurationLimit, VelocityLimit
from pink.tasks import FrameTask, PostureTask

from teleop.utils import config

logger = logging.getLogger("ik_solver")


class ArmIkSolver:
    """IK 전용 축소 모델 사본(한계 적용)과 Pink 작업·한계, solve 호출 상태(이전 해)를 소유한다."""

    def __init__(self, robot_model):
        self._robot_model = robot_model
        self._q_prev = robot_model.q_ready.copy()

        # 팔별 축소 모델 q 인덱스 (미체결 팔 관절 고정용)
        arm_q_indices = robot_model.arm_q_indices()
        self._side_q_indices = {
            side: np.array([idx for name, idx in arm_q_indices.items() if name.startswith(f"{side}_arm_")])
            for side in ("left", "right")
        }

        # 축소 모델 q 인덱스 → 관절명 (한계 밖 관절 로그용)
        self._joint_name_by_idx = {idx: name for name, idx in arm_q_indices.items()}

        # 경고 종류별 누적 횟수와 마지막 경고 시각 (경고 요약용)
        self._warn_counts = {}
        self._warn_times = {}

        # 마지막 sync에서 IK 관절 한계 밖이던 관절명 (상태 변화 로그용)
        self._sync_out_of_bounds = ()

        self._build_ik()

    @property
    def vel_limit(self) -> np.ndarray:
        """축소 모델 q 순서의 팔 관절 속도 상한(rad/s) 사본."""
        return self._vel_limit.copy()

    def _build_ik(self):
        """IK 전용 모델 사본에 한계를 적용하고 Pink 작업·한계를 구성한다. 반환값 없음."""
        # IK 전용 모델 사본: 위치 한계 = URDF ∩ 안전 한계, 속도 한계 = URDF × IK_VEL_FRACTION (상한 IK_MAX_JOINT_VEL)
        self._ik_model = pin.Model(self._robot_model.model)
        self._lb, self._ub = self._robot_model.joint_bounds()
        self._vel_limit = np.minimum(
            self._robot_model.velocity_limits() * config.IK_VEL_FRACTION, config.IK_MAX_JOINT_VEL
        )
        self._ik_model.lowerPositionLimit = self._lb.copy()
        self._ik_model.upperPositionLimit = self._ub.copy()
        self._ik_model.velocityLimit = self._vel_limit.copy()
        self._configuration = Configuration(self._ik_model, self._ik_model.createData(), self._q_prev)

        # 손 TCP 추종 작업 (팔별)
        self._hand_tasks = {
            side: FrameTask(
                frame_name,
                position_cost=config.IK_POSITION_COST,
                orientation_cost=config.IK_ORIENTATION_COST,
                lm_damping=config.IK_LM_DAMPING,
            )
            for side, frame_name in self._robot_model.tcp_frame_names.items()
        }

        # ready pose 유지 작업
        self._posture_task = PostureTask(cost=config.IK_POSTURE_COST)
        self._posture_task.set_target(self._robot_model.q_ready)

        # 관절 위치·속도 한계 (QP 제약)
        self._limits = [
            ConfigurationLimit(self._ik_model, config_limit_gain=config.IK_CONFIG_LIMIT_GAIN),
            VelocityLimit(self._ik_model),
        ]
        logger.info(
            f"IK(Pink) 구성 완료: nq={self._ik_model.nq}, QP {config.IK_QP_SOLVER}, "
            f"비용 위치 {config.IK_POSITION_COST}·자세 {config.IK_ORIENTATION_COST}·ready {config.IK_POSTURE_COST}"
        )

    def solve(self, targets: dict, dt: float | None = None) -> np.ndarray:
        """목표 dict로 관절각을 한 스텝 계산해 반환한다 (실패 시 이전 해 유지).

        targets: {side: {wrist_pos(3), wrist_quat(4)} | None} — 로봇 base 프레임의 손 TCP 목표.
        side가 None이면(클러치 미체결) 해당 팔은 관절각을 이전 해 그대로 고정한다.
        dt: 이전 solve 후 경과 시간(초). QP 적분 스텝으로 사용 (기본: IK 주기,
        상한: IK 주기 × IK_MAX_DT_FACTOR). 비정상 목표는 해당 팔 미체결(None)로 처리한다.
        """
        # 목표 검사: 비정상 목표는 해당 팔 미체결로 처리
        targets = {side: self._validated_target(side, targets.get(side)) for side in ("left", "right")}
        held = [side for side in ("left", "right") if targets[side] is None]

        # 양팔 모두 미체결이면 이전 해 유지
        if len(held) == 2:
            return self._q_prev.copy()

        # 스텝 시간: 경과 시간을 IK 주기 × IK_MAX_DT_FACTOR 이하로 자른다
        ik_period = 1.0 / config.IK_RATE_HZ
        step_dt = ik_period if dt is None else min(dt, config.IK_MAX_DT_FACTOR * ik_period)
        if step_dt <= 0.0:
            return self._q_prev.copy()

        # 작업 구성: 체결된 팔의 손 TCP 목표 + ready 자세 유지
        tasks = [self._posture_task]
        for side, target in targets.items():
            if target is None:
                continue
            target_rotation = pin.Quaternion(
                target["wrist_quat"][3], target["wrist_quat"][0], target["wrist_quat"][1], target["wrist_quat"][2]
            ).normalized().toRotationMatrix()
            self._hand_tasks[side].set_target(pin.SE3(target_rotation, target["wrist_pos"]))
            tasks.append(self._hand_tasks[side])

        # 이전 해에서 관절 속도 QP 풀기 (실패 시 이전 해 유지)
        self._configuration.update(self._q_prev)
        try:
            velocity = solve_ik(
                self._configuration, tasks, step_dt,
                solver=config.IK_QP_SOLVER, damping=config.IK_DAMPING, limits=self._limits, safety_break=False,
            )
        except Exception as error:
            self._warn_throttled("solve_error", f"IK QP 실패, 이전 해 유지: {type(error).__name__}: {error}")
            return self._q_prev.copy()

        # 한 스텝 적분
        q_next = pin.integrate(self._ik_model, self._q_prev, velocity * step_dt)

        # 비정상(NaN/inf) 결과는 버리고 이전 해 유지
        if not np.all(np.isfinite(q_next)):
            self._warn_throttled("solve_nonfinite", "IK 결과 비정상(NaN/inf), 이전 해 유지")
            return self._q_prev.copy()

        # 미체결 팔 고정: 해당 팔 관절각을 이전 해로 되돌린다
        for side in held:
            q_next[self._side_q_indices[side]] = self._q_prev[self._side_q_indices[side]]

        self._q_prev = q_next
        return q_next.copy()

    def _validated_target(self, side: str, target: dict | None) -> dict | None:
        """한 팔 목표의 형식·값을 검사해 정규화한 목표를 반환한다 (미체결·비정상이면 None)."""
        if target is None:
            return None

        # 손 TCP 목표: 위치(3)·쿼터니언(4) 배열 변환
        try:
            wrist_pos = np.asarray(target["wrist_pos"], dtype=float).reshape(3)
            wrist_quat = np.asarray(target["wrist_quat"], dtype=float).reshape(4)
        except (KeyError, TypeError, ValueError, IndexError):
            self._warn_throttled(f"target_format_{side}", f"IK 목표 형식 오류({side}), 해당 팔 고정")
            return None

        # 손 TCP 목표: 유한값·쿼터니언 노름 검사
        wrist_finite = np.all(np.isfinite(wrist_pos)) and np.all(np.isfinite(wrist_quat))
        if not wrist_finite or np.linalg.norm(wrist_quat) < 1e-6:
            self._warn_throttled(f"target_value_{side}", f"IK 목표 비정상(NaN/inf/영 쿼터니언, {side}), 해당 팔 고정")
            return None

        return {"wrist_pos": wrist_pos, "wrist_quat": wrist_quat}

    def _warn_throttled(self, kind: str, message: str):
        """경고를 종류별로 세고 IK_WARN_INTERVAL_S마다 누적 횟수와 함께 로그로 남긴다. 반환값 없음."""
        self._warn_counts[kind] = self._warn_counts.get(kind, 0) + 1

        # 마지막 경고 후 요약 주기가 지났을 때만 출력하고 횟수를 초기화
        now = time.monotonic()
        if now - self._warn_times.get(kind, -np.inf) < config.IK_WARN_INTERVAL_S:
            return
        logger.warning(f"{message} (최근 {self._warn_counts[kind]}회)")
        self._warn_counts[kind] = 0
        self._warn_times[kind] = now

    def reset(self):
        """이전 해 상태를 ready pose로 되돌린다. 반환값 없음."""
        self._q_prev = self._robot_model.q_ready.copy()

    def sync(self, q: np.ndarray):
        """이전 해 상태를 외부 관절각으로 맞춘다 (ready 복귀 등 외부 궤적 후 호출). 반환값 없음."""
        self._q_prev = np.asarray(q, dtype=float).copy()

        # IK 관절 한계 밖 관절 검사: 목록이 바뀔 때만 로그
        out_of_bounds_idx = np.flatnonzero((self._q_prev < self._lb) | (self._q_prev > self._ub))
        out_of_bounds = tuple(self._joint_name_by_idx[int(idx)] for idx in out_of_bounds_idx)
        if out_of_bounds == self._sync_out_of_bounds:
            return
        self._sync_out_of_bounds = out_of_bounds

        # 한계 밖 관절이 있으면 관절별 값·한계를 경고, 없으면 복귀 로그
        if out_of_bounds:
            joints_text = ", ".join(
                f"{self._joint_name_by_idx[int(idx)]}={self._q_prev[idx]:.3f} "
                f"[{self._lb[idx]:.3f}, {self._ub[idx]:.3f}]"
                for idx in out_of_bounds_idx
            )
            logger.warning(f"동기화 관절각이 IK 한계 밖 — 체결 시 한계 안으로 이동: {joints_text}")
        else:
            logger.info("동기화 관절각이 IK 한계 안으로 복귀")
