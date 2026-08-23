"""양팔 IK 솔버: EE(그리퍼) 포즈 + 팔꿈치 방향 트래킹. CasADi 심볼릭 FK 기반 NLP를 warm start로 푼다.

정식화:
    min_q  Σ_arm [ w_pos·|p_tcp(q) - p_target|² + w_rot·|R_ee(q) - R_target|²_F
                   + w_elbow·|(p_elbow - p_shoulder) - v_target|² ]
           + w_reg·|q - q_prev|² + w_ready·|q - q_ready|²
    s.t.   q_lower ≤ q ≤ q_upper (URDF ∩ 안전 한계)

- p_tcp: 그리퍼 TCP(집는 지점, ee 로컬 GRIPPER_TCP_OFFSET) — 손 위치가 여기에 대응된다
- 자세는 회전행렬 Frobenius 오차 (log6는 오차 0에서 미분 NaN이라 배제)
- 팔꿈치 코스트: 어깨→팔꿈치 벡터를 목표(사람 팔 방향×로봇 상완)에 정렬. 가중치를
  파라미터로 받아 목표가 없으면(w=0) 자동 비활성 — 그때는 ready 바이어스가 여유 자유도 해소
"""

import logging

import casadi
import numpy as np
from pinocchio import casadi as cpin

from teleop.utils import config
from teleop.utils.transforms import pose_to_se3

logger = logging.getLogger("ik_solver")


class ArmIkSolver:
    """축소 모델(14 DOF) 기반 NLP를 보유하고 solve 호출 상태(warm start)를 유지한다."""

    def __init__(self, robot_model):
        self._rm = robot_model
        self._q_prev = robot_model.q_ready.copy()
        self._build_nlp()

    def _build_nlp(self):
        """CasADi 심볼릭 FK로 코스트·파라미터를 구성하고 SQP 솔버를 생성한다. 반환값 없음."""
        model = self._rm.model
        cmodel = cpin.Model(model)
        cdata = cmodel.createData()

        cq = casadi.SX.sym("q", model.nq)
        cpin.framesForwardKinematics(cmodel, cdata, cq)

        cost = 0
        params = []
        for side in ("left", "right"):
            ids = self._rm.frame_ids[side]

            # EE 목표: 4x4 동차 행렬 파라미터. 위치는 그리퍼 TCP 기준
            T_target = casadi.SX.sym(f"T_{side}", 4, 4)
            params.append(casadi.vec(T_target))
            M_wrist = cdata.oMf[ids["wrist"]]
            p_tcp = M_wrist.translation + M_wrist.rotation @ casadi.SX(list(config.GRIPPER_TCP_OFFSET))
            cost += config.IK_W_POS * casadi.sumsqr(p_tcp - T_target[:3, 3])
            cost += config.IK_W_ROT * casadi.sumsqr(M_wrist.rotation - T_target[:3, :3])

            # 팔꿈치 목표: 어깨→팔꿈치 벡터 차이. 가중치를 파라미터로 두어 샘플별 on/off
            v_target = casadi.SX.sym(f"v_{side}", 3)
            w_elbow = casadi.SX.sym(f"w_{side}")
            params += [v_target, w_elbow]
            elbow_vec = cdata.oMf[ids["elbow"]].translation - cdata.oMf[ids["shoulder"]].translation
            cost += w_elbow * casadi.sumsqr(elbow_vec - v_target)

        # 이전 해 정규화(연속성) + ready pose 바이어스(여유 자유도 해소)
        q_prev = casadi.SX.sym("q_prev", model.nq)
        params.append(q_prev)
        cost += config.IK_W_REG * casadi.sumsqr(cq - q_prev)
        cost += config.IK_W_READY * casadi.sumsqr(cq - casadi.SX(self._rm.q_ready.tolist()))

        nlp = {"x": cq, "p": casadi.vertcat(*params), "f": cost}
        # sqpmethod+qrqp: warm start 추종에서 ipopt(30ms대) 대비 잔차 동일·솔브 1ms 미만.
        # convexify_strategy 필수 — exact Hessian이 비볼록이라 없으면 QP가 발산한다
        opts = {
            "qpsol": "qrqp",
            "max_iter": config.IK_MAX_ITER,
            "convexify_strategy": "eigen-reflect",
            "print_time": 0,
            "print_iteration": False,
            "print_status": False,
            "print_header": False,
            "error_on_fail": False,
            "qpsol_options": {"print_iter": False, "print_header": False, "print_info": False, "error_on_fail": False},
        }
        self._solver = casadi.nlpsol("ik", "sqpmethod", nlp, opts)
        self._lb, self._ub = self._rm.joint_bounds()
        # 관절별 속도 상한: URDF 한계의 일정 비율 + 전역 상한 (하드웨어 한계 초과 방지)
        self._vel_limit = np.minimum(
            self._rm.velocity_limits() * config.IK_VEL_FRACTION, config.IK_MAX_JOINT_VEL
        )
        logger.info(f"IK NLP 구성 완료: nq={model.nq}, sqpmethod max_iter={config.IK_MAX_ITER}")

    def solve(self, targets: dict, dt: float | None = None) -> np.ndarray:
        """목표 dict로 관절각을 계산해 반환한다 (실패 시 이전 해 유지).

        targets: {side: {wrist_pos(3), wrist_quat(4), elbow_vec(3)|None} | None} — 로봇 base 프레임.
        wrist는 TCP 목표, elbow_vec은 어깨 기준 팔꿈치 목표 벡터(없으면 팔꿈치 코스트 비활성).
        side가 None이면(클러치 미체결) 해당 팔은 현재 자세를 목표로 삼아 유지한다.
        dt: 이전 solve 후 경과 시간(초). 관절 속도 제한의 스텝 크기로 사용 (기본: IK 주기).
        """
        # 미체결 팔의 유지 목표로 쓸 현재 손목 포즈 (이전 해 기준 FK)
        hold_fk = self._rm.fk_frames(self._q_prev)

        p = []
        for side in ("left", "right"):
            t = targets[side]
            if t is None:
                pos, quat = hold_fk[side]["wrist"]
                t = {"wrist_pos": pos, "wrist_quat": quat}
            p.append(pose_to_se3(t["wrist_pos"], t["wrist_quat"]).reshape(-1, order="F"))
            if t.get("elbow_vec") is not None:
                p.append(np.asarray(t["elbow_vec"], dtype=float))
                p.append([config.IK_W_ELBOW])
            else:
                p.append(np.zeros(3))
                p.append([0.0])
        p.append(self._q_prev)

        try:
            sol = self._solver(
                x0=self._q_prev,
                p=np.concatenate([np.asarray(x, dtype=float).ravel() for x in p]),
                lbx=self._lb,
                ubx=self._ub,
            )
            q = np.asarray(sol["x"]).ravel()
        except RuntimeError as e:
            # 솔버 예외 시 로봇을 멈추는 대신 이전 해를 유지 (제어 연속성)
            logger.warning(f"IK 솔버 실패, 이전 해 유지: {e}")
            return self._q_prev.copy()

        # QP 실패 등으로 비정상 값이 나오면 버린다 (error_on_fail=False의 방어)
        if not np.all(np.isfinite(q)):
            logger.warning("IK 결과 비정상(NaN/inf), 이전 해 유지")
            return self._q_prev.copy()

        # 급격한 점프 차단: 관절별 속도 상한 × 실제 경과 시간으로 스텝 제한
        # (고정 스텝이면 solve가 느려질 때 속도 상한까지 같이 떨어져 지연이 커진다)
        max_step = self._vel_limit * (dt if dt is not None else 1.0 / config.IK_RATE_HZ)
        dq = np.clip(q - self._q_prev, -max_step, max_step)
        q = self._q_prev + dq

        self._q_prev = q
        return q.copy()

    def reset(self):
        """warm start 상태를 ready pose로 되돌린다. 반환값 없음."""
        self._q_prev = self._rm.q_ready.copy()

    def sync(self, q: np.ndarray):
        """warm start 상태를 외부 관절각으로 맞춘다 (ready 복귀 등 외부 궤적 후 호출). 반환값 없음."""
        self._q_prev = np.asarray(q, dtype=float).copy()
