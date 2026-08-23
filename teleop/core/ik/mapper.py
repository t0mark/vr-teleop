"""사람 포즈 → IK 목표 매핑: grip 클러치 + 상대 델타 방식의 EE(그리퍼) 트래킹.

동작 방식 (실사용 검증된 oss/custom-teleop과 동일한 구조):
- **grip(squeeze) > 임계값을 쥐고 있는 동안만 추종한다** (클러치)
- engagement 순간의 손 포즈와 로봇 TCP 포즈를 앵커로 잡고, 이후 손의 **델타**만 이식:
    목표 위치 = TCP앵커 + (손 현재 - 손앵커)      [1m = 1m]
    목표 자세 = Δ자세 · TCP앵커자세,  Δ자세 = R_손현재 · R_손앵커⁻¹  (월드 프레임 좌곱)
- 목표가 항상 로봇의 현재 자세에서 연속적으로 시작하므로 VR 진입·트래킹 점프 시
  팔이 급변할 수 없다 (절대 매핑의 진입 폭주를 구조적으로 차단)
- 절대 자세 오프셋 캘리브레이션(자연 자세 앵커)이 필요 없다 — 델타만 쓰므로
- grip을 놓으면 해당 팔은 현재 자세 유지 (solver의 hold)
- **팔꿈치 트래킹**: 체결 중 body-tracking이 잡히면 사람 (어깨 볼→팔꿈치) 단위벡터 × 로봇 상완을
  팔꿈치 목표로 함께 전달 (자세 유도 소프트 코스트, 미검출 시 자동 비활성)

좌표 처리:
- 손 포즈는 로봇 좌표계(webxr_to_robot 상사변환 완료) 입력
- 델타·팔꿈치 방향은 사람 몸통 프레임(어깨선+중력)으로 재표현 → 서 있는 방향 무관
- 조인트 의미: 어깨 볼 = *-arm-upper 원점 (Meta 계층: Shoulder(쇄골)→Scapula→ArmUpper)
  ※ *-shoulder(쇄골)를 어깨로 쓰면 팔꿈치 방향이 편향되어 관절 고착 유발 (겪은 회귀)
"""

import logging

import numpy as np
from scipy.spatial.transform import Rotation as R

from teleop.utils import config

logger = logging.getLogger("ik_mapper")

# 어깨 볼 조인트: arm-upper 우선, shoulder(쇄골)는 폴백
_SHOULDER_BALL = {"left": "left-arm-upper", "right": "right-arm-upper"}
_SHOULDER_FALLBACK = {"left": "left-shoulder", "right": "right-shoulder"}
# 팔꿈치 = arm-lower 원점
_ELBOW = {"left": "left-arm-lower", "right": "right-arm-lower"}


def _get_shoulder(joints: dict, side: str):
    """어깨 볼 조인트를 반환한다 (arm-upper 우선, 없으면 shoulder 폴백, 둘 다 없으면 None)."""
    return joints.get(_SHOULDER_BALL[side]) or joints.get(_SHOULDER_FALLBACK[side])


class TargetMapper:
    """클러치 engagement 상태(손·TCP 앵커)를 관리하고 샘플→IK 목표 변환을 수행한다."""

    def __init__(self, robot_model):
        self._rm = robot_model
        self._robot_upper = robot_model.upper_arm_length()
        self._engaged = {"left": False, "right": False}
        # engagement 앵커: 손(몸통 프레임 pos·rot), TCP(로봇 프레임 pos·rot)
        self._hand_anchor = {"left": None, "right": None}
        self._tcp_anchor = {"left": None, "right": None}

    def map(self, sample: dict, current_fk: dict) -> dict:
        """샘플과 현재 로봇 FK로 양팔 IK 목표를 만든다.

        sample: PoseHub.latest_robot 형식. current_fk: RobotModel.fk_frames(현재 q).
        반환: {side: {wrist_pos, wrist_quat} | None} — None이면 해당 팔 현재 자세 유지.
        """
        hands = sample.get("hands", {})
        joints = sample.get("body", {}).get("joints", {})
        human_chest = self._human_chest(joints)
        R_body = self._human_torso_rotation(joints)

        targets = {}
        for side in ("left", "right"):
            hand = hands.get(side)
            gripping = hand is not None and hand.get("squeeze", 0.0) > config.GRIP_ENGAGE_THRESHOLD
            if not gripping:
                # 클러치 해제: 앵커를 버리고 팔은 현재 자세 유지
                if self._engaged[side]:
                    logger.info(f"클러치 해제({side}) — 현재 자세 유지")
                self._engaged[side] = False
                targets[side] = None
                continue

            # 손 포즈를 몸통 프레임으로 재표현 (서 있는 방향 무관한 델타 계산용)
            p_body = R_body.T @ (np.asarray(hand["pos"]) - human_chest)
            R_hand_body = R_body.T @ R.from_quat(hand["quat"]).as_matrix()

            # engagement 순간: 손·TCP 앵커 캡처 → 이후 델타가 여기서부터 누적
            if not self._engaged[side]:
                self._engaged[side] = True
                self._hand_anchor[side] = (p_body.copy(), R_hand_body.copy())
                tcp_pos, tcp_quat = current_fk[side]["wrist"]
                self._tcp_anchor[side] = (tcp_pos.copy(), R.from_quat(tcp_quat).as_matrix())
                logger.info(f"클러치 engagement({side}): TCP 앵커 {np.round(tcp_pos, 3)}")

            # 델타 이식: 위치는 평행이동, 자세는 월드(몸통) 프레임 델타의 좌곱
            p_anchor, R_anchor = self._hand_anchor[side]
            tcp_p, tcp_R = self._tcp_anchor[side]
            dR = R_hand_body @ R_anchor.T
            targets[side] = {
                "wrist_pos": tcp_p + (p_body - p_anchor),
                "wrist_quat": R.from_matrix(dR @ tcp_R).as_quat(),
                "elbow_vec": self._elbow_vector(joints, side, R_body),
            }
        return targets

    def _elbow_vector(self, joints: dict, side: str, R_body: np.ndarray) -> np.ndarray | None:
        """몸통 프레임 기준 (어깨 볼→팔꿈치) 단위벡터를 로봇 상완 길이로 스케일해 반환한다.

        팔꿈치 자세 유도용 (절대 방향 — 델타 앵커와 무관하게 사람 팔 방향을 그대로 따름).
        조인트 미검출·퇴화 시 None을 반환해 솔버가 팔꿈치 코스트를 끈다.
        """
        sh, el = _get_shoulder(joints, side), joints.get(_ELBOW[side])
        if sh is None or el is None:
            return None
        v = R_body.T @ (np.asarray(el["pos"]) - np.asarray(sh["pos"]))
        norm = np.linalg.norm(v)
        if norm < 1e-6:
            return None
        return v / norm * self._robot_upper

    def disengage_all(self):
        """전체 클러치를 해제한다 (트래킹 두절 시 서비스가 호출). 반환값 없음."""
        for side in ("left", "right"):
            self._engaged[side] = False

    def _human_chest(self, joints: dict) -> np.ndarray:
        """사람 가슴 위치(어깨 볼 중점)를 반환한다. 어깨가 없으면 설정 기본 높이로 대체한다.

        델타 방식에서는 가슴이 몸통 프레임 재표현의 기준점으로만 쓰여, 절대 위치 오차가
        engagement 앵커에 흡수되므로 폴백의 정확도는 중요하지 않다.
        """
        l, r = _get_shoulder(joints, "left"), _get_shoulder(joints, "right")
        if l is not None and r is not None:
            return (np.asarray(l["pos"]) + np.asarray(r["pos"])) / 2.0
        return np.array([0.0, 0.0, config.FALLBACK_CHEST_HEIGHT])

    def _human_torso_rotation(self, joints: dict) -> np.ndarray:
        """사람 몸통 기준 회전(3x3, 열 = [전방, 좌, 상])을 로봇 프레임 기준으로 구성한다.

        y축 = 오른어깨→왼어깨 방향의 수평 성분(사람이 서 있다고 가정하고 중력 z와 직교화),
        z축 = 중력 상방향, x축 = y×z (오른손 규약으로 전방). 어깨가 없거나 어깨선이
        수직에 가까우면 항등을 반환한다 (룸 좌표 그대로 — 델타 방식이라 동작에는 지장 없음).
        """
        l, r = _get_shoulder(joints, "left"), _get_shoulder(joints, "right")
        if l is None or r is None:
            return np.eye(3)

        # 어깨선을 수평면에 투영해 좌축을 얻는다 (기울인 자세에서도 yaw만 반영)
        y = np.asarray(l["pos"]) - np.asarray(r["pos"])
        y[2] = 0.0
        norm = np.linalg.norm(y)
        if norm < 1e-6:
            return np.eye(3)
        y /= norm

        z = np.array([0.0, 0.0, 1.0])
        # x = y × z: y=(0,1,0), z=(0,0,1)일 때 x=(1,0,0) — 로봇 축 규약과 일치
        x = np.cross(y, z)
        return np.column_stack([x, y, z])
