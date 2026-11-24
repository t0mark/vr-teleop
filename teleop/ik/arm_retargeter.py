"""팔 리타게팅: follow 클러치 + 상대 델타 방식으로 조작자 손 포즈를 로봇 손 TCP 목표로 바꾼다.

동작 방식:
- **공통 샘플의 follow가 True인 동안만 추종한다** (클러치 — VR 손 추적은 추적 중 항상, 수동 조작은 grip을 켠 동안)
- engagement 순간의 손 포즈와 로봇 TCP 포즈를 앵커로 잡고, 이후 손의 **델타**만 이식:
    목표 위치 = TCP앵커 + R_몸통앵커ᵀ·(손 현재 - 손앵커)      [1m = 1m]
    목표 자세 = Δ자세 · TCP앵커자세,  Δ자세 = R_몸통앵커ᵀ·(R_손현재 · R_손앵커⁻¹)·R_몸통앵커
- 목표가 항상 로봇의 현재 자세에서 연속적으로 시작한다
- follow가 꺼지거나 손 포즈가 비정상이면 해당 팔은 현재 자세 유지 (solver의 hold)
- 손 위치·자세만 목표로 넘긴다 — 팔 자세(팔꿈치)는 IK가 ready 자세 쪽으로 정한다

좌표 처리:
- 손 포즈는 로봇 좌표계(webxr_to_robot 상사변환 완료) 입력
- 손 델타는 룸(트래킹 공간) 기준으로 잰다
- 델타의 방향은 engagement 순간의 사람 몸통 yaw(어깨선+중력)로 재표현하고, 몸통 프레임은 체결 동안 고정한다
- 조인트 의미: 어깨 볼 = *-arm-upper 원점
"""

import logging
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

from teleop.utils import config
from teleop.utils.inputs import hand_follow, hand_pose

logger = logging.getLogger("arm_retargeter")

# 어깨 볼 = arm-upper 원점
_SHOULDER_BALL = {"left": "left-arm-upper", "right": "right-arm-upper"}


def _joint_position(joints: dict, name: str) -> np.ndarray | None:
    """body 조인트의 위치(3)를 반환한다 (없거나 형식·값이 비정상이면 None)."""
    joint = joints.get(name)
    if not isinstance(joint, dict):
        return None

    # 위치 배열 변환 후 유한값 검사
    try:
        position = np.asarray(joint["pos"], dtype=float).reshape(3)
    except (KeyError, TypeError, ValueError):
        return None
    if not np.all(np.isfinite(position)):
        return None
    return position


def _human_torso_rotation(joints: dict) -> np.ndarray | None:
    """사람 몸통 기준 회전(3x3, 열 = [전방, 좌, 상])을 로봇 프레임 기준으로 구성해 반환한다.

    좌축 = 오른어깨→왼어깨 방향의 수평 성분, 상축 = 중력 상방향, 전방축 = 좌축×상축.
    어깨가 없거나 어깨선이 수직에 가까우면 None을 반환한다.
    """
    left_shoulder = _joint_position(joints, _SHOULDER_BALL["left"])
    right_shoulder = _joint_position(joints, _SHOULDER_BALL["right"])
    if left_shoulder is None or right_shoulder is None:
        return None

    # 어깨선을 수평면에 투영해 좌축을 얻는다
    left_axis = left_shoulder - right_shoulder
    left_axis[2] = 0.0
    norm = np.linalg.norm(left_axis)
    if norm < 1e-6:
        return None
    left_axis /= norm

    # 상축은 중력 상방향, 전방축은 좌축 × 상축
    up_axis = np.array([0.0, 0.0, 1.0])
    forward_axis = np.cross(left_axis, up_axis)
    return np.column_stack([forward_axis, left_axis, up_axis])


class ArmRetargeter:
    """클러치 engagement 상태(손·TCP·몸통 앵커)를 관리하고 샘플→손 TCP 목표 변환을 수행한다."""

    def __init__(self):
        self._engaged = {"left": False, "right": False}

        # engagement 앵커: 손(룸 프레임 pos·rot), TCP(로봇 프레임 pos·rot), 몸통 yaw 회전(3x3)
        self._hand_anchor = {"left": None, "right": None}
        self._tcp_anchor = {"left": None, "right": None}
        self._body_anchor = {"left": None, "right": None}

        # 경고 종류별 누적 횟수와 마지막 경고 시각 (경고 요약용)
        self._warn_counts = {}
        self._warn_times = {}

    def map(self, sample: dict, current_fk: dict) -> dict:
        """샘플과 현재 로봇 FK로 양팔 손 TCP 목표를 만든다.

        sample: RtcHub.latest_robot 형식. current_fk: RobotModel.fk_frames(현재 q).
        반환: {side: {wrist_pos, wrist_quat} | None} — None이면 해당 팔 현재 자세 유지.
        """
        joints = (sample.get("body") or {}).get("joints", {})
        torso_rotation = _human_torso_rotation(joints)

        targets = {}
        for side in ("left", "right"):
            pose = hand_pose(sample, side)
            following = pose is not None and hand_follow(sample, side)

            # 손 포즈 검사: 추종 중이 아니거나 비정상이면 클러치를 해제하고 현재 자세 유지
            hand = self._parse_hand_pose(side, pose) if following else None
            if hand is None:
                self._release(side, "추종 해제")
                targets[side] = None
                continue
            hand_pos, hand_rot = hand

            # engagement 순간: 손·TCP·몸통 앵커 캡처
            if not self._engaged[side]:
                self._engaged[side] = True
                self._hand_anchor[side] = (hand_pos.copy(), hand_rot.copy())
                self._body_anchor[side] = torso_rotation.copy() if torso_rotation is not None else np.eye(3)
                tcp_pos, tcp_quat = current_fk[side]["wrist"]
                self._tcp_anchor[side] = (tcp_pos.copy(), R.from_quat(tcp_quat).as_matrix())
                logger.info(f"클러치 engagement({side}): TCP 앵커 {np.round(tcp_pos, 3)}")

            # 델타 이식: 룸 프레임 손 델타를 앵커 몸통 프레임으로 재표현해 TCP 앵커에 적용
            hand_anchor_pos, hand_anchor_rot = self._hand_anchor[side]
            tcp_anchor_pos, tcp_anchor_rot = self._tcp_anchor[side]
            body_anchor_rot = self._body_anchor[side]
            hand_delta_pos = body_anchor_rot.T @ (hand_pos - hand_anchor_pos)
            hand_delta_rot = body_anchor_rot.T @ (hand_rot @ hand_anchor_rot.T) @ body_anchor_rot
            targets[side] = {
                "wrist_pos": tcp_anchor_pos + config.ARM_SCALE * hand_delta_pos,
                "wrist_quat": R.from_matrix(hand_delta_rot @ tcp_anchor_rot).as_quat(),
            }
        return targets

    def _parse_hand_pose(self, side: str, pose: tuple | None) -> tuple[np.ndarray, np.ndarray] | None:
        """손 포즈를 (위치(3), 회전행렬(3x3))로 변환해 반환한다 (없거나 형식·값이 비정상이면 None)."""
        if pose is None:
            return None

        # 위치(3)·쿼터니언(4) 배열 변환
        try:
            hand_pos = np.asarray(pose[0], dtype=float).reshape(3)
            hand_quat = np.asarray(pose[1], dtype=float).reshape(4)
        except (TypeError, ValueError, IndexError):
            self._warn_throttled(f"hand_format_{side}", f"손 포즈 형식 오류({side}), 추종 해제")
            return None

        # 유한값·쿼터니언 노름 검사
        pose_finite = np.all(np.isfinite(hand_pos)) and np.all(np.isfinite(hand_quat))
        if not pose_finite or np.linalg.norm(hand_quat) < 1e-6:
            self._warn_throttled(f"hand_value_{side}", f"손 포즈 비정상(NaN/inf/영 쿼터니언, {side}), 추종 해제")
            return None
        return hand_pos, R.from_quat(hand_quat).as_matrix()

    def disengage_all(self):
        """전체 클러치를 해제한다 (트래킹 두절 시 서비스가 호출). 반환값 없음."""
        for side in ("left", "right"):
            self._release(side, "전체 해제")

    def _release(self, side: str, reason: str):
        """해당 팔의 클러치를 해제하고 앵커를 버린다 (체결 상태였을 때만 로그). 반환값 없음."""
        if not self._engaged[side]:
            return
        self._engaged[side] = False
        self._hand_anchor[side] = None
        self._tcp_anchor[side] = None
        self._body_anchor[side] = None
        logger.info(f"클러치 해제({side}, {reason}) — 현재 자세 유지")

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
