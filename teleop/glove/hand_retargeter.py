"""손 리타게팅: 조작자 손 키포인트 21개를 dex-retargeting(DexPilot)으로 Inspire 손 관절각으로 바꾸고 굽힘 6값으로 정규화한다.

입력 키포인트 순서 (MediaPipe 규약, 위치 m, 오른손 좌표계면 어느 프레임이든 가능):
    0 손목, 1-4 엄지(CMC·MCP·IP·끝), 5-8 검지(MCP·PIP·DIP·끝), 9-12 중지, 13-16 약지, 17-20 새끼
처리:
    1. 손목·검지 MCP·중지 MCP로 손목 프레임을 추정해 MANO 규약(x 손바닥 법선, y 검지 쪽, z 손가락 방향)으로 옮긴다
       (dex-retargeting 공식 예제와 같은 방식 — 로봇 손 URDF 프레임도 같은 규약)
    2. DexPilot이 손목→손끝·손끝↔손끝 벡터를 로봇 손 FK와 맞추는 관절각을 푼다 (손끝 사이 관계 보존 → 집기 정확도)
    3. 구동 관절 6개를 관절 한계로 0-1 정규화해 공통 형식 [little, ring, middle, index, thumb_bend, thumb_rotate]으로 낸다
       (0 = 펼침·엄지 바깥, 1 = 굽힘·엄지 손바닥 쪽 — Inspire 레지스터 변환은 inspire_hand.py가 한다)
"""

import logging

import numpy as np
import pinocchio as pin
from dex_retargeting.retargeting_config import RetargetingConfig

from teleop.utils import config

logger = logging.getLogger("hand_retargeter")

# 손목 프레임 → MANO 규약 회전 (dex-retargeting 예제 single_hand_detector.py와 같은 값)
_OPERATOR2MANO = {
    "right": np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float),
    "left": np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]], dtype=float),
}

# 공통 형식 손가락 순서에 대응하는 Inspire 구동 관절 (dex-urdf 이름)
FINGER_JOINT_NAMES = (
    "pinky_proximal_joint",
    "ring_proximal_joint",
    "middle_proximal_joint",
    "index_proximal_joint",
    "thumb_proximal_pitch_joint",
    "thumb_proximal_yaw_joint",
)

# 손끝 링크 (엄지·검지·중지·약지·새끼)와 손목 링크 (dex-urdf 이름)
_FINGER_TIP_LINK_NAMES = ["thumb_tip", "index_tip", "middle_tip", "ring_tip", "pinky_tip"]
_WRIST_LINK_NAME = "base"

# 키포인트 개수
KEYPOINT_COUNT = 21


def wrist_frame(keypoints: np.ndarray) -> np.ndarray:
    """키포인트(21,3)에서 손목 프레임 회전(3x3, 열 = 손목 방향·손바닥 법선·검지 쪽)을 추정해 반환한다."""
    points = keypoints[[0, 5, 9], :]

    # 중지 MCP → 손목 방향
    wrist_direction = points[0] - points[2]

    # 손목·검지 MCP·중지 MCP 평면의 법선 (SVD)
    centered = points - np.mean(points, axis=0, keepdims=True)
    _, _, vh = np.linalg.svd(centered)
    normal = vh[2, :]

    # 그람-슈미트 직교화
    x_axis = wrist_direction - np.dot(wrist_direction, normal) * normal
    x_axis = x_axis / np.linalg.norm(x_axis)
    z_axis = np.cross(x_axis, normal)

    # z축이 검지 쪽(중지 MCP → 검지 MCP)을 향하도록 법선 부호 결정
    if np.dot(z_axis, points[1] - points[2]) < 0:
        normal = -normal
        z_axis = -z_axis
    return np.stack([x_axis, normal, z_axis], axis=1)


class HandRetargeter:
    """한 손의 dex-retargeting 최적화기(이전 해·저역 통과 상태)와 구동 관절 정규화 범위를 소유한다."""

    def __init__(self, side: str):
        self._side = side

        # DexPilot 리타게팅 구성 (dex-retargeting 공식 Inspire 설정과 같은 관절·링크)
        urdf_path = config.HAND_RETARGET_URDF_DIR / f"inspire_hand_{side}.urdf"
        retargeting_config = RetargetingConfig.from_dict({
            "type": "DexPilot",
            "urdf_path": str(urdf_path),
            "target_joint_names": list(FINGER_JOINT_NAMES),
            "wrist_link_name": _WRIST_LINK_NAME,
            "finger_tip_link_names": _FINGER_TIP_LINK_NAMES,
            "scaling_factor": config.HAND_RETARGET_SCALING,
            "low_pass_alpha": config.HAND_RETARGET_LOW_PASS_ALPHA,
        })
        self._retargeting = retargeting_config.build()
        self._human_indices = self._retargeting.optimizer.target_link_human_indices

        # 결과 관절 벡터에서 구동 관절 6개의 위치와 정규화 범위 (URDF 한계)
        joint_names = self._retargeting.joint_names
        self._finger_joint_indices = [joint_names.index(name) for name in FINGER_JOINT_NAMES]
        hand_model = pin.buildModelFromUrdf(str(urdf_path))
        self._joint_lower = np.array([
            hand_model.lowerPositionLimit[hand_model.joints[hand_model.getJointId(name)].idx_q] for name in FINGER_JOINT_NAMES
        ])
        self._joint_upper = np.array([
            hand_model.upperPositionLimit[hand_model.joints[hand_model.getJointId(name)].idx_q] for name in FINGER_JOINT_NAMES
        ])
        logger.info(f"손 리타게팅({side}) 구성: DexPilot, {urdf_path.name}, 배율 {config.HAND_RETARGET_SCALING}")

    def retarget(self, keypoints: np.ndarray) -> list[float]:
        """키포인트(21,3)를 공통 형식 굽힘 6값(0-1)으로 바꿔 반환한다."""
        # 손목 기준 MANO 규약 좌표로 변환
        relative = keypoints - keypoints[0]
        mano_keypoints = relative @ wrist_frame(keypoints) @ _OPERATOR2MANO[self._side]

        # DexPilot 입력 벡터(손목→손끝·손끝↔손끝)로 관절각 풀기
        reference = mano_keypoints[self._human_indices[1, :]] - mano_keypoints[self._human_indices[0, :]]
        joint_positions = self._retargeting.retarget(reference)

        # 구동 관절을 관절 한계로 0-1 정규화
        finger_positions = joint_positions[self._finger_joint_indices]
        curl = (finger_positions - self._joint_lower) / (self._joint_upper - self._joint_lower)
        return np.clip(curl, 0.0, 1.0).tolist()

    def reset(self):
        """이전 해·저역 통과 상태를 초기화한다 (장갑 재접속·손 미수신 후 재개 시). 반환값 없음."""
        self._retargeting.reset()
