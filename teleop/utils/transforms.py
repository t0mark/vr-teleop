"""좌표 변환: WebXR 좌표계와 로봇 좌표계 사이의 포즈 변환.

좌표계 규약 (둘 다 오른손 좌표계, 단위 m):
- WebXR: x=우, y=상, -z=전방 (사용자 기준, y-up)
- 로봇(RBY1 base, ROS 규약): x=전방, y=좌, z=상 (z-up)

축 대응 (사용자가 로봇과 같은 방향을 본다고 가정):
    robot_x = -webxr_z,  robot_y = -webxr_x,  robot_z = webxr_y
사용자 정면과 로봇 정면 사이의 yaw 정렬(캘리브레이션)은 IK 단계에서 별도 처리한다.

쿼터니언 규약: (x, y, z, w) — WebXR·scipy 동일.
"""

import numpy as np
from scipy.spatial.transform import Rotation

# WebXR 좌표를 로봇 좌표로 보내는 기저 변환 행렬 (행 = 로봇 축, det=+1)
_R_ROBOT_WEBXR = np.array([
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])

# 업링크 쿼터니언 최소 크기 (이보다 작으면 형식 오류)
_MIN_QUAT_NORM = 1e-6


def pose_to_se3(pos, quat) -> np.ndarray:
    """위치(3)·쿼터니언(x,y,z,w)을 4x4 동차 변환 행렬로 조립해 반환한다."""
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(quat).as_matrix()
    transform[:3, 3] = pos
    return transform


def se3_to_pose(transform: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """4x4 동차 변환 행렬을 (위치(3), 쿼터니언(x,y,z,w))로 분해해 반환한다."""
    return transform[:3, 3].copy(), Rotation.from_matrix(transform[:3, :3]).as_quat()


def webxr_to_robot(pos, quat) -> tuple[np.ndarray, np.ndarray]:
    """WebXR 좌표계의 포즈를 로봇 좌표계로 변환해 (위치(3), 쿼터니언(x,y,z,w))을 반환한다.

    기저 변환 행렬 C에 대해 포즈 행렬은 상사 변환 T_r = C·T_w·C⁻¹ 를 따른다
    (회전부 R_r = C·R_w·Cᵀ, 위치부 p_r = C·p_w).
    """
    basis = _R_ROBOT_WEBXR

    # 위치는 기저 변환, 회전은 상사 변환
    robot_pos = basis @ np.asarray(pos, dtype=float)
    robot_rot_matrix = basis @ Rotation.from_quat(quat).as_matrix() @ basis.T
    return robot_pos, Rotation.from_matrix(robot_rot_matrix).as_quat()


def robot_to_webxr(pos, quat) -> tuple[np.ndarray, np.ndarray]:
    """로봇 좌표계의 포즈를 WebXR 좌표계로 변환해 (위치(3), 쿼터니언(x,y,z,w))을 반환한다.

    webxr_to_robot의 역변환이다 (C가 직교 행렬이므로 C⁻¹ = Cᵀ).
    """
    basis = _R_ROBOT_WEBXR

    # 위치는 역기저 변환, 회전은 역상사 변환
    webxr_pos = basis.T @ np.asarray(pos, dtype=float)
    webxr_rot_matrix = basis.T @ Rotation.from_quat(quat).as_matrix() @ basis
    return webxr_pos, Rotation.from_matrix(webxr_rot_matrix).as_quat()


def robot_points_to_webxr_matrix() -> np.ndarray:
    """로봇 좌표 점을 WebXR 좌표로 보내는 3x3 행렬 Cᵀ를 반환한다 (열 벡터 기준 p_w = Cᵀ·p_r).

    점 N개를 행으로 쌓은 배열 P_r(N×3)은 P_w = P_r @ C 로 변환한다 (반환 행렬의 전치를 오른쪽에 곱한다).
    포즈 단위 변환은 robot_to_webxr를 쓴다.
    """
    return _R_ROBOT_WEBXR.T.copy()


def parse_pose(entry, label: str) -> tuple[np.ndarray, np.ndarray]:
    """포즈 항목 {pos[3], quat[x,y,z,w]}을 검증해 (위치(3), 쿼터니언(4)) 배열로 반환한다 (형식 오류면 ValueError)."""
    if not isinstance(entry, dict):
        raise ValueError(f"{label}: 포즈가 dict가 아님 ({type(entry).__name__})")

    # 숫자 배열 변환
    try:
        pos = np.asarray(entry["pos"], dtype=float)
        quat = np.asarray(entry["quat"], dtype=float)
    except (KeyError, TypeError, ValueError) as e:
        raise ValueError(f"{label}: pos·quat 누락 또는 숫자 아님 ({e!r})") from e

    # 길이·유한성·쿼터니언 크기 검사
    if pos.shape != (3,) or quat.shape != (4,):
        raise ValueError(f"{label}: pos[3]·quat[4] 형식 아님 (pos {pos.shape}, quat {quat.shape})")
    if not (np.all(np.isfinite(pos)) and np.all(np.isfinite(quat))):
        raise ValueError(f"{label}: 유한하지 않은 값 포함")
    if np.linalg.norm(quat) < _MIN_QUAT_NORM:
        raise ValueError(f"{label}: 쿼터니언 크기가 0에 가까움")
    return pos, quat


def webxr_sample_to_robot(sample: dict) -> dict:
    """프론트 업링크 샘플(JSON dict) 전체를 로봇 좌표계로 변환한 사본을 반환한다 (형식 오류면 ValueError).

    입력 구조: 브라우저 공통 샘플 형식 (teleop/utils/inputs.py) — head·body·hands는 None일 수 있다.
    포즈 필드만 변환하고 follow·fingers·command 등 나머지 필드는 그대로 유지한다.
    """
    if not isinstance(sample, dict):
        raise ValueError(f"샘플이 dict가 아님 ({type(sample).__name__})")
    robot_sample = {"t": sample.get("t"), "command": sample.get("command"), "body": {"joints": {}}, "hands": {}, "head": None}

    # 조작자 시점 포즈
    head = sample.get("head")
    if head is not None:
        robot_pos, robot_quat = webxr_to_robot(*parse_pose(head, "head"))
        robot_sample["head"] = {"pos": robot_pos.tolist(), "quat": robot_quat.tolist()}

    # body-tracking 조인트 포즈
    body = sample.get("body")
    if body is not None and not isinstance(body, dict):
        raise ValueError(f"body가 dict가 아님 ({type(body).__name__})")
    joints = (body or {}).get("joints") or {}
    if not isinstance(joints, dict):
        raise ValueError(f"body.joints가 dict가 아님 ({type(joints).__name__})")
    for joint_name, joint in joints.items():
        robot_pos, robot_quat = webxr_to_robot(*parse_pose(joint, f"body.joints.{joint_name}"))
        robot_sample["body"]["joints"][joint_name] = {"pos": robot_pos.tolist(), "quat": robot_quat.tolist()}

    # 손 포즈 (follow·fingers 등 포즈 외 입력값은 원본 유지)
    hands = sample.get("hands") or {}
    if not isinstance(hands, dict):
        raise ValueError(f"hands가 dict가 아님 ({type(hands).__name__})")
    for side, hand in hands.items():
        if hand is None:
            robot_sample["hands"][side] = None
            continue
        robot_pos, robot_quat = webxr_to_robot(*parse_pose(hand, f"hands.{side}"))
        robot_sample["hands"][side] = {**hand, "pos": robot_pos.tolist(), "quat": robot_quat.tolist()}

    return robot_sample
