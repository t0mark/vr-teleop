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
from scipy.spatial.transform import Rotation as R

# 로봇 좌표 = _R_ROBOT_WEBXR @ WebXR 좌표. 위 축 대응을 행렬로 옮긴 것 (행 = 로봇 축, det=+1)
_R_ROBOT_WEBXR = np.array([
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])


def pose_to_se3(pos, quat) -> np.ndarray:
    """위치(3)·쿼터니언(x,y,z,w)을 4x4 동차 변환 행렬로 조립해 반환한다."""
    T = np.eye(4)
    T[:3, :3] = R.from_quat(quat).as_matrix()
    T[:3, 3] = pos
    return T


def se3_to_pose(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """4x4 동차 변환 행렬을 (위치(3), 쿼터니언(x,y,z,w))로 분해해 반환한다."""
    return T[:3, 3].copy(), R.from_matrix(T[:3, :3]).as_quat()


def webxr_to_robot(pos, quat) -> tuple[np.ndarray, np.ndarray]:
    """WebXR 좌표계의 포즈를 로봇 좌표계로 변환해 (위치(3), 쿼터니언(x,y,z,w))을 반환한다.

    유도: 좌표계 기저 변환 C에 대해 포즈 행렬은 상사 변환 T_r = C·T_w·C⁻¹ 를 따른다
    (회전부 R_r = C·R_w·Cᵀ, 위치부 p_r = C·p_w).
    """
    C = _R_ROBOT_WEBXR
    p_r = C @ np.asarray(pos, dtype=float)
    R_r = C @ R.from_quat(quat).as_matrix() @ C.T
    return p_r, R.from_matrix(R_r).as_quat()


def robot_to_webxr(pos, quat) -> tuple[np.ndarray, np.ndarray]:
    """로봇 좌표계의 포즈를 WebXR 좌표계로 변환해 (위치(3), 쿼터니언(x,y,z,w))을 반환한다.

    webxr_to_robot의 역변환: C가 직교 행렬이므로 C⁻¹ = Cᵀ.
    """
    C = _R_ROBOT_WEBXR
    p_w = C.T @ np.asarray(pos, dtype=float)
    R_w = C.T @ R.from_quat(quat).as_matrix() @ C
    return p_w, R.from_matrix(R_w).as_quat()


def robot_points_to_webxr_matrix() -> np.ndarray:
    """로봇 좌표 점 p_r을 WebXR 좌표로 보내는 3x3 행렬(Cᵀ)을 반환한다.

    대량의 점(포인트클라우드)을 한 번의 행렬곱으로 변환할 때 사용: P_w = P_r @ Cᵀᵀ = P_r @ C
    (행 벡터 기준). 포즈 단위 변환은 robot_to_webxr를 쓴다.
    """
    return _R_ROBOT_WEBXR.T.copy()


def webxr_sample_to_robot(sample: dict) -> dict:
    """프론트 업링크 샘플(JSON dict) 전체를 로봇 좌표계로 변환한 사본을 반환한다.

    입력 구조: { body: {joints: {이름: {pos, quat}}}, hands: {left/right: {pos, quat, ...}} }
    포즈 필드만 변환하고 트리거·버튼 등 나머지 필드는 그대로 유지한다.
    """
    out = {"t": sample.get("t"), "body": {"joints": {}}, "hands": {}}

    for name, j in sample.get("body", {}).get("joints", {}).items():
        p, q = webxr_to_robot(j["pos"], j["quat"])
        out["body"]["joints"][name] = {"pos": p.tolist(), "quat": q.tolist()}

    for hand, c in sample.get("hands", {}).items():
        if c is None:
            out["hands"][hand] = None
            continue
        p, q = webxr_to_robot(c["pos"], c["quat"])
        # 포즈 외 입력값(trigger 등)은 좌표계와 무관하므로 원본 유지
        out["hands"][hand] = {**c, "pos": p.tolist(), "quat": q.tolist()}

    return out
