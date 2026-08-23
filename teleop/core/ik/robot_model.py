"""RBY1 로봇 모델: URDF 로드·양팔 축소 모델·FK·시각화 메타데이터를 관리한다.

좌표계: 모든 FK 출력은 로봇 base 프레임 (x=전방, y=좌, z=상, 단위 m).
축소 모델: 양팔 14관절(arm_0-6 x2)만 자유, 나머지(바퀴·토르소·머리·그리퍼)는 ready pose로 고정.
"""

import logging
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation as R

from teleop.utils import config
from teleop.utils.transforms import robot_to_webxr

logger = logging.getLogger("robot_model")

# ready pose (기존 Unity teleop 예제의 Right-A 초기 자세, deg)
READY_TORSO_DEG = [0.0, 45.0, -80.0, 45.0, 0.0, 0.0]
READY_RIGHT_ARM_DEG = [0.0, -15.0, 0.0, -120.0, 0.0, 30.0, 0.0]
READY_LEFT_ARM_DEG = [0.0, 15.0, 0.0, -120.0, 0.0, 30.0, 0.0]
# 머리 고정 자세 (rad): head_0=yaw 0, head_1=pitch. custom-teleop 기본값 0.7은 바닥을 과하게 봐서
# 0.4(약 23°)로 완화 — 카메라가 손 작업 공간과 전방을 함께 담는 각도
READY_HEAD_RAD = [0.0, 0.4]

# IK가 제어하는 관절. 주의: 축소 모델의 실제 q 순서는 pinocchio가 정하며(left 먼저임이 확인됨)
# 이 리스트 순서와 다르다 — 모든 q 접근은 반드시 이름 기반(idx_q 조회)으로 할 것
ARM_JOINT_NAMES = [f"right_arm_{i}" for i in range(7)] + [f"left_arm_{i}" for i in range(7)]

# IK 코스트에 쓰는 프레임: 손목 목표는 그리퍼(ee) 프레임 — 플랜지(link_*_arm_6)를 쓰면
# 사용자 손이 그리퍼가 아니라 15cm 뒤 손목에 매핑되는 위화감이 있음 (실기기 피드백으로 확인).
# ee_*는 fixed joint 체인(플랜지→FT센서→그리퍼)이라 축소 모델에서도 프레임이 유지된다
FRAME_NAMES = {
    "right": {"wrist": "ee_right", "elbow": "link_right_arm_3", "shoulder": "link_right_arm_0"},
    "left": {"wrist": "ee_left", "elbow": "link_left_arm_3", "shoulder": "link_left_arm_0"},
}


def load_fixed_urdf_xml(path) -> str:
    """URDF를 읽어 표준 urdfdom이 거부하는 wheel limit(effort 누락)을 보정한 XML을 반환한다."""
    tree = ET.parse(path)
    root = tree.getroot()
    # effort 없는 limit에 기본값 주입 (해당 관절은 IK에서 잠그므로 값 자체는 무의미)
    for joint in root.iter("joint"):
        limit = joint.find("limit")
        if limit is not None and "effort" not in limit.attrib:
            limit.set("effort", "0")
    return ET.tostring(root, encoding="unicode")


def _origin_to_webxr(origin_el) -> dict:
    """URDF visual origin을 WebXR 기저의 {pos, quat}로 변환해 반환한다 (없으면 identity).

    링크 포즈를 WebXR 좌표계로 상사변환해 보내므로, 링크 로컬 변환(origin)도
    같은 기저 재정의(Cᵀ·T·C)를 거쳐야 브라우저에서 일관되게 합성된다.
    URDF rpy는 고정축 x→y→z(extrinsic) 규약 = scipy 'xyz'.
    """
    if origin_el is None:
        return {"pos": [0.0, 0.0, 0.0], "quat": [0.0, 0.0, 0.0, 1.0]}
    xyz = [float(v) for v in origin_el.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in origin_el.get("rpy", "0 0 0").split()]
    p, q = robot_to_webxr(xyz, R.from_euler("xyz", rpy).as_quat())
    return {"pos": p.tolist(), "quat": q.tolist()}


class RobotModel:
    """RBY1 축소 모델(양팔 14 DOF)과 FK·시각화 메타를 소유한다."""

    def __init__(self, urdf_path=config.RBY1_URDF):
        self._urdf_path = urdf_path
        xml = load_fixed_urdf_xml(urdf_path)
        full_model = pin.buildModelFromXML(xml)

        # ready pose를 전체 모델 기준 벡터로 구성 (잠긴 관절은 이 값으로 고정된다)
        q_ref = pin.neutral(full_model)
        ready_map = dict(
            zip(
                [f"torso_{i}" for i in range(6)] + ARM_JOINT_NAMES + ["head_0", "head_1"],
                np.concatenate([
                    np.deg2rad(READY_TORSO_DEG + READY_RIGHT_ARM_DEG + READY_LEFT_ARM_DEG),
                    READY_HEAD_RAD,
                ]),
            )
        )
        for name, value in ready_map.items():
            jid = full_model.getJointId(name)
            q_ref[full_model.joints[jid].idx_q] = value

        # 양팔 외 전 관절을 잠가 14 DOF 축소 모델 생성 (잠긴 링크의 프레임은 유지됨 → FK 시각화 가능)
        locked_ids = [
            jid for jid in range(1, full_model.njoints)
            if full_model.names[jid] not in ARM_JOINT_NAMES
        ]
        self.model = pin.buildReducedModel(full_model, locked_ids, q_ref)
        self.data = self.model.createData()

        # ready pose에서의 팔 관절각 (IK 초기값·정규화 중심)
        self.q_ready = np.zeros(self.model.nq)
        for name, value in ready_map.items():
            if name in ARM_JOINT_NAMES:
                jid = self.model.getJointId(name)
                self.q_ready[self.model.joints[jid].idx_q] = value

        # IK 코스트용 프레임 id
        self.frame_ids = {
            side: {k: self.model.getFrameId(v) for k, v in frames.items()}
            for side, frames in FRAME_NAMES.items()
        }

        self._visuals = self._parse_visuals(xml)
        self._fingers = self._parse_finger_joints(xml)
        self._log_geometry()

    def _parse_visuals(self, xml: str) -> dict:
        """URDF에서 링크별 visual(메시 파일·origin)을 추출해 반환한다.

        반환: {링크명: {mesh: glb 파일명, origin: {pos, quat(WebXR 기저)}}} — dae는 동명 glb로
        치환 (rby1-sdk meshes/에 전 파일의 glb 대응본 존재 확인됨).
        """
        visuals = {}
        root = ET.fromstring(xml)
        for link in root.iter("link"):
            visual = link.find("visual")
            if visual is None:
                continue
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            filename = mesh.get("filename", "").split("/")[-1].replace(".dae", ".glb")
            visuals[link.get("name")] = {
                "mesh": filename,
                "origin": _origin_to_webxr(visual.find("origin")),
            }
        return visuals

    def _log_geometry(self):
        """모델 로드 결과와 기준 기하(어깨 위치·상완 길이)를 로그로 남긴다."""
        poses = self.fk_frames(self.q_ready)
        r_sh, l_sh = poses["right"]["shoulder"][0], poses["left"]["shoulder"][0]
        logger.info(
            f"축소 모델 로드: nq={self.model.nq}, visual 링크 {len(self._visuals)}개, "
            f"오른어깨 {np.round(r_sh, 3)}, 왼어깨 {np.round(l_sh, 3)}, 상완 {self.upper_arm_length():.3f}m"
        )

    def fk_frames(self, q: np.ndarray) -> dict:
        """관절각 q(14)로 IK 관련 프레임 포즈를 계산해 반환한다.

        반환: {side: {wrist/elbow/shoulder: (pos(3), quat(x,y,z,w))}} — 로봇 base 프레임.
        wrist 위치는 그리퍼 TCP(집는 지점, ee 로컬 GRIPPER_TCP_OFFSET 적용) 기준이다.
        """
        pin.framesForwardKinematics(self.model, self.data, q)
        tcp_offset = np.asarray(config.GRIPPER_TCP_OFFSET)
        out = {}
        for side, ids in self.frame_ids.items():
            out[side] = {}
            for key, fid in ids.items():
                M = self.data.oMf[fid]
                pos = M.translation.copy()
                # 손 목표는 그리퍼 몸체가 아니라 집는 지점에 대응시킨다
                if key == "wrist":
                    pos = pos + M.rotation @ tcp_offset
                out[side][key] = (pos, pin.Quaternion(M.rotation).coeffs().copy())
        return out

    def _parse_finger_joints(self, xml: str) -> list[dict]:
        """URDF에서 그리퍼 손가락(프리즘) 관절 정보를 추출해 반환한다.

        축소 모델에서는 손가락 관절이 잠겨 있어(q=0=닫힘) FK가 못 움직이므로,
        시각화에서 열림 정도(openness)에 따라 수동 합성하기 위한 메타데이터.
        반환: [{child, parent, side, T_origin(4x4), axis(3, 관절 프레임), q_open(열림 시 관절값)}]
        """
        fingers = []
        root = ET.fromstring(xml)
        for joint in root.iter("joint"):
            if joint.get("type") != "prismatic":
                continue
            parent = joint.find("parent").get("link")
            if not parent.startswith("ee_"):
                continue
            origin = joint.find("origin")
            xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
            rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
            T = np.eye(4)
            T[:3, :3] = R.from_euler("xyz", rpy).as_matrix()
            T[:3, 3] = xyz
            limit = joint.find("limit")
            lo, hi = float(limit.get("lower")), float(limit.get("upper"))
            # 열림 방향 관절값: 한계 중 절대값이 큰 쪽 (URDF상 q=0이 닫힘)
            q_open = lo if abs(lo) > abs(hi) else hi
            fingers.append({
                "child": joint.find("child").get("link"),
                "parent": parent,
                "side": "right" if parent.endswith("right") else "left",
                "T_origin": T,
                "axis": np.array([float(v) for v in joint.find("axis").get("xyz").split()]),
                "q_open": q_open,
            })
        return fingers

    def fk_visual_links(self, q: np.ndarray, gripper_open: dict | None = None) -> dict:
        """관절각 q(14)로 visual이 있는 모든 링크의 포즈를 계산해 반환한다.

        gripper_open: {left/right: 열림 정도 0(닫힘)-1(열림)} — 손가락 링크 포즈를 이 값으로 합성.
        반환: {링크명: (pos(3), quat(x,y,z,w))} — 로봇 base 프레임. 브라우저 시각화용.
        """
        pin.framesForwardKinematics(self.model, self.data, q)
        out = {}
        for name in self._visuals:
            fid = self.model.getFrameId(name)
            M = self.data.oMf[fid]
            out[name] = (M.translation.copy(), pin.Quaternion(M.rotation).coeffs().copy())

        # 손가락은 잠긴 관절이라 열림 정도에 따라 수동 합성: T_child = T_ee · T_origin · Trans(axis·q)
        if gripper_open is not None:
            for f in self._fingers:
                if f["child"] not in self._visuals:
                    continue
                openness = float(np.clip(gripper_open.get(f["side"], 0.0), 0.0, 1.0))
                M_ee = self.data.oMf[self.model.getFrameId(f["parent"])]
                T_ee = np.eye(4)
                T_ee[:3, :3] = M_ee.rotation
                T_ee[:3, 3] = M_ee.translation
                T_slide = np.eye(4)
                T_slide[:3, 3] = f["axis"] * (f["q_open"] * openness)
                T = T_ee @ f["T_origin"] @ T_slide
                out[f["child"]] = (T[:3, 3].copy(), R.from_matrix(T[:3, :3]).as_quat())
        return out

    def visual_config(self) -> dict:
        """브라우저 1회 전송용 시각화 메타(링크별 메시 파일·origin)를 반환한다."""
        return self._visuals

    def shoulder_positions(self) -> dict:
        """ready pose 기준 좌·우 어깨(arm_0 원점) 위치를 반환한다 (토르소 고정이라 상수)."""
        poses = self.fk_frames(self.q_ready)
        return {side: poses[side]["shoulder"][0] for side in ("left", "right")}

    def upper_arm_length(self) -> float:
        """ready pose 기준 어깨-팔꿈치 거리(m)를 반환한다 (팔꿈치 방향 코스트의 스케일)."""
        poses = self.fk_frames(self.q_ready)
        return float(np.linalg.norm(poses["right"]["elbow"][0] - poses["right"]["shoulder"][0]))

    def camera_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """머리 장착 카메라의 광학 프레임 포즈 (pos(3), R(3x3))를 로봇 base 프레임으로 반환한다.

        장착 링크(config.CAMERA_MOUNT_LINK)의 FK에 장착 오프셋·광학 축 대응(config.CAMERA_MOUNT_*)을
        합성: p = p_link + R_link·t_mount,  R = R_link·R_mount. 머리·토르소가 잠겨 있어 상수다.
        """
        pin.framesForwardKinematics(self.model, self.data, self.q_ready)
        M = self.data.oMf[self.model.getFrameId(config.CAMERA_MOUNT_LINK)]
        R_mount = np.asarray(config.CAMERA_MOUNT_ROT, dtype=float)
        t_mount = np.asarray(config.CAMERA_MOUNT_POS, dtype=float)
        return M.translation + M.rotation @ t_mount, M.rotation @ R_mount

    def camera_visual_config(self) -> dict:
        """브라우저 카메라 표시용 메타 {link, origin{pos,quat}(WebXR 기저, 링크 로컬), mesh_base, mesh}를 반환한다.

        D455 GLB는 ROS camera_link 프레임(x=전방, y=좌, z=상 — 장착 링크와 같은 축 규약)으로 구워져 있어
        링크 로컬 변환은 평행이동(CAMERA_MOUNT_POS)뿐이다. visual origin과 같은 이유로 WebXR 기저로
        재정의(Cᵀ·T·C)해 보내고, 클라이언트는 로봇 링크 GLB와 동일한 기저 보정 체인으로 붙인다.
        """
        p, q = robot_to_webxr(config.CAMERA_MOUNT_POS, [0.0, 0.0, 0.0, 1.0])
        return {
            "link": config.CAMERA_MOUNT_LINK,
            "origin": {"pos": p.tolist(), "quat": q.tolist()},
            "mesh_base": config.CAMERA_MESH_URL_PREFIX,
            "mesh": config.CAMERA_MESH_FILE,
        }

    def joint_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        """축소 모델의 관절 하한·상한(rad)을 반환한다.

        URDF 한계에 SAFETY_JOINT_LIMITS(실사용 검증된 안전 한계)를 교집합으로 적용한다 —
        URDF 원값은 팔꿈치 완전 신전·손목 역꺾임 등 콜리전 영역 진입을 허용하기 때문.
        """
        lb = self.model.lowerPositionLimit.copy()
        ub = self.model.upperPositionLimit.copy()
        for name, (lo, hi) in config.SAFETY_JOINT_LIMITS.items():
            idx = self.model.joints[self.model.getJointId(name)].idx_q
            lb[idx] = max(lb[idx], lo)
            ub[idx] = min(ub[idx], hi)
        return lb, ub

    def velocity_limits(self) -> np.ndarray:
        """축소 모델의 관절 속도 한계(rad/s, URDF velocity)를 반환한다."""
        return self.model.velocityLimit.copy()

    def joint_names_in_q_order(self) -> list[str]:
        """q 벡터 순서대로 관절 이름을 반환한다 (로봇 브리지의 이름 기반 매핑용)."""
        return [self.model.names[jid] for jid in range(1, self.model.njoints)]
