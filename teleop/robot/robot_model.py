"""RBY1 로봇 모델: URDF 로드, IK용 축소 모델, 시각화용 전체 모델 FK를 관리한다.

좌표계: 모든 FK 출력은 로봇 base 프레임 (x=전방, y=좌, z=상, 단위 m).

모델 두 벌을 함께 들고 있다.
- 전체 모델(full): URDF 그대로(nq=30). **시각화·카메라 외부 파라미터는 전부 이쪽**을 쓴다.
  실로봇에서 읽은 관절값(토르소·머리 포함)을 그대로 반영해야 하기 때문이다.
- 축소 모델(reduced): 양팔 14관절만 자유, 나머지는 ready pose 고정. IK 최적화 전용이다.

손: URDF의 그리퍼(ee_* 몸체·프리즘 손가락)를 떼고 Inspire RH56E2 손 URDF 조각(assets/inspire_hand)을 붙인다.
손가락 굽힘 6값은 hand_joint_positions()로 손 관절각(손가락 마디·엄지)으로 바꿔 시각화 FK에 넣는다.

관절값 입출력은 항상 {관절명: 각도(rad)} 사전으로 주고받는다. pinocchio의 q 순서는
모델 구성에 따라 달라지고(축소 모델은 left가 먼저) 연속 관절(바퀴)은 q를 2칸(cos·sin) 차지하므로,
인덱스로 다루면 좌우가 뒤바뀌는 함정이 있다.
"""

import logging
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation as R

from teleop.utils import config
from teleop.utils.transforms import robot_to_webxr

logger = logging.getLogger("robot_model")

# ready pose (deg): 토르소·오른팔·왼팔
READY_TORSO_DEG = [0.0, 45.0, -80.0, 45.0, 0.0, 0.0]
READY_RIGHT_ARM_DEG = [0.0, -15.0, 0.0, -120.0, 0.0, 30.0, 0.0]
READY_LEFT_ARM_DEG = [0.0, 15.0, 0.0, -120.0, 0.0, 30.0, 0.0]
# ready pose 머리 (rad): head_0=yaw, head_1=pitch(약 23° 아래)
READY_HEAD_RAD = [0.0, 0.4]

# IK가 제어하는 관절
ARM_JOINT_NAMES = [f"right_arm_{i}" for i in range(7)] + [f"left_arm_{i}" for i in range(7)]

# 손 장착(ee) 프레임과, 여기에 HAND_TCP_OFFSET을 더해 축소 모델에 추가하는 손 TCP(파지 중심) 프레임
EE_FRAME_NAMES = {"right": "ee_right", "left": "ee_left"}
TCP_FRAME_NAMES = {"right": "tcp_right", "left": "tcp_left"}

# IK·매핑에 쓰는 프레임: 손 목표는 손 TCP 프레임, 팔꿈치·어깨는 팔 링크
FRAME_NAMES = {
    "right": {"wrist": TCP_FRAME_NAMES["right"], "elbow": "link_right_arm_3", "shoulder": "link_right_arm_0"},
    "left": {"wrist": TCP_FRAME_NAMES["left"], "elbow": "link_left_arm_3", "shoulder": "link_left_arm_0"},
}


def load_fixed_urdf_xml(path) -> str:
    """URDF를 읽어 보정한 XML을 반환한다: wheel limit(effort 누락) 보정 + 그리퍼를 Inspire 손으로 교체."""
    tree = ET.parse(path)
    root = tree.getroot()

    # effort가 없는 limit에 기본값 0을 넣는다
    for joint in root.iter("joint"):
        limit = joint.find("limit")
        if limit is not None and "effort" not in limit.attrib:
            limit.set("effort", "0")

    _attach_inspire_hands(root)
    return ET.tostring(root, encoding="unicode")


def _attach_inspire_hands(root: ET.Element):
    """그리퍼 손가락 관절·링크와 그리퍼 몸체 메시를 떼고 손 URDF 조각을 붙인다. 반환값 없음 (root 수정).

    손 메시 경로는 브라우저가 그대로 쓰도록 URL(HAND_MESH_URL_PREFIX)로 바꿔 둔다.
    """
    # 그리퍼 손가락: ee_*에 붙은 프리즘 관절과 그 자식 링크
    finger_links = set()
    for joint in list(root.findall("joint")):
        if joint.get("type") == "prismatic" and joint.find("parent").get("link").startswith("ee_"):
            finger_links.add(joint.find("child").get("link"))
            root.remove(joint)

    # 그리퍼 손가락 링크 제거 + ee 링크의 그리퍼 몸체 메시 제거 (링크·프레임은 유지)
    for link in list(root.findall("link")):
        if link.get("name") in finger_links:
            root.remove(link)
        elif link.get("name") in ("ee_right", "ee_left"):
            for visual in link.findall("visual"):
                link.remove(visual)

    # 손 URDF 조각 (장착 관절 parent = ee_{side})
    for side in ("right", "left"):
        fragment = ET.parse(config.HAND_MODEL_DIR / f"inspire_hand_{side}.urdf").getroot()
        for element in fragment:
            for mesh in element.iter("mesh"):
                mesh.set("filename", config.HAND_MESH_URL_PREFIX + mesh.get("filename").split("/")[-1])
            root.append(element)


def _origin_to_webxr(origin_el) -> dict:
    """URDF visual origin을 WebXR 기저의 {pos, quat}로 변환해 반환한다 (없으면 identity).

    링크 포즈를 WebXR 좌표계로 상사변환해 보내므로, 링크 로컬 변환(origin)도
    같은 기저 재정의(Cᵀ·T·C)를 거쳐야 브라우저에서 일관되게 합성된다.
    URDF rpy는 고정축 x→y→z(extrinsic) 규약 = scipy 'xyz'.
    """
    if origin_el is None:
        return {"pos": [0.0, 0.0, 0.0], "quat": [0.0, 0.0, 0.0, 1.0]}
    xyz = [float(token) for token in origin_el.get("xyz", "0 0 0").split()]
    rpy = [float(token) for token in origin_el.get("rpy", "0 0 0").split()]
    webxr_pos, webxr_quat = robot_to_webxr(xyz, R.from_euler("xyz", rpy).as_quat())
    return {"pos": webxr_pos.tolist(), "quat": webxr_quat.tolist()}


class RobotModel:
    """RBY1 전체 모델(시각화)과 축소 모델(IK), 그리고 두 모델의 FK를 소유한다."""

    def __init__(self, urdf_path=config.RBY1_URDF):
        self._urdf_path = urdf_path
        xml = load_fixed_urdf_xml(urdf_path)

        # 전체 모델: 시각화·카메라 외부 파라미터용. 실로봇 관절값을 그대로 반영한다
        self._full_model = pin.buildModelFromXML(xml)
        self._full_data = self._full_model.createData()

        # ready pose 관절값 사전과 전체 모델 q
        ready_map = dict(
            zip(
                [f"torso_{i}" for i in range(6)] + ARM_JOINT_NAMES + ["head_0", "head_1"],
                np.concatenate([
                    np.deg2rad(READY_TORSO_DEG + READY_RIGHT_ARM_DEG + READY_LEFT_ARM_DEG),
                    READY_HEAD_RAD,
                ]),
            )
        )
        self._ready_map = ready_map
        self._q_full_ready = self._build_q_full(ready_map)

        # 축소 모델: 양팔 외 전 관절을 ready로 잠근 14 DOF (잠긴 링크의 프레임은 유지됨)
        locked_ids = [
            joint_id for joint_id in range(1, self._full_model.njoints)
            if self._full_model.names[joint_id] not in ARM_JOINT_NAMES
        ]
        self._model = pin.buildReducedModel(self._full_model, locked_ids, self._q_full_ready)
        self._add_tcp_frames()
        self._data = self._model.createData()

        # ready pose에서의 팔 관절각 (IK 초기값·정규화 중심)
        self._q_ready = np.zeros(self._model.nq)
        for name in ARM_JOINT_NAMES:
            self._q_ready[self._model.joints[self._model.getJointId(name)].idx_q] = ready_map[name]

        # IK 코스트용 프레임 id
        self._frame_ids = {
            side: {key: self._model.getFrameId(frame_name) for key, frame_name in frames.items()}
            for side, frames in FRAME_NAMES.items()
        }

        # 시각화 메타·자기 몸 마스크용 메시와 visual 링크 프레임 id
        self._visuals = self._parse_visuals(xml)
        self._visual_meshes = self._parse_visual_meshes(xml)
        self._visual_frame_ids = {name: self._full_model.getFrameId(name) for name in self._visuals}
        self._log_geometry()

    def _add_tcp_frames(self):
        """축소 모델의 손 장착(ee) 프레임에 HAND_TCP_OFFSET을 더한 손 TCP 프레임을 추가한다. 반환값 없음.

        TCP 프레임은 ee 프레임과 같은 자세이고 위치만 파지 중심으로 옮긴 것이다 (data 생성 전에 호출).
        """
        tcp_offset = pin.SE3(np.eye(3), np.asarray(config.HAND_TCP_OFFSET, dtype=float))
        for side, tcp_frame_name in TCP_FRAME_NAMES.items():
            ee_frame_id = self._model.getFrameId(EE_FRAME_NAMES[side])
            ee_frame = self._model.frames[ee_frame_id]
            self._model.addFrame(pin.Frame(
                tcp_frame_name, ee_frame.parentJoint, ee_frame_id, ee_frame.placement * tcp_offset, pin.FrameType.OP_FRAME,
            ))

    # ── 속성 ────────────────────────────────────────────────────────────────

    @property
    def tcp_frame_names(self) -> dict[str, str]:
        """축소 모델의 손 TCP 프레임 이름 {side: 프레임명}을 반환한다 (IK 손 작업 대상)."""
        return dict(TCP_FRAME_NAMES)

    @property
    def full_model(self):
        """전체 모델(pinocchio Model)을 반환한다."""
        return self._full_model

    @property
    def full_data(self):
        """전체 모델 FK 결과(pinocchio Data)를 반환한다."""
        return self._full_data

    @property
    def model(self):
        """축소 모델(양팔 14 DOF, pinocchio Model)을 반환한다."""
        return self._model

    @property
    def data(self):
        """축소 모델 FK 결과(pinocchio Data)를 반환한다."""
        return self._data

    @property
    def q_ready(self) -> np.ndarray:
        """ready pose의 축소 모델 q(14)를 반환한다."""
        return self._q_ready

    @property
    def q_full_ready(self) -> np.ndarray:
        """ready pose의 전체 모델 q를 반환한다."""
        return self._q_full_ready

    @property
    def frame_ids(self) -> dict:
        """IK 코스트용 프레임 id {side: {wrist/elbow/shoulder: id}}를 반환한다."""
        return self._frame_ids

    # ── 관절값 변환 ──────────────────────────────────────────────────────────

    def _build_q_full(self, named: dict) -> np.ndarray:
        """{관절명: 각도(rad)} 사전을 전체 모델 q 벡터로 변환해 반환한다.

        사전에 없는 관절은 pin.neutral 값(0)을 유지한다. 연속 관절(바퀴)은 pinocchio가
        q를 (cos θ, sin θ) 두 칸으로 표현하므로 nq에 따라 나눠 채운다.
        """
        q_full = pin.neutral(self._full_model)
        for name, value in named.items():
            if not self._full_model.existJointName(name):
                continue
            joint = self._full_model.joints[self._full_model.getJointId(name)]
            if joint.nq == 2:
                q_full[joint.idx_q] = np.cos(value)
                q_full[joint.idx_q + 1] = np.sin(value)
            else:
                q_full[joint.idx_q] = value
        return q_full

    def q_full_from_named(self, named: dict | None) -> np.ndarray:
        """측정 관절값 사전을 전체 모델 q로 변환해 반환한다 (누락 관절은 ready 값 유지).

        실로봇은 본체 관절만 보고하고 손 관절은 값이 없다.
        누락분을 ready로 채워야 링크가 원점으로 무너지지 않는다.
        """
        if not named:
            return self._q_full_ready.copy()
        merged = dict(self._ready_map)
        merged.update(named)
        return self._build_q_full(merged)

    def arm_q_indices(self) -> dict[str, int]:
        """축소 모델 q 벡터에서 각 팔 관절의 인덱스 {관절명: idx_q}를 반환한다 (측정값 동기화용)."""
        return {
            name: self._model.joints[self._model.getJointId(name)].idx_q
            for name in ARM_JOINT_NAMES
        }

    def arm_positions(self, q_reduced: np.ndarray) -> dict:
        """축소 모델 q(14)를 {관절명: 각도} 사전으로 변환해 반환한다 (시뮬 시각화 입력용)."""
        return {
            name: float(q_reduced[self._model.joints[self._model.getJointId(name)].idx_q])
            for name in ARM_JOINT_NAMES
        }

    # ── URDF 파싱 ───────────────────────────────────────────────────────────

    def _parse_visuals(self, xml: str) -> dict:
        """URDF에서 링크별 visual(메시 파일·origin)을 추출해 반환한다 (브라우저 전송용).

        반환: {링크명: {mesh: glb 파일명, origin: {pos, quat(WebXR 기저)}, mesh_base(선택)}} — dae는 동명 glb로
        치환한다. 메시 경로가 손 메시 URL(HAND_MESH_URL_PREFIX로 시작)이면 그 디렉터리를 mesh_base로 따로
        실어 보낸다 (로봇 공통 mesh_base 대신 사용).
        """
        visuals = {}
        root = ET.fromstring(xml)
        for link in root.iter("link"):
            # visual 메시가 있는 링크만 대상
            visual = link.find("visual")
            if visual is None:
                continue
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue

            # 메시 파일명(glb)·origin, 손 메시 URL이면 mesh_base
            filename = mesh.get("filename", "")
            entry = {
                "mesh": filename.split("/")[-1].replace(".dae", ".glb"),
                "origin": _origin_to_webxr(visual.find("origin")),
            }
            if filename.startswith(config.HAND_MESH_URL_PREFIX):
                entry["mesh_base"] = filename.rsplit("/", 1)[0] + "/"
            visuals[link.get("name")] = entry
        return visuals

    def _parse_visual_meshes(self, xml: str) -> dict:
        """링크별 visual 메시의 디스크 경로와 링크 로컬 origin 변환을 반환한다 (자기 몸 마스크용).

        반환: {링크명: (메시 GLB 경로(Path), origin 4x4)}. 손 메시(HAND_MESH_URL_PREFIX 경로)는 HAND_MESH_DIR, 나머지는 RBY1_MESH_DIR.
        """
        meshes = {}
        root = ET.fromstring(xml)
        for link in root.iter("link"):
            # visual 메시가 있는 링크만 대상
            visual = link.find("visual")
            mesh = visual.find("geometry/mesh") if visual is not None else None
            if mesh is None:
                continue

            # 메시 디스크 경로
            filename = mesh.get("filename", "")
            mesh_file = filename.split("/")[-1].replace(".dae", ".glb")
            directory = config.HAND_MESH_DIR if filename.startswith(config.HAND_MESH_URL_PREFIX) else config.RBY1_MESH_DIR

            # 링크 로컬 origin 4x4 변환
            origin = visual.find("origin")
            origin_transform = np.eye(4)
            if origin is not None:
                rpy = [float(token) for token in origin.get("rpy", "0 0 0").split()]
                origin_transform[:3, :3] = R.from_euler("xyz", rpy).as_matrix()
                origin_transform[:3, 3] = [float(token) for token in origin.get("xyz", "0 0 0").split()]
            meshes[link.get("name")] = (directory / mesh_file, origin_transform)
        return meshes

    def visual_meshes(self) -> dict:
        """링크별 (메시 GLB 경로, 링크 로컬 origin 4x4)를 반환한다."""
        return dict(self._visual_meshes)

    # ── 손 ──────────────────────────────────────────────────────────────────

    @staticmethod
    def hand_joint_positions(fingers: dict | None) -> dict[str, float]:
        """손가락 굽힘 {left/right: [6] 0-1}을 손 모델 관절각 {관절명: rad}으로 바꿔 반환한다 (없는 손은 빠진다).

        순서 [little, ring, middle, index, thumb_bend, thumb_rotate]. 네 손가락은 근위·원위 마디를 같은 각으로
        굽히고, 엄지는 기준 굽힘 값(모델 기준 자세)에서의 차이에 범위를 곱한다 (config.HAND_*).
        """
        joint_positions = {}
        finger_max = np.radians(config.HAND_FINGER_MAX_DEG)
        rotate_ref, rotate_span = config.HAND_THUMB_ROTATE
        bend_ref, bend_span = config.HAND_THUMB_BEND
        for side, values in (fingers or {}).items():
            if values is None:
                continue
            curl = np.clip(np.asarray(values, dtype=float), 0.0, 1.0)

            # 네 손가락: 근위·원위 마디를 같은 각으로
            for name, index in (("little", 0), ("ring", 1), ("middle", 2), ("index", 3)):
                angle = float(finger_max * curl[index])
                joint_positions[f"{side}_{name}_proximal_joint"] = angle
                joint_positions[f"{side}_{name}_distal_joint"] = angle

            # 엄지: 기준 값에서의 차이 × 범위
            joint_positions[f"{side}_thumb_yaw_joint"] = float(np.radians(rotate_span) * (curl[5] - rotate_ref))
            joint_positions[f"{side}_thumb_distal_joint"] = float(np.radians(bend_span) * (curl[4] - bend_ref))
        return joint_positions

    # ── 시각화 FK (전체 모델) ────────────────────────────────────────────────

    def fk_visual(self, named: dict | None, fingers: dict | None = None) -> dict:
        """관절값·손가락 굽힘으로 visual이 있는 모든 링크의 포즈를 계산해 반환한다.

        named: {관절명: 각도(rad)} (실로봇 측정값 또는 IK 결과). None이면 ready pose.
        fingers: {left/right: [6] 굽힘 0-1} — 손 모델 관절각으로 바꿔 함께 반영 (없으면 손은 기준 자세).
        반환: {링크명: (pos(3), quat(x,y,z,w))} — 로봇 base 프레임.
        """
        self.full_fk(named, fingers)
        link_poses = {}
        for name, frame_id in self._visual_frame_ids.items():
            placement = self._full_data.oMf[frame_id]
            link_poses[name] = (placement.translation.copy(), pin.Quaternion(placement.rotation).coeffs().copy())
        return link_poses

    def full_fk(self, named: dict | None, fingers: dict | None = None):
        """관절값·손가락 굽힘으로 전체 모델 프레임 FK를 갱신한다 (결과는 full_data.oMf). 반환값 없음."""
        merged = dict(named or {})
        merged.update(self.hand_joint_positions(fingers))
        q_full = self.q_full_from_named(merged if merged else None)
        pin.framesForwardKinematics(self._full_model, self._full_data, q_full)

    def camera_pose(self, named: dict | None = None) -> tuple[np.ndarray, np.ndarray]:
        """머리 장착 카메라의 광학 프레임 포즈 (pos(3), R(3x3))를 로봇 base 프레임으로 반환한다.

        장착 링크(config.CAMERA_MOUNT_LINK)의 FK에 장착 오프셋·광학 축 대응(config.CAMERA_MOUNT_*)을
        합성: p = p_link + R_link·t_mount,  R = R_link·R_mount.
        머리·토르소가 움직이면 값이 바뀌므로 관절값이 갱신될 때마다 다시 호출해야 한다.
        """
        q_full = self.q_full_from_named(named)
        pin.framesForwardKinematics(self._full_model, self._full_data, q_full)
        link_placement = self._full_data.oMf[self._full_model.getFrameId(config.CAMERA_MOUNT_LINK)]
        mount_rotation = np.asarray(config.CAMERA_MOUNT_ROT, dtype=float)
        mount_translation = np.asarray(config.CAMERA_MOUNT_POS, dtype=float)
        return (
            link_placement.translation + link_placement.rotation @ mount_translation,
            link_placement.rotation @ mount_rotation,
        )

    def camera_link_pose(self, named: dict | None = None) -> tuple[np.ndarray, np.ndarray]:
        """카메라 장착 링크 자체의 포즈 (pos(3), quat(x,y,z,w))를 로봇 base 프레임으로 반환한다.

        VR에서 사용자 시점을 로봇 머리에 붙일 때 쓴다 (광학 축 대응 없이 링크 프레임 그대로).
        """
        q_full = self.q_full_from_named(named)
        pin.framesForwardKinematics(self._full_model, self._full_data, q_full)
        link_placement = self._full_data.oMf[self._full_model.getFrameId(config.CAMERA_MOUNT_LINK)]
        return link_placement.translation.copy(), pin.Quaternion(link_placement.rotation).coeffs().copy()

    # ── IK 프레임 FK (축소 모델) ────────────────────────────────────────────

    def fk_frames(self, q: np.ndarray) -> dict:
        """축소 모델 관절각 q(14)로 IK 관련 프레임 포즈를 계산해 반환한다.

        반환: {side: {wrist/elbow/shoulder: (pos(3), quat(x,y,z,w))}} — 로봇 base 프레임.
        wrist는 손 TCP 프레임(파지 중심, ee 로컬 HAND_TCP_OFFSET 적용) 포즈다.
        """
        pin.framesForwardKinematics(self._model, self._data, q)
        frame_poses = {}
        for side, ids in self._frame_ids.items():
            frame_poses[side] = {}
            for key, frame_id in ids.items():
                placement = self._data.oMf[frame_id]
                frame_poses[side][key] = (placement.translation.copy(), pin.Quaternion(placement.rotation).coeffs().copy())
        return frame_poses

    # ── 메타데이터·한계 ─────────────────────────────────────────────────────

    def _log_geometry(self):
        """모델 로드 결과와 기준 기하(어깨 위치·상완 길이)를 로그로 남긴다."""
        poses = self.fk_frames(self._q_ready)
        right_shoulder, left_shoulder = poses["right"]["shoulder"][0], poses["left"]["shoulder"][0]
        logger.info(
            f"모델 로드: 전체 nq={self._full_model.nq}, 축소 nq={self._model.nq}, "
            f"visual 링크 {len(self._visuals)}개, 오른어깨 {np.round(right_shoulder, 3)}, "
            f"왼어깨 {np.round(left_shoulder, 3)}, 상완 {self.upper_arm_length():.3f}m"
        )

    def visual_config(self) -> dict:
        """브라우저 1회 전송용 시각화 메타(링크별 메시 파일·origin)를 반환한다."""
        return self._visuals

    def camera_visual_config(self) -> dict:
        """브라우저 카메라 표시용 메타 {link, origin{pos,quat}(WebXR 기저, 링크 로컬), mesh_base, mesh}를 반환한다.

        D455 GLB는 ROS camera_link 프레임(x=전방, y=좌, z=상 — 장착 링크와 같은 축 규약)으로 구워져 있어
        링크 로컬 변환은 평행이동(CAMERA_MOUNT_POS)뿐이다. visual origin과 같은 이유로 WebXR 기저로
        재정의(Cᵀ·T·C)해 보내고, 클라이언트는 로봇 링크 GLB와 동일한 기저 보정 체인으로 붙인다.
        """
        webxr_pos, webxr_quat = robot_to_webxr(config.CAMERA_MOUNT_POS, [0.0, 0.0, 0.0, 1.0])
        return {
            "link": config.CAMERA_MOUNT_LINK,
            "origin": {"pos": webxr_pos.tolist(), "quat": webxr_quat.tolist()},
            "mesh_base": config.CAMERA_MESH_URL_PREFIX,
            "mesh": config.CAMERA_MESH_FILE,
        }

    def shoulder_positions(self) -> dict:
        """ready pose 기준 좌·우 어깨(arm_0 원점) 위치를 반환한다 (토르소 고정이라 상수)."""
        poses = self.fk_frames(self._q_ready)
        return {side: poses[side]["shoulder"][0] for side in ("left", "right")}

    def upper_arm_length(self) -> float:
        """ready pose 기준 어깨-팔꿈치 거리(m)를 반환한다 (팔꿈치 방향 코스트의 스케일)."""
        poses = self.fk_frames(self._q_ready)
        return float(np.linalg.norm(poses["right"]["elbow"][0] - poses["right"]["shoulder"][0]))

    def joint_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        """축소 모델의 관절 하한·상한(rad)을 반환한다.

        URDF 한계에 SAFETY_JOINT_LIMITS(안전 한계)를 교집합으로 적용한다.
        """
        lower_bounds = self._model.lowerPositionLimit.copy()
        upper_bounds = self._model.upperPositionLimit.copy()
        for name, (safety_lower, safety_upper) in config.SAFETY_JOINT_LIMITS.items():
            q_index = self._model.joints[self._model.getJointId(name)].idx_q
            lower_bounds[q_index] = max(lower_bounds[q_index], safety_lower)
            upper_bounds[q_index] = min(upper_bounds[q_index], safety_upper)
        return lower_bounds, upper_bounds

    def velocity_limits(self) -> np.ndarray:
        """축소 모델의 관절 속도 한계(rad/s, URDF velocity)를 반환한다."""
        return self._model.velocityLimit.copy()

    def velocity_limits_named(self) -> dict[str, float]:
        """전체 모델의 1자유도 관절 속도 한계 {관절명: rad/s}를 반환한다 (로봇 명령 안전 상한용)."""
        limits = {}
        for joint_id in range(1, self._full_model.njoints):
            joint = self._full_model.joints[joint_id]
            if joint.nv == 1:
                limits[self._full_model.names[joint_id]] = float(self._full_model.velocityLimit[joint.idx_v])
        return limits

    def position_limits_named(self) -> dict[str, tuple[float, float]]:
        """전체 모델의 1자유도 관절 위치 한계 {관절명: (하한, 상한) rad}를 반환한다 (로봇 명령 범위 제한용)."""
        limits = {}
        for joint_id in range(1, self._full_model.njoints):
            joint = self._full_model.joints[joint_id]
            if joint.nq == 1:
                limits[self._full_model.names[joint_id]] = (
                    float(self._full_model.lowerPositionLimit[joint.idx_q]),
                    float(self._full_model.upperPositionLimit[joint.idx_q]),
                )
        return limits

    def ready_positions(self) -> dict[str, float]:
        """ready pose의 관절값 {관절명: rad}(토르소·양팔·머리)를 반환한다."""
        return dict(self._ready_map)

    def joint_names_in_q_order(self) -> list[str]:
        """축소 모델 q 벡터 순서대로 관절 이름을 반환한다 (로봇 브리지의 이름 기반 매핑용)."""
        return [self._model.names[joint_id] for joint_id in range(1, self._model.njoints)]
