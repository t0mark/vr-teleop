"""Inspire RH56E2 손 STEP → 링크별 GLB + 손 URDF 조각 생성 도구 (1회성, 결과는 assets/inspire_hand/에 둔다).

사용 (호스트, 도구 전용 의존성: pip install cadquery-ocp open3d):
    python3 tools/build_inspire_hand.py <RH56E2-R-T1.STEP 경로>

입력은 오른손 STEP 하나다. 왼손 STEP은 오른손을 z_s로 거울 반사한 형상이라(부품 위치 일치 확인),
왼손은 손 로컬 좌표에서 y 거울로 만든다 (관절축은 굽힘 방향이 유지되게 -M·a).

좌표 규약 (STEP mm → 손 로컬 m): 원점 = 손목 장착 플랜지 중심, 손가락 +y_s → -z, 손바닥 앞 -z_s → -x, x_s → -y.
손 로컬 프레임은 ee_{side} 프레임에 장착 회전(z축 ∓90°, 오른손 -90°)으로 붙는다 — 녹화 depth에 손 메시를 투영해
엄지·손가락 위치가 실제와 겹치는 각도로 정했다. 장착 회전은 URDF 조각의 장착 관절 rpy에 들어간다.

링크 (부품은 STEP 조립 경로의 NAUO 번호):
    hand_base       손바닥 본체·손목 어댑터·손가락 축 받침
    {finger}_proximal / {finger}_distal   검지·중지·약지·새끼 (근위·원위 마디, 핀 구멍 원 검출로 관절 위치 확인)
    thumb_yaw / thumb_distal              엄지 본체 / 끝마디 (STEP에서 엄지가 두 부품으로 단순화되어 있음)
관절 각 0 = STEP 자세. 손가락 굽힘 값 → 관절각 변환은 teleop/robot/robot_model.py가 한다.
"""

import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import open3d as o3d
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS

REPO_ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = REPO_ROOT / "assets" / "inspire_hand"

# STEP(mm) → 손 로컬(m): 행 = 로컬 축, 열 = STEP 축. 원점 = 장착 플랜지 중심 (STEP 좌표)
FLANGE_MM = np.array([-8.3, -111.0, 111.8])
R_LOCAL_STEP = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
# ee 프레임 기준 장착 회전 (z축, rad): 오른손 -90°, 왼손 +90° (거울)
MOUNT_YAW = {"right": -np.pi / 2, "left": np.pi / 2}

# 손가락: (근위 부품, 원위 부품, 손가락 x 중심 mm, 근위 핀 (y, z) mm, 원위 핀 (y, z) mm)
FINGERS = {
    "index": ("NAUO184", "NAUO178", -40.5, (45.43, 111.27), (78.6, 108.14)),
    "middle": ("NAUO183", "NAUO177", -19.0, (46.04, 111.27), (78.6, 108.14)),
    "ring": ("NAUO182", "NAUO181", 3.5, (45.17, 111.27), (78.6, 108.14)),
    "little": ("NAUO186", "NAUO176", 25.5, (42.82, 111.27), (76.0, 108.14)),
}
BASE_PARTS = ("NAUO168", "NAUO169", "NAUO170", "NAUO175", "NAUO179", "NAUO180")
# 손가락 굽힘축 (STEP): x_s 음의 방향 회전 = +y_s 손가락이 손바닥 앞(-z_s)으로 굽는다
FINGER_AXIS_STEP = np.array([-1.0, 0.0, 0.0])
# 엄지: 회전축 = 손바닥 평면과 평행한 y_s 방향 (x -55, z 74.5). 음의 y_s 회전이 손바닥 앞·손가락 쪽으로 대향
THUMB_YAW_PIVOT_MM = np.array([-55.0, -22.0, 74.5])
THUMB_YAW_AXIS_STEP = np.array([0.0, -1.0, 0.0])
# 엄지 굽힘 핀 (세 축 투영 모두에서 같은 반지름 2.65 mm 원으로 검출된 점)
THUMB_BEND_PIVOT_MM = np.array([-93.5, -6.4, 36.8])

# 관절 범위 (rad, URDF limit): 손가락 마디 0-80°, 엄지 회전·굽힘은 STEP 자세를 기준으로 한 범위
FINGER_LIMIT = (0.0, np.radians(80.0))
THUMB_YAW_LIMIT = (np.radians(-52.5), np.radians(22.5))
THUMB_BEND_LIMIT = (np.radians(-37.4), np.radians(45.7))

# 링크별 삼각형 목표 (웹 표시·마스크 계산 부담 기준)
TRI_TARGET = {"hand_base": 8000, "proximal": 1500, "distal": 1200, "thumb_yaw": 3000, "thumb_distal": 1200}
# 표시 색 (실물 검정 무광에 가깝게, 조명에서 형상이 보이는 정도의 짙은 회색)
MESH_COLOR = (0.22, 0.22, 0.24)


def read_step_parts(path: str) -> dict:
    """STEP의 모든 부품을 {조립 경로: (verts mm (N,3), faces (M,3))}로 읽어 반환한다."""
    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(path) != 1:
        raise SystemExit(f"STEP 읽기 실패: {path}")
    reader.Transfer(doc)
    tool = XCAFTool(doc)
    roots = TDF_LabelSequence()
    tool.st.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        tool.walk(roots.Value(i), TopLoc_Location(), [])
    return tool.parts


class XCAFTool:
    """XCAF 조립 트리를 내려가며 부품 형상을 테셀레이션해 모은다."""

    def __init__(self, doc):
        from OCP.XCAFDoc import XCAFDoc_DocumentTool
        self.st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
        self.parts = {}

    @staticmethod
    def name(label) -> str:
        attr = TDataStd_Name()
        return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else "?"

    def walk(self, label, loc, trail):
        """참조는 위치를 누적해 따라가고, 조립품은 자식으로, 부품은 테셀레이션해 저장한다."""
        if self.st.IsReference_s(label):
            ref = TDF_Label()
            self.st.GetReferredShape_s(label, ref)
            self.walk(ref, loc.Multiplied(self.st.GetLocation_s(label)), trail + [self.name(label)])
            return
        if self.st.IsAssembly_s(label):
            comps = TDF_LabelSequence()
            self.st.GetComponents_s(label, comps, False)
            for i in range(1, comps.Length() + 1):
                self.walk(comps.Value(i), loc, trail)
            return

        # 부품 테셀레이션 (선형 편차 0.3 mm, 각도 0.4 rad), 뒤집힌 면은 감김 방향 교정
        shape = self.st.GetShape_s(label).Moved(loc)
        BRepMesh_IncrementalMesh(shape, 0.3, False, 0.4, True)
        verts, faces = [], []
        explorer = TopExp_Explorer(shape, TopAbs_FACE)
        while explorer.More():
            face = TopoDS.Face_s(explorer.Current())
            face_loc = TopLoc_Location()
            tri = BRep_Tool.Triangulation_s(face, face_loc)
            if tri is not None:
                tr = face_loc.Transformation()
                base = len(verts)
                for i in range(1, tri.NbNodes() + 1):
                    p = tri.Node(i).Transformed(tr)
                    verts.append((p.X(), p.Y(), p.Z()))
                reversed_face = face.Orientation() == TopAbs_REVERSED
                for i in range(1, tri.NbTriangles() + 1):
                    a, b, c = tri.Triangle(i).Get()
                    faces.append((base + a - 1, base + c - 1, base + b - 1) if reversed_face
                                 else (base + a - 1, base + b - 1, base + c - 1))
            explorer.Next()
        key = "/".join(trail + [self.name(label)])
        self.parts[key] = (np.array(verts, float).reshape(-1, 3), np.array(faces, np.int64).reshape(-1, 3))


def to_local(p_mm) -> np.ndarray:
    """STEP 좌표(mm) 점들을 손 로컬 좌표(m)로 변환해 반환한다."""
    return (np.asarray(p_mm, float) - FLANGE_MM) @ R_LOCAL_STEP.T / 1000.0


def merge_parts(parts: dict, keys) -> tuple[np.ndarray, np.ndarray]:
    """조립 경로에 /NAUO###/가 들어간 부품들을 하나의 메시로 합쳐 반환한다."""
    verts, faces, offset = [], [], 0
    for key in keys:
        matched = [v for path, v in parts.items() if f"/{key}/" in path]
        if len(matched) != 1:
            raise SystemExit(f"부품 {key} 매칭 {len(matched)}개 — STEP 구성 확인 필요")
        v, f = matched[0]
        verts.append(v)
        faces.append(f + offset)
        offset += len(v)
    return np.concatenate(verts), np.concatenate(faces)


def thumb_bend_axis(parts: dict) -> np.ndarray:
    """엄지 굽힘축(STEP)을 반환한다: 본체 방향(회전 피벗→굽힘 피벗)과 끝마디 방향이 이루는 평면의 법선.

    양의 회전이 본체-끝마디 사이 굽힘을 키우는 방향이다.
    """
    tip, _ = merge_parts(parts, ["NAUO173"])
    body_dir = THUMB_BEND_PIVOT_MM - THUMB_YAW_PIVOT_MM
    tip_dir = tip.mean(axis=0) - THUMB_BEND_PIVOT_MM
    axis = np.cross(body_dir, tip_dir)
    return axis / np.linalg.norm(axis)


def link_specs(parts: dict) -> list[dict]:
    """링크 정의 목록을 반환한다: 이름, 부모, 부품, 관절 피벗(STEP mm), 관절축(STEP), 범위, 삼각형 목표."""
    specs = [{"name": "hand_base", "parent": None, "parts": BASE_PARTS, "pivot": None, "axis": None,
              "limit": None, "target": TRI_TARGET["hand_base"]}]
    for finger, (prox, dist, x, (yp, zp), (yd, zd)) in FINGERS.items():
        specs.append({"name": f"{finger}_proximal", "parent": "hand_base", "parts": (prox,),
                      "pivot": (x, yp, zp), "axis": FINGER_AXIS_STEP, "limit": FINGER_LIMIT,
                      "target": TRI_TARGET["proximal"]})
        specs.append({"name": f"{finger}_distal", "parent": f"{finger}_proximal", "parts": (dist,),
                      "pivot": (x, yd, zd), "axis": FINGER_AXIS_STEP, "limit": FINGER_LIMIT,
                      "target": TRI_TARGET["distal"]})
    specs.append({"name": "thumb_yaw", "parent": "hand_base", "parts": ("NAUO172",),
                  "pivot": THUMB_YAW_PIVOT_MM, "axis": THUMB_YAW_AXIS_STEP, "limit": THUMB_YAW_LIMIT,
                  "target": TRI_TARGET["thumb_yaw"]})
    specs.append({"name": "thumb_distal", "parent": "thumb_yaw", "parts": ("NAUO173",),
                  "pivot": THUMB_BEND_PIVOT_MM, "axis": thumb_bend_axis(parts), "limit": THUMB_BEND_LIMIT,
                  "target": TRI_TARGET["thumb_distal"]})
    return specs


def simplify(verts: np.ndarray, faces: np.ndarray, target: int) -> o3d.geometry.TriangleMesh:
    """중복·퇴화를 정리하고 삼각형 수를 target 이하로 줄인 메시를 반환한다."""
    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(verts), o3d.utility.Vector3iVector(faces))
    mesh.merge_close_vertices(1e-5)
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    if len(mesh.triangles) > target:
        mesh = mesh.simplify_quadric_decimation(target)
    return mesh


def write_glb(path: Path, verts: np.ndarray, faces: np.ndarray):
    """정점(링크 로컬 m)·면으로 법선·색을 붙인 GLB를 쓴다. 반환값 없음."""
    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(verts), o3d.utility.Vector3iVector(faces))
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color(MESH_COLOR)
    if not o3d.io.write_triangle_mesh(str(path), mesh):
        raise SystemExit(f"GLB 쓰기 실패: {path}")


def build(parts: dict):
    """양손 GLB와 URDF 조각을 만든다. 반환값 없음."""
    specs = link_specs(parts)
    mesh_dir = OUT_DIR / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)

    # 오른손 기준 링크 메시(링크 원점 기준 로컬 m)와 관절 원점·축
    local = {}
    pivots = {"hand_base": np.zeros(3)}
    for spec in specs:
        v, f = merge_parts(parts, spec["parts"])
        origin = to_local(spec["pivot"]) if spec["pivot"] is not None else np.zeros(3)
        pivots[spec["name"]] = origin
        mesh = simplify(to_local(v) - origin, f, spec["target"])
        local[spec["name"]] = (np.asarray(mesh.vertices), np.asarray(mesh.triangles))
        print(f"{spec['name']:16s} 삼각형 {len(f):6d} → {len(mesh.triangles):5d}, 관절 원점(손 로컬) {np.round(origin, 4)}")

    for side in ("right", "left"):
        # 왼손: 손 로컬 y 거울 (정점 반사 + 감김 교정, 관절축 -M·a)
        mirror = np.diag([1.0, -1.0, 1.0]) if side == "left" else np.eye(3)
        robot = ET.Element("robot", name=f"inspire_hand_{side}")
        mount = ET.SubElement(robot, "joint", name=f"{side}_hand_mount", type="fixed")
        ET.SubElement(mount, "parent", link=f"ee_{side}")
        ET.SubElement(mount, "child", link=f"{side}_hand_base")
        ET.SubElement(mount, "origin", xyz="0 0 0", rpy=f"0 0 {MOUNT_YAW[side]:.6f}")

        for spec in specs:
            name = f"{side}_{spec['name']}"
            verts, faces = local[spec["name"]]
            verts = verts @ mirror.T
            if side == "left":
                faces = faces[:, [0, 2, 1]]
            mesh_file = f"{name}.glb"
            write_glb(mesh_dir / mesh_file, verts, faces)

            link = ET.SubElement(robot, "link", name=name)
            visual = ET.SubElement(link, "visual")
            ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
            geometry = ET.SubElement(visual, "geometry")
            ET.SubElement(geometry, "mesh", filename=f"./meshes/{mesh_file}")
            if spec["parent"] is None:
                continue

            # 관절: 원점 = 부모 링크 원점 기준 피벗, 축 = 손 로컬 (왼손은 -M·a)
            origin = mirror @ (pivots[spec["name"]] - pivots[spec["parent"]])
            axis = R_LOCAL_STEP @ spec["axis"]
            axis = -(mirror @ axis) if side == "left" else axis
            joint = ET.SubElement(robot, "joint", name=f"{name}_joint", type="revolute")
            ET.SubElement(joint, "parent", link=f"{side}_{spec['parent']}")
            ET.SubElement(joint, "child", link=name)
            ET.SubElement(joint, "origin", xyz=" ".join(f"{v:.6f}" for v in origin), rpy="0 0 0")
            ET.SubElement(joint, "axis", xyz=" ".join(f"{v:.6f}" for v in axis))
            lower, upper = spec["limit"]
            ET.SubElement(joint, "limit", lower=f"{lower:.6f}", upper=f"{upper:.6f}", effort="10", velocity="5")

        ET.indent(robot)
        urdf = OUT_DIR / f"inspire_hand_{side}.urdf"
        urdf.write_text(ET.tostring(robot, encoding="unicode") + "\n")
        print(f"작성: {urdf.relative_to(REPO_ROOT)}")


def main():
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    step = sys.argv[1]
    print(f"STEP 읽기: {os.path.basename(step)}")
    build(read_step_parts(step))


if __name__ == "__main__":
    main()
