"""로봇 자기 몸 마스크: 카메라에 찍힌 로봇 팔·손 픽셀을 찾아 재구성 메시에 구워지지 않게 한다.

팔·손 링크 visual 메시(RBY1 팔 GLB + Inspire 손 GLB) 표면을 균일 샘플링한 점을 측정 관절값·손가락 굽힘의 FK로 배치하고,
depth 격자(PIXEL_MASK_STRIDE)에 투영해 픽셀마다 가장 가까운 로봇 표면 깊이(z-buffer)를 만든다.
점 사이 빈틈은 주변 칸의 최소 깊이로 메운다. 링크 그룹(팔·손, config.SELF_MASK_GROUPS)마다 z-buffer를 따로 만들고,
관측 깊이가 그 그룹 표면보다 그룹 여유 넘게 앞이 아니면 로봇 몸(또는 그 뒤)으로 보고 지운다 —
손에 쥔 물체처럼 손보다 카메라 쪽에 있는 표면은 남는다.

FK(link_poses)는 RobotModel의 pinocchio Data를 공유하므로 이벤트 루프 스레드에서 호출하고,
투영·판정(pixel_mask)은 순수 numpy라 executor에서 돌려도 된다.
"""

import base64
import json
import logging
import struct

import numpy as np

from teleop.camera.camera_interface import Intrinsics
from teleop.perception.projection import upsample_mask
from teleop.utils import config

logger = logging.getLogger("self_mask")

# glTF 접근자 성분 형식 → numpy dtype (POSITION은 FLOAT, 인덱스는 UNSIGNED_SHORT/INT)
_GLTF_DTYPES = {5126: np.float32, 5123: np.uint16, 5125: np.uint32}
_GLTF_WIDTH = {"VEC3": 3, "SCALAR": 1}


def _read_accessor(gltf: dict, data: bytes, bin_offset: int, index: int) -> np.ndarray:
    """접근자 하나를 (count, width) 배열로 읽어 반환한다 (bufferView 간격 고려)."""
    accessor = gltf["accessors"][index]
    dtype = np.dtype(_GLTF_DTYPES[accessor["componentType"]])
    width = _GLTF_WIDTH[accessor["type"]]
    view = gltf["bufferViews"][accessor["bufferView"]]
    start = bin_offset + view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
    stride = view.get("byteStride", dtype.itemsize * width)
    raw = np.frombuffer(data, dtype=np.uint8, count=stride * (accessor["count"] - 1) + dtype.itemsize * width,
                        offset=start)
    rows = np.lib.stride_tricks.as_strided(raw, (accessor["count"], dtype.itemsize * width), (stride, 1))
    return rows.copy().view(dtype).reshape(accessor["count"], width)


def read_glb_mesh(path) -> tuple[np.ndarray, np.ndarray]:
    """GLB의 모든 프리미티브를 합친 (정점 (N,3) float32, 삼각형 (M,3) int64)를 반환한다.

    압축 확장·노드 변환이 없는 단일 노드 GLB(RBY1 메시, build_inspire_hand 출력)를 전제로 한다.
    버퍼는 BIN 청크(RBY1 메시) 또는 base64 data URI(Open3D 출력) 둘 다 읽는다.
    """
    raw_file = open(path, "rb").read()
    json_len, _ = struct.unpack_from("<II", raw_file, 12)
    gltf = json.loads(raw_file[20:20 + json_len])
    uri = gltf["buffers"][0].get("uri")
    if uri is not None:
        data, bin_offset = base64.b64decode(uri.split(",", 1)[1]), 0
    else:
        data, bin_offset = raw_file, 20 + json_len + 8

    verts, faces, offset = [], [], 0
    for mesh in gltf["meshes"]:
        for primitive in mesh["primitives"]:
            v = _read_accessor(gltf, data, bin_offset, primitive["attributes"]["POSITION"])
            if "indices" in primitive:
                f = _read_accessor(gltf, data, bin_offset, primitive["indices"]).astype(np.int64).reshape(-1, 3)
            else:
                f = np.arange(len(v), dtype=np.int64).reshape(-1, 3)
            verts.append(v)
            faces.append(f + offset)
            offset += len(v)
    return np.concatenate(verts), np.concatenate(faces)


def sample_surface(verts: np.ndarray, faces: np.ndarray, spacing: float, rng) -> np.ndarray:
    """삼각형 면적에 비례해 표면을 균일 샘플링한 점 (K,3)을 반환한다 (점 하나가 spacing² 면적을 맡는다)."""
    a, b, c = verts[faces[:, 0]], verts[faces[:, 1]], verts[faces[:, 2]]
    areas = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    count = max(int(areas.sum() / spacing ** 2), len(faces) // 4, 1)
    chosen = rng.choice(len(faces), count, p=areas / areas.sum())
    # 삼각형 안 균일 무게중심 좌표
    r1, r2 = rng.random(count), rng.random(count)
    flip = r1 + r2 > 1.0
    r1[flip], r2[flip] = 1.0 - r1[flip], 1.0 - r2[flip]
    return a[chosen] + r1[:, None] * (b[chosen] - a[chosen]) + r2[:, None] * (c[chosen] - a[chosen])


class SelfMask:
    """대상 링크의 표면 샘플 점(링크 로컬)과 카메라 격자 파라미터를 보관하고 로봇 몸 픽셀을 판정한다."""

    def __init__(self, robot_model, intr: Intrinsics):
        self._rm = robot_model
        self._intr = intr
        self._stride = config.PIXEL_MASK_STRIDE
        self._grid_h = (intr.height + self._stride - 1) // self._stride
        self._grid_w = (intr.width + self._stride - 1) // self._stride

        # 링크별 표면 균일 샘플 점 (visual origin까지 적용한 링크 로컬 좌표)과 소속 그룹 번호
        rng = np.random.default_rng(0)
        self._groups = list(config.SELF_MASK_GROUPS.items())
        self._links: list[tuple[str, int, np.ndarray]] = []
        self._link_group: list[int] = []
        total = 0
        for link, (path, origin) in robot_model.visual_meshes().items():
            group = next((g for g, (prefixes, _) in enumerate(self._groups) if link.startswith(prefixes)), None)
            if group is None:
                continue
            self._link_group.append(group)
            verts, faces = read_glb_mesh(path)
            points = sample_surface(verts.astype(np.float64), faces, config.SELF_MASK_POINT_SPACING, rng)
            points = points @ origin[:3, :3].T + origin[:3, 3]
            self._links.append((link, robot_model.full_model.getFrameId(link), points.astype(np.float32)))
            total += len(points)
        logger.info(f"자기 몸 마스크: 링크 {len(self._links)}개, 표면 점 {total}개, 격자 {self._grid_w}x{self._grid_h}")

    def link_poses(self, joints: dict, fingers: dict | None) -> list[tuple[np.ndarray, np.ndarray]]:
        """관절값·손가락 굽힘으로 대상 링크 포즈 [(R(3x3), t(3))]를 계산해 반환한다 (루프 스레드)."""
        self._rm.full_fk(joints, fingers)
        data = self._rm.full_data
        return [(data.oMf[fid].rotation.copy(), data.oMf[fid].translation.copy()) for _, fid, _ in self._links]

    def pixel_mask(self, depth: np.ndarray, cam_pos: np.ndarray, cam_rot: np.ndarray, poses) -> np.ndarray:
        """로봇 몸으로 판정된 픽셀 (H, W) bool 마스크를 반환한다 (executor에서 실행).

        depth: (H, W) float32 m. poses: link_poses() 결과.
        """
        s = self._stride
        observed = depth[::s, ::s]
        robot = np.zeros(observed.shape, dtype=bool)
        for group, (_, margin) in enumerate(self._groups):
            # 그룹 표면 점 → 카메라 광학 프레임
            members = [i for i, g in enumerate(self._link_group) if g == group]
            if not members:
                continue
            world = np.concatenate([self._links[i][2] @ poses[i][0].T + poses[i][1] for i in members])
            zbuf = self._zbuffer((world - cam_pos) @ cam_rot)

            # 관측이 그룹 표면보다 여유 넘게 앞이 아니면 로봇 몸 (무효 깊이는 판정 대상 아님)
            robot |= np.isfinite(zbuf) & (observed > 0) & (observed > zbuf - margin)

        # 실루엣 경계의 섞인 깊이까지 지우도록 팽창
        return upsample_mask(self._dilate(robot), s, depth.shape)

    @staticmethod
    def _dilate(mask: np.ndarray) -> np.ndarray:
        """bool 격자 마스크를 SELF_MASK_DILATE_CELLS 반경(정사각형)으로 팽창해 반환한다."""
        radius = config.SELF_MASK_DILATE_CELLS
        padded = np.pad(mask, radius)
        out = mask.copy()
        h, w = mask.shape
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                out |= padded[radius + dy:radius + dy + h, radius + dx:radius + dx + w]
        return out

    def _zbuffer(self, cam: np.ndarray) -> np.ndarray:
        """카메라 광학 프레임 점들을 격자에 투영해 칸마다 가장 가까운 깊이(z-buffer, 빈틈 메움)를 반환한다."""
        cam = cam[cam[:, 2] > 0.05]
        intr, s = self._intr, self._stride
        u = ((intr.fx * cam[:, 0] / cam[:, 2] + intr.ppx) / s).astype(np.int64)
        v = ((intr.fy * cam[:, 1] / cam[:, 2] + intr.ppy) / s).astype(np.int64)
        inside = (u >= 0) & (u < self._grid_w) & (v >= 0) & (v < self._grid_h)
        zbuf = np.full(self._grid_h * self._grid_w, np.inf, dtype=np.float32)
        np.minimum.at(zbuf, v[inside] * self._grid_w + u[inside], cam[inside, 2].astype(np.float32))
        return self._fill_gaps(zbuf.reshape(self._grid_h, self._grid_w))

    @staticmethod
    def _fill_gaps(zbuf: np.ndarray) -> np.ndarray:
        """표면 점 사이 빈 칸을 주변(SELF_MASK_FILL_CELLS 반경) 최소 깊이로 메운 z-buffer를 반환한다."""
        radius = config.SELF_MASK_FILL_CELLS
        padded = np.pad(zbuf, radius, constant_values=np.inf)
        filled = zbuf.copy()
        h, w = zbuf.shape
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                np.minimum(filled, padded[radius + dy:radius + dy + h, radius + dx:radius + dx + w], out=filled)
        return filled
