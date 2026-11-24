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
# glTF 접근자 형식 → 원소당 성분 수
_GLTF_WIDTH = {"VEC3": 3, "SCALAR": 1}
# GLB 헤더(12 B)와 청크 헤더(8 B) 크기
_GLB_HEADER_BYTES = 12
_GLB_CHUNK_HEADER_BYTES = 8
# 투영에 쓰는 표면 점의 최소 깊이(m)
_MIN_PROJECT_DEPTH = 0.05


def _read_accessor(gltf: dict, data: bytes, bin_offset: int, index: int) -> np.ndarray:
    """접근자 하나를 (count, width) 배열로 읽어 반환한다 (bufferView 간격 고려)."""
    # 접근자·bufferView에서 형식·시작 위치·간격 확인
    accessor = gltf["accessors"][index]
    dtype = np.dtype(_GLTF_DTYPES[accessor["componentType"]])
    width = _GLTF_WIDTH[accessor["type"]]
    buffer_view = gltf["bufferViews"][accessor["bufferView"]]
    start = bin_offset + buffer_view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
    row_bytes = dtype.itemsize * width
    stride = buffer_view.get("byteStride", row_bytes)

    # 간격을 건너뛰며 행을 모아 dtype 배열로 변환
    raw = np.frombuffer(data, dtype=np.uint8, count=stride * (accessor["count"] - 1) + row_bytes, offset=start)
    rows = np.lib.stride_tricks.as_strided(raw, (accessor["count"], row_bytes), (stride, 1))
    return rows.copy().view(dtype).reshape(accessor["count"], width)


def read_glb_mesh(path) -> tuple[np.ndarray, np.ndarray]:
    """GLB의 모든 프리미티브를 합친 (정점 (N,3) float32, 삼각형 (M,3) int64)를 반환한다.

    압축 확장·노드 변환이 없는 단일 노드 GLB(RBY1 메시, build_inspire_hand 출력)를 전제로 한다.
    버퍼는 BIN 청크(RBY1 메시) 또는 base64 data URI(Open3D 출력) 둘 다 읽는다.
    """
    # JSON 청크 읽기
    with open(path, "rb") as glb_file:
        raw_file = glb_file.read()
    json_len, _ = struct.unpack_from("<II", raw_file, _GLB_HEADER_BYTES)
    json_start = _GLB_HEADER_BYTES + _GLB_CHUNK_HEADER_BYTES
    gltf = json.loads(raw_file[json_start:json_start + json_len])

    # 버퍼: data URI면 디코드, 아니면 BIN 청크
    uri = gltf["buffers"][0].get("uri")
    if uri is not None:
        data, bin_offset = base64.b64decode(uri.split(",", 1)[1]), 0
    else:
        data, bin_offset = raw_file, json_start + json_len + _GLB_CHUNK_HEADER_BYTES

    # 프리미티브별 정점·삼각형을 정점 번호를 이어 붙여 합치기
    vert_parts, face_parts, vert_offset = [], [], 0
    for mesh in gltf["meshes"]:
        for primitive in mesh["primitives"]:
            positions = _read_accessor(gltf, data, bin_offset, primitive["attributes"]["POSITION"])
            if "indices" in primitive:
                indices = _read_accessor(gltf, data, bin_offset, primitive["indices"])
                primitive_faces = indices.astype(np.int64).reshape(-1, 3)
            else:
                primitive_faces = np.arange(len(positions), dtype=np.int64).reshape(-1, 3)
            vert_parts.append(positions)
            face_parts.append(primitive_faces + vert_offset)
            vert_offset += len(positions)
    return np.concatenate(vert_parts), np.concatenate(face_parts)


def sample_surface(verts: np.ndarray, faces: np.ndarray, spacing: float, rng) -> np.ndarray:
    """삼각형 면적에 비례해 표면을 균일 샘플링한 점 (K,3)을 반환한다 (점 하나가 spacing² 면적을 맡는다)."""
    # 면적 비례로 샘플 삼각형 선택
    corner_a, corner_b, corner_c = verts[faces[:, 0]], verts[faces[:, 1]], verts[faces[:, 2]]
    areas = 0.5 * np.linalg.norm(np.cross(corner_b - corner_a, corner_c - corner_a), axis=1)
    count = max(int(areas.sum() / spacing ** 2), len(faces) // 4, 1)
    chosen = rng.choice(len(faces), count, p=areas / areas.sum())

    # 삼각형 안 균일 무게중심 좌표
    weight_b, weight_c = rng.random(count), rng.random(count)
    flip = weight_b + weight_c > 1.0
    weight_b[flip], weight_c[flip] = 1.0 - weight_b[flip], 1.0 - weight_c[flip]
    return (corner_a[chosen] + weight_b[:, None] * (corner_b[chosen] - corner_a[chosen])
            + weight_c[:, None] * (corner_c[chosen] - corner_a[chosen]))


class SelfMask:
    """대상 링크의 표면 샘플 점(링크 로컬)과 카메라 격자 파라미터를 보관하고 로봇 몸 픽셀을 판정한다."""

    def __init__(self, robot_model, intrinsics: Intrinsics):
        """robot_model: 링크 visual 메시·FK 제공자. intrinsics: color(=정렬 기준) 내부 파라미터."""
        # 카메라 격자 파라미터
        self._robot_model = robot_model
        self._intrinsics = intrinsics
        self._stride = config.PIXEL_MASK_STRIDE
        self._grid_h = (intrinsics.height + self._stride - 1) // self._stride
        self._grid_w = (intrinsics.width + self._stride - 1) // self._stride

        # 링크별 표면 균일 샘플 점 (visual origin까지 적용한 링크 로컬 좌표)과 소속 그룹 번호
        rng = np.random.default_rng(0)
        self._groups = list(config.SELF_MASK_GROUPS.items())
        self._links: list[tuple[str, int, np.ndarray]] = []
        self._link_group: list[int] = []
        total_points = 0
        for link, (path, origin) in robot_model.visual_meshes().items():
            group = next(
                (group_index for group_index, (prefixes, _) in enumerate(self._groups) if link.startswith(prefixes)),
                None,
            )
            if group is None:
                continue
            self._link_group.append(group)
            verts, faces = read_glb_mesh(path)
            points = sample_surface(verts.astype(np.float64), faces, config.SELF_MASK_POINT_SPACING, rng)
            points = points @ origin[:3, :3].T + origin[:3, 3]
            self._links.append((link, robot_model.full_model.getFrameId(link), points.astype(np.float32)))
            total_points += len(points)

        # 그룹별 소속 링크 번호 목록
        self._group_members = [
            [link_index for link_index, link_group in enumerate(self._link_group) if link_group == group_index]
            for group_index in range(len(self._groups))
        ]
        logger.info(f"자기 몸 마스크: 링크 {len(self._links)}개, 표면 점 {total_points}개, 격자 {self._grid_w}x{self._grid_h}")

    def link_poses(self, joints: dict, fingers: dict | None) -> list[tuple[np.ndarray, np.ndarray]]:
        """관절값·손가락 굽힘으로 대상 링크 포즈 [(R(3x3), t(3))]를 계산해 반환한다 (루프 스레드)."""
        self._robot_model.full_fk(joints, fingers)
        data = self._robot_model.full_data
        return [
            (data.oMf[frame_id].rotation.copy(), data.oMf[frame_id].translation.copy())
            for _, frame_id, _ in self._links
        ]

    def pixel_mask(self, depth: np.ndarray, cam_pos: np.ndarray, cam_rot: np.ndarray, poses) -> np.ndarray:
        """로봇 몸으로 판정된 픽셀 (H, W) bool 마스크를 반환한다 (executor에서 실행).

        depth: (H, W) float32 m. poses: link_poses() 결과.
        """
        observed = depth[::self._stride, ::self._stride]
        robot = np.zeros(observed.shape, dtype=bool)
        for (_, margin), members in zip(self._groups, self._group_members):
            if not members:
                continue

            # 그룹 표면 점 → base → 카메라 광학 프레임 z-buffer
            world = np.concatenate([
                self._links[link_index][2] @ poses[link_index][0].T + poses[link_index][1] for link_index in members
            ])
            zbuf = self._zbuffer((world - cam_pos) @ cam_rot)

            # 관측이 그룹 표면보다 여유 넘게 앞이 아니면 로봇 몸 (무효 깊이 제외)
            robot |= np.isfinite(zbuf) & (observed > 0) & (observed > zbuf - margin)

        # 실루엣 경계까지 팽창 후 원 해상도로 확장
        return upsample_mask(self._dilate(robot), self._stride, depth.shape)

    @staticmethod
    def _dilate(mask: np.ndarray) -> np.ndarray:
        """bool 격자 마스크를 SELF_MASK_DILATE_CELLS 반경(정사각형)으로 팽창해 반환한다."""
        radius = config.SELF_MASK_DILATE_CELLS
        padded = np.pad(mask, radius)
        dilated = mask.copy()
        height, width = mask.shape
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                dilated |= padded[radius + dy:radius + dy + height, radius + dx:radius + dx + width]
        return dilated

    def _zbuffer(self, cam_points: np.ndarray) -> np.ndarray:
        """카메라 광학 프레임 점들을 격자에 투영해 칸마다 가장 가까운 깊이(z-buffer, 빈틈 메움)를 반환한다."""
        # 카메라 앞 점을 격자 칸 좌표로 투영
        cam_points = cam_points[cam_points[:, 2] > _MIN_PROJECT_DEPTH]
        intrinsics = self._intrinsics
        grid_u = np.floor((intrinsics.fx * cam_points[:, 0] / cam_points[:, 2] + intrinsics.ppx) / self._stride)
        grid_v = np.floor((intrinsics.fy * cam_points[:, 1] / cam_points[:, 2] + intrinsics.ppy) / self._stride)
        grid_u = grid_u.astype(np.int64)
        grid_v = grid_v.astype(np.int64)
        inside = (grid_u >= 0) & (grid_u < self._grid_w) & (grid_v >= 0) & (grid_v < self._grid_h)

        # 칸마다 최소 깊이 기록 후 빈틈 메움
        zbuf = np.full(self._grid_h * self._grid_w, np.inf, dtype=np.float32)
        np.minimum.at(zbuf, grid_v[inside] * self._grid_w + grid_u[inside], cam_points[inside, 2].astype(np.float32))
        return self._fill_gaps(zbuf.reshape(self._grid_h, self._grid_w))

    @staticmethod
    def _fill_gaps(zbuf: np.ndarray) -> np.ndarray:
        """표면 점 사이 빈 칸을 주변(SELF_MASK_FILL_CELLS 반경) 최소 깊이로 메운 z-buffer를 반환한다."""
        radius = config.SELF_MASK_FILL_CELLS
        padded = np.pad(zbuf, radius, constant_values=np.inf)
        filled = zbuf.copy()
        height, width = zbuf.shape
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                np.minimum(filled, padded[radius + dy:radius + dy + height, radius + dx:radius + dx + width], out=filled)
        return filled
