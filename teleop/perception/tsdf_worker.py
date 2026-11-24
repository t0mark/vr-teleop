"""TSDF 워커: 별도 프로세스에서 depth 프레임을 TSDF로 적분하고 바뀐 블록의 메시를 돌려준다.

메인 프로세스(asyncio 제어 루프)와 분리하는 이유: Open3D 메시 추출(수십 ms)이 GIL·CPU를 잡아
30 Hz 로봇 제어 루프를 흔들지 않게 하기 위함이다. Open3D는 워커 프로세스에서만 import한다.

두 층으로 재구성한다 (TsdfLayer):
    층 0 환경     — 작업 영역 상자 밖, 복셀 TSDF_VOXEL (방 전체를 가볍게)
    층 1 작업 영역 — WORKSPACE_BOX 안, 복셀 TSDF_WS_VOXEL (손 작업 물체 형상을 살리게)
층마다 블록은 자기 영역 픽셀(작업 영역 마스크로 구분)의 표면 근처에만 만들고, 적분에는 전체 depth를 쓴다 —
어느 광선이든 그 층의 빈 공간을 지울 수 있게 하기 위함이다. 추출한 메시는 삼각형 무게중심으로 상자 안/밖을
잘라 두 층이 겹치지 않게 한다.

프로세스 간 데이터:
    입력 큐 (메인 → 워커): (depth uint16 mm (H,W), color uint8 (H,W,3), camera-to-base 4x4, 작업 영역 픽셀 (H,W) bool)
        가득 차 있으면 새 프레임을 버린다 (지연 누적 없음).
    출력 큐 (워커 → 메인): {"changed": [(key, verts, colors, tris)], "removed": [key], "stats": {층 이름: {...}}}
        key = (층, bx, by, bz). TSDF_EXTRACT_S마다 층별 전체 메시를 추출해 층별 전송 블록(MESH_SPLIT_SIZE·_WS) 격자로 나누고,
        내용 요약(digest)이 이전과 다른 블록·사라진 블록만 보낸다.
블록 소속은 삼각형 무게중심으로 정한다 — 경계 삼각형은 한쪽 블록에만 들어가 중복되지 않는다.

동적 물체 지우기: 표면 근처 블록만 적분하면 물체가 떠난 자리(카메라와 새 표면 사이의 빈 공간)는 다시
갱신되지 않아 유령으로 남는다. 그래서 시야 안의 기존 블록도 매 프레임 적분 대상에 넣어 빈 공간으로 갱신하고,
복셀 가중치에 상한(TSDF_MAX_WEIGHT)을 둬 오래 본 표면도 몇 초 안에 바뀌게 한다. 적분의 관측 깊이 상한은
블록 생성 범위보다 넓게(TSDF_CARVE_DEPTH_MAX) 둔다 — 먼 벽까지 뻗은 광선도 그 앞 공간을 지우되
먼 곳에 새 블록은 만들지 않는다. 깊이 0(무효·로봇 몸 마스크) 화소는 적분에서 빠지므로 로봇 팔 뒤의 맵은 유지된다.
"""

import hashlib
import logging
import multiprocessing as mp
import os
import queue
import time

import numpy as np

from teleop.camera.camera_interface import Intrinsics
from teleop.utils import config
from teleop.utils.log import setup_logging

logger = logging.getLogger("tsdf_worker")

# TSDF 블록 한 변의 복셀 수 (VoxelBlockGrid block_resolution)
_BLOCK_RESOLUTION = 16
_STOP_JOIN_S = 3.0
# 워커 프로세스 연산 스레드 수를 정하는 환경 변수 (OpenMP·BLAS)
_THREAD_ENV = ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS")
# 층 번호 (메시 블록 키의 첫 원소, 브라우저도 같은 번호를 쓴다)
LAYER_ENVIRONMENT = 0
LAYER_WORKSPACE = 1


class TsdfWorker:
    """워커 프로세스와 입출력 큐를 소유한다 (메인 프로세스 쪽 핸들)."""

    def __init__(self, intr: Intrinsics):
        self._intr = intr
        self._ctx = mp.get_context("spawn")
        self._in_q = None
        self._out_q = None
        self._stop_event = None
        self._process = None
        self._dropped = 0

    @property
    def alive(self) -> bool:
        """워커 프로세스가 살아 있으면 True."""
        return self._process is not None and self._process.is_alive()

    def take_dropped(self) -> int:
        """입력 큐가 가득 차 버린 프레임 수를 반환하고 0으로 되돌린다."""
        dropped, self._dropped = self._dropped, 0
        return dropped

    def start(self):
        """워커 프로세스를 띄운다. 반환값 없음."""
        self._in_q = self._ctx.Queue(maxsize=2)
        self._out_q = self._ctx.Queue()
        self._stop_event = self._ctx.Event()
        intr = (self._intr.width, self._intr.height, self._intr.fx, self._intr.fy, self._intr.ppx, self._intr.ppy)
        self._process = self._ctx.Process(
            target=run_worker, args=(intr, self._in_q, self._out_q, self._stop_event), name="tsdf-worker", daemon=True,
        )

        # 자식의 연산 스레드 풀(numpy BLAS, Open3D TBB)은 자식이 모듈을 import하는 시점에 만들어지므로 띄우기 전에 제한한다.
        # - 스레드 수 환경 변수: 부모 환경을 잠시 바꿨다가 되돌린다
        # - CPU affinity: 자식은 띄운 스레드의 affinity를 물려받고 TBB는 그만큼만 스레드를 만든다 — 이 스레드만 잠시 좁힌다
        saved_env = {name: os.environ.get(name) for name in _THREAD_ENV}
        saved_cpus = os.sched_getaffinity(0)
        worker_cpus = set(sorted(saved_cpus)[-config.TSDF_WORKER_CPUS:])
        os.environ.update({name: "1" for name in _THREAD_ENV})
        os.sched_setaffinity(0, worker_cpus)
        try:
            self._process.start()
        finally:
            os.sched_setaffinity(0, saved_cpus)
            for name, value in saved_env.items():
                if value is None:
                    os.environ.pop(name, None)
                else:
                    os.environ[name] = value
        logger.info(f"TSDF 워커 시작 (pid {self._process.pid}, CPU 코어 {sorted(worker_cpus)})")

    def stop(self):
        """워커를 멈추고 기다린다 (블로킹 — executor에서 호출). 반환값 없음."""
        if self._process is None:
            return
        self._stop_event.set()
        self._process.join(_STOP_JOIN_S)
        if self._process.is_alive():
            logger.warning("TSDF 워커가 제때 끝나지 않아 강제 종료")
            self._process.terminate()
            self._process.join(_STOP_JOIN_S)
        # 남은 큐 자원 정리 (feeder 스레드가 종료를 막지 않게)
        for q in (self._in_q, self._out_q):
            q.cancel_join_thread()
            q.close()
        self._process = None
        logger.info("TSDF 워커 정지")

    def submit(self, depth_mm: np.ndarray, color: np.ndarray, cam_to_base: np.ndarray, workspace: np.ndarray) -> bool:
        """프레임 하나를 워커로 보낸다. 반환: 전달 여부 (큐가 가득 차면 버리고 False).

        workspace: 관측 점이 작업 영역 상자 안인 픽셀 (H, W) bool.
        """
        try:
            self._in_q.put_nowait((depth_mm, color, cam_to_base, workspace))
            return True
        except queue.Full:
            self._dropped += 1
            return False

    def poll(self) -> list[dict]:
        """워커가 보낸 결과를 모두 꺼내 반환한다 (없으면 빈 목록)."""
        results = []
        while True:
            try:
                results.append(self._out_q.get_nowait())
            except queue.Empty:
                return results


class TsdfLayer:
    """한 층의 VoxelBlockGrid, 영역 규칙, 블록 요약(digest)을 소유한다 (워커 프로세스 전용)."""

    def __init__(self, layer_id: int, name: str, voxel: float, depth_max: float, keep_inside: bool, K: np.ndarray,
                 split_size: float):
        """keep_inside: True면 작업 영역 상자 안, False면 밖이 이 층의 영역이다. split_size: 전송 블록 한 변(m)."""
        import open3d as o3d
        import open3d.core as o3c

        self._o3d = o3d
        self._o3c = o3c
        self.layer_id = layer_id
        self.name = name
        self._voxel = voxel
        self._depth_max = depth_max
        self._keep_inside = keep_inside
        self._K = K
        self._intrinsic = o3c.Tensor(K, dtype=o3c.float64)
        self._block_size = voxel * _BLOCK_RESOLUTION
        self._split_size = split_size
        self._vbg = o3d.t.geometry.VoxelBlockGrid(
            attr_names=("tsdf", "weight", "color"),
            attr_dtypes=(o3c.float32, o3c.float32, o3c.float32),
            attr_channels=((1), (1), (3)),
            voxel_size=voxel,
            block_resolution=_BLOCK_RESOLUTION,
            block_count=config.TSDF_BLOCK_COUNT,
            device=o3c.Device("CPU:0"),
        )
        self._digests: dict[tuple, bytes] = {}
        self._integrate_ms = 0.0
        self._integrated = 0
        logger.info(f"TSDF 층 {name}: 복셀 {voxel * 1000:.0f} mm, TSDF 블록 {self._block_size:.3f} m, 블록 생성 깊이 ≤{depth_max:.1f} m")

    def integrate(self, depth_mm: np.ndarray, region: np.ndarray, color, cam_to_base: np.ndarray):
        """프레임 하나를 적분한다: 블록 생성은 이 층 영역 픽셀로, 적분(빈 공간 갱신 포함)은 전체 depth로. 반환값 없음.

        color: Open3D Image (층 공용으로 한 번 만든 것).
        """
        o3d, o3c = self._o3d, self._o3c
        t0 = time.perf_counter()
        height, width = depth_mm.shape
        extrinsic = o3c.Tensor(np.linalg.inv(cam_to_base), dtype=o3c.float64)
        full = o3d.t.geometry.Image(o3c.Tensor(depth_mm))

        # 블록 생성용 depth: 이 층 영역 픽셀만
        own = np.where(region, depth_mm, 0).astype(np.uint16)
        surface = self._surface_blocks(own, extrinsic)

        # 적분 블록 = 관측 표면 근처 블록 ∪ 시야 안 기존 블록 (빈 공간 갱신)
        hashmap = self._vbg.hashmap()
        existing = hashmap.key_tensor()[hashmap.active_buf_indices().to(o3c.int64)].numpy()
        visible = frustum_blocks(existing, self._block_size, cam_to_base, self._K, width, height, self._depth_max)
        coords = np.unique(np.concatenate([surface, visible]), axis=0).astype(np.int32)
        if len(coords) > 0:
            self._vbg.integrate(o3c.Tensor(coords), full, color, self._intrinsic, self._intrinsic, extrinsic,
                                1000.0, config.TSDF_CARVE_DEPTH_MAX)

            # 가중치 상한: 텐서 메모리를 공유하는 numpy 뷰에서 바로 자른다
            weight = self._vbg.attribute("weight").numpy()
            np.minimum(weight, config.TSDF_MAX_WEIGHT, out=weight)
        self._integrate_ms += (time.perf_counter() - t0) * 1000
        self._integrated += 1

    def _surface_blocks(self, own_depth_mm: np.ndarray, extrinsic) -> np.ndarray:
        """이 층 영역 depth의 관측 표면 근처 블록 키 (N,3) int32를 반환한다 (표면이 없으면 빈 배열).

        Open3D는 건드린 블록이 하나도 없으면 "No block is touched" 예외로 알린다 — 이 층 영역에 이번 프레임
        표면이 없다는 정상 상황이므로 빈 결과로 바꾸고, 다른 오류는 그대로 올린다.
        """
        o3d, o3c = self._o3d, self._o3c
        if not np.any(own_depth_mm):
            return np.zeros((0, 3), dtype=np.int32)
        try:
            return self._vbg.compute_unique_block_coordinates(
                o3d.t.geometry.Image(o3c.Tensor(own_depth_mm)), self._intrinsic, extrinsic, 1000.0, self._depth_max,
            ).numpy()
        except RuntimeError as e:
            if "No block is touched" not in str(e):
                raise
            return np.zeros((0, 3), dtype=np.int32)

    def extract_changes(self) -> tuple[list, list, dict]:
        """메시를 추출해 영역 밖 삼각형을 버리고 블록 비교 결과 (바뀐 블록, 사라진 키, 통계)를 반환한다."""
        t0 = time.perf_counter()
        mesh = self._vbg.extract_triangle_mesh(weight_threshold=config.TSDF_WEIGHT_THRESHOLD)
        verts = mesh.vertex.positions.numpy() if "positions" in mesh.vertex else np.zeros((0, 3), np.float32)
        colors = mesh.vertex.colors.numpy() if "colors" in mesh.vertex else np.zeros((0, 3), np.float32)
        tris = mesh.triangle.indices.numpy() if "indices" in mesh.triangle else np.zeros((0, 3), np.int64)
        extract_ms = (time.perf_counter() - t0) * 1000

        # 층 영역 삼각형만 남기고 블록 분할·비교
        t0 = time.perf_counter()
        tris = crop_triangles(verts, tris, config.WORKSPACE_BOX, self._keep_inside)
        blocks = split_blocks(verts, colors, tris, self._split_size)
        changed, removed = diff_blocks(blocks, self._digests)
        split_ms = (time.perf_counter() - t0) * 1000

        stats = {
            "frames": self._integrated,
            "integrate_ms": self._integrate_ms / max(self._integrated, 1),
            "extract_ms": extract_ms,
            "split_ms": split_ms,
            "blocks": len(self._digests),
            "tris": len(tris),
            "changed": len(changed),
            "removed": len(removed),
        }
        self._integrate_ms = 0.0
        self._integrated = 0
        changed = [((self.layer_id, *key), *data) for key, *data in changed]
        removed = [(self.layer_id, *key) for key in removed]
        return changed, removed, stats


def run_worker(intr, in_q, out_q, stop_event):
    """워커 프로세스 본체: 두 층 적분 → 주기적 메시 추출·블록 비교 → 결과 송신."""
    setup_logging()
    import open3d as o3d
    import open3d.core as o3c

    width, height, fx, fy, ppx, ppy = intr
    K = np.array([[fx, 0, ppx], [0, fy, ppy], [0, 0, 1]], dtype=np.float64)
    layers = [
        TsdfLayer(LAYER_ENVIRONMENT, "환경", config.TSDF_VOXEL, config.TSDF_DEPTH_MAX, False, K, config.MESH_SPLIT_SIZE),
        TsdfLayer(LAYER_WORKSPACE, "작업 영역", config.TSDF_WS_VOXEL, config.TSDF_WS_DEPTH_MAX, True, K,
                  config.MESH_SPLIT_SIZE_WS),
    ]
    logger.info(
        f"TSDF 워커 준비: 작업 영역 {config.WORKSPACE_BOX}, 빈 공간 갱신 ≤{config.TSDF_CARVE_DEPTH_MAX:.1f} m, "
        f"가중치 상한 {config.TSDF_MAX_WEIGHT:.0f}, 추출 주기 {config.TSDF_EXTRACT_S:.1f}초"
    )

    integrated = 0
    last_extract = time.monotonic()
    while not stop_event.is_set():
        # 프레임 적분 (대기는 짧게 — 추출 주기와 종료 신호를 놓치지 않게)
        try:
            depth_mm, color, cam_to_base, workspace = in_q.get(timeout=0.1)
        except queue.Empty:
            depth_mm = None
        if depth_mm is not None:
            image = o3d.t.geometry.Image(o3c.Tensor(color))
            for layer in layers:
                region = workspace if layer.layer_id == LAYER_WORKSPACE else ~workspace
                layer.integrate(depth_mm, region, image, cam_to_base)
            integrated += 1

        # 주기적 메시 추출: 새로 적분한 프레임이 있을 때만
        now = time.monotonic()
        if integrated == 0 or now - last_extract < config.TSDF_EXTRACT_S:
            continue
        changed, removed, stats = [], [], {}
        for layer in layers:
            layer_changed, layer_removed, stats[layer.name] = layer.extract_changes()
            changed += layer_changed
            removed += layer_removed
        out_q.put({"changed": changed, "removed": removed, "stats": stats})
        integrated = 0
        last_extract = now
    logger.info("TSDF 워커 종료")


def frustum_blocks(keys: np.ndarray, block_size: float, cam_to_base: np.ndarray, K: np.ndarray,
                   width: int, height: int, depth_max: float) -> np.ndarray:
    """기존 블록 키 (N,3) 중 시야 안(외접구가 이미지와 겹치고 depth_max 이내)인 것만 반환한다."""
    if len(keys) == 0:
        return keys.astype(np.int32).reshape(0, 3)

    # 블록 중심을 카메라 광학 프레임으로
    centers = (keys.astype(np.float64) + 0.5) * block_size
    base_to_cam = np.linalg.inv(cam_to_base)
    pc = centers @ base_to_cam[:3, :3].T + base_to_cam[:3, 3]
    radius = block_size * np.sqrt(3.0) / 2.0

    # 깊이 범위 + 이미지 영역(블록 외접구 반지름만큼 여유) 판정
    z = pc[:, 2]
    inside = (z > -radius) & (z < depth_max + radius)
    zc = np.maximum(z, 0.05)
    u = K[0, 0] * pc[:, 0] / zc + K[0, 2]
    v = K[1, 1] * pc[:, 1] / zc + K[1, 2]
    margin = K[0, 0] * radius / zc
    inside &= (u > -margin) & (u < width + margin) & (v > -margin) & (v < height + margin)
    return keys[inside].astype(np.int32)


def crop_triangles(verts: np.ndarray, tris: np.ndarray, box, keep_inside: bool) -> np.ndarray:
    """삼각형 무게중심이 상자 안(keep_inside) 또는 밖인 삼각형만 남겨 반환한다."""
    if len(tris) == 0:
        return tris
    centroids = verts[tris].mean(axis=1)
    inside = np.ones(len(tris), dtype=bool)
    for axis, (lower, upper) in enumerate(box):
        inside &= (centroids[:, axis] > lower) & (centroids[:, axis] < upper)
    return tris[inside if keep_inside else ~inside]


def split_blocks(verts: np.ndarray, colors: np.ndarray, tris: np.ndarray, block_size: float) -> dict:
    """메시를 삼각형 무게중심 기준 블록으로 나눠 {key: (verts (n,3) f32, colors (n,3) u8, tris (m,3) u32)}로 반환한다."""
    if len(tris) == 0:
        return {}
    centroids = verts[tris].mean(axis=1)
    keys = np.floor(centroids / block_size).astype(np.int32)
    uniq, inverse = np.unique(keys, axis=0, return_inverse=True)
    inverse = inverse.ravel()
    order = np.argsort(inverse, kind="stable")
    bounds = np.searchsorted(inverse[order], np.arange(len(uniq) + 1))
    colors_u8 = np.clip(np.rint(colors * 255.0), 0, 255).astype(np.uint8)

    blocks = {}
    for b, key in enumerate(uniq):
        block_tris = tris[order[bounds[b]:bounds[b + 1]]]
        # 블록이 쓰는 정점만 모아 인덱스를 다시 매긴다
        used, local = np.unique(block_tris.ravel(), return_inverse=True)
        blocks[tuple(int(k) for k in key)] = (
            verts[used].astype(np.float32),
            colors_u8[used],
            local.reshape(-1, 3).astype(np.uint32),
        )
    return blocks


def diff_blocks(blocks: dict, digests: dict) -> tuple[list, list]:
    """이전 요약(digests)과 비교해 (바뀐 블록 목록, 사라진 키 목록)을 반환하고 digests를 갱신한다.

    요약은 mm 정수화한 정점·색을 정렬해 만든다 — 추출할 때마다 정점 순서가 달라져도 같은 형상이면 같은 값.
    """
    changed = []
    for key, (verts, colors, tris) in blocks.items():
        rows = np.concatenate([np.rint(verts * 1000.0).astype(np.int32), colors.astype(np.int32)], axis=1)
        rows = rows[np.lexsort(rows.T[::-1])]
        digest = hashlib.blake2b(rows.tobytes() + len(tris).to_bytes(4, "little"), digest_size=16).digest()
        if digests.get(key) != digest:
            digests[key] = digest
            changed.append((key, verts, colors, tris))

    removed = [key for key in digests if key not in blocks]
    for key in removed:
        del digests[key]
    return changed, removed
