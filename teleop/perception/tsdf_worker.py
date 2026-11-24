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
        한 장만 담고, 가득 차 있으면 대기 중인 옛 프레임을 버리고 새 프레임으로 바꾼다 (최신 우선, 지연 누적 없음).
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
# 워커 프로세스 종료 대기 한도(초)
_STOP_JOIN_S = 3.0
# 워커 프로세스 연산 스레드 수를 정하는 환경 변수 (OpenMP·BLAS)
_THREAD_ENV = ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS")
# 메시 추출이 추출 주기보다 오래 걸릴 때 경고 로그의 최소 간격(초)
_SLOW_EXTRACT_LOG_S = 10.0
# 전송 블록 키 정수 인코딩: 축마다 비트 수와 음수 좌표를 양수로 옮기는 편향
_KEY_AXIS_BITS = 21
_KEY_AXIS_BIAS = 1 << (_KEY_AXIS_BITS - 1)
_KEY_AXIS_MASK = (1 << _KEY_AXIS_BITS) - 1
# 변경 판정 요약의 정점 위치 양자화 값을 축마다 담는 비트 수와 편향
_DIGEST_AXIS_BITS = 16
_DIGEST_AXIS_BIAS = 1 << (_DIGEST_AXIS_BITS - 1)
# 투영에 쓰는 최소 깊이(m)
_MIN_PROJECT_DEPTH = 0.05
# 층 번호 (메시 블록 키의 첫 원소, 브라우저도 같은 번호를 쓴다)
LAYER_ENVIRONMENT = 0
LAYER_WORKSPACE = 1


class TsdfWorker:
    """워커 프로세스와 입출력 큐를 소유한다 (메인 프로세스 쪽 핸들)."""

    def __init__(self, intrinsics: Intrinsics):
        """intrinsics: color(=정렬 기준) 내부 파라미터."""
        self._intrinsics = intrinsics
        self._ctx = mp.get_context("spawn")

        # 입출력 큐·종료 신호·프로세스 (start에서 생성)와 버린 프레임 수
        self._in_q = None
        self._out_q = None
        self._stop_event = None
        self._process = None
        self._dropped = 0

    @property
    def alive(self) -> bool:
        """워커 프로세스가 살아 있으면 True."""
        return self._process is not None and self._process.is_alive()

    @property
    def exit_code(self) -> int | None:
        """워커 프로세스 종료 코드 (실행 중이거나 시작 전이면 None)."""
        return None if self._process is None else self._process.exitcode

    def take_dropped(self) -> int:
        """입력 큐가 가득 차 버린 프레임 수를 반환하고 0으로 되돌린다."""
        dropped, self._dropped = self._dropped, 0
        return dropped

    def start(self):
        """워커 프로세스를 띄운다. 반환값 없음."""
        # 입출력 큐·종료 신호·프로세스 생성
        self._in_q = self._ctx.Queue(maxsize=1)
        self._out_q = self._ctx.Queue()
        self._stop_event = self._ctx.Event()
        intrinsics = self._intrinsics
        intrinsics_tuple = (intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx,
                            intrinsics.ppy)
        self._process = self._ctx.Process(
            target=run_worker, args=(intrinsics_tuple, self._in_q, self._out_q, self._stop_event), name="tsdf-worker",
            daemon=True,
        )

        # 스레드 수 환경 변수·CPU affinity를 워커용으로 잠시 바꿔 띄운 뒤 되돌림
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

        # 종료 신호 후 대기, 시간 안에 안 끝나면 강제 종료
        self._stop_event.set()
        self._process.join(_STOP_JOIN_S)
        if self._process.is_alive():
            logger.warning("TSDF 워커가 제때 끝나지 않아 강제 종료")
            self._process.terminate()
            self._process.join(_STOP_JOIN_S)

        # 큐 feeder 스레드 대기 해제 후 큐 닫기
        for worker_queue in (self._in_q, self._out_q):
            worker_queue.cancel_join_thread()
            worker_queue.close()
        self._process = None
        logger.info("TSDF 워커 정지")

    def submit(self, depth_mm: np.ndarray, color: np.ndarray, cam_to_base: np.ndarray, workspace: np.ndarray) -> bool:
        """프레임 하나를 워커로 보낸다. 반환: 전달 여부 (옛 프레임을 꺼낼 수 없어 새 프레임을 버리면 False).

        큐가 가득 차 있으면 대기 중인 옛 프레임을 꺼내 버리고 새 프레임을 넣는다 (버린 수는 take_dropped로 센다).
        workspace: 관측 점이 작업 영역 상자 안인 픽셀 (H, W) bool.
        """
        # 빈 자리가 있으면 바로 넣기
        item = (depth_mm, color, cam_to_base, workspace)
        try:
            self._in_q.put_nowait(item)
            return True
        except queue.Full:
            pass

        # 대기 중인 옛 프레임을 버리고 새 프레임으로 교체
        self._dropped += 1
        try:
            self._in_q.get_nowait()
        except queue.Empty:
            pass
        try:
            self._in_q.put_nowait(item)
            return True
        except queue.Full:
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

    def __init__(self, layer_id: int, name: str, voxel: float, depth_max: float, keep_inside: bool,
                 camera_matrix: np.ndarray, split_size: float):
        """keep_inside: True면 작업 영역 상자 안, False면 밖이 이 층의 영역이다. split_size: 전송 블록 한 변(m).

        camera_matrix: 핀홀 내부 파라미터 3x3 행렬.
        """
        # Open3D 모듈 (워커 프로세스에서만 로드)
        import open3d as o3d
        import open3d.core as o3c

        # 층 설정
        self._o3d = o3d
        self._o3c = o3c
        self._layer_id = layer_id
        self._name = name
        self._voxel = voxel
        self._depth_max = depth_max
        self._keep_inside = keep_inside
        self._camera_matrix = camera_matrix
        self._intrinsic = o3c.Tensor(camera_matrix, dtype=o3c.float64)
        self._block_size = voxel * _BLOCK_RESOLUTION
        self._split_size = split_size

        # 복셀 블록 격자·블록 요약·적분 통계
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

    @property
    def layer_id(self) -> int:
        """층 번호 (LAYER_ENVIRONMENT·LAYER_WORKSPACE)."""
        return self._layer_id

    @property
    def name(self) -> str:
        """층 이름 (로그·통계 키)."""
        return self._name

    def integrate(self, depth_mm: np.ndarray, region: np.ndarray, color, cam_to_base: np.ndarray):
        """프레임 하나를 적분한다: 블록 생성은 이 층 영역 픽셀로, 적분(빈 공간 갱신 포함)은 전체 depth로. 반환값 없음.

        color: Open3D Image (층 공용으로 한 번 만든 것).
        """
        o3d, o3c = self._o3d, self._o3c
        integrate_start = time.perf_counter()
        height, width = depth_mm.shape
        extrinsic = o3c.Tensor(np.linalg.inv(cam_to_base), dtype=o3c.float64)
        full_depth = o3d.t.geometry.Image(o3c.Tensor(depth_mm))

        # 블록 생성용 depth: 이 층 영역 픽셀만
        own_depth = np.where(region, depth_mm, 0).astype(np.uint16)
        surface = self._surface_blocks(own_depth, extrinsic)

        # 적분 블록 = 관측 표면 근처 블록 ∪ 시야 안 기존 블록 (빈 공간 갱신)
        hashmap = self._vbg.hashmap()
        existing = hashmap.key_tensor()[hashmap.active_buf_indices().to(o3c.int64)].numpy()
        visible = frustum_blocks(existing, self._block_size, cam_to_base, self._camera_matrix, width, height,
                                 self._depth_max)
        coords = np.unique(np.concatenate([surface, visible]), axis=0).astype(np.int32)
        if len(coords) > 0:
            self._vbg.integrate(o3c.Tensor(coords), full_depth, color, self._intrinsic, self._intrinsic, extrinsic,
                                1000.0, config.TSDF_CARVE_DEPTH_MAX)

            # 이번 프레임에 적분한 블록의 가중치를 상한으로 자르기
            buf_indices, found = self._vbg.hashmap().find(o3c.Tensor(coords))
            touched = buf_indices.to(o3c.int64).numpy()[found.numpy()]
            weight = self._vbg.attribute("weight").numpy()
            weight[touched] = np.minimum(weight[touched], config.TSDF_MAX_WEIGHT)

        # 적분 통계 누적
        self._integrate_ms += (time.perf_counter() - integrate_start) * 1000
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
        except RuntimeError as error:
            if "No block is touched" not in str(error):
                raise
            return np.zeros((0, 3), dtype=np.int32)

    def extract_changes(self) -> tuple[list, list, dict]:
        """메시를 추출해 영역 밖 삼각형을 버리고 블록 비교 결과 (바뀐 블록, 사라진 키, 통계)를 반환한다."""
        # 층 전체 메시 추출
        extract_start = time.perf_counter()
        mesh = self._vbg.extract_triangle_mesh(weight_threshold=config.TSDF_WEIGHT_THRESHOLD)
        verts = mesh.vertex.positions.numpy() if "positions" in mesh.vertex else np.zeros((0, 3), np.float32)
        colors = mesh.vertex.colors.numpy() if "colors" in mesh.vertex else np.zeros((0, 3), np.float32)
        tris = mesh.triangle.indices.numpy() if "indices" in mesh.triangle else np.zeros((0, 3), np.int64)
        extract_ms = (time.perf_counter() - extract_start) * 1000

        # 층 영역 삼각형만 남기고 블록 분할·비교
        split_start = time.perf_counter()
        tris = crop_triangles(verts, tris, config.WORKSPACE_BOX, self._keep_inside)
        blocks = split_blocks(verts, colors, tris, self._split_size)
        changed, removed = diff_blocks(blocks, self._digests)
        split_ms = (time.perf_counter() - split_start) * 1000

        # 층 통계 작성 후 적분 통계 초기화
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

        # 블록 키 앞에 층 번호 붙이기
        changed = [((self._layer_id, *key), *data) for key, *data in changed]
        removed = [(self._layer_id, *key) for key in removed]
        return changed, removed, stats


def run_worker(intrinsics_tuple, in_q, out_q, stop_event):
    """워커 프로세스 본체: 두 층 적분 → 주기적 메시 추출·블록 비교 → 결과 송신.

    intrinsics_tuple: (width, height, fx, fy, ppx, ppy).
    """
    setup_logging()

    # Open3D 모듈 (워커 프로세스에서만 로드)
    import open3d as o3d
    import open3d.core as o3c

    # 프로세스 종료 시 출력 큐의 미전송 결과 flush 대기를 생략
    out_q.cancel_join_thread()

    # 두 층(환경·작업 영역) 생성
    _, _, fx, fy, ppx, ppy = intrinsics_tuple
    camera_matrix = np.array([[fx, 0, ppx], [0, fy, ppy], [0, 0, 1]], dtype=np.float64)
    layers = [
        TsdfLayer(LAYER_ENVIRONMENT, "환경", config.TSDF_VOXEL, config.TSDF_DEPTH_MAX, False, camera_matrix,
                  config.MESH_SPLIT_SIZE),
        TsdfLayer(LAYER_WORKSPACE, "작업 영역", config.TSDF_WS_VOXEL, config.TSDF_WS_DEPTH_MAX, True, camera_matrix,
                  config.MESH_SPLIT_SIZE_WS),
    ]
    logger.info(
        f"TSDF 워커 준비: 작업 영역 {config.WORKSPACE_BOX}, 빈 공간 갱신 ≤{config.TSDF_CARVE_DEPTH_MAX:.1f} m, "
        f"가중치 상한 {config.TSDF_MAX_WEIGHT:.0f}, 추출 주기 {config.TSDF_EXTRACT_S:.1f}초"
    )

    integrated = 0
    last_extract = time.monotonic()
    last_slow_log = 0.0
    while not stop_event.is_set():
        # 프레임 하나를 받아 두 층에 적분 (0.1초 대기)
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

        # 층별 바뀐 블록·사라진 블록을 모아 송신
        changed, removed, stats = [], [], {}
        for layer in layers:
            layer_changed, layer_removed, stats[layer.name] = layer.extract_changes()
            changed += layer_changed
            removed += layer_removed
        out_q.put({"changed": changed, "removed": removed, "stats": stats})
        integrated = 0
        last_extract = time.monotonic()

        # 추출이 추출 주기보다 오래 걸리면 경고 (간격 제한)
        extract_s = last_extract - now
        if extract_s > config.TSDF_EXTRACT_S and last_extract - last_slow_log >= _SLOW_EXTRACT_LOG_S:
            logger.warning(f"메시 추출 {extract_s * 1000:.0f} ms가 추출 주기 {config.TSDF_EXTRACT_S:.1f}초보다 김 — 적분 프레임 감소")
            last_slow_log = last_extract
    logger.info("TSDF 워커 종료")


def frustum_blocks(keys: np.ndarray, block_size: float, cam_to_base: np.ndarray, camera_matrix: np.ndarray,
                   width: int, height: int, depth_max: float) -> np.ndarray:
    """기존 블록 키 (N,3) 중 시야 안(외접구가 이미지와 겹치고 depth_max 이내)인 것만 반환한다."""
    if len(keys) == 0:
        return keys.astype(np.int32).reshape(0, 3)

    # 블록 중심을 카메라 광학 프레임으로
    centers = (keys.astype(np.float64) + 0.5) * block_size
    base_to_cam = np.linalg.inv(cam_to_base)
    cam_centers = centers @ base_to_cam[:3, :3].T + base_to_cam[:3, 3]
    radius = block_size * np.sqrt(3.0) / 2.0

    # 깊이 범위 + 이미지 영역(블록 외접구 반지름만큼 여유) 판정
    depth = cam_centers[:, 2]
    inside = (depth > -radius) & (depth < depth_max + radius)
    safe_depth = np.maximum(depth, _MIN_PROJECT_DEPTH)
    pixel_u = camera_matrix[0, 0] * cam_centers[:, 0] / safe_depth + camera_matrix[0, 2]
    pixel_v = camera_matrix[1, 1] * cam_centers[:, 1] / safe_depth + camera_matrix[1, 2]
    margin = camera_matrix[0, 0] * radius / safe_depth
    inside &= (pixel_u > -margin) & (pixel_u < width + margin) & (pixel_v > -margin) & (pixel_v < height + margin)
    return keys[inside].astype(np.int32)


def crop_triangles(verts: np.ndarray, tris: np.ndarray, box, keep_inside: bool) -> np.ndarray:
    """삼각형 무게중심이 상자 안(keep_inside) 또는 밖인 삼각형만 남겨 반환한다."""
    if len(tris) == 0:
        return tris

    # 무게중심의 상자 안 여부 판정
    centroids = verts[tris].mean(axis=1)
    inside = np.ones(len(tris), dtype=bool)
    for axis, (lower, upper) in enumerate(box):
        inside &= (centroids[:, axis] > lower) & (centroids[:, axis] < upper)
    return tris[inside if keep_inside else ~inside]


def split_blocks(verts: np.ndarray, colors: np.ndarray, tris: np.ndarray, block_size: float) -> dict:
    """메시를 삼각형 무게중심 기준 블록으로 나눠 {key: (verts (n,3) f32, colors (n,3) u8, tris (m,3) u32)}로 반환한다."""
    if len(tris) == 0:
        return {}
    tris = tris.astype(np.int64)

    # 삼각형마다 블록 키를 정수 하나로 인코딩해 블록 순으로 정렬
    centroids = verts[tris].mean(axis=1)
    tri_codes = encode_block_keys(np.floor(centroids / block_size).astype(np.int64))
    order = np.argsort(tri_codes, kind="stable")
    sorted_codes = tri_codes[order]
    sorted_tris = tris[order]
    tri_starts = np.flatnonzero(np.r_[True, sorted_codes[1:] != sorted_codes[:-1]])
    tri_bounds = np.r_[tri_starts, len(order)]
    block_keys = decode_block_keys(sorted_codes[tri_starts])
    tri_block = np.repeat(np.arange(len(tri_starts), dtype=np.int64), np.diff(tri_bounds))

    # 블록이 쓰는 정점을 (블록, 정점) 쌍으로 모아 블록별 지역 인덱스를 한 번에 매긴다
    pair_codes = tri_block[:, None] * len(verts) + sorted_tris
    used_pairs, pair_local = np.unique(pair_codes.ravel(), return_inverse=True)
    used_block = used_pairs // len(verts)
    used_verts = used_pairs % len(verts)
    vert_bounds = np.searchsorted(used_block, np.arange(len(tri_starts) + 1))
    local_tris = (pair_local.reshape(-1, 3) - vert_bounds[tri_block][:, None]).astype(np.uint32)
    block_verts = verts[used_verts].astype(np.float32)
    block_colors = np.clip(np.rint(colors[used_verts] * 255.0), 0, 255).astype(np.uint8)

    # 블록별로 정점·색·삼각형 구간을 잘라 담기
    blocks = {}
    for block_index, key in enumerate(block_keys):
        vert_slice = slice(vert_bounds[block_index], vert_bounds[block_index + 1])
        tri_slice = slice(tri_bounds[block_index], tri_bounds[block_index + 1])
        blocks[key] = (block_verts[vert_slice], block_colors[vert_slice], local_tris[tri_slice])
    return blocks


def encode_block_keys(keys: np.ndarray) -> np.ndarray:
    """블록 키 (N,3) 정수를 축마다 _KEY_AXIS_BITS 비트로 묶은 int64 (N,)로 반환한다."""
    biased = (keys.astype(np.int64) + _KEY_AXIS_BIAS) & _KEY_AXIS_MASK
    return (biased[:, 0] << (2 * _KEY_AXIS_BITS)) | (biased[:, 1] << _KEY_AXIS_BITS) | biased[:, 2]


def decode_block_keys(codes: np.ndarray) -> list[tuple[int, int, int]]:
    """encode_block_keys로 묶은 int64 (N,)를 블록 키 튜플 목록으로 되돌려 반환한다."""
    axes = np.stack([codes >> (2 * _KEY_AXIS_BITS), codes >> _KEY_AXIS_BITS, codes], axis=1) & _KEY_AXIS_MASK
    return [tuple(int(k) for k in key) for key in (axes - _KEY_AXIS_BIAS)]


def diff_blocks(blocks: dict, digests: dict) -> tuple[list, list]:
    """이전 요약(digests)과 비교해 (바뀐 블록 목록, 사라진 키 목록)을 반환하고 digests를 갱신한다.

    요약은 양자화한 정점 위치·색(MESH_DIGEST_POS_STEP·MESH_DIGEST_COLOR_BITS)을 정렬해 만든다 —
    추출할 때마다 정점 순서가 달라져도 같은 형상이면 같은 값.
    """
    color_shift = 8 - config.MESH_DIGEST_COLOR_BITS
    changed = []
    for key, (verts, colors, tris) in blocks.items():
        # 정점 위치·색을 정수 코드 두 개로 양자화
        quantized = np.rint(verts / config.MESH_DIGEST_POS_STEP).astype(np.int64) + _DIGEST_AXIS_BIAS
        quantized = np.clip(quantized, 0, (1 << _DIGEST_AXIS_BITS) - 1)
        pos_codes = ((quantized[:, 0] << (2 * _DIGEST_AXIS_BITS)) | (quantized[:, 1] << _DIGEST_AXIS_BITS)
                     | quantized[:, 2])
        color_bits = colors.astype(np.int64) >> color_shift
        color_codes = (color_bits[:, 0] << 16) | (color_bits[:, 1] << 8) | color_bits[:, 2]

        # 정점 순서와 무관하도록 정렬해 요약
        sort_order = np.lexsort((color_codes, pos_codes))
        rows = np.stack([pos_codes[sort_order], color_codes[sort_order]], axis=1)
        digest = hashlib.blake2b(rows.tobytes() + len(tris).to_bytes(4, "little"), digest_size=16).digest()
        if digests.get(key) != digest:
            digests[key] = digest
            changed.append((key, verts, colors, tris))

    removed = [key for key in digests if key not in blocks]
    for key in removed:
        del digests[key]
    return changed, removed
