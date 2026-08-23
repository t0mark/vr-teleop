"""복셀 격자 시간 누적: 프레임마다 들어오는 점을 복셀 단위로 병합해 중복 없이 최근 관측만 유지한다.

규칙:
- 같은 복셀에 여러 점이 있으면 **가장 최근 프레임의 점 하나**만 남긴다 (표면이 프레임마다 중복 쌓이지 않음)
- 복셀이 최근 PCD_ACCUM_FRAMES 프레임 동안 한 번도 관측되지 않으면 버린다 (움직인 물체의 잔상 소멸)
- 같은 프레임 안의 중복은 격자 다운샘플 효과로 함께 제거된다
복셀 키: 축별 정수 인덱스(i, j, k)를 int64 하나로 패킹 — 좌표 범위 ±(GRID/2·voxel) m 가정.
"""

import numpy as np

from teleop.utils import config

# 축별 격자 칸 수 (2^21 → 0.02 m 복셀이면 ±21 km, 키 패킹 시 3축 합 63비트로 int64에 들어감)
_GRID_BITS = 21
_GRID_HALF = 1 << (_GRID_BITS - 1)


class VoxelAccumulator:
    """병합된 점·색·관측 프레임 번호 배열을 보관한다 (프레임 간 유지 상태)."""

    def __init__(self, voxel_m: float = config.PCD_VOXEL_M, horizon: int = config.PCD_ACCUM_FRAMES):
        self._inv_voxel = 1.0 / voxel_m
        self._horizon = horizon
        self._frame = 0
        self._keys = np.empty(0, dtype=np.int64)
        self._pts = np.empty((0, 3), dtype=np.float32)
        self._cols = np.empty((0, 3), dtype=np.uint8)
        self._stamp = np.empty(0, dtype=np.int64)

    @property
    def size(self) -> int:
        """현재 보관 중인 복셀(점) 수."""
        return self._keys.shape[0]

    def _voxel_keys(self, pts: np.ndarray) -> np.ndarray:
        """점 (N,3)을 복셀 정수 키 (N,) int64로 변환한다."""
        idx = np.floor(pts * self._inv_voxel).astype(np.int64) + _GRID_HALF
        # 각 축을 _GRID_BITS 비트 필드에 담아 충돌 없는 단일 키로 패킹
        return (idx[:, 0] << (2 * _GRID_BITS)) | (idx[:, 1] << _GRID_BITS) | idx[:, 2]

    def add(self, pts: np.ndarray, cols: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """새 프레임의 점·색을 병합하고, 누적 결과 (points (M,3) float32, colors (M,3) uint8)를 반환한다."""
        self._frame += 1
        new_keys = self._voxel_keys(pts)
        new_stamp = np.full(new_keys.shape[0], self._frame, dtype=np.int64)

        # 새 점을 앞에 두고 안정 unique(첫 등장 유지)를 쓰면 같은 복셀에서 최신 관측이 이긴다
        keys = np.concatenate([new_keys, self._keys])
        all_pts = np.concatenate([pts, self._pts])
        all_cols = np.concatenate([cols, self._cols])
        stamp = np.concatenate([new_stamp, self._stamp])
        _, first = np.unique(keys, return_index=True)

        # 기한(horizon) 안에 관측된 복셀만 유지
        alive = first[stamp[first] > self._frame - self._horizon]
        self._keys = keys[alive]
        self._pts = all_pts[alive]
        self._cols = all_cols[alive]
        self._stamp = stamp[alive]
        return self._pts, self._cols
