"""depth 역투영: 정렬된 depth 픽셀을 카메라 포즈로 로봇 base 프레임 3D 점으로 바꾸고 픽셀 판정(상자·확대)을 돕는다.

핀홀 역투영 (광학 프레임 x=우, y=하, z=전방):
    X = (u - ppx) / fx · d,  Y = (v - ppy) / fy · d,  Z = d
정규화 좌표 (u-ppx)/fx 는 프레임마다 같으므로 스트라이드 격자에 대해 1회 계산해 둔다.
base 프레임 점 = R_cam · p_광학 + t_cam (R_cam·t_cam: 광학 프레임 → base, PosedFrame의 카메라 포즈).
"""

import numpy as np

from teleop.camera.camera_interface import Intrinsics


class DepthProjector:
    """스트라이드 격자의 정규화 이미지 좌표를 보관하고 depth → base 점 변환을 수행한다."""

    def __init__(self, intr: Intrinsics, stride: int):
        """intr: color(=정렬 기준) 내부 파라미터, stride: 픽셀 다운샘플 간격."""
        self._stride = stride
        u = np.arange(0, intr.width, stride, dtype=np.float32)
        v = np.arange(0, intr.height, stride, dtype=np.float32)
        uu, vv = np.meshgrid(u, v)
        self._xn = (uu - intr.ppx) / intr.fx
        self._yn = (vv - intr.ppy) / intr.fy

    @property
    def stride(self) -> int:
        """픽셀 다운샘플 간격."""
        return self._stride

    def project_grid(self, depth: np.ndarray, cam_pos: np.ndarray, cam_rot: np.ndarray) -> np.ndarray:
        """스트라이드 격자 전체를 base 점 (H/s, W/s, 3) float32로 역투영해 반환한다 (무효 깊이 포함).

        depth: (H, W) float32 m. 픽셀 단위 판정(마스크 등)에 쓴다.
        """
        s = self._stride
        d = depth[::s, ::s]
        pts_cam = np.stack([self._xn * d, self._yn * d, d], axis=-1)
        rot = np.asarray(cam_rot, dtype=np.float32)
        pos = np.asarray(cam_pos, dtype=np.float32)
        return pts_cam @ rot.T + pos


def box_mask(points: np.ndarray, box) -> np.ndarray:
    """점 배열 (..., 3)이 base 프레임 상자 ((x), (y), (z)) 안에 있는지 bool (...)로 반환한다."""
    inside = np.ones(points.shape[:-1], dtype=bool)
    for axis, (lower, upper) in enumerate(box):
        inside &= (points[..., axis] > lower) & (points[..., axis] < upper)
    return inside


def upsample_mask(grid: np.ndarray, stride: int, shape: tuple[int, int]) -> np.ndarray:
    """스트라이드 격자 bool 마스크를 원 해상도 (H, W)로 확장해 반환한다 (격자 칸을 stride만큼 반복)."""
    full = np.repeat(np.repeat(grid, stride, axis=0), stride, axis=1)
    return full[: shape[0], : shape[1]]
