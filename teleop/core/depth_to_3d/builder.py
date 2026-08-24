"""정렬된 RGB-D 프레임을 WebXR 좌표계 색상 포인트클라우드로 변환한다.

핀홀 역투영: 픽셀 (u, v)와 깊이 d에 대해 광학 프레임 점은
    X = (u - ppx) / fx · d,  Y = (v - ppy) / fy · d,  Z = d
(광학 프레임: x=우, y=하, z=전방). 정규화 좌표 (u-ppx)/fx 는 프레임마다 같으므로 1회 계산해 둔다.
이후 카메라 외부 파라미터(로봇 base 기준 광학 프레임 포즈, RobotModel.camera_pose()에서 FK로 유도)로
로봇 base 프레임, 다시 기저 변환으로 WebXR 프레임까지 하나의 3x3 행렬·오프셋으로 합성해
한 번의 행렬곱으로 처리한다.
"""

import numpy as np

from teleop.utils import config
from teleop.utils.camera.base import Intrinsics, RGBDFrame
from teleop.utils.transforms import robot_points_to_webxr_matrix


class PointCloudBuilder:
    """내부 파라미터에서 유도한 역투영 격자와 합성 변환을 보관한다 (프레임 간 재사용)."""

    def __init__(self, intr: Intrinsics, cam_pos: np.ndarray, cam_rot: np.ndarray,
                 stride: int = config.PCD_STRIDE):
        """intr: color 내부 파라미터, cam_pos(3)·cam_rot(3x3): 로봇 base 기준 광학 프레임 포즈."""
        self._stride = stride
        # 스트라이드 격자의 정규화 이미지 좌표 (H/s, W/s)
        u = np.arange(0, intr.width, stride, dtype=np.float32)
        v = np.arange(0, intr.height, stride, dtype=np.float32)
        uu, vv = np.meshgrid(u, v)
        self._xn = (uu - intr.ppx) / intr.fx
        self._yn = (vv - intr.ppy) / intr.fy

        # 로봇→WebXR 기저 변환은 상수이므로 미리 잡아 둔다
        self._Ct = robot_points_to_webxr_matrix().astype(np.float32)
        self.set_extrinsics(cam_pos, cam_rot)

    def set_extrinsics(self, cam_pos: np.ndarray, cam_rot: np.ndarray):
        """카메라 외부 파라미터를 갱신한다. 반환값 없음.

        머리·토르소 관절이 움직이면 광학 프레임 포즈가 바뀌므로 프레임마다 다시 넣어야
        누적 점군이 월드에서 어긋나지 않는다.
        합성 변환: p_w = A·p_c + b,  A = Cᵀ·R_cr,  b = Cᵀ·t_cr
        (R_cr, t_cr: 광학→로봇 외부 파라미터). 행 벡터 연산을 위해 A는 전치해 보관한다.
        """
        R_cr = np.asarray(cam_rot, dtype=np.float32)
        t_cr = np.asarray(cam_pos, dtype=np.float32)
        self._A_T = (self._Ct @ R_cr).T
        self._b = self._Ct @ t_cr

    @property
    def max_points(self) -> int:
        """한 프레임에서 나올 수 있는 최대 점 개수 (격자 크기)."""
        return self._xn.size

    def build(self, frame: RGBDFrame) -> tuple[np.ndarray, np.ndarray]:
        """프레임에서 (points (N,3) float32 WebXR m, colors (N,3) uint8)을 만들어 반환한다.

        깊이가 유효 범위 밖(0 포함)인 픽셀은 제외한다.
        """
        s = self._stride
        d = frame.depth[::s, ::s]
        c = frame.color[::s, ::s]

        # 무효(0)·너무 가까운·너무 먼 깊이 제거 — 노이즈와 불필요한 원거리 점을 한 번에 걸러낸다
        mask = (d > config.PCD_MIN_DEPTH) & (d < config.PCD_MAX_DEPTH)
        d = d[mask]

        # 광학 프레임 역투영 후 합성 변환 적용
        pts_cam = np.stack([self._xn[mask] * d, self._yn[mask] * d, d], axis=1)
        pts = pts_cam @ self._A_T + self._b
        return pts.astype(np.float32, copy=False), c[mask]
