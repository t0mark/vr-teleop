"""프레임 포즈: RGB-D 프레임에 촬영 시각의 관절값·카메라 포즈를 붙인다.

머리가 움직이는 중에는 "최신 관절값"과 "프레임 촬영 시각"이 수십 ms 어긋나 점군이 번진다. 관절 측정값을
고주파로 폴링해 시각과 함께 이력에 쌓아 두고, 프레임 수신 시각(t_mono)에서 선형 보간한 관절값으로 FK를 한다.
머리 관절 속도도 함께 구해 소비자(TSDF)가 빠르게 움직이는 프레임을 거를 수 있게 한다.

FK는 RobotModel의 pinocchio Data를 다른 서비스와 공유하므로 이벤트 루프 스레드에서만 호출한다.
"""

import asyncio
import bisect
import contextlib
import logging
import time
from dataclasses import dataclass

import numpy as np

from teleop.camera.camera_interface import RGBDFrame
from teleop.robot.robot_interface import JointSource
from teleop.utils import config

logger = logging.getLogger("posed_frame")

_HEAD_JOINTS = ("head_0", "head_1")


@dataclass(frozen=True)
class PosedFrame:
    """촬영 시각 기준 관절값·카메라 포즈가 붙은 프레임.

    cam_pos(3)·cam_rot(3x3): 광학 프레임 → base. head_speed: 머리 관절 최대 속도(rad/s).
    """

    frame: RGBDFrame
    joints: dict
    cam_pos: np.ndarray
    cam_rot: np.ndarray
    head_speed: float


class PosedFrameBuilder:
    """관절 측정 이력(시각, 관절값)과 폴링 태스크를 소유하고 프레임에 포즈를 붙인다."""

    def __init__(self, robot_model, joint_source: JointSource):
        self._rm = robot_model
        self._source = joint_source
        self._times: list[float] = []
        self._joints: list[dict] = []
        self._task: asyncio.Task | None = None

    def start(self):
        """관절 폴링 태스크를 시작한다. 반환값 없음."""
        self._task = asyncio.create_task(self._poll_loop())
        logger.info(f"프레임 포즈 시작: 관절 폴링 {config.POSED_JOINT_POLL_HZ:.0f} Hz, 이력 {config.POSED_HISTORY_S:.1f}초")

    async def stop(self):
        """폴링 태스크를 멈춘다. 반환값 없음."""
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._task
            self._task = None

    async def _poll_loop(self):
        """관절값이 바뀔 때마다 (시각, 값)을 이력에 쌓고 오래된 항목을 버린다 (루프 본체)."""
        period = 1.0 / config.POSED_JOINT_POLL_HZ
        while True:
            joints = self._source.joint_positions()
            now = time.monotonic()
            if joints is not None and (not self._joints or joints != self._joints[-1]):
                self._times.append(now)
                self._joints.append(joints)

            # 보관 길이를 넘은 이력 제거
            cutoff = bisect.bisect_left(self._times, now - config.POSED_HISTORY_S)
            if cutoff > 0:
                del self._times[:cutoff]
                del self._joints[:cutoff]
            await asyncio.sleep(period)

    def build(self, frame: RGBDFrame) -> PosedFrame | None:
        """프레임 수신 시각의 관절값을 보간해 카메라 포즈를 붙여 반환한다 (관절 이력이 없거나 끊겼으면 None)."""
        if not self._joints:
            return None
        t = frame.t_mono

        # 최신 관절이 오래됐으면 측정 두절로 본다
        if t - self._times[-1] > config.ROBOT_STATE_STALE_S:
            return None

        joints, head_speed = self._interpolate(t)
        cam_pos, cam_rot = self._rm.camera_pose(joints)
        return PosedFrame(frame=frame, joints=joints, cam_pos=cam_pos, cam_rot=cam_rot, head_speed=head_speed)

    def _interpolate(self, t: float) -> tuple[dict, float]:
        """시각 t의 관절값(이웃 두 샘플 선형 보간, 범위 밖이면 가장 가까운 샘플)과 머리 속도를 반환한다."""
        i = bisect.bisect_left(self._times, t)
        if i == 0:
            return dict(self._joints[0]), self._head_speed(0, min(1, len(self._joints) - 1))
        if i >= len(self._times):
            last = len(self._times) - 1
            return dict(self._joints[last]), self._head_speed(max(0, last - 1), last)

        t0, t1 = self._times[i - 1], self._times[i]
        j0, j1 = self._joints[i - 1], self._joints[i]
        alpha = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        joints = {name: j0[name] + alpha * (j1[name] - j0[name]) for name in j0 if name in j1}
        return joints, self._head_speed(i - 1, i)

    def _head_speed(self, a: int, b: int) -> float:
        """이력 a·b 사이 머리 관절 최대 속도(rad/s)를 반환한다 (같은 샘플이면 0)."""
        dt = self._times[b] - self._times[a]
        if dt <= 0.0:
            return 0.0
        return max(abs(self._joints[b][j] - self._joints[a][j]) / dt for j in _HEAD_JOINTS)
