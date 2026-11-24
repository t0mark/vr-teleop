"""프레임 포즈: RGB-D 프레임에 촬영 시각의 관절값·카메라 포즈를 붙인다.

머리가 움직이는 중에는 "최신 관절값"과 "프레임 촬영 시각"이 수십 ms 어긋나 점군이 번진다. 관절 측정값을
고주파로 폴링해 시각과 함께 이력에 쌓아 두고, 프레임 수신 시각(t_mono)에서 선형 보간한 관절값으로 FK를 한다.
머리 관절 속도도 함께 구해 소비자(TSDF)가 빠르게 움직이는 프레임을 거를 수 있게 한다.
관절 공급자가 None을 반환하면 측정 두절로 보고 이력을 비운 뒤 포즈를 붙이지 않는다.

FK(RobotModel의 공유 pinocchio Data)는 이벤트 루프 스레드에서만 호출한다.
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

# 머리 속도 계산 대상 관절
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
        """robot_model: 카메라 포즈 FK 제공자. joint_source: 관절 측정 공급자."""
        self._robot_model = robot_model
        self._joint_source = joint_source

        # 관절 측정 이력 (시각 오름차순)과 폴링 태스크
        self._times: list[float] = []
        self._joints: list[dict] = []
        self._task: asyncio.Task | None = None

        # 측정 두절 상태 (첫 측정 전도 두절)와 머리 관절 누락 경고 여부
        self._outage = True
        self._head_missing_logged = False

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
        """폴링마다 (시각, 관절값)을 이력에 쌓고, 측정 두절이면 이력을 비우고, 오래된 항목을 버린다 (루프 본체)."""
        period = 1.0 / config.POSED_JOINT_POLL_HZ
        while True:
            joints = self._joint_source.joint_positions()
            now = time.monotonic()

            # 측정 두절 시작: 이력을 비우고 포즈 부여 중단
            if joints is None:
                if not self._outage:
                    logger.warning(f"관절 측정 두절 — 이력 {len(self._times)}개 비움, 프레임 포즈 중단")
                    self._times.clear()
                    self._joints.clear()
                    self._outage = True

            # 측정 수신: 두절에서 복구됐으면 알리고 이력에 추가
            else:
                if self._outage:
                    logger.info("관절 측정 수신 — 프레임 포즈 시작")
                    self._outage = False
                self._times.append(now)
                self._joints.append(joints)

            # 보관 길이를 넘은 이력 제거
            cutoff = bisect.bisect_left(self._times, now - config.POSED_HISTORY_S)
            if cutoff > 0:
                del self._times[:cutoff]
                del self._joints[:cutoff]
            await asyncio.sleep(period)

    def build(self, frame: RGBDFrame) -> PosedFrame | None:
        """프레임 수신 시각의 관절값을 보간해 카메라 포즈를 붙여 반환한다 (측정 두절이거나 이력이 없으면 None)."""
        if self._outage or not self._joints:
            return None

        # 수신 시각 관절값 보간 → 카메라 포즈
        joints, head_speed = self._interpolate(frame.t_mono)
        cam_pos, cam_rot = self._robot_model.camera_pose(joints)
        return PosedFrame(frame=frame, joints=joints, cam_pos=cam_pos, cam_rot=cam_rot, head_speed=head_speed)

    def _interpolate(self, query_time: float) -> tuple[dict, float]:
        """시각 query_time의 관절값(이웃 두 샘플 선형 보간, 범위 밖이면 가장 가까운 샘플)과 머리 속도를 반환한다."""
        # 이력 범위 밖이면 가장 가까운 샘플 사용
        after = bisect.bisect_left(self._times, query_time)
        if after == 0:
            return dict(self._joints[0]), self._head_speed(min(1, len(self._joints) - 1))
        if after >= len(self._times):
            last = len(self._times) - 1
            return dict(self._joints[last]), self._head_speed(last)

        # 이웃 두 샘플 사이 선형 보간
        time_before, time_after = self._times[after - 1], self._times[after]
        joints_before, joints_after = self._joints[after - 1], self._joints[after]
        alpha = (query_time - time_before) / (time_after - time_before) if time_after > time_before else 0.0
        joints = {
            name: joints_before[name] + alpha * (joints_after[name] - joints_before[name])
            for name in joints_before if name in joints_after
        }
        return joints, self._head_speed(after)

    def _head_speed(self, end: int) -> float:
        """이력 end 샘플까지 POSED_HEAD_SPEED_WINDOW_S 구간의 머리 관절 최대 속도(rad/s)를 반환한다 (구간이 없으면 0)."""
        # 구간 시작 샘플 (최소 한 샘플 앞)
        start = bisect.bisect_left(self._times, self._times[end] - config.POSED_HEAD_SPEED_WINDOW_S)
        start = min(start, max(end - 1, 0))
        elapsed_s = self._times[end] - self._times[start]
        if elapsed_s <= 0.0:
            return 0.0

        # 측정에 없는 머리 관절은 빼고 계산 (처음 한 번 경고)
        names = [name for name in _HEAD_JOINTS if name in self._joints[start] and name in self._joints[end]]
        if len(names) < len(_HEAD_JOINTS) and not self._head_missing_logged:
            logger.warning(f"관절 측정에 머리 관절 일부가 없음 ({_HEAD_JOINTS}) — 없는 관절은 머리 속도에서 제외")
            self._head_missing_logged = True
        return max(
            (abs(self._joints[end][name] - self._joints[start][name]) / elapsed_s for name in names), default=0.0
        )
