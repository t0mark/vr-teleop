"""로봇 상태 서비스: 관절 측정값을 주기적으로 읽어 FK로 링크 포즈를 만들고 방송한다.

데이터 흐름: JointSource.joint_positions() → RobotModel.fk_visual(전체 모델)
             → WebXR 좌표 변환 → RtcHub.broadcast_state({type: robot_state})

공급자(JointSource)는 로봇 백엔드(실물 또는 시뮬)다. 브라우저는
어느 쪽인지 몰라도 되지만, 무엇을 보고 있는지 알 수 있게 메시지에 source 필드를 실어 보낸다.

그리퍼는 로봇 본체 gRPC에 실려 오지 않으므로(별도 버스), 값이 없으면 조작 입력(트리거) 쪽
공급자로 대체한다.
"""

import asyncio
import logging
import math
import time

from teleop.robot.robot_interface import JointSource
from teleop.utils import config
from teleop.utils.transforms import robot_to_webxr

logger = logging.getLogger("robot_state_broadcaster")


def _log_task_exception(task: asyncio.Task):
    """루프 태스크가 예외로 끝나면 로그에 남긴다 (조용한 정지 방지). 반환값 없음."""
    if task.cancelled() or task.exception() is None:
        return
    logger.error(f"로봇 상태 루프 종료: {task.exception()!r}", exc_info=task.exception())


class RobotStateBroadcaster:
    """상태 방송 루프와 시각화 메타(robot_config) 등록을 소유한다."""

    def __init__(self, rtc_hub, robot_model, joint_source: JointSource,
                 gripper_source=None, control_source=None,
                 rate_hz: float = config.ROBOT_STATE_RATE_HZ, source_name: str = "sim"):
        """gripper_source: 관절 공급자에 그리퍼 값이 없을 때 대신 쓸 공급자 — gripper_openness()만 있으면 된다 (RobotController의 트리거 해석 등).

        control_source: 제어 상태 표시용 — armed 속성만 있으면 된다 (RobotController).
        """
        self._hub = rtc_hub
        self._rm = robot_model
        self._source = joint_source
        self._gripper_source = gripper_source
        self._control_source = control_source
        self._period = 1.0 / rate_hz
        self._source_name = source_name
        self._task: asyncio.Task | None = None
        self._sent = 0
        self._had_joints = False
        self._stats_t = 0.0

    def start(self):
        """시각화 메타를 허브에 등록하고 방송 루프를 시작한다. 반환값 없음."""
        # 브라우저가 접속 시 1회 받는 로봇 메시 구성 + IK 브리지용 q 순서 관절명
        self._hub.robot_config = {
            "type": "robot_config",
            "mesh_base": config.MESH_URL_PREFIX,
            "links": self._rm.visual_config(),
            "joint_names": self._rm.joint_names_in_q_order(),
            "camera": self._rm.camera_visual_config(),
        }
        self._stats_t = time.monotonic()
        self._task = asyncio.create_task(self._loop())
        self._task.add_done_callback(_log_task_exception)
        logger.info(f"로봇 상태 서비스 시작: {1.0 / self._period:.0f} Hz, 공급자 {self._source_name}")

    async def stop(self):
        """방송 루프를 정지한다. 반환값 없음."""
        if self._task is not None:
            self._task.cancel()
            self._task = None

    async def _loop(self):
        """고정 주기로 관절값을 읽어 링크 포즈를 방송한다 (루프 본체)."""
        while True:
            t0 = time.monotonic()

            named = self._source.joint_positions()
            gripper = self._resolve_gripper()
            # 측정값이 없으면 ready pose로 표시한다 (fk_visual이 None을 ready로 채운다).
            # 링크를 원점에 겹쳐 두는 것보다 자세가 보이는 편이 진단에 낫다
            await self._hub.broadcast_state(self._make_state(named, gripper))

            self._sent += 1
            self._had_joints = self._had_joints or named is not None
            self._log_stats()
            await asyncio.sleep(max(0.0, self._period - (time.monotonic() - t0)))

    def _resolve_gripper(self) -> dict:
        """그리퍼 열림 정도를 결정해 반환한다 (공급자 값 우선, 없으면 대체 공급자, 그래도 없으면 열림)."""
        value = self._source.gripper_openness()
        if value is None and self._gripper_source is not None:
            value = self._gripper_source.gripper_openness()
        return value if value is not None else {"left": 1.0, "right": 1.0}

    def _make_state(self, named: dict | None, gripper: dict) -> dict:
        """관절값 사전의 FK 결과를 WebXR 좌표계 링크 포즈 메시지로 변환해 반환한다.

        joints 필드는 관절명 사전 그대로 실어 보낸다 — 인덱스 순서에 의존하지 않게 하기 위함이며
        HUD 진단에도 쓴다.
        """
        links = {}
        for name, (pos, quat) in self._rm.fk_visual(named, gripper_open=gripper).items():
            p, r = robot_to_webxr(pos, quat)
            links[name] = {"pos": p.tolist(), "quat": r.tolist()}
        state = {
            "type": "robot_state",
            "source": self._source_name,
            "live": named is not None,
            "links": links,
            "joints": named or {},
            "gripper": dict(gripper),
        }
        # 제어 상태(대기/제어 중): HUD가 "왜 안 따라오는지"를 보여 주는 핵심 표시
        if self._control_source is not None:
            state["armed"] = bool(self._control_source.armed)
        # 주행으로 누적된 베이스 포즈 (시뮬 백엔드만 제공, WebXR 기저로 변환해 실음)
        base = self._source.base_pose() if hasattr(self._source, "base_pose") else None
        if base is not None:
            x, y, yaw = base
            p, r = robot_to_webxr([x, y, 0.0], [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
            state["base"] = {"pos": p.tolist(), "quat": r.tolist()}
        return state

    def _log_stats(self):
        """주기적으로 방송률과 측정값 수신 여부를 로그로 남긴다. 반환값 없음."""
        now = time.monotonic()
        if now - self._stats_t < config.POSE_STATS_INTERVAL_S:
            return
        dt = now - self._stats_t
        logger.info(
            f"로봇 상태 방송 {self._sent / dt:.1f} Hz, 공급자 {self._source_name}, "
            f"측정값 {'수신' if self._had_joints else '없음(ready 표시)'}"
        )
        self._sent = 0
        self._had_joints = False
        self._stats_t = now
