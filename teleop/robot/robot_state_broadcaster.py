"""로봇 상태 서비스: 관절 측정값을 주기적으로 읽어 FK로 링크 포즈를 만들고 방송한다.

데이터 흐름: JointSource.joint_positions() → RobotModel.fk_visual(전체 모델)
             → WebXR 좌표 변환 → RtcHub.broadcast_state({type: robot_state})

공급자(JointSource)는 로봇 백엔드(실물 또는 시뮬)다. 브라우저는
어느 쪽인지 몰라도 되지만, 무엇을 보고 있는지 알 수 있게 메시지에 source 필드를 실어 보낸다.

손은 로봇 본체 gRPC에 실려 오지 않으므로(별도 버스), 손별로 값이 없으면 조작 입력(fingers) 쪽
공급자로 대체한다. 손가락 굽힘은 RobotModel이 Inspire 손 모델 관절각으로 바꿔 손 링크 포즈에 반영한다.
"""

import asyncio
import contextlib
import logging
import time

from teleop.robot.robot_interface import JointSource
from teleop.utils import config
from teleop.utils.inputs import FINGER_COUNT
from teleop.utils.transforms import robot_to_webxr

logger = logging.getLogger("robot_state_broadcaster")

# robot_state 메시지 크기를 줄이기 위한 링크 포즈·관절값 반올림 소수 자릿수
_STATE_DECIMALS = 4


def _log_task_exception(task: asyncio.Task):
    """루프 태스크가 예외로 끝나면 로그에 남긴다. 반환값 없음."""
    if task.cancelled() or task.exception() is None:
        return
    logger.error(f"로봇 상태 루프 종료: {task.exception()!r}", exc_info=task.exception())


class RobotStateBroadcaster:
    """상태 방송 루프와 시각화 메타(robot_config) 등록을 소유한다."""

    def __init__(self, rtc_hub, robot_model, joint_source: JointSource,
                 hand_source=None, control_source=None,
                 rate_hz: float = config.ROBOT_STATE_RATE_HZ, source_name: str = "sim"):
        """hand_source: 관절 공급자에 손 값이 없을 때 대신 쓸 공급자 — hand_fingers()만 있으면 된다 (RobotController의 손가락 목표 등).

        control_source: 제어 상태 표시용 — armed 속성만 있으면 된다 (RobotController).
        """
        self._hub = rtc_hub
        self._rm = robot_model
        self._source = joint_source
        self._hand_source = hand_source
        self._control_source = control_source
        self._period = 1.0 / rate_hz
        self._source_name = source_name
        self._task: asyncio.Task | None = None

        # 통계: 방송 횟수·측정값 수신 여부·통계 시작 시각
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

        # 방송 루프 시작
        self._stats_t = time.monotonic()
        self._task = asyncio.create_task(self._loop())
        self._task.add_done_callback(_log_task_exception)
        logger.info(f"로봇 상태 서비스 시작: {1.0 / self._period:.0f} Hz, 공급자 {self._source_name}")

    async def stop(self):
        """방송 루프를 정지한다. 반환값 없음."""
        # 루프 태스크 취소 후 종료까지 대기
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await self._task
            self._task = None

    async def _loop(self):
        """고정 주기로 관절값을 읽어 링크 포즈를 방송한다 (루프 본체)."""
        while True:
            cycle_start = time.monotonic()

            # 관절값·손가락 굽힘으로 상태 메시지를 만들어 방송 (측정값이 없으면 ready pose로 표시)
            named = self._source.joint_positions()
            fingers = self._resolve_fingers()
            await self._hub.broadcast_state(self._make_state(named, fingers))

            # 방송 통계
            self._sent += 1
            self._had_joints = self._had_joints or named is not None
            self._log_stats()
            await asyncio.sleep(max(0.0, self._period - (time.monotonic() - cycle_start)))

    def _resolve_fingers(self) -> dict:
        """손별로 손가락 굽힘 {left/right: [6]}을 결정해 반환한다 (측정값 우선, 없으면 대체 공급자, 그래도 없으면 펼침)."""
        # 측정값과 대체 공급자 값
        measured = self._source.hand_fingers() or {}
        fallback = (self._hand_source.hand_fingers() if self._hand_source is not None else None) or {}

        # 손마다 측정 → 대체 공급자 → 펼침 순으로 고른다
        resolved = {}
        for side in ("left", "right"):
            values = measured.get(side)
            if values is None:
                values = fallback.get(side)
            if values is None:
                values = [0.0] * FINGER_COUNT
            resolved[side] = values
        return resolved

    def _make_state(self, named: dict | None, fingers: dict) -> dict:
        """관절값 사전의 FK 결과를 WebXR 좌표계 링크 포즈 메시지로 변환해 반환한다.

        joints 필드는 관절명 사전 그대로 실어 보낸다 (HUD 진단에도 쓴다).
        """
        # 링크 포즈 FK → WebXR 좌표 변환·반올림
        links = {}
        for name, (pos, quat) in self._rm.fk_visual(named, fingers=fingers).items():
            webxr_pos, webxr_quat = robot_to_webxr(pos, quat)
            links[name] = {
                "pos": [round(value, _STATE_DECIMALS) for value in webxr_pos.tolist()],
                "quat": [round(value, _STATE_DECIMALS) for value in webxr_quat.tolist()],
            }

        # 관절값 반올림과 메시지 조립
        joints = {joint_name: round(float(value), _STATE_DECIMALS) for joint_name, value in (named or {}).items()}
        state = {
            "type": "robot_state",
            "source": self._source_name,
            "live": named is not None,
            "links": links,
            "joints": joints,
            "hand_fingers": {side: list(values) for side, values in fingers.items()},
        }

        # 제어 상태(대기/제어 중) 표시
        if self._control_source is not None:
            state["armed"] = bool(self._control_source.armed)
        return state

    def _log_stats(self):
        """주기적으로 방송률과 측정값 수신 여부를 로그로 남긴다. 반환값 없음."""
        now = time.monotonic()
        if now - self._stats_t < config.POSE_STATS_INTERVAL_S:
            return
        elapsed = now - self._stats_t
        logger.info(
            f"로봇 상태 방송 {self._sent / elapsed:.1f} Hz, 공급자 {self._source_name}, "
            f"측정값 {'수신' if self._had_joints else '없음(ready 표시)'}"
        )
        self._sent = 0
        self._had_joints = False
        self._stats_t = now
