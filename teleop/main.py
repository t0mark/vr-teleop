"""백엔드 엔트리포인트: 모듈을 조립해 서버를 기동한다.

사용법 (컨테이너 내부):
    python -m teleop.main

구성: 포즈 허브 + IK 서비스 + 로봇 상태 서비스(시각화 방송) + 로봇 제어 서비스(실로봇 명령)
      + 그리퍼 + D455 포인트클라우드 + HTTPS/WSS 서버.

실로봇 연결 성공 여부로 기동 시 한 번 갈라진다.
- 연결: 측정 관절값을 그대로 그리고(토르소·머리 포함), 제어 서비스가 IK 결과를 명령으로 내보낸다.
  로봇은 조작자가 오른손 A로 제어를 시작하기 전까지 움직이지 않는다.
- 실패: IK 결과를 그리는 시뮬레이션 (로봇 없이도 프론트·파이프라인을 검증할 수 있어야 하므로)
카메라가 없으면 포인트클라우드만 건너뛰고 나머지는 정상 기동한다.
"""

import asyncio
import logging

from teleop.core.depth_to_3d.service import PointCloudService
from teleop.core.ik.mapper import TargetMapper
from teleop.core.ik.robot_model import RobotModel
from teleop.core.ik.service import IkService
from teleop.core.ik.solver import ArmIkSolver
from teleop.net.web_api.pose_hub import PoseHub
from teleop.net.web_api.server import WebAPIServer
from teleop.robot_sdk.control import RobotControlService
from teleop.robot_sdk.rby1.gripper import Rby1Gripper
from teleop.robot_sdk.rby1.robot import Rby1Backend
from teleop.robot_sdk.service import RobotStateService
from teleop.utils.camera.d455 import D455Camera
from teleop.utils.log import setup_logging

logger = logging.getLogger("main")


def _start_camera() -> D455Camera | None:
    """D455를 열어 반환한다. 장치가 없거나 열기에 실패하면 경고 후 None (서버는 계속 기동)."""
    camera = D455Camera()
    try:
        camera.start()
    except RuntimeError as e:
        logger.warning(f"카메라 시작 실패, 포인트클라우드 비활성: {e}")
        return None
    return camera


def _start_robot() -> tuple[Rby1Backend, Rby1Gripper] | tuple[None, None]:
    """실로봇에 연결해 (백엔드, 그리퍼)를 반환한다. 연결 실패면 (None, None) (시뮬레이션으로 진행).

    그리퍼는 본체와 별도 버스라 본체 연결 성공 후에만 시작한다 (호밍이 실제 동작을 수반한다).
    """
    gripper = Rby1Gripper()
    backend = Rby1Backend(gripper=gripper)
    if not backend.connect():
        logger.warning("실로봇 미연결 — IK 결과를 그리는 시뮬레이션 시각화로 진행")
        return None, None
    # 그리퍼 모터 전원은 툴 플랜지에서 나오므로 먼저 켜고, 버스 초기화는 그리퍼 스레드가 잠시 뒤 시작한다
    if backend.power_tool_flange():
        gripper.start()
    else:
        logger.warning("툴 플랜지 전원 실패 — 그리퍼 비활성")
    return backend, gripper


async def run():
    """모듈을 조립·기동하고 종료 신호(Ctrl+C)까지 대기한다. 반환값 없음."""
    # IK 스택: 모델 로드(수 초) → 솔버 NLP 구성 → 매퍼
    robot_model = RobotModel()
    solver = ArmIkSolver(robot_model)
    mapper = TargetMapper(robot_model)

    pose_hub = PoseHub()
    ik_service = IkService(pose_hub, robot_model, solver, mapper)

    # 시각화 공급자 선택: 실로봇 측정값 우선, 없으면 IK 시뮬레이션.
    # 그리퍼 표시는 측정값(그리퍼 버스)이 있으면 그것을, 없으면 조작 입력(트리거)을 쓴다
    robot, gripper = _start_robot()
    joint_source = robot if robot is not None else ik_service
    state_service = RobotStateService(
        pose_hub, robot_model, joint_source,
        gripper_source=ik_service,
        source_name="robot" if robot is not None else "sim",
    )
    control_service = (
        RobotControlService(pose_hub, robot_model, ik_service, robot, gripper)
        if robot is not None else None
    )

    server = WebAPIServer(pose_hub)

    # 카메라는 선택 사항 — D455가 없어도 포즈·IK·상태 파이프라인은 동작해야 한다
    camera = _start_camera()
    pcd_service = (
        PointCloudService(camera, pose_hub, robot_model, joint_source)
        if camera is not None else None
    )

    pose_hub.start()
    ik_service.start()
    state_service.start()
    if control_service is not None:
        control_service.start()
    if pcd_service is not None:
        pcd_service.start()
    await server.start()

    # 상시 서비스이므로 취소(Ctrl+C) 전까지 유휴 대기
    try:
        await asyncio.Event().wait()
    finally:
        await server.stop()
        if pcd_service is not None:
            await pcd_service.stop()
        if control_service is not None:
            await control_service.stop()
        await state_service.stop()
        await ik_service.stop()
        await pose_hub.stop()
        if camera is not None:
            camera.stop()
        if gripper is not None:
            gripper.stop()
        if robot is not None:
            robot.disconnect()


def main():
    """로깅을 초기화하고 asyncio 루프를 실행한다."""
    setup_logging()
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        logger.info("종료 요청 수신, 정상 종료")


if __name__ == "__main__":
    main()
