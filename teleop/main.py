"""백엔드 엔트리포인트: 모듈을 조립해 서버를 기동한다.

사용법 (컨테이너 내부):
    python -m teleop.main

현 단계 구성: 포즈 허브 + IK 서비스(시뮬레이션 시각화) + D455 포인트클라우드 + HTTPS/WSS 서버.
카메라가 없으면 포인트클라우드 서비스만 건너뛰고 나머지는 정상 기동한다.
로봇 제어(robot_sdk)는 내장 PC 이전 후 IkService 출력에 연결한다.
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


async def run():
    """모듈을 조립·기동하고 종료 신호(Ctrl+C)까지 대기한다. 반환값 없음."""
    # IK 스택: 모델 로드(수 초) → 솔버 NLP 구성 → 매퍼
    robot_model = RobotModel()
    solver = ArmIkSolver(robot_model)
    mapper = TargetMapper(robot_model)

    pose_hub = PoseHub()
    ik_service = IkService(pose_hub, robot_model, solver, mapper)
    server = WebAPIServer(pose_hub)

    # 카메라는 선택 사항 — 개발 PC에 D455가 없어도 포즈·IK 파이프라인은 동작해야 한다
    camera = _start_camera()
    pcd_service = PointCloudService(camera, pose_hub, robot_model) if camera is not None else None

    pose_hub.start()
    ik_service.start()
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
        await ik_service.stop()
        await pose_hub.stop()
        if camera is not None:
            camera.stop()


def main():
    """로깅을 초기화하고 asyncio 루프를 실행한다."""
    setup_logging()
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        logger.info("종료 요청 수신, 정상 종료")


if __name__ == "__main__":
    main()
