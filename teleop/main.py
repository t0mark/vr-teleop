"""백엔드 엔트리포인트: 공통 스택 조립 + 모드(dev/deploy) 부품의 조립·런타임 전환.

사용법 (컨테이너 내부):
    TELEOP_MODE=dev    python -m teleop.main   # 개발: 시뮬 로봇 + 시뮬 그리퍼 + 합성 카메라 (기본값)
    TELEOP_MODE=deploy python -m teleop.main   # 운용: 실로봇(gRPC) + 실물 D455 (로봇 내장 PC)

TELEOP_MODE는 초기 모드일 뿐이다 — 기동 후에는 웹 데스크톱 뷰의 모드 버튼으로 dev↔deploy를
런타임 전환할 수 있다 (아래 ModeManager).

구성: [모드 무관] WebRTC 허브(RtcHub — 시그널링·DataChannel 관리) + HTTPS 서버 +
                  IK 계산기(solver·mapper 객체) — 기동 시 1회 조립
      [모드 종속] 로봇 백엔드·그리퍼·카메라 + 제어기·상태 방송·포인트클라우드 — ModeManager가 소유,
                  모드가 바뀌면 통째로 교체된다 (허브 자체는 클라이언트 접속을 유지한 채 재사용)
IK는 스스로 돌지 않는다 — RobotController의 30Hz 루프가 매 주기 solver를 호출한다.
"""

import asyncio
import logging

from teleop.ik.pose_to_target import TargetMapper
from teleop.ik.ik_solver import ArmIkSolver
from teleop.net.rtc_hub import RtcHub
from teleop.net.web_server import WebAPIServer
from teleop.pointcloud.pcd_streamer import PointCloudStreamer
from teleop.robot.robot_controller import RobotController
from teleop.robot.robot_model import RobotModel
from teleop.robot.robot_state_broadcaster import RobotStateBroadcaster
from teleop.utils import config
from teleop.utils.log import setup_logging

logger = logging.getLogger("main")


class ModeManager:
    """dev/deploy 부품 3종(로봇·그리퍼·카메라)과 종속 서비스 3종(상태 방송·명령·포인트클라우드)을
    소유하고 런타임 전환을 수행한다.

    전환 절차 (웹의 set_mode 요청 → request_switch):
        1. 새 부품을 먼저 준비한다 (deploy: 로봇 연결 — 실패하면 현재 모드를 그대로 유지하고 오류만 방송)
        2. 준비가 되면 기존 서비스 정지 → 기존 부품 해제 → 새 부품 배선 → 서비스 재기동
        3. 결과를 mode_status로 전 클라이언트에 방송한다 (신규 접속에도 1회 전송)

    안전 규약:
    - 제어 중(armed)에는 전환을 거부한다 — 먼저 B로 해제해야 한다
    - deploy 해제 시 스트림만 닫는다 (서보는 켜진 채 현재 자세 유지 — B 해제와 동일)
    - 최초 기동이 deploy인데 로봇 연결이 실패하면 SystemExit (폴백 금지, config.TELEOP_MODE 참고)
    """

    def __init__(self, rtc_hub, robot_model, solver, mapper):
        """solver·mapper: 모드 무관 IK 계산기 — 전환 시 재구성하지 않고 새 제어기에 그대로 넘긴다."""
        self._hub = rtc_hub
        self._rm = robot_model
        self._solver = solver
        self._mapper = mapper
        self._mode: str | None = None
        self._backend = None
        self._gripper = None
        self._camera = None
        self._state_broadcaster = None
        self._controller = None
        self._pcd_streamer = None
        # 전환 중 재요청 무시용 (전환은 한 번에 하나만)
        self._switching = False

    @property
    def mode(self) -> str | None:
        """현재 모드 (dev/deploy, 기동 전이면 None)."""
        return self._mode

    # ── 기동·종료 ────────────────────────────────────────────────────────────

    async def start(self, mode: str):
        """최초 모드로 부품·서비스를 기동한다. deploy 로봇 연결 실패는 SystemExit. 반환값 없음."""
        if mode not in ("dev", "deploy"):
            raise SystemExit(f"알 수 없는 TELEOP_MODE: {mode!r} (dev 또는 deploy)")
        loop = asyncio.get_running_loop()
        parts, error = await loop.run_in_executor(
            None, self._build_dev if mode == "dev" else self._build_deploy
        )
        if parts is None:
            raise SystemExit(f"{mode} 모드 기동 실패: {error} — 개발 목적이면 TELEOP_MODE=dev로 기동한다")
        self._wire(mode, parts)
        await self._broadcast_status(True, f"{mode} 모드로 기동")

    async def stop(self):
        """서비스·부품을 모두 정리한다 (서버 종료 경로). 반환값 없음."""
        await self._teardown()

    # ── 런타임 전환 (웹 set_mode 요청) ──────────────────────────────────────

    async def request_switch(self, mode: str):
        """웹 요청으로 모드를 전환한다. 실패하면 현재 모드를 유지하고 오류를 방송한다. 반환값 없음."""
        if mode not in ("dev", "deploy"):
            await self._broadcast_status(False, f"알 수 없는 모드: {mode!r}")
            return
        if self._switching:
            logger.warning(f"전환 진행 중이라 무시: {mode}")
            return
        if mode == self._mode:
            await self._broadcast_status(True, f"이미 {mode} 모드")
            return
        # 로봇이 움직이는 중의 부품 교체는 금지 — 조작자가 명시적으로 해제부터 해야 한다
        if self._controller is not None and self._controller.armed:
            await self._broadcast_status(False, "제어 중에는 전환 불가 — 오른손 B로 해제 후 다시 시도")
            return

        self._switching = True
        try:
            logger.warning(f"모드 전환 시작: {self._mode} → {mode}")
            await self._broadcast_status(True, f"{mode} 모드로 전환 중…", switching=True)

            # 1) 새 부품 준비 (블로킹: 로봇 연결·카메라 오픈) — 실패해도 기존 모드는 아직 살아 있다
            loop = asyncio.get_running_loop()
            parts, error = await loop.run_in_executor(
                None, self._build_dev if mode == "dev" else self._build_deploy
            )
            if parts is None:
                logger.error(f"모드 전환 실패 ({mode}): {error}")
                await self._broadcast_status(False, f"{mode} 전환 실패: {error} — {self._mode} 모드 유지")
                return

            # 2) 기존 것 내리고 새것 배선
            await self._teardown()
            self._wire(mode, parts)
            logger.warning(f"모드 전환 완료: {mode}")
            await self._broadcast_status(True, f"{mode} 모드로 전환 완료")
        finally:
            self._switching = False

    # ── 부품 빌드 (executor에서 실행되는 블로킹 함수) ────────────────────────

    def _build_dev(self):
        """dev 부품(시뮬 로봇·시뮬 그리퍼·합성 카메라)을 만들어 ((백엔드, 그리퍼, 카메라), None)을 반환한다.

        시뮬 부품은 실패 경로가 없다. 하드웨어 접근 없음.
        """
        from teleop.camera.synthetic_camera import SyntheticCamera
        from teleop.robot.sim_gripper import SimGripper
        from teleop.robot.sim_robot import SimRobotBackend

        gripper = SimGripper()
        backend = SimRobotBackend(gripper=gripper)
        backend.connect()
        # 실기기와 같은 기동 순서 유지: 툴 플랜지 전원 → 그리퍼 호밍
        backend.power_tool_flange()
        gripper.start()
        camera = SyntheticCamera()
        camera.start()
        return (backend, gripper, camera), None

    def _build_deploy(self):
        """deploy 부품(실로봇·실물 그리퍼·D455)을 만들어 ((백엔드, 그리퍼, 카메라), None)을 반환한다.

        로봇 연결 실패면 (None, 사유) — 호출부가 현재 모드 유지 또는 기동 실패로 처리한다.
        카메라는 없으면 None (포인트클라우드만 비활성).
        """
        from teleop.camera.d455_camera import D455Camera
        from teleop.robot.rby1_gripper import Rby1Gripper
        from teleop.robot.rby1_robot import Rby1Backend

        gripper = Rby1Gripper()
        backend = Rby1Backend(gripper=gripper)
        if not backend.connect():
            return None, f"로봇({config.ROBOT_ADDRESS}) 연결 실패"
        # 그리퍼 모터 전원은 툴 플랜지에서 나오므로 먼저 켜고, 버스 초기화(호밍)는 그리퍼 스레드가 수행
        if backend.power_tool_flange():
            gripper.start()
        else:
            logger.warning("툴 플랜지 전원 실패 — 그리퍼 비활성")

        camera = D455Camera()
        try:
            camera.start()
        except RuntimeError as e:
            logger.warning(f"카메라 시작 실패, 포인트클라우드 비활성: {e}")
            camera = None
        return (backend, gripper, camera), None

    # ── 배선·해체 ────────────────────────────────────────────────────────────

    def _wire(self, mode: str, parts):
        """부품을 보관하고 종속 서비스를 만들어 시작한다. 반환값 없음.

        시각화는 백엔드 측정값을 그린다. 그리퍼 표시는 측정값(그리퍼 버스)이 있으면 그것을,
        없으면(호밍 전 등) 조작 입력(트리거) 쪽을 쓴다.
        """
        self._backend, self._gripper, self._camera = parts
        self._mode = mode
        self._controller = RobotController(
            self._hub, self._rm, self._solver, self._mapper, self._backend, self._gripper,
        )
        self._state_broadcaster = RobotStateBroadcaster(
            self._hub, self._rm, self._backend,
            gripper_source=self._controller,
            control_source=self._controller,
            source_name="sim" if mode == "dev" else "robot",
        )
        self._pcd_streamer = (
            PointCloudStreamer(self._camera, self._hub, self._rm, self._backend)
            if self._camera is not None else None
        )
        self._state_broadcaster.start()
        self._controller.start()
        if self._pcd_streamer is not None:
            self._pcd_streamer.start()

    async def _teardown(self):
        """종속 서비스를 멈추고 부품을 해제한다 (없으면 무시). 반환값 없음.

        명령 서비스 stop이 스트림을 닫고(서보 유지), 백엔드 disconnect가 구독을 정리한다.
        """
        if self._controller is not None:
            await self._controller.stop()
            self._controller = None
        if self._state_broadcaster is not None:
            await self._state_broadcaster.stop()
            self._state_broadcaster = None
        if self._pcd_streamer is not None:
            await self._pcd_streamer.stop()
            self._pcd_streamer = None
        if self._camera is not None:
            self._camera.stop()
            self._camera = None
        if self._gripper is not None:
            self._gripper.stop()
            self._gripper = None
        if self._backend is not None:
            self._backend.disconnect()
            self._backend = None

    async def _broadcast_status(self, ok: bool, message: str, switching: bool = False):
        """모드 상태를 보관(신규 접속용)하고 전 클라이언트에 방송한다. 반환값 없음."""
        status = {
            "type": "mode_status",
            "mode": self._mode,
            "switching": switching,
            "ok": ok,
            "message": message,
        }
        self._hub.mode_status = status
        await self._hub.broadcast_state(status)


async def run():
    """공통 스택을 조립·기동하고 종료 신호(Ctrl+C)까지 대기한다. 반환값 없음."""
    logger.info(f"초기 실행 모드: {config.TELEOP_MODE} (웹 데스크톱 뷰에서 런타임 전환 가능)")

    # IK 스택: 모델 로드(수 초) → 솔버 NLP 구성 → 매퍼
    robot_model = RobotModel()
    solver = ArmIkSolver(robot_model)
    mapper = TargetMapper(robot_model)

    rtc_hub = RtcHub()
    server = WebAPIServer(rtc_hub)

    # 모드 종속 부품·서비스는 매니저가 소유하고, 웹의 set_mode 요청을 매니저로 잇는다
    mode_manager = ModeManager(rtc_hub, robot_model, solver, mapper)
    rtc_hub.on_set_mode = mode_manager.request_switch

    rtc_hub.start()
    await mode_manager.start(config.TELEOP_MODE)
    await server.start()

    # 상시 서비스이므로 취소(Ctrl+C) 전까지 유휴 대기
    try:
        await asyncio.Event().wait()
    finally:
        await server.stop()
        await mode_manager.stop()
        await rtc_hub.close_all()
        await rtc_hub.stop()


def main():
    """로깅을 초기화하고 asyncio 루프를 실행한다."""
    setup_logging()
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        logger.info("종료 요청 수신, 정상 종료")


if __name__ == "__main__":
    main()
