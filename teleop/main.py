"""백엔드 엔트리포인트: 공통 스택 조립 + 모드(dev/deploy) 부품의 조립·런타임 전환.

사용법 (컨테이너 내부):
    TELEOP_MODE=dev    python -m teleop.main   # 개발: 시뮬 로봇 + 시뮬 손 + 합성 카메라 (기본값)
    TELEOP_MODE=deploy python -m teleop.main   # 운용: 실로봇(gRPC) + 실물 D455 (로봇 내장 PC)

TELEOP_MODE는 초기 모드일 뿐이다 — 기동 후에는 웹 데스크톱 뷰의 모드 버튼으로 dev↔deploy를
런타임 전환할 수 있다 (아래 ModeManager).

구성: [모드 무관] WebRTC 허브(RtcHub — 시그널링·DataChannel 관리) + HTTPS 서버 +
                  IK 계산기(solver·mapper 객체) + Manus 장갑(조작자 손가락) — 기동 시 1회 조립
      [모드 종속] 로봇 백엔드·손·카메라 + 제어기·상태 방송·인식(재구성 메시: 환경·작업 영역) — ModeManager가 소유,
                  모드가 바뀌면 통째로 교체된다 (허브 자체는 클라이언트 접속을 유지한 채 재사용)
IK는 스스로 돌지 않는다 — RobotController의 30Hz 루프가 매 주기 solver를 호출한다.
"""

import asyncio
import logging

from teleop.glove.manus_glove import ManusGlove
from teleop.ik.pose_to_target import TargetMapper
from teleop.ik.ik_solver import ArmIkSolver
from teleop.net.rtc_hub import RtcHub
from teleop.net.web_server import WebAPIServer
from teleop.perception.perception_service import PerceptionService
from teleop.robot.robot_controller import RobotController
from teleop.robot.robot_model import RobotModel
from teleop.robot.robot_state_broadcaster import RobotStateBroadcaster
from teleop.utils import config
from teleop.utils.log import setup_logging

logger = logging.getLogger("main")


class ModeManager:
    """dev/deploy 부품 3종(로봇·손·카메라)과 종속 서비스 3종(상태 방송·명령·인식)을
    소유하고 런타임 전환을 수행한다.

    전환 절차 (웹의 set_mode 요청 → request_switch):
        1. 새 부품을 먼저 준비한다 (deploy: 로봇 연결 — 실패하면 현재 모드를 그대로 유지하고 오류만 방송)
        2. 준비가 되면 기존 서비스 정지 → 기존 부품 해제 → 새 부품 배선 → 서비스 재기동
        3. 결과를 mode_status로 전 클라이언트에 방송한다 (신규 접속에도 1회 전송)

    안전 규약:
    - 제어 중(armed)에는 전환을 거부한다 — 먼저 VR을 종료해 제어를 해제해야 한다
    - deploy 해제 시 스트림만 닫는다 (서보는 켜진 채 현재 자세 유지 — stop 해제와 동일)
    - 최초 기동이 deploy인데 로봇 연결이 실패하면 SystemExit (폴백 금지, config.TELEOP_MODE 참고)
    """

    def __init__(self, rtc_hub, robot_model, solver, mapper, glove=None):
        """solver·mapper: 모드 무관 IK 계산기 — 전환 시 재구성하지 않고 새 제어기에 그대로 넘긴다.

        glove: 모드 무관 조작자 손가락 입력(ManusGlove) — 전환 시에도 유지하고 새 제어기에 그대로 넘긴다.
        """
        self._hub = rtc_hub
        self._rm = robot_model
        self._solver = solver
        self._mapper = mapper
        self._glove = glove
        self._mode: str | None = None
        self._backend = None
        self._hand = None
        self._camera = None
        self._state_broadcaster = None
        self._controller = None
        self._perception = None
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
            await self._broadcast_status(False, "제어 중에는 전환 불가 — VR을 종료해 제어를 해제한 뒤 다시 시도")
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
        """dev 부품(시뮬 로봇·시뮬 손·합성 카메라)을 만들어 ((백엔드, 손, 카메라), None)을 반환한다.

        시뮬 부품은 실패 경로가 없다. 하드웨어 접근 없음.
        """
        from teleop.camera.synthetic_camera import SyntheticCamera
        from teleop.robot.sim_hand import SimHand
        from teleop.robot.sim_robot import SimRobotBackend

        hand = SimHand()
        backend = SimRobotBackend(hand=hand)
        backend.connect()
        hand.start()
        camera = SyntheticCamera()
        camera.start()
        return (backend, hand, camera), None

    def _build_deploy(self):
        """deploy 부품(실로봇·실물 손·D455)을 만들어 ((백엔드, 손, 카메라), None)을 반환한다.

        로봇 연결 실패면 (None, 사유) — 호출부가 현재 모드 유지 또는 기동 실패로 처리한다.
        카메라는 없으면 None (인식만 비활성). 손은 접속에 실패해도 백그라운드에서 재시도한다.
        """
        from teleop.camera.d455_camera import D455Camera
        from teleop.robot.inspire_hand import InspireHand
        from teleop.robot.rby1_robot import Rby1Backend

        hand = InspireHand()
        backend = Rby1Backend(hand=hand)
        if not backend.connect():
            return None, f"로봇({config.ROBOT_ADDRESS}) 연결 실패"

        # 손 접속은 백그라운드 스레드가 재시도하므로 실패해도 팔 제어를 막지 않는다
        hand.start()

        camera = D455Camera()
        try:
            camera.start()
        except RuntimeError as e:
            logger.warning(f"카메라 시작 실패, 인식(재구성 메시) 비활성: {e}")
            camera = None
        return (backend, hand, camera), None

    # ── 배선·해체 ────────────────────────────────────────────────────────────

    def _wire(self, mode: str, parts):
        """부품을 보관하고 종속 서비스를 만들어 시작한다. 반환값 없음.

        시각화는 백엔드 측정값을 그린다. 손 표시는 측정값(손 버스)이 있으면 그것을,
        없으면(호밍 전·손 없음) 조작 입력(fingers) 쪽을 쓴다.
        """
        self._backend, self._hand, self._camera = parts
        self._mode = mode
        self._controller = RobotController(
            self._hub, self._rm, self._solver, self._mapper, self._backend, self._hand, glove=self._glove,
        )
        self._state_broadcaster = RobotStateBroadcaster(
            self._hub, self._rm, self._backend,
            hand_source=self._controller,
            control_source=self._controller,
            source_name="sim" if mode == "dev" else "robot",
        )
        self._perception = (
            PerceptionService(self._camera, self._hub, self._rm, self._backend)
            if self._camera is not None else None
        )
        self._state_broadcaster.start()
        self._controller.start()
        if self._perception is not None:
            self._perception.start()

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
        if self._perception is not None:
            await self._perception.stop()
            self._perception = None

        # 부품 해제는 블로킹(스레드 join·gRPC)이라 executor에서
        loop = asyncio.get_running_loop()
        if self._camera is not None:
            await loop.run_in_executor(None, self._camera.stop)
            self._camera = None
        if self._hand is not None:
            await loop.run_in_executor(None, self._hand.stop)
            self._hand = None
        if self._backend is not None:
            await loop.run_in_executor(None, self._backend.disconnect)
            self._backend = None
        logger.info("모드 부품 해제 완료")

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

    # 조작자 손가락 입력(Manus 장갑)은 모드 무관 — 기동 시 1회 시작 (SDK·장갑이 없어도 서버는 계속 돈다)
    glove = ManusGlove()
    glove.start()

    # 모드 종속 부품·서비스는 매니저가 소유하고, 웹의 set_mode 요청을 매니저로 잇는다
    mode_manager = ModeManager(rtc_hub, robot_model, solver, mapper, glove=glove)
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
        await asyncio.get_running_loop().run_in_executor(None, glove.stop)
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
