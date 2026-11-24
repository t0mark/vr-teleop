"""백엔드 엔트리포인트: 공통 스택 조립 + 모드(dev/deploy) 부품의 조립·런타임 전환.

사용법 (컨테이너 내부):
    TELEOP_MODE=dev    python -m teleop.main   # 개발: 시뮬 로봇 + 시뮬 손 + 합성 카메라 (기본값)
    TELEOP_MODE=deploy python -m teleop.main   # 운용: 실로봇(gRPC) + Inspire 손(Modbus TCP) + 실물 D455 (로봇 내장 PC)

TELEOP_MODE는 초기 모드일 뿐이다 — 기동 후에는 웹 데스크톱 뷰의 모드 버튼으로 dev↔deploy를
런타임 전환할 수 있다 (아래 ModeManager).

구성: [모드 무관] WebRTC 허브(RtcHub — 시그널링·DataChannel 관리) + HTTPS 서버 +
                  IK 계산기(solver·arm_retargeter 객체) + Manus 장갑(조작자 손가락) — 기동 시 1회 조립
      [모드 종속] 로봇 백엔드·손·카메라 + 제어기·상태 방송·인식(재구성 메시: 환경·작업 영역) — ModeManager가 소유,
                  모드가 바뀌면 통째로 교체된다 (허브 자체는 클라이언트 접속을 유지한 채 재사용)
IK는 스스로 돌지 않는다 — RobotController의 30Hz 루프가 매 주기 solver를 호출한다.
SIGTERM(docker stop)·SIGINT(Ctrl+C)를 받으면 종료 정리 경로를 거쳐 끝난다.
"""

import asyncio
import inspect
import logging
import signal

from teleop.glove.manus_glove import ManusGlove
from teleop.ik.arm_retargeter import ArmRetargeter
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
        2. 준비가 되면 제어 중 여부를 다시 확인하고, 기존 서비스 정지 → 기존 부품 해제 → 새 부품 배선 → 서비스 재기동
        3. 결과를 mode_status로 전 클라이언트에 방송한다 (신규 접속에도 1회 전송)

    안전 규약:
    - 제어 중(armed)에는 전환을 거부한다 — 먼저 VR을 종료해 제어를 해제해야 한다
    - deploy 해제 시 스트림만 닫는다 (서보는 켜진 채 현재 자세 유지 — stop 해제와 동일)
    - 최초 기동이 deploy인데 로봇 연결이 실패하면 SystemExit (폴백 금지, config.TELEOP_MODE 참고)
    - 전환 전에 래치된 명령은 새 제어기에 넘기지 않는다
    """

    def __init__(self, rtc_hub, robot_model, solver, arm_retargeter, glove=None):
        """solver·arm_retargeter: 모드 무관 IK 계산기 — 전환 시 재구성하지 않고 새 제어기에 그대로 넘긴다.

        glove: 모드 무관 조작자 손가락 입력(ManusGlove) — 전환 시에도 유지하고 새 제어기에 그대로 넘긴다.
        """
        # 모드 무관 공유 객체
        self._hub = rtc_hub
        self._robot_model = robot_model
        self._solver = solver
        self._arm_retargeter = arm_retargeter
        self._glove = glove

        # 현재 모드와 모드 종속 부품·서비스 (전환 시 교체)
        self._mode: str | None = None
        self._backend = None
        self._hand = None
        self._camera = None
        self._state_broadcaster = None
        self._controller = None
        self._perception = None

        # 전환 진행 여부 (진행 중이면 재요청 무시)
        self._switching = False
        # 진행 중인 전환 태스크 (종료 시 완료 대기용)
        self._switch_task: asyncio.Task | None = None
        # 종료 경로 진입 여부 (진입 후 새 전환 거부)
        self._stopping = False

    @property
    def mode(self) -> str | None:
        """현재 모드 (dev/deploy, 기동 전·서비스 없음이면 None)."""
        return self._mode

    def is_armed(self) -> bool:
        """현재 제어기가 제어 중이면 True (제어기가 없으면 False)."""
        return self._controller is not None and self._controller.armed

    # ── 기동·종료 ────────────────────────────────────────────────────────────

    async def start(self, mode: str):
        """최초 모드로 부품·서비스를 기동한다. deploy 로봇 연결 실패는 SystemExit. 반환값 없음."""
        # 모드 이름 검사
        if mode not in ("dev", "deploy"):
            raise SystemExit(f"알 수 없는 TELEOP_MODE: {mode!r} (dev 또는 deploy)")

        # 부품 준비 (실패하면 기동 실패)
        parts, error = await self._build_parts(mode)
        if parts is None:
            raise SystemExit(f"{mode} 모드 기동 실패: {error} — 개발 목적이면 TELEOP_MODE=dev로 기동한다")

        # 서비스 배선·기동 (실패하면 부품을 정리하고 기동 실패)
        error = await self._activate(mode, parts)
        if error is not None:
            raise SystemExit(f"{mode} 모드 기동 실패: {error}")
        await self._broadcast_status(True, f"{mode} 모드로 기동")

    async def stop(self):
        """진행 중인 전환을 기다린 뒤 서비스·부품을 모두 정리한다 (서버 종료 경로). 반환값 없음."""
        # 새 전환 요청 차단
        self._stopping = True

        # 진행 중인 전환 완료 대기
        switch_task = self._switch_task
        if switch_task is not None and not switch_task.done():
            logger.warning("진행 중인 모드 전환 완료 대기 후 종료")
            await asyncio.wait({switch_task})

        # 서비스·부품 정리
        await self._teardown()

    # ── 런타임 전환 (웹 set_mode 요청) ──────────────────────────────────────

    async def request_switch(self, mode: str):
        """웹 요청으로 모드를 전환한다. 실패하면 현재 모드를 유지하고 오류를 방송한다. 반환값 없음."""
        # 요청 검사: 모드 이름·종료 중·전환 중·같은 모드
        if mode not in ("dev", "deploy"):
            await self._broadcast_status(False, f"알 수 없는 모드: {mode!r}")
            return
        if self._stopping:
            logger.warning(f"서버 종료 중이라 전환 요청 무시: {mode}")
            return
        if self._switching:
            logger.warning(f"전환 진행 중이라 무시: {mode}")
            return
        if mode == self._mode:
            await self._broadcast_status(True, f"이미 {mode} 모드")
            return
        # 제어 중이면 전환 거부
        if self.is_armed():
            await self._broadcast_status(False, "제어 중에는 전환 불가 — VR을 종료해 제어를 해제한 뒤 다시 시도")
            return

        # 전환 진행 표시 후 절차 수행
        self._switching = True
        self._switch_task = asyncio.current_task()
        try:
            logger.warning(f"모드 전환 시작: {self._mode} → {mode}")
            await self._broadcast_status(True, f"{mode} 모드로 전환 중…", switching=True)

            # 1) 새 부품 준비 (실패하면 현재 모드 유지)
            parts, error = await self._build_parts(mode)
            if parts is None:
                logger.error(f"모드 전환 실패 ({mode}): {error}")
                await self._broadcast_status(False, f"{mode} 전환 실패: {error} — {self._mode} 모드 유지")
                return

            # 2) 준비 중 제어 시작·서버 종료가 생겼으면 새 부품을 정리하고 전환 취소
            if self._stopping or self.is_armed():
                reason = "서버 종료 중" if self._stopping else "준비 중 제어가 시작됨"
                logger.warning(f"모드 전환 취소 ({mode}): {reason} — 새 부품 정리")
                await asyncio.get_running_loop().run_in_executor(None, self._release_parts, parts)
                await self._broadcast_status(False, f"{mode} 전환 취소: {reason} — {self._mode} 모드 유지")
                return

            # 3) 기존 서비스·부품 해제 후 새 부품 배선·기동
            await self._teardown()
            error = await self._activate(mode, parts)
            if error is not None:
                logger.error(f"모드 전환 실패 ({mode}): {error} — 기동 중인 모드 없음")
                await self._broadcast_status(False, f"{mode} 전환 실패: {error} — 기동 중인 모드 없음")
                return
            logger.warning(f"모드 전환 완료: {mode}")
            await self._broadcast_status(True, f"{mode} 모드로 전환 완료")
        except Exception as e:
            logger.exception(f"모드 전환 중 예외 ({mode})")
            await self._broadcast_status(False, f"{mode} 전환 실패: {type(e).__name__}: {e}")
        finally:
            self._switching = False
            self._switch_task = None

    # ── 부품 빌드 (executor에서 실행되는 블로킹 함수) ────────────────────────

    async def _build_parts(self, mode: str):
        """모드에 맞는 부품을 executor에서 만들어 ((백엔드, 손, 카메라), None) 또는 (None, 사유)를 반환한다."""
        # 모드별 빌드 함수 선택
        build = self._build_dev if mode == "dev" else self._build_deploy

        # executor에서 빌드 (예외는 실패 사유로 변환)
        try:
            return await asyncio.get_running_loop().run_in_executor(None, build)
        except Exception as e:
            logger.exception(f"{mode} 부품 준비 중 예외")
            return None, f"부품 준비 예외: {type(e).__name__}: {e}"

    def _build_dev(self):
        """dev 부품(시뮬 로봇·시뮬 손·합성 카메라)을 만들어 ((백엔드, 손, 카메라), None)을 반환한다.

        시뮬 부품은 실패 경로가 없다. 하드웨어 접근 없음.
        """
        # dev 전용 부품 모듈 로드
        from teleop.camera.synthetic_camera import SyntheticCamera
        from teleop.robot.sim_hand import SimHand
        from teleop.robot.sim_robot import SimRobotBackend

        # 시뮬 로봇·손 연결과 합성 카메라 시작
        hand = SimHand()
        backend = SimRobotBackend(hand=hand)
        backend.connect()
        hand.start()
        camera = SyntheticCamera()
        camera.start()
        return (backend, hand, camera), None

    def _build_deploy(self):
        """deploy 부품(실로봇·실물 손·D455)을 만들어 ((백엔드, 손, 카메라), None)을 반환한다.

        로봇 연결 실패·부품 준비 예외면 이미 만든 부품을 정리하고 (None, 사유) — 호출부가 현재 모드 유지 또는 기동 실패로 처리한다.
        카메라는 없으면 None (인식만 비활성). 손은 접속에 실패해도 백그라운드에서 재시도한다.
        """
        # deploy 전용 부품 모듈 로드 (하드웨어 SDK 의존)
        from teleop.camera.d455_camera import D455Camera
        from teleop.robot.inspire_hand import InspireHand
        from teleop.robot.rby1_robot import Rby1Backend

        # 로봇 연결 (예외로 끝나면 부분 연결을 정리)
        hand = InspireHand()
        backend = Rby1Backend(hand=hand)
        try:
            connected = backend.connect()
        except Exception as e:
            logger.exception("로봇 연결 중 예외 — 부분 연결 정리")
            self._release_parts((backend, None, None))
            return None, f"로봇({config.ROBOT_ADDRESS}) 연결 예외: {type(e).__name__}: {e}"
        if not connected:
            return None, f"로봇({config.ROBOT_ADDRESS}) 연결 실패"

        # 손·카메라 시작 (손 접속 실패는 백그라운드 재시도, 카메라 열기 실패는 카메라 없이 진행)
        camera = None
        try:
            hand.start()
            camera = D455Camera()
            try:
                camera.start()
            except RuntimeError as e:
                logger.warning(f"카메라 시작 실패, 인식(재구성 메시) 비활성: {e}")
                camera = None
        except Exception as e:
            logger.exception("deploy 부품 준비 중 예외 — 이미 만든 부품 정리")
            self._release_parts((backend, hand, camera))
            return None, f"deploy 부품 준비 예외: {type(e).__name__}: {e}"
        return (backend, hand, camera), None

    def _release_parts(self, parts):
        """부품 (백엔드, 손, 카메라)을 카메라 → 손 → 백엔드 순으로 해제한다 (블로킹, None은 무시). 반환값 없음."""
        backend, hand, camera = parts

        # 해제 단계 목록 구성
        release_steps = []
        if camera is not None:
            release_steps.append(("카메라", camera.stop))
        if hand is not None:
            release_steps.append(("손", hand.stop))
        if backend is not None:
            release_steps.append(("로봇 백엔드", backend.disconnect))

        # 단계별 해제 (실패해도 다음 단계 진행)
        for part_name, release in release_steps:
            try:
                release()
            except Exception:
                logger.exception(f"{part_name} 해제 실패")

    # ── 배선·해체 ────────────────────────────────────────────────────────────

    async def _activate(self, mode: str, parts) -> str | None:
        """부품으로 종속 서비스를 배선·기동한다. 반환: 실패 사유 (성공이면 None, 실패 시 부품·서비스 정리)."""
        loop = asyncio.get_running_loop()

        # 서비스 생성 (실패하면 부품만 정리)
        try:
            self._wire(mode, parts)
        except Exception as e:
            logger.exception(f"{mode} 서비스 생성 실패 — 부품 정리")
            await loop.run_in_executor(None, self._release_parts, parts)
            return f"서비스 생성 실패: {type(e).__name__}: {e}"

        # 서비스 기동 (실패하면 서비스·부품 모두 정리)
        try:
            self._start_services()
        except Exception as e:
            logger.exception(f"{mode} 서비스 기동 실패 — 서비스·부품 정리")
            await self._teardown()
            return f"서비스 기동 실패: {type(e).__name__}: {e}"
        return None

    def _wire(self, mode: str, parts):
        """종속 서비스를 만들고, 모두 성공하면 부품·서비스·모드를 보관한다 (기동은 하지 않는다). 반환값 없음.

        시각화는 백엔드 측정값을 그린다. 손 표시는 측정값(손 버스)이 있으면 그것을,
        없으면(호밍 전·손 없음) 조작 입력(fingers) 쪽을 쓴다.
        """
        backend, hand, camera = parts

        # 종속 서비스 생성: 제어기 → 상태 방송 → 인식(카메라 있을 때만)
        controller = RobotController(
            self._hub, self._robot_model, self._solver, self._arm_retargeter, backend, hand, glove=self._glove,
        )
        state_broadcaster = RobotStateBroadcaster(
            self._hub, self._robot_model, backend,
            hand_source=controller,
            control_source=controller,
            source_name="sim" if mode == "dev" else "robot",
        )
        perception = (
            PerceptionService(camera, self._hub, self._robot_model, backend)
            if camera is not None else None
        )

        # 생성이 모두 성공한 뒤 부품·서비스·모드 보관
        self._backend, self._hand, self._camera = parts
        self._controller = controller
        self._state_broadcaster = state_broadcaster
        self._perception = perception
        self._mode = mode

    def _start_services(self):
        """보관된 종속 서비스를 기동한다 (이전에 래치된 명령은 폐기). 반환값 없음."""
        # 전환 전에 래치된 명령 폐기
        stale_command = self._hub.take_command()
        if stale_command is not None:
            logger.warning(f"모드 기동 전 래치된 명령 폐기: {stale_command}")

        # 상태 방송 → 제어기 → 인식 순으로 기동
        self._state_broadcaster.start()
        self._controller.start()
        if self._perception is not None:
            self._perception.start()

    async def _teardown(self):
        """종속 서비스를 멈추고 부품을 해제한다 (없으면 무시, 단계별 실패는 로그만 남김). 반환값 없음.

        명령 서비스 stop이 스트림을 닫고(서보 유지), 백엔드 disconnect가 구독을 정리한다.
        """
        # 서비스 정지: 제어기 → 상태 방송 → 인식
        services = (
            ("제어기", self._controller),
            ("상태 방송", self._state_broadcaster),
            ("인식", self._perception),
        )
        for service_name, service in services:
            if service is None:
                continue
            try:
                await service.stop()
            except Exception:
                logger.exception(f"{service_name} 정지 실패")
        self._controller = None
        self._state_broadcaster = None
        self._perception = None

        # 부품·모드 보관 해제 후 executor에서 부품 해제
        parts = (self._backend, self._hand, self._camera)
        self._backend = None
        self._hand = None
        self._camera = None
        self._mode = None
        await asyncio.get_running_loop().run_in_executor(None, self._release_parts, parts)
        logger.info("모드 부품 해제 완료")

    async def _broadcast_status(self, ok: bool, message: str, switching: bool = False):
        """모드 상태를 보관(신규 접속용)하고 전 클라이언트에 방송한다. 반환값 없음."""
        # 상태 메시지 구성
        status = {
            "type": "mode_status",
            "mode": self._mode,
            "switching": switching,
            "ok": ok,
            "message": message,
        }

        # 신규 접속용 보관 후 방송
        self._hub.mode_status = status
        await self._hub.broadcast_state(status)


def request_shutdown(shutdown_event: asyncio.Event, received_signal: signal.Signals):
    """종료 시그널 수신을 기록하고 종료 이벤트를 세운다. 반환값 없음."""
    logger.warning(f"종료 시그널 수신: {received_signal.name}")
    shutdown_event.set()


async def shutdown_services(server, mode_manager, glove, rtc_hub):
    """서버 → 모드 → 장갑 → 허브 순으로 정리한다 (단계별 실패는 로그만 남기고 다음 단계 진행). 반환값 없음."""
    loop = asyncio.get_running_loop()

    # 정리 단계 목록
    shutdown_steps = (
        ("HTTPS 서버 정지", server.stop),
        ("모드 부품·서비스 정리", mode_manager.stop),
        ("Manus 장갑 정지", glove.stop),
        ("WebRTC 연결 정리", rtc_hub.close_all),
        ("WebRTC 허브 정지", rtc_hub.stop),
    )

    # 단계별 정리: 코루틴은 직접 await, 블로킹 함수는 executor에서
    for step_name, step in shutdown_steps:
        try:
            if inspect.iscoroutinefunction(step):
                await step()
            else:
                await loop.run_in_executor(None, step)
        except Exception:
            logger.exception(f"종료 단계 실패: {step_name}")
    logger.info("종료 정리 완료")


async def run():
    """공통 스택을 조립·기동하고 종료 신호(SIGTERM·SIGINT)까지 대기한다. 반환값 없음."""
    logger.info(f"초기 실행 모드: {config.TELEOP_MODE} (웹 데스크톱 뷰에서 런타임 전환 가능)")

    # IK 스택 조립: 모델 로드 → 솔버(Pink) 구성 → 팔 리타게터
    robot_model = RobotModel()
    solver = ArmIkSolver(robot_model)
    arm_retargeter = ArmRetargeter()

    # WebRTC 허브와 HTTPS 서버 생성
    rtc_hub = RtcHub()
    server = WebAPIServer(rtc_hub)

    # 모드 무관 조작자 손가락 입력(Manus 장갑) 생성
    glove = ManusGlove()

    # 모드 매니저 생성과 허브의 set_mode 요청·제어 중 조회 연결
    mode_manager = ModeManager(rtc_hub, robot_model, solver, arm_retargeter, glove=glove)
    rtc_hub.on_set_mode = mode_manager.request_switch
    rtc_hub.armed_provider = mode_manager.is_armed

    # SIGTERM·SIGINT 수신 시 종료 이벤트를 세워 정리 경로로 진입
    shutdown_event = asyncio.Event()
    loop = asyncio.get_running_loop()
    for shutdown_signal in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(shutdown_signal, request_shutdown, shutdown_event, shutdown_signal)

    # 기동 후 종료 시그널 대기, 종료·실패 시 정리
    try:
        glove.start()
        rtc_hub.start()
        await mode_manager.start(config.TELEOP_MODE)
        await server.start()
        logger.info("서버 기동 완료 — 종료 시그널 대기")
        await shutdown_event.wait()
    finally:
        await shutdown_services(server, mode_manager, glove, rtc_hub)


def main():
    """로깅을 초기화하고 asyncio 루프를 실행한다."""
    setup_logging(log_file=config.LOG_FILE)

    # 서버 실행과 종료 사유 로그
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        logger.info("종료 요청 수신, 정상 종료")
    except SystemExit as e:
        logger.critical(f"기동 실패로 종료: {e}")
        raise
    except Exception:
        logger.exception("서버 비정상 종료")
        raise


if __name__ == "__main__":
    main()
