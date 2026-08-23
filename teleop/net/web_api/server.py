"""HTTPS 정적 파일 서버 + WSS 라우트: web/ 프론트엔드 서빙과 포즈 파이프라인 소켓을 연다.

접속 편의를 위해 3개 포트를 연다.
- 8443: HTTPS 본 포트 (WSS 포즈 업링크·모니터 다운링크 포함)
- 443: HTTPS 별칭 — 주소창에 https://IP 만 입력해도 접속되게 함
- 80: HTTP → HTTPS 리다이렉트 — 주소창에 IP만 입력(기본 http)해도 접속되게 함
"""

import logging
import ssl

from aiohttp import web

from teleop.net.web_api.pose_hub import PoseHub
from teleop.utils import config

logger = logging.getLogger("web_api")


class WebAPIServer:
    """aiohttp 기반 HTTPS 서버(+별칭·리다이렉트 포트)를 소유·관리한다 (기동/종료 상태 유지)."""

    def __init__(self, pose_hub: PoseHub, host: str = config.HTTP_HOST, port: int = config.HTTPS_PORT):
        self._pose_hub = pose_hub
        self._host = host
        self._port = port
        self._runner: web.AppRunner | None = None
        self._redirect_runner: web.AppRunner | None = None
        self._std_https_active = False

    @staticmethod
    @web.middleware
    async def _no_cache_middleware(request: web.Request, handler):
        """모든 응답에 no-cache를 설정한다.

        브라우저 휴리스틱 캐시가 수정 전 JS를 계속 서빙해 프론트 수정이 반영되지 않는
        문제가 있었음 (Quest·PC 공통). 재검증(304)은 허용하므로 대용량 파일 부담은 없다.
        """
        resp = await handler(request)
        resp.headers["Cache-Control"] = "no-cache"
        return resp

    def _build_app(self) -> web.Application:
        """라우트가 구성된 aiohttp Application을 생성해 반환한다."""
        app = web.Application(middlewares=[self._no_cache_middleware])
        # WSS 라우트를 정적 라우트보다 먼저 등록해야 /ws/* 가 파일 경로로 오인되지 않는다
        app.router.add_get(config.POSE_WS_PATH, self._pose_hub.handle_pose_ws)
        app.router.add_get(config.MONITOR_WS_PATH, self._pose_hub.handle_monitor_ws)
        # 로봇 시각화용 GLB 메시 (rby1-sdk 동봉 리소스를 읽기 전용 서빙)
        app.router.add_static(config.MESH_URL_PREFIX, config.RBY1_MESH_DIR, show_index=False)
        # 루트 접속은 index.html로 연결하고, 나머지는 web/ 이하 정적 파일 그대로 서빙
        app.router.add_get("/", self._handle_index)
        app.router.add_static("/", config.WEB_ROOT, show_index=False)
        return app

    async def _handle_index(self, request: web.Request) -> web.FileResponse:
        """루트 경로 요청에 index.html을 반환한다."""
        # 접속 확인이 곧 연결 진단이므로 클라이언트 주소를 실행 로그로 남긴다
        logger.info(f"페이지 요청: {request.remote}")
        return web.FileResponse(config.WEB_ROOT / "index.html")

    async def _handle_redirect(self, request: web.Request) -> web.Response:
        """평문 HTTP 요청을 같은 호스트의 HTTPS로 301 리다이렉트한다."""
        # 443 별칭이 살아 있으면 포트 없는 깔끔한 URL로, 아니면 본 포트로 보낸다
        hostname = request.host.split(":")[0]
        port_part = "" if self._std_https_active else f":{self._port}"
        target = f"https://{hostname}{port_part}{request.rel_url}"
        logger.info(f"HTTP 리다이렉트: {request.remote} → {target}")
        raise web.HTTPMovedPermanently(target)

    def _build_ssl_context(self) -> ssl.SSLContext:
        """web/certs/의 자체 서명 인증서로 TLS 컨텍스트를 생성해 반환한다."""
        if not (config.CERT_FILE.exists() and config.KEY_FILE.exists()):
            raise FileNotFoundError(
                f"인증서 없음: {config.CERT_DIR} — 먼저 python -m teleop.net.web_api.gen_certs 실행"
            )
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(config.CERT_FILE, config.KEY_FILE)
        return ctx

    async def start(self):
        """HTTPS 본 포트·443 별칭·HTTP 리다이렉트를 기동한다. 반환값 없음."""
        ssl_ctx = self._build_ssl_context()
        runner = web.AppRunner(self._build_app())
        await runner.setup()
        await web.TCPSite(runner, self._host, self._port, ssl_context=ssl_ctx).start()
        self._runner = runner
        logger.info(f"HTTPS 서버 기동: https://{self._host}:{self._port} (web root: {config.WEB_ROOT})")

        # 별칭·리다이렉트는 편의 기능이므로 실패(포트 점유 등)해도 본 서버는 계속 동작
        try:
            await web.TCPSite(runner, self._host, config.HTTPS_STD_PORT, ssl_context=ssl_ctx).start()
            self._std_https_active = True
            logger.info(f"HTTPS 별칭 포트 기동: {config.HTTPS_STD_PORT}")
        except OSError as e:
            logger.warning(f"HTTPS 별칭 포트({config.HTTPS_STD_PORT}) 기동 실패, 건너뜀: {e}")

        try:
            redirect_app = web.Application()
            redirect_app.router.add_route("*", "/{tail:.*}", self._handle_redirect)
            redirect_runner = web.AppRunner(redirect_app)
            await redirect_runner.setup()
            await web.TCPSite(redirect_runner, self._host, config.HTTP_REDIRECT_PORT).start()
            self._redirect_runner = redirect_runner
            logger.info(f"HTTP 리다이렉트 기동: {config.HTTP_REDIRECT_PORT} → HTTPS")
        except OSError as e:
            logger.warning(f"HTTP 리다이렉트 포트({config.HTTP_REDIRECT_PORT}) 기동 실패, 건너뜀: {e}")

    async def stop(self):
        """서버들을 종료하고 리소스를 해제한다. 반환값 없음."""
        for runner in (self._runner, self._redirect_runner):
            if runner is not None:
                await runner.cleanup()
        self._runner = None
        self._redirect_runner = None
        self._std_https_active = False
        logger.info("HTTPS 서버 종료")
