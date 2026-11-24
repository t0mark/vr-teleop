"""HTTPS 정적 파일 서버 + WebRTC 시그널링 라우트: web/ 프론트엔드 서빙과 offer/answer 교환을 연다.

포트 3개를 연다.
- 8443: HTTPS 본 포트 (WebRTC 시그널링 POST 포함)
- 443: HTTPS 별칭 (https://IP 로 접속)
- 80: HTTP → HTTPS 리다이렉트 (IP만 입력해 접속)

정적 자산 사전 압축(기동 시): 벤더 JS·assets/ 의 큰 파일마다 `<파일>.gz` 형제 파일을 만들어 두면,
aiohttp 정적 핸들러가 `Accept-Encoding: gzip` 요청에 그것을 대신 보낸다. `web/src/`는 압축 대상에서 제외한다.
"""

import asyncio
import gzip
import logging
import shutil
import ssl
from pathlib import Path

from aiohttp import web

from teleop.net.rtc_hub import RtcHub
from teleop.utils import config

logger = logging.getLogger("web_server")

# 사전 압축 대상 확장자
_COMPRESSIBLE_SUFFIXES = (".js", ".css", ".glb", ".gltf", ".json", ".svg")
# 사전 압축 최소 파일 크기 (바이트, 미만은 건너뜀)
_COMPRESS_MIN_BYTES = 32 * 1024


def _compress_dir(root: Path) -> tuple[int, int]:
    """root 아래 대상 파일마다 `.gz` 형제 파일을 만든다. 반환: (생성 수, 절감 바이트).

    원본이 더 최근이면 다시 만든다. 쓰기 실패한 파일은 경고만 남기고 원본으로 서빙한다.
    """
    if not root.is_dir():
        return 0, 0

    created_count = 0
    saved_bytes = 0
    for source in root.rglob("*"):
        # 압축 대상 확장자·크기 필터
        if source.suffix not in _COMPRESSIBLE_SUFFIXES or not source.is_file():
            continue
        if source.stat().st_size < _COMPRESS_MIN_BYTES:
            continue

        # 원본보다 최신인 .gz가 있으면 재사용
        target = source.with_name(source.name + ".gz")
        if target.exists() and target.stat().st_mtime >= source.stat().st_mtime:
            saved_bytes += source.stat().st_size - target.stat().st_size
            continue

        # 임시 파일에 압축한 뒤 rename으로 교체한다
        temp_target = source.with_name(source.name + ".gz.tmp")
        try:
            with source.open("rb") as source_file, gzip.open(temp_target, "wb", compresslevel=6) as gzip_file:
                shutil.copyfileobj(source_file, gzip_file)
            temp_target.replace(target)
        except OSError as e:
            logger.warning(f"압축 실패, 원본으로 서빙한다 ({source.name}): {e}")
            temp_target.unlink(missing_ok=True)
            continue

        created_count += 1
        saved_bytes += source.stat().st_size - target.stat().st_size
    return created_count, saved_bytes


def _compress_static_assets(roots: list[Path]):
    """여러 정적 루트를 사전 압축하고 결과를 로그로 남긴다. 반환값 없음."""
    total_created = 0
    total_saved = 0
    for root in roots:
        created_count, saved_bytes = _compress_dir(root)
        total_created += created_count
        total_saved += saved_bytes
    logger.info(f"정적 자산 사전 압축: 신규 {total_created}개, 전송 절감 {total_saved / 1e6:.1f} MB")


class WebAPIServer:
    """aiohttp 기반 HTTPS 서버(+별칭·리다이렉트 포트)를 소유·관리한다 (기동/종료 상태 유지)."""

    def __init__(self, rtc_hub: RtcHub, host: str = config.HTTP_HOST, port: int = config.HTTPS_PORT):
        self._hub = rtc_hub
        self._host = host
        self._port = port
        self._runner: web.AppRunner | None = None
        self._redirect_runner: web.AppRunner | None = None
        self._std_https_active = False

    @staticmethod
    @web.middleware
    async def _no_cache_middleware(request: web.Request, handler):
        """모든 응답에 no-cache(사용 전 재검증)를 설정한다."""
        response = await handler(request)
        response.headers["Cache-Control"] = "no-cache"
        return response

    def _build_app(self) -> web.Application:
        """라우트가 구성된 aiohttp Application을 생성해 반환한다."""
        app = web.Application(middlewares=[self._no_cache_middleware])

        # WebRTC 시그널링 라우트 (정적 라우트보다 먼저 등록)
        app.router.add_post(config.RTC_OFFER_PATH, self._handle_rtc_offer)

        # 로봇 시각화용 RBY1 GLB 메시 서빙
        app.router.add_static(config.MESH_ROUTE_PREFIX, config.RBY1_MESH_DIR, show_index=False)

        # 공용 정적 자산(아바타·카메라·손 GLB 등) 서빙
        app.router.add_static(config.ASSETS_ROUTE_PREFIX, config.ASSETS_DIR, show_index=False)

        # 루트 접속은 index.html, 나머지는 web/ 이하 정적 파일 서빙
        app.router.add_get("/", self._handle_index)
        app.router.add_static("/", config.WEB_ROOT, show_index=False)
        return app

    async def _handle_index(self, request: web.Request) -> web.FileResponse:
        """루트 경로 요청에 index.html을 반환한다."""
        # 페이지 요청 클라이언트 주소 기록
        logger.info(f"페이지 요청: {request.remote}")
        return web.FileResponse(config.WEB_ROOT / "index.html")

    async def _handle_rtc_offer(self, request: web.Request) -> web.Response:
        """POST /rtc/offer: SDP offer를 받아 WebRTC 협상 후 answer를 JSON으로 반환한다."""
        # 요청 본문 검증 (형식 오류는 400)
        try:
            offer = await request.json()
            sdp = offer["sdp"]
            sdp_type = offer["type"]
        except (ValueError, KeyError, TypeError) as e:
            logger.warning(f"잘못된 offer 요청 ({request.remote}): {type(e).__name__}: {e}")
            raise web.HTTPBadRequest(text=f"잘못된 offer 요청: {e}")

        # WebRTC 협상 (협상 실패는 500)
        try:
            answer = await self._hub.handle_offer(sdp, sdp_type, request.remote)
        except Exception as e:
            logger.error(f"WebRTC 협상 실패 ({request.remote}): {type(e).__name__}: {e}")
            raise web.HTTPInternalServerError(text=f"WebRTC 협상 실패: {e}")

        logger.info(f"WebRTC 협상 완료: {request.remote}")
        return web.json_response(answer)

    async def _handle_redirect(self, request: web.Request) -> web.Response:
        """평문 HTTP 요청을 같은 호스트의 HTTPS로 307 리다이렉트한다."""
        # Host 헤더에서 포트를 뗀 호스트명 (IPv6는 대괄호 유지)
        hostname = request.url.raw_host or ""
        if ":" in hostname:
            hostname = f"[{hostname}]"

        # 443 별칭이 살아 있으면 포트 없는 URL로, 아니면 본 포트로 보낸다
        port_part = "" if self._std_https_active else f":{self._port}"
        target = f"https://{hostname}{port_part}{request.rel_url}"
        logger.info(f"HTTP 리다이렉트: {request.remote} → {target}")
        raise web.HTTPTemporaryRedirect(target)

    def _build_ssl_context(self) -> ssl.SSLContext:
        """저장소 루트 certs/의 자체 서명 인증서로 TLS 컨텍스트를 생성해 반환한다."""
        if not (config.CERT_FILE.exists() and config.KEY_FILE.exists()):
            raise FileNotFoundError(
                f"인증서 없음: {config.CERT_DIR} — 먼저 python tools/make_https_cert.py 실행"
            )

        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_context.load_cert_chain(config.CERT_FILE, config.KEY_FILE)
        return ssl_context

    async def start(self):
        """HTTPS 본 포트·443 별칭·HTTP 리다이렉트를 기동한다. 반환값 없음."""
        # 벤더 JS·assets/ 사전 압축 (executor에서 실행)
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(
            None, _compress_static_assets, [config.WEB_ROOT / "vendor", config.ASSETS_DIR]
        )

        # HTTPS 본 포트 기동
        ssl_context = self._build_ssl_context()
        runner = web.AppRunner(self._build_app())
        await runner.setup()
        await web.TCPSite(runner, self._host, self._port, ssl_context=ssl_context).start()
        self._runner = runner
        logger.info(f"HTTPS 서버 기동: https://{self._host}:{self._port} (web root: {config.WEB_ROOT})")

        # HTTPS 별칭 포트 기동 (실패해도 본 서버는 계속 동작)
        try:
            await web.TCPSite(runner, self._host, config.HTTPS_STD_PORT, ssl_context=ssl_context).start()
            self._std_https_active = True
            logger.info(f"HTTPS 별칭 포트 기동: {config.HTTPS_STD_PORT}")
        except OSError as e:
            logger.warning(f"HTTPS 별칭 포트({config.HTTPS_STD_PORT}) 기동 실패, 건너뜀: {e}")

        # HTTP 리다이렉트 포트 기동 (실패해도 본 서버는 계속 동작)
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
