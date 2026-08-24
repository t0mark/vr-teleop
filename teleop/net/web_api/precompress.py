"""정적 자산 사전 압축: aiohttp가 자동으로 집어 가는 `.gz` 형제 파일을 만들어 둔다.

aiohttp의 정적 핸들러는 요청이 `Accept-Encoding: gzip`을 포함하면 `<파일>.gz`가 옆에 있을 때
그것을 대신 보낸다(Content-Encoding: gzip). 즉 서버 코드를 바꾸지 않고 전송량만 줄일 수 있다.

대상은 **크고 잘 안 바뀌는** 자산(벤더 JS, GLB 메시)뿐이다. 자주 고치는 `web/src/`는 제외한다 —
서버 기동 때만 압축본을 갱신하므로, 편집이 잦은 파일까지 포함하면 낡은 .gz가 서빙되는 함정이 생긴다
(합계 수십 KB라 압축 이득도 없다).

호출 사이 유지되는 상태가 없어 모듈 수준 함수로 둔다.
"""

import gzip
import logging
import shutil
from pathlib import Path

logger = logging.getLogger("precompress")

# 압축 이득이 있는 확장자 (GLB는 정점 데이터가 반복이 많아 40% 수준까지 줄어든다)
COMPRESSIBLE_SUFFIXES = (".js", ".css", ".glb", ".gltf", ".json", ".svg")
# 이보다 작은 파일은 압축·보관 비용 대비 이득이 미미해 건너뛴다
MIN_SIZE_BYTES = 32 * 1024


def _needs_update(source: Path, target: Path) -> bool:
    """압축본을 (다시) 만들어야 하는지 반환한다. 원본이 더 최근이면 갱신 대상이다."""
    return not target.exists() or target.stat().st_mtime < source.stat().st_mtime


def precompress_dir(root: Path) -> tuple[int, int]:
    """root 아래 대상 파일마다 `.gz`를 생성한다. 반환: (생성 수, 절감 바이트).

    쓰기가 불가능한 위치(읽기 전용 마운트 등)면 조용히 건너뛴다 — 압축은 최적화일 뿐이라
    실패가 서버 기동을 막아서는 안 된다.
    """
    if not root.is_dir():
        return 0, 0

    count = 0
    saved = 0
    for source in root.rglob("*"):
        if source.suffix not in COMPRESSIBLE_SUFFIXES or not source.is_file():
            continue
        if source.stat().st_size < MIN_SIZE_BYTES:
            continue
        target = source.with_name(source.name + ".gz")
        if not _needs_update(source, target):
            saved += source.stat().st_size - target.stat().st_size
            continue
        try:
            with source.open("rb") as fin, gzip.open(target, "wb", compresslevel=6) as fout:
                shutil.copyfileobj(fin, fout)
        except OSError as e:
            logger.warning(f"압축 실패, 원본으로 서빙한다 ({source.name}): {e}")
            continue
        count += 1
        saved += source.stat().st_size - target.stat().st_size
    return count, saved


def precompress_all(roots: list[Path]):
    """여러 정적 루트를 사전 압축하고 결과를 로그로 남긴다. 반환값 없음."""
    total_new = 0
    total_saved = 0
    for root in roots:
        new, saved = precompress_dir(root)
        total_new += new
        total_saved += saved
    logger.info(f"정적 자산 사전 압축: 신규 {total_new}개, 전송 절감 {total_saved / 1e6:.1f} MB")
