"""실행 중 모니터링 로그 포맷 통일.

모든 엔트리포인트는 setup_logging()을 호출한 뒤 logging.getLogger(모듈명)를 사용한다.
회전 파일 로그는 서버 엔트리포인트(teleop.main)만 log_file을 넘겨 기록한다.
"""

import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path

from teleop.utils import config


def setup_logging(level: int = logging.INFO, log_file: Path | None = None):
    """루트 로거를 공통 포맷(시각-레벨-모듈-메시지)으로 초기화한다 (log_file이 있으면 회전 파일에도 기록). 반환값 없음."""
    handlers: list[logging.Handler] = [logging.StreamHandler()]

    # 파일 경로가 주어지면 크기 기준 회전 파일 핸들러 추가
    if log_file is not None:
        log_file.parent.mkdir(parents=True, exist_ok=True)
        handlers.append(RotatingFileHandler(
            log_file,
            maxBytes=config.LOG_MAX_BYTES,
            backupCount=config.LOG_BACKUP_COUNT,
            encoding="utf-8",
        ))

    logging.basicConfig(
        level=level,
        format="%(asctime)s - %(levelname)-8s - %(name)s - %(message)s",
        handlers=handlers,
    )
