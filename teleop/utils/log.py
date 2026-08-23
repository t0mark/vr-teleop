"""실행 중 모니터링 로그 포맷 통일.

모든 엔트리포인트는 setup_logging()을 호출한 뒤 logging.getLogger(모듈명)를 사용한다.
"""

import logging


def setup_logging(level: int = logging.INFO):
    """루트 로거를 공통 포맷(시각-레벨-모듈-메시지)으로 초기화한다. 반환값 없음."""
    logging.basicConfig(
        level=level,
        format="%(asctime)s - %(levelname)-8s - %(name)s - %(message)s",
    )
