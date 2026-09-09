"""표준 로깅 설정."""
from __future__ import annotations

import logging
import sys
from pathlib import Path
from typing import Optional


def setup_logging(level: str = "INFO",
                  log_file: Optional[Path] = None,
                  fmt: Optional[str] = None) -> None:
    """루트 로거 설정.

    콘솔 + (선택적) 파일 핸들러를 부착한다.
    """
    fmt = fmt or "%(asctime)s [%(levelname)5s] %(name)s: %(message)s"
    handlers: list = [logging.StreamHandler(sys.stdout)]

    if log_file is not None:
        log_file = Path(log_file)
        log_file.parent.mkdir(parents=True, exist_ok=True)
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))

    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format=fmt,
        handlers=handlers,
        force=True,
    )

    # Isaac Sim 자체 로그 너무 시끄러움 방지
    for noisy in ("omni", "carb", "OmniGraph", "kit"):
        logging.getLogger(noisy).setLevel(logging.WARNING)
