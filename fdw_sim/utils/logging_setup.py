"""표준 로깅 설정."""
from __future__ import annotations

import logging
import sys
import time
from pathlib import Path
from typing import Optional


def setup_logging(level: str = "INFO",
                  log_file: Optional[Path] = None,
                  fmt: Optional[str] = None,
                  disable_file_log: bool = False) -> None:
    """루트 로거 설정.

    콘솔 + 파일 핸들러를 부착한다.

    log_file을 명시하지 않으면 fdw_sim/logs/run_<timestamp>.log에 자동으로
    기록한다 (KPILogger의 run_%Y%m%d_%H%M%S 명명 규칙과 동일해 상호 대조
    가능). Isaac Sim의 SimulationApp이 sys.stdout을 자체 콘솔 캡처용으로
    가로채는 환경(Kit)에서는 logger.exception()의 멀티라인 traceback이
    터미널에 온전히 안 보이는 경우가 실측됨(2026-09-28 DGX Spark) — 파일
    핸들러는 그 가로채기와 무관하게 직접 파일 I/O로 기록되므로 항상 전체
    traceback을 보존한다. disable_file_log=True로 끌 수 있다(예: 테스트).
    """
    fmt = fmt or "%(asctime)s [%(levelname)5s] %(name)s: %(message)s"
    handlers: list = [logging.StreamHandler(sys.stdout)]

    if log_file is None and not disable_file_log:
        log_file = Path("fdw_sim") / "logs" / time.strftime("run_%Y%m%d_%H%M%S.log")

    if log_file is not None:
        log_file = Path(log_file)
        log_file.parent.mkdir(parents=True, exist_ok=True)
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
        print(f"[LOG] full log (with tracebacks) written to: {log_file}",
              file=sys.stderr)

    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format=fmt,
        handlers=handlers,
        force=True,
    )

    # Isaac Sim 자체 로그 너무 시끄러움 방지
    for noisy in ("omni", "carb", "OmniGraph", "kit"):
        logging.getLogger(noisy).setLevel(logging.WARNING)
