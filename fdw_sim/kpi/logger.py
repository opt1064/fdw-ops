"""KPI Logger.

모든 KPIRecord를 구독하여:
    - 메모리 내 시계열 누적
    - JSONL 파일에 append
    - 종료 시 요약 리포트 생성
"""
from __future__ import annotations

from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional
import json
import logging
import time

from fdw_sim.messaging.bus import MessageBus, Topics
from fdw_sim.messaging.schemas import KPIRecord

logger = logging.getLogger(__name__)


class KPILogger:
    """KPI 수집기.

    버스의 KPI_LOG 토픽을 구독하여 모든 KPIRecord를:
        - 메모리에 누적
        - JSONL 파일에 append
        - 종료 시 summary.json으로 출력
    """

    def __init__(self,
                 log_dir: Path = Path("./logs"),
                 run_name: Optional[str] = None,
                 flush_every: int = 50) -> None:
        self.log_dir: Path = Path(log_dir)
        self.run_name: Optional[str] = run_name
        self.flush_every: int = flush_every

        self._records: List[KPIRecord] = []
        self._by_metric: Dict[str, List[KPIRecord]] = defaultdict(list)

        self.log_dir.mkdir(parents=True, exist_ok=True)
        run = self.run_name or time.strftime("run_%Y%m%d_%H%M%S")
        self._path: Path = self.log_dir / f"{run}_kpi.jsonl"
        self._file_handle: Optional[Any] = open(self._path, "a", encoding="utf-8")
        logger.info("[KPI] writing to %s", self._path)

    # ------------------------------------------------------------------ wiring
    def attach(self, bus: MessageBus) -> None:
        bus.subscribe(Topics.KPI_LOG, self._on_record)

    def _on_record(self, rec: KPIRecord) -> None:
        self._records.append(rec)
        self._by_metric[rec.metric].append(rec)
        if self._file_handle is not None:
            self._file_handle.write(json.dumps(rec.to_dict(), ensure_ascii=False) + "\n")
            if len(self._records) % self.flush_every == 0:
                self._file_handle.flush()

    # ------------------------------------------------------------------ query
    def metrics(self) -> List[str]:
        return list(self._by_metric.keys())

    def latest(self, metric: str) -> Optional[KPIRecord]:
        if metric not in self._by_metric or not self._by_metric[metric]:
            return None
        return self._by_metric[metric][-1]

    def summary(self) -> Dict[str, Dict[str, float]]:
        out: Dict[str, Dict[str, float]] = {}
        for m, recs in self._by_metric.items():
            vals = [r.value for r in recs]
            if not vals:
                continue
            out[m] = {
                "count": float(len(vals)),
                "min": min(vals),
                "max": max(vals),
                "avg": sum(vals) / len(vals),
                "last": vals[-1],
            }
        return out

    # ------------------------------------------------------------------ output
    def write_summary(self, path: Optional[Path] = None) -> Path:
        if path is None:
            path = self.log_dir / f"{self._path.stem.replace('_kpi', '')}_summary.json"
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.summary(), f, indent=2, ensure_ascii=False)
        logger.info("[KPI] summary written to %s", path)
        return path

    def close(self) -> None:
        if self._file_handle is not None:
            self._file_handle.flush()
            self._file_handle.close()
            self._file_handle = None
        self.write_summary()
