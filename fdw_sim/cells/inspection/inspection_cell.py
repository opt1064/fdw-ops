"""Inspection Cell — 검사/품질판정 분산지능 셀.

PoC-1 구현 범위:
    - 부품 수령 → 3D 스캔(Vision/Depth) → 치수/외관 검사 → 합격/재작업 판정
    - 직전 셀(Welding 등)에서 전달된 quality_prediction을 입력으로 사용
    - Inspection Agent: 다중 메트릭(score, defect_risk)을 종합해 PASS/FAIL/REWORK 판정

Level 2/3에서는 Replicator 합성데이터, 실제 카메라/Depth 센서 + ML 모델로 확장.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional
import logging
import random

from fdw_sim.cells.base.cell_base import (
    CellConfig,
    DistributedIntelligenceCell,
)
from fdw_sim.messaging.bus import MessageBus
from fdw_sim.messaging.schemas import (
    CellState,
    CellType,
    DispatchCommand,
    QualityPrediction,
)

logger = logging.getLogger(__name__)


@dataclass
class InspectionAgent:
    """간단한 검사 판정 surrogate.

    quality.score가 임계값 이상이면 PASS,
    그 미만이지만 defect_risk가 낮으면 REWORK,
    심각하면 FAIL.
    """
    pass_threshold: float = 0.85
    rework_threshold: float = 0.65

    def judge(self, q: QualityPrediction, noise_std: float = 0.03) -> Dict[str, Any]:
        # 측정 노이즈
        measured_score = max(0.0, min(1.0, q.score + random.gauss(0.0, noise_std)))
        if measured_score >= self.pass_threshold:
            verdict = "PASS"
        elif measured_score >= self.rework_threshold:
            verdict = "REWORK"
        else:
            verdict = "FAIL"
        return {
            "verdict": verdict,
            "measured_score": measured_score,
            "defect_risk": q.defect_risk,
        }


class InspectionCell(DistributedIntelligenceCell):
    """검사/품질판정 셀."""

    def __init__(self, config: CellConfig, bus: MessageBus,
                 default_cycle_time: float = 6.0) -> None:
        super().__init__(config, bus)
        self.default_cycle_time = default_cycle_time
        self.agent = InspectionAgent()

        # 누적 통계
        self.total_inspected: int = 0
        self.total_pass: int = 0
        self.total_rework: int = 0
        self.total_fail: int = 0

        # 입력 시점에 직전 셀로부터 받은 품질 예측을 보존
        self._incoming_quality: QualityPrediction = QualityPrediction()
        self.last_verdict: Optional[str] = None

    # ------------------------------------------------------------------ setup
    def configure(self) -> None:
        logger.info("[%s] InspectionCell configured (cycle=%.1fs)",
                    self.cell_id, self.default_cycle_time)

    # 부품을 받을 때 직전 셀의 quality_prediction을 함께 받기 위한 확장 hook
    def receive_part_with_quality(self, part_id: str,
                                  quality: Optional[QualityPrediction]) -> bool:
        ok = self.receive_part(part_id)
        if ok and quality is not None:
            self._incoming_quality = quality
        return ok

    # =========================================================================
    # Command 처리
    # =========================================================================
    def on_command(self, command: DispatchCommand) -> bool:
        op = command.operation
        if op == "START_INSPECTION":
            if not self.input_buffer.occupied:
                logger.warning("[%s] cannot START_INSPECTION: input buffer empty", self.cell_id)
                return False
            if self.output_buffer.occupied:
                logger.warning("[%s] cannot START_INSPECTION: output buffer full", self.cell_id)
                return False
            recipe = command.recipe or {}
            self.cycle_time_estimate = float(recipe.get("cycle_time", self.default_cycle_time))
            self.progress = 0.0
            return True

        if op == "RESET":
            self.progress = 0.0
            return True

        return False

    # =========================================================================
    # PROCESSING 진행
    # =========================================================================
    def step_processing(self, dt: float) -> None:
        if self.cycle_time_estimate <= 0:
            return
        self.progress += dt / self.cycle_time_estimate
        if self.progress < 1.0:
            return

        # 검사 판정
        result = self.agent.judge(self._incoming_quality)
        verdict = result["verdict"]
        measured_score = result["measured_score"]
        self.last_verdict = verdict

        self.quality = QualityPrediction(
            score=measured_score,
            defect_risk=result["defect_risk"],
            confidence=0.95,
        )

        # 통계 업데이트
        self.total_inspected += 1
        if verdict == "PASS":
            self.total_pass += 1
        elif verdict == "REWORK":
            self.total_rework += 1
        else:
            self.total_fail += 1

        # KPI 기록
        self.log_kpi("inspection_measured_score", measured_score)
        self.log_kpi("inspection_verdict", 1.0,
                     tags={"verdict": verdict, "part_id": str(self.input_buffer.part_id)})
        if self.total_inspected > 0:
            self.log_kpi("quality_pass_rate", self.total_pass / self.total_inspected)

        logger.info("[%s] verdict=%s score=%.3f (pass=%d/%d)",
                    self.cell_id, verdict, measured_score,
                    self.total_pass, self.total_inspected)

        self._complete_job()
