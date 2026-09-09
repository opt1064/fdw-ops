"""Welding Cell — 용접 분산지능 셀.

PoC-1 구현 범위:
    - 부품 수령 → 지그 고정(가상) → 비전 측정(Gap 예측 surrogate)
    - Tool Selection Agent (Laser vs Arc)
    - 용접 경로 수행 (시간 모델)
    - 품질 예측 surrogate model

실제 열변형/용융풀 해석은 하지 않고, surrogate function으로 품질 점수를 산출.
이후 Level 2/3에서 Isaac Sim 로봇 모션, 합성데이터, RL로 확장 가능.
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


# =============================================================================
# Edge AI Agents (surrogate models)
# =============================================================================
@dataclass
class GapPredictionAgent:
    """비전 기반 용접 갭 예측 에이전트 (surrogate)."""
    noise_std: float = 0.05

    def predict_gap_mm(self, part_id: str) -> float:
        # 실제로는 RGB+Depth 입력 → CNN 추론. 여기서는 부품 ID 기반 결정론적 + 노이즈.
        seed = sum(ord(c) for c in part_id) % 1000
        rng = random.Random(seed)
        base = rng.uniform(0.1, 0.6)         # mm
        return max(0.0, base + rng.gauss(0.0, self.noise_std))


@dataclass
class ToolSelectionAgent:
    """Gap 크기에 따라 Laser / Arc 툴을 선택."""
    gap_threshold_mm: float = 0.3

    def select(self, gap_mm: float) -> str:
        # gap이 작으면 Laser(고정밀), 크면 Arc(갭 메우기 능력)
        return "laser" if gap_mm < self.gap_threshold_mm else "arc"


@dataclass
class QualityPredictionAgent:
    """공정 파라미터 + Gap → 품질 점수 surrogate."""

    def predict(self, *, tool: str, gap_mm: float, speed: float, power: float) -> QualityPrediction:
        # 정규화
        gap_penalty = max(0.0, gap_mm - 0.4) * 0.8        # gap이 클수록 감점
        # 툴별 sweet spot
        if tool == "laser":
            speed_opt, power_opt = 0.10, 1500.0
        else:  # arc
            speed_opt, power_opt = 0.08, 220.0
        speed_penalty = abs(speed - speed_opt) / max(speed_opt, 1e-3) * 0.3
        power_penalty = abs(power - power_opt) / max(power_opt, 1e-3) * 0.3

        score = max(0.0, min(1.0, 1.0 - gap_penalty - speed_penalty - power_penalty))
        defect_risk = 1.0 - score
        return QualityPrediction(score=score, defect_risk=defect_risk, confidence=0.85)


# =============================================================================
# Welding Cell
# =============================================================================
class WeldingCell(DistributedIntelligenceCell):
    """용접 분산지능 셀."""

    def __init__(self, config: CellConfig, bus: MessageBus,
                 default_cycle_time: float = 18.0) -> None:
        super().__init__(config, bus)
        self.default_cycle_time = default_cycle_time

        # Edge AI Agents
        self.gap_agent = GapPredictionAgent()
        self.tool_agent = ToolSelectionAgent()
        self.quality_agent = QualityPredictionAgent()

        # 현재 활성 툴 / 레시피
        self.active_tool: str = "laser"
        self.active_recipe: Dict[str, Any] = {}

        # 측정값
        self.predicted_gap_mm: float = 0.0

    # ------------------------------------------------------------------ setup
    def configure(self) -> None:
        logger.info("[%s] WeldingCell configured (default_cycle=%.1fs)",
                    self.cell_id, self.default_cycle_time)

    # =========================================================================
    # Command 처리
    # =========================================================================
    def on_command(self, command: DispatchCommand) -> bool:
        op = command.operation
        if op == "START_WELDING":
            if not self.input_buffer.occupied:
                logger.warning("[%s] cannot START_WELDING: input buffer empty", self.cell_id)
                return False
            if self.output_buffer.occupied:
                logger.warning("[%s] cannot START_WELDING: output buffer full", self.cell_id)
                return False

            recipe = command.recipe or {}
            # 1) Gap 예측
            self.predicted_gap_mm = self.gap_agent.predict_gap_mm(self.input_buffer.part_id)
            # 2) Tool 선택 (recipe에서 강제 지정 시 우선)
            self.active_tool = recipe.get("tool") or self.tool_agent.select(self.predicted_gap_mm)
            # 3) 사이클 타임 결정 (속도/길이 기반 단순화)
            speed = float(recipe.get("speed", 0.10))
            length_mm = float(recipe.get("path_length_mm", 200.0))
            self.cycle_time_estimate = max(5.0, length_mm / 1000.0 / max(speed, 1e-3))
            self.active_recipe = {
                "tool": self.active_tool,
                "speed": speed,
                "power": float(recipe.get("power", 1500.0 if self.active_tool == "laser" else 220.0)),
                "path_length_mm": length_mm,
                "predicted_gap_mm": self.predicted_gap_mm,
            }
            logger.info("[%s] recipe locked: tool=%s, gap=%.2fmm, cycle=%.1fs",
                        self.cell_id, self.active_tool, self.predicted_gap_mm,
                        self.cycle_time_estimate)
            self.progress = 0.0
            return True

        if op == "TOOL_CHANGE":
            target_tool = (command.recipe or {}).get("tool")
            if target_tool in ("laser", "arc"):
                self.active_tool = target_tool
                logger.info("[%s] tool changed to %s", self.cell_id, target_tool)
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

        # 작업 완료 → 품질 예측
        self.quality = self.quality_agent.predict(
            tool=self.active_recipe.get("tool", self.active_tool),
            gap_mm=self.predicted_gap_mm,
            speed=self.active_recipe.get("speed", 0.10),
            power=self.active_recipe.get("power", 1500.0),
        )
        self.log_kpi("predicted_gap_mm", self.predicted_gap_mm, unit="mm")
        self.log_kpi("active_tool", 1.0, tags={"tool": self.active_tool})
        self._complete_job()
