"""FDW-OPS 공통 메시지 스키마.

모든 분산지능 셀은 이 스키마를 따라 통신해야 합니다.
이 스키마는 추후 ROS 2 메시지(.msg)나 Protobuf로 1:1 변환 가능하도록
플랫(flat)하고 명시적으로 설계되었습니다.
"""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import Any, Dict, Optional
import json
import time
import uuid


# =============================================================================
# Enums
# =============================================================================
class CellState(str, Enum):
    """셀 상태머신 정의."""
    IDLE = "IDLE"
    READY = "READY"
    PROCESSING = "PROCESSING"
    BLOCKED = "BLOCKED"      # 입출력 버퍼 막힘
    FAULT = "FAULT"          # 이상 발생
    RECOVERY = "RECOVERY"    # 복구 중
    OFFLINE = "OFFLINE"


class CellType(str, Enum):
    """셀 유형 (FDW 분산지능 셀 분류 기준)."""
    MATERIAL = "material"        # 소재/이송
    FORMING = "forming"          # 소성가공
    WELDING = "welding"          # 용접
    WAAM = "waam"                # 적층
    MACHINING = "machining"      # 정밀가공
    INSPECTION = "inspection"    # 검사/품질


class CommandType(str, Enum):
    START = "START"
    PAUSE = "PAUSE"
    ABORT = "ABORT"
    RESET = "RESET"
    LOAD_RECIPE = "LOAD_RECIPE"
    TOOL_CHANGE = "TOOL_CHANGE"


# =============================================================================
# Message Schemas
# =============================================================================
@dataclass
class BufferStatus:
    """단일 입출력 버퍼 상태."""
    occupied: bool = False
    part_id: Optional[str] = None
    capacity: int = 1
    queue_length: int = 0


@dataclass
class QualityPrediction:
    """셀 출력물의 품질 예측 결과."""
    score: float = 1.0           # 0.0 ~ 1.0 (1.0 = 완벽)
    defect_risk: float = 0.0     # 0.0 ~ 1.0 (0.0 = 결함 없음)
    confidence: float = 1.0      # 모델 신뢰도


@dataclass
class HealthStatus:
    """장비 헬스 상태."""
    status: str = "normal"       # normal | warning | critical
    alarm_code: Optional[str] = None
    message: Optional[str] = None


@dataclass
class CellStatusMessage:
    """셀 → FDW-OPS 상태 보고 메시지.

    /fdw/cell_status 토픽에 발행됨.
    """
    cell_id: str
    cell_type: CellType
    state: CellState = CellState.IDLE
    current_job_id: Optional[str] = None
    input_buffer: BufferStatus = field(default_factory=BufferStatus)
    output_buffer: BufferStatus = field(default_factory=BufferStatus)
    estimated_remaining_time_sec: float = 0.0
    quality_prediction: QualityPrediction = field(default_factory=QualityPrediction)
    health: HealthStatus = field(default_factory=HealthStatus)
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["cell_type"] = self.cell_type.value
        d["state"] = self.state.value
        return d

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False)


@dataclass
class DispatchCommand:
    """FDW-OPS → 특정 셀 작업 지시.

    /fdw/dispatch_command 토픽에 발행됨.
    """
    target_cell: str
    job_id: str
    operation: str                              # 예: "START_WELDING"
    recipe: Dict[str, Any] = field(default_factory=dict)
    next_cell: Optional[str] = None
    priority: str = "normal"                    # low | normal | high | critical
    command_id: str = field(default_factory=lambda: f"CMD_{uuid.uuid4().hex[:8].upper()}")
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False)


@dataclass
class MaterialTransferCommand:
    """FDW-OPS → 이송장비(AMR) 이송 지시.

    /fdw/material_transfer 토픽에 발행됨.
    """
    from_cell: str
    to_cell: str
    part_id: str
    carrier: str                                # 예: "AMR_02"
    priority: str = "normal"
    command_id: str = field(default_factory=lambda: f"MOVE_{uuid.uuid4().hex[:8].upper()}")
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False)


@dataclass
class KPIRecord:
    """KPI 단일 레코드 (셀 또는 시스템 레벨)."""
    timestamp: float
    cell_id: Optional[str]              # None이면 시스템 전역 KPI
    metric: str                         # cycle_time, utilization, quality_pass_rate, ...
    value: float
    unit: str = ""
    tags: Dict[str, str] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class JobSpec:
    """오케스트레이터가 발행하는 작업(Job) 명세.

    한 Job = 하나의 부품이 입고~출하까지 거치는 공정 시퀀스.
    """
    job_id: str
    part_id: str
    part_type: str                              # 예: "tubular_frame_A"
    process_route: list                         # 예: ["forming", "welding", "machining", "inspection"]
    recipe_overrides: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    priority: str = "normal"
    created_at: float = field(default_factory=time.time)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)
