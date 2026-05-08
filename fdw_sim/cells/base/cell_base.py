"""DistributedIntelligenceCell — 모든 FDW 분산지능 셀의 베이스 클래스.

각 공정 셀(material/welding/forming/waam/machining/inspection)은 이 클래스를
상속하여 다음 5개 컴포넌트를 채워 넣는다:

    1. Physical Asset       : 로봇/설비/지그/툴/버퍼 (USD prim 참조)
    2. Sensor Layer         : RGB / Depth / LiDAR / Force / Proximity
    3. Local Controller     : 동작 시퀀스 / 모션 / 툴 체인지
    4. Edge AI Agent        : 품질예측 / 이상탐지 / 파라미터 추천
    5. KPI Logger           : cycle_time / utilization / quality_pass_rate
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional
import logging
import time

from fdw_sim.messaging.bus import MessageBus, Topics
from fdw_sim.messaging.schemas import (
    BufferStatus,
    CellState,
    CellStatusMessage,
    CellType,
    DispatchCommand,
    HealthStatus,
    JobSpec,
    KPIRecord,
    QualityPrediction,
)
from fdw_sim.cells.base.state_machine import CellStateMachine

logger = logging.getLogger(__name__)


@dataclass
class CellConfig:
    """셀 인스턴스 생성을 위한 설정."""
    cell_id: str
    cell_type: CellType
    prim_path: str = ""                 # 예: "/World/FDW/Cells/Welding_Cell_01"
    input_capacity: int = 1
    output_capacity: int = 1
    status_publish_hz: float = 5.0      # 상태 발행 주기
    extra: Dict[str, Any] = field(default_factory=dict)


class DistributedIntelligenceCell(ABC):
    """모든 분산지능 셀의 공통 베이스.

    Lifecycle:
        configure()       -> Isaac Sim 자산 로드, 컨트롤러/에이전트 초기화
        receive_command() -> FDW-OPS의 DispatchCommand 수신 시 호출됨
        step(dt)          -> 매 시뮬레이션 틱마다 호출됨
        publish_status()  -> 주기적으로 CellStatusMessage 발행
    """

    def __init__(self, config: CellConfig, bus: MessageBus) -> None:
        self.config = config
        self.bus = bus
        self.cell_id = config.cell_id
        self.cell_type = config.cell_type

        # 상태머신
        self.fsm = CellStateMachine(self.cell_id, initial=CellState.OFFLINE)

        # 입출력 버퍼
        self.input_buffer = BufferStatus(capacity=config.input_capacity)
        self.output_buffer = BufferStatus(capacity=config.output_capacity)

        # 현재 진행 중인 Job
        self.current_job: Optional[JobSpec] = None
        self.current_command: Optional[DispatchCommand] = None

        # 진행률(0.0 ~ 1.0)과 예상 완료 시간(초)
        self.progress: float = 0.0
        self.cycle_time_estimate: float = 0.0

        # KPI / 품질 / 헬스
        self.quality = QualityPrediction()
        self.health = HealthStatus()
        self.kpi_buffer: List[KPIRecord] = []

        # 발행 주기 관리
        self._last_status_publish_t: float = 0.0
        self._publish_period: float = 1.0 / max(config.status_publish_hz, 1e-3)

        # 명령 토픽 구독 (FDW-OPS → 자기 자신)
        self.bus.subscribe(Topics.DISPATCH_COMMAND, self._on_dispatch)

    # =========================================================================
    # 서브클래스가 구현해야 하는 핵심 메서드
    # =========================================================================
    @abstractmethod
    def configure(self) -> None:
        """Isaac Sim에서 자산(USD prim)을 스폰/로드하고 컨트롤러/에이전트를 초기화."""

    @abstractmethod
    def on_command(self, command: DispatchCommand) -> bool:
        """DispatchCommand를 해석하여 실제 작업을 시작한다.

        Returns:
            True  : 명령을 수락하고 PROCESSING 상태로 진입
            False : 명령을 거절 (현재 상태 유지)
        """

    @abstractmethod
    def step_processing(self, dt: float) -> None:
        """PROCESSING 상태일 때 호출되는 작업 진행 로직.

        self.progress 를 업데이트하고, 완료되면 self._complete_job() 호출.
        """

    # =========================================================================
    # 공통 lifecycle
    # =========================================================================
    def boot(self) -> None:
        """셀 부팅: OFFLINE -> IDLE."""
        self.configure()
        self.fsm.transition(CellState.IDLE)
        self.health = HealthStatus(status="normal")
        logger.info("[%s] booted (%s)", self.cell_id, self.cell_type.value)

    def step(self, dt: float, sim_time: float) -> None:
        """매 시뮬레이션 틱마다 호출."""
        state = self.fsm.state

        if state == CellState.PROCESSING:
            self.step_processing(dt)

        # 주기적 상태 발행
        if sim_time - self._last_status_publish_t >= self._publish_period:
            self.publish_status()
            self._last_status_publish_t = sim_time

    # =========================================================================
    # 작업 흐름 헬퍼
    # =========================================================================
    def receive_part(self, part_id: str) -> bool:
        """입력 버퍼에 부품을 적재 (이송 시스템이 호출)."""
        if self.input_buffer.occupied:
            return False
        self.input_buffer.occupied = True
        self.input_buffer.part_id = part_id
        if self.fsm.state == CellState.IDLE:
            self.fsm.transition(CellState.READY)
        logger.debug("[%s] received part %s", self.cell_id, part_id)
        return True

    def takeout_part(self) -> Optional[str]:
        """출력 버퍼에서 부품을 꺼냄 (이송 시스템이 호출)."""
        if not self.output_buffer.occupied:
            return None
        part_id = self.output_buffer.part_id
        self.output_buffer.occupied = False
        self.output_buffer.part_id = None
        # 출력 버퍼가 비고 작업이 끝났으면 IDLE 복귀
        if self.fsm.state == CellState.BLOCKED:
            self.fsm.transition(CellState.IDLE if not self.input_buffer.occupied else CellState.READY)
        logger.debug("[%s] released part %s", self.cell_id, part_id)
        return part_id

    def _complete_job(self) -> None:
        """작업 완료 처리: 부품을 출력 버퍼로 이동하고 KPI 기록."""
        part_id = self.input_buffer.part_id
        # 출력 버퍼로 이동
        self.input_buffer.occupied = False
        self.input_buffer.part_id = None
        self.output_buffer.occupied = True
        self.output_buffer.part_id = part_id

        # KPI 기록
        self.log_kpi("cycle_time", self.cycle_time_estimate, unit="sec")
        self.log_kpi("quality_score", self.quality.score)
        self.log_kpi("defect_risk", self.quality.defect_risk)

        # 다음 셀로 전달 가능한 상태인지 판단
        if self.output_buffer.occupied:
            self.fsm.transition(CellState.BLOCKED)  # AMR이 가져갈 때까지 대기
        else:
            self.fsm.transition(CellState.IDLE)

        logger.info("[%s] job %s completed (q=%.2f)",
                    self.cell_id, self.current_job.job_id if self.current_job else "?",
                    self.quality.score)
        self.current_job = None
        self.current_command = None
        self.progress = 0.0

    # =========================================================================
    # 메시징
    # =========================================================================
    def _on_dispatch(self, command: DispatchCommand) -> None:
        """버스 콜백: 자신을 타겟으로 하는 명령만 처리."""
        if command.target_cell != self.cell_id:
            return
        accepted = self.on_command(command)
        if accepted:
            self.current_command = command
            if self.fsm.state in (CellState.IDLE, CellState.READY):
                self.fsm.transition(CellState.READY) if self.fsm.state == CellState.IDLE else None
                self.fsm.transition(CellState.PROCESSING)
            logger.info("[%s] accepted command %s (%s)", self.cell_id, command.command_id, command.operation)
        else:
            logger.warning("[%s] rejected command %s", self.cell_id, command.command_id)

    def publish_status(self) -> None:
        msg = CellStatusMessage(
            cell_id=self.cell_id,
            cell_type=self.cell_type,
            state=self.fsm.state,
            current_job_id=self.current_job.job_id if self.current_job else None,
            input_buffer=self.input_buffer,
            output_buffer=self.output_buffer,
            estimated_remaining_time_sec=max(
                0.0, self.cycle_time_estimate * (1.0 - self.progress)
            ),
            quality_prediction=self.quality,
            health=self.health,
        )
        self.bus.publish(Topics.CELL_STATUS, msg)

    def log_kpi(self, metric: str, value: float, unit: str = "",
                tags: Optional[Dict[str, str]] = None) -> None:
        rec = KPIRecord(
            timestamp=time.time(),
            cell_id=self.cell_id,
            metric=metric,
            value=value,
            unit=unit,
            tags=tags or {},
        )
        self.kpi_buffer.append(rec)
        self.bus.publish(Topics.KPI_LOG, rec)

    # =========================================================================
    # 디버깅
    # =========================================================================
    def __repr__(self) -> str:
        return (f"<{self.__class__.__name__} id={self.cell_id} "
                f"type={self.cell_type.value} state={self.fsm.state.value}>")
