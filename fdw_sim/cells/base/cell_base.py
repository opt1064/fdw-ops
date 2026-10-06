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
from dataclasses import dataclass, field, replace
from typing import Any, Dict, List, Optional, Tuple
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

        # Logical mode needs no robot acknowledgements. A visualizer with an
        # intended robot explicitly enables motion mode before receiving work.
        self._motion_required: bool = False
        self._placement_key: Optional[Tuple[str, str]] = None
        self._placement_ready: bool = True
        self._placement_failed: bool = False
        self._process_motion_key: Optional[Tuple[str, str]] = None
        self._process_motion_complete: bool = False
        self._process_recipe_complete: bool = False
        self._process_elapsed: float = 0.0
        self.max_process_motion_wait_sec: float = 120.0
        self._accepted_command_ids: set[str] = set()

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
            self._process_elapsed += max(0.0, dt)
            if self._process_recipe_complete:
                self._complete_job()
            else:
                self.step_processing(dt)
            if (self.fsm.state == CellState.PROCESSING and self._motion_required
                    and not self._process_motion_complete
                    and self._process_elapsed >= self.cycle_time_estimate
                    + self.max_process_motion_wait_sec):
                self._motion_fault("process_motion_timeout", "Process motion completion was not confirmed")

        # 주기적 상태 발행
        if sim_time - self._last_status_publish_t >= self._publish_period:
            self.publish_status()
            self._last_status_publish_t = sim_time

    # =========================================================================
    # 작업 흐름 헬퍼
    # =========================================================================
    @property
    def motion_required(self) -> bool:
        return self._motion_required

    @property
    def placement_ready(self) -> bool:
        return self._placement_ready and not self._placement_failed

    def set_motion_required(self, required: bool) -> None:
        """Choose logical versus acknowledged motion before admitting a part.

        Never switch an in-flight robot operation to logical success after an
        error. Recovery must resolve custody and restart the affected work.
        """
        if bool(required) == self._motion_required:
            return
        if self.current_command is not None or self.input_buffer.occupied or self.output_buffer.occupied:
            raise RuntimeError("Cannot change motion mode while a part is in the cell")
        self._motion_required = bool(required)

    def is_part_ready(self, part_id: Optional[str]) -> bool:
        """Readiness is both logical ownership and successful placement."""
        return bool(part_id and self.input_buffer.occupied
                    and self.input_buffer.part_id == part_id
                    and self.placement_ready
                    and self.fsm.state in (CellState.IDLE, CellState.READY))

    def mark_placement_pending(self, part_id: str, transfer_command_id: str) -> bool:
        """Bind a just-received input to its AMR transfer before publishing it.

        The input is reserved while the AMR/robot still has physical custody.
        An old transfer cannot replace a live placement request.
        """
        if (not part_id or not transfer_command_id or self._placement_key is not None
                or self._placement_failed or not self.input_buffer.occupied
                or self.input_buffer.part_id != part_id
                or self.fsm.state not in (CellState.IDLE, CellState.READY, CellState.BLOCKED)):
            return False
        self._placement_key = (part_id, transfer_command_id)
        self._placement_ready = False
        if self.fsm.state == CellState.READY:
            self.fsm.transition(CellState.IDLE)
        return True

    def confirm_placement(self, part_id: str, transfer_command_id: str,
                          success: bool = True, *, reason: str = "") -> bool:
        """Accept exactly one result for the live part and transfer.

        A failure is terminal for this placement; a late success cannot turn a
        timeout or controller failure into successful physical custody.
        """
        if (self._placement_key != (part_id, transfer_command_id)
                or self._placement_ready or self._placement_failed
                or not self.input_buffer.occupied or self.input_buffer.part_id != part_id
                or self.fsm.state not in (CellState.IDLE, CellState.READY, CellState.BLOCKED)):
            return False
        if not success:
            self._placement_failed = True
            self._motion_fault("placement_failed", reason or "Part placement was not confirmed")
            return True
        self._placement_ready = True
        if self.fsm.state == CellState.IDLE:
            self.fsm.transition(CellState.READY)
        return True

    def confirm_process_motion(self, command_id: str, part_id: str,
                               success: bool = True, *, reason: str = "") -> bool:
        """Accept a controller result only for the live dispatch and its part."""
        if (not self._motion_required or self.fsm.state != CellState.PROCESSING
                or self._process_motion_key != (command_id, part_id)
                or self._process_motion_complete or not self.input_buffer.occupied
                or self.input_buffer.part_id != part_id):
            return False
        if not success:
            self._motion_fault("process_motion_failed", reason or "Process motion failed")
            return True
        self._process_motion_complete = True
        if self._process_recipe_complete:
            self._complete_job()
        return True

    def _motion_fault(self, code: str, message: str) -> None:
        self.health = HealthStatus(status="critical", alarm_code=code, message=message)
        if self.fsm.can_transition(CellState.FAULT):
            self.fsm.transition(CellState.FAULT)
        logger.error("[%s] %s: %s", self.cell_id, code, message)

    def receive_part(self, part_id: str) -> bool:
        """입력 버퍼에 부품을 적재 (이송 시스템이 호출)."""
        if (not part_id or self.input_buffer.occupied
                or self.fsm.state not in (CellState.IDLE, CellState.READY, CellState.BLOCKED)):
            return False
        self.input_buffer.occupied = True
        self.input_buffer.part_id = part_id
        self._placement_key = None
        self._placement_ready = not self._motion_required
        self._placement_failed = False
        if self.fsm.state == CellState.IDLE and self.placement_ready:
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
            self.fsm.transition(CellState.READY if self.input_buffer.occupied and self.placement_ready
                                else CellState.IDLE)
        logger.debug("[%s] released part %s", self.cell_id, part_id)
        return part_id

    def _complete_job(self) -> None:
        """작업 완료 처리: 부품을 출력 버퍼로 이동하고 KPI 기록."""
        if self.fsm.state != CellState.PROCESSING or not self.input_buffer.occupied:
            return
        self._process_recipe_complete = True
        self.progress = 1.0
        if self._motion_required and not self._process_motion_complete:
            return
        if self.output_buffer.occupied:
            return
        part_id = self.input_buffer.part_id
        # 출력 버퍼로 이동
        self.input_buffer.occupied = False
        self.input_buffer.part_id = None
        self._placement_key = None
        self._placement_ready = True
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
        self._process_motion_key = None
        self._process_motion_complete = False
        self._process_recipe_complete = False
        self.progress = 0.0

    # =========================================================================
    # 메시징
    # =========================================================================
    def _on_dispatch(self, command: DispatchCommand) -> None:
        """버스 콜백: 자신을 타겟으로 하는 명령만 처리."""
        if command.target_cell != self.cell_id:
            return
        if command.command_id in self._accepted_command_ids:
            return
        starts_process = command.operation.startswith("START")
        if starts_process and (not self.is_part_ready(self.input_buffer.part_id)
                               or self.output_buffer.occupied
                               or (self.motion_required and not command.part_id)
                               or (command.part_id is not None
                                   and command.part_id != self.input_buffer.part_id)):
            logger.warning("[%s] rejected unready/mismatched command %s", self.cell_id, command.command_id)
            return
        if self.fsm.state in (CellState.PROCESSING, CellState.FAULT):
            logger.warning("[%s] rejected command %s while %s", self.cell_id, command.command_id, self.fsm.state.value)
            return
        accepted = self.on_command(command)
        if accepted:
            self._accepted_command_ids.add(command.command_id)
            if starts_process:
                self.current_command = command
                self._process_motion_key = (command.command_id, self.input_buffer.part_id)
                self._process_motion_complete = False
                self._process_recipe_complete = False
                self._process_elapsed = 0.0
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
            current_job_id=(self.current_job.job_id if self.current_job else
                            self.current_command.job_id if self.current_command else None),
            input_buffer=replace(self.input_buffer),
            output_buffer=replace(self.output_buffer),
            estimated_remaining_time_sec=max(
                0.0, self.cycle_time_estimate * (1.0 - self.progress)
            ),
            quality_prediction=(replace(self.quality) if isinstance(self.quality, QualityPrediction)
                                else QualityPrediction(score=0.0, defect_risk=1.0, confidence=0.0)),
            health=replace(self.health),
            inspection_verdict=getattr(self, "last_verdict", None),
            verdict_part_id=getattr(self, "verdict_part_id", None),
            verdict_reason=getattr(self, "last_verdict_reason", None),
            placement_ready=self.placement_ready,
            placement_transfer_command_id=self._placement_key[1] if self._placement_key else None,
            current_command_id=self.current_command.command_id if self.current_command else None,
            current_part_id=self.input_buffer.part_id if self.current_command else None,
            motion_required=self.motion_required,
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
