"""FDW-OPS Orchestrator (Rule-based, PoC-1).

역할:
    - JobSpec(작업 명세)을 받아 process_route를 따라 셀 간 dispatch + transfer 명령 생성
    - 모든 셀의 CellStatusMessage를 구독하여 라우팅 의사결정
    - 병목/이상 발생 시 단순 복구 시도 (재시도, 우회 라우팅 — 후속 PoC에서 확장)

이 단계는 RL이나 MILP 없이 룰 기반으로만 동작한다.
이후 PoC-2/3에서 동일 인터페이스를 유지한 채 AI 라우팅 모듈로 교체 가능하다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional
import logging
import time

from fdw_sim.messaging.bus import MessageBus, Topics
from fdw_sim.messaging.schemas import (
    CellState,
    CellStatusMessage,
    CellType,
    DispatchCommand,
    JobSpec,
    KPIRecord,
    MaterialTransferCommand,
)

logger = logging.getLogger(__name__)


# =============================================================================
# Config
# =============================================================================
@dataclass
class OrchestratorConfig:
    """오케스트레이터 동작 파라미터."""
    decision_period_sec: float = 0.2          # 1~5 Hz
    material_cell_id: str = "MATERIAL_CELL_01"
    # PoC safety policy: FAIL, REWORK, or missing verdict -> logical quarantine.
    # Automatic rework is deliberately unsupported (zero automatic retries).
    inspection_nonpass_disposition: str = "quarantine"
    # process_type -> 기본 cell_id 매핑 (라우팅 테이블)
    process_to_cell: Dict[str, str] = field(default_factory=lambda: {
        "welding":    "WELDING_CELL_01",
        "inspection": "INSPECTION_CELL_01",
        "forming":    "FORMING_CELL_01",
        "waam":       "WAAM_CELL_01",
        "machining":  "MACHINING_CELL_01",
    })
    # process_type -> 기본 operation 이름
    process_to_operation: Dict[str, str] = field(default_factory=lambda: {
        "welding":    "START_WELDING",
        "inspection": "START_INSPECTION",
        "forming":    "START_FORMING",
        "waam":       "START_DEPOSITION",
        "machining":  "START_MACHINING",
    })


    def __post_init__(self) -> None:
        if self.inspection_nonpass_disposition != "quarantine":
            raise ValueError("inspection_nonpass_disposition must be quarantine; automatic rework is unsupported")


# =============================================================================
# Internal Job Tracker
# =============================================================================
@dataclass
class JobTracker:
    """단일 Job의 진행 상황 추적."""
    job: JobSpec
    route_index: int = 0
    started_at: float = 0.0                        # sim_time 기준 (wall-clock 아님)
    completed_at: Optional[float] = None           # sim_time 기준
    last_step_completed: Optional[str] = None      # 마지막 완료된 process_type
    current_location: Optional[str] = None         # 현재 부품이 위치한 cell_id
    pending_dispatch: bool = False
    pending_transfer: bool = False
    history: List[str] = field(default_factory=list)
    disposition: Optional[str] = None
    disposition_reason: Optional[str] = None

    @property
    def is_done(self) -> bool:
        return self.route_index >= len(self.job.process_route)

    @property
    def next_process(self) -> Optional[str]:
        if self.is_done:
            return None
        return self.job.process_route[self.route_index]


# =============================================================================
# Orchestrator
# =============================================================================
class FDWOrchestrator:
    """룰 기반 FDW-OPS 오케스트레이터.

    동작 순서:
        1) Job 등록 -> Material cell의 smart rack에서 첫 process cell로 이송 명령
        2) 각 셀의 상태 메시지를 받아 cell_state_cache에 저장
        3) decision_period마다 모든 활성 Job의 다음 단계 추진
            - 부품이 셀 input_buffer에 도착했으면 DispatchCommand 발행
            - 셀이 작업을 끝내고 BLOCKED(또는 output_buffer.occupied)이면 다음 셀로 이송 명령
        4) 모든 process_route를 끝낸 Job은 종료 처리
    """

    def __init__(self, bus: MessageBus, config: Optional[OrchestratorConfig] = None) -> None:
        self.bus = bus
        self.config = config or OrchestratorConfig()

        # 셀 상태 캐시 (cell_id -> 최신 CellStatusMessage)
        self.cell_state_cache: Dict[str, CellStatusMessage] = {}

        # 활성 Job 추적
        self.active_jobs: Dict[str, JobTracker] = {}
        self.completed_jobs: List[JobTracker] = []
        self.quarantined_jobs: List[JobTracker] = []
        self.quarantined_parts: Dict[str, Dict[str, Any]] = {}

        # 시뮬레이션 시간
        self._sim_time: float = 0.0
        self._last_decision_t: float = 0.0

        # 발행한 transfer/dispatch 중복 방지
        self._dispatched_jobs_at_cell: Dict[str, str] = {}  # cell_id -> job_id
        self._transferred_keys: set = set()                 # (job_id, route_index, from, to)

        # 셀 직접 참조 (final takeout, 디버깅 용도)
        # 시뮬레이션 매니저가 register_cell()을 호출할 때 채워준다.
        self.cell_registry: Dict[str, Any] = {}

        # 구독
        self.bus.subscribe(Topics.CELL_STATUS, self._on_cell_status)
        self.bus.subscribe(Topics.JOB_CREATED, self._on_job_created)

    def register_cell(self, cell: Any) -> None:
        """셀 인스턴스 등록 (final takeout 등 직접 호출 용도)."""
        self.cell_registry[cell.cell_id] = cell

    # =========================================================================
    # 외부 API
    # =========================================================================
    def submit_job(self, job: JobSpec) -> None:
        """새로운 Job을 시스템에 투입. 부품은 이미 Material cell smart_rack에 있어야 한다."""
        if job.job_id in self.active_jobs:
            logger.warning("Job %s already active, ignored", job.job_id)
            return
        tracker = JobTracker(
            job=job,
            current_location=self.config.material_cell_id,
            started_at=self._sim_time,
        )
        self.active_jobs[job.job_id] = tracker
        logger.info("[ORCH] submitted job %s route=%s", job.job_id, job.process_route)
        # 즉시 첫 이송 명령 발행
        self._issue_next_transfer(tracker)

    def step(self, dt: float, sim_time: float) -> None:
        """시뮬레이션 매니저가 매 틱 호출."""
        self._sim_time = sim_time
        if sim_time - self._last_decision_t < self.config.decision_period_sec:
            return
        self._last_decision_t = sim_time
        self._tick_decisions()

    # =========================================================================
    # 콜백
    # =========================================================================
    def _on_cell_status(self, msg: CellStatusMessage) -> None:
        self.cell_state_cache[msg.cell_id] = msg

    def _on_job_created(self, job: JobSpec) -> None:
        self.submit_job(job)

    # =========================================================================
    # 의사결정 루프
    # =========================================================================
    def _tick_decisions(self) -> None:
        finished: List[str] = []

        for job_id, tracker in self.active_jobs.items():
            if tracker.is_done:
                tracker.completed_at = self._sim_time
                tracker.disposition = "shipped"
                self.completed_jobs.append(tracker)
                finished.append(job_id)
                self._publish_job_completed(tracker)
                continue

            current_proc = tracker.next_process
            target_cell_id = self.config.process_to_cell.get(current_proc)
            if target_cell_id is None:
                logger.error("[ORCH] unknown process '%s' in job %s", current_proc, job_id)
                finished.append(job_id)
                continue

            target_state = self.cell_state_cache.get(target_cell_id)

            # 1) 부품이 target cell의 input_buffer에 도착했으면 dispatch
            if target_state and target_state.input_buffer.occupied \
                    and target_state.input_buffer.part_id == tracker.job.part_id \
                    and target_state.state in (CellState.READY, CellState.IDLE):
                if self._dispatched_jobs_at_cell.get(target_cell_id) != job_id:
                    self._issue_dispatch(tracker, target_cell_id, current_proc)
                    self._dispatched_jobs_at_cell[target_cell_id] = job_id
                continue

            # 2) target cell이 작업을 끝내고 output_buffer에 부품 있으면 → 다음 단계로 이송
            if target_state and target_state.output_buffer.occupied \
                    and target_state.output_buffer.part_id == tracker.job.part_id:
                tracker.last_step_completed = current_proc
                tracker.history.append(current_proc)
                tracker.current_location = target_cell_id
                self._dispatched_jobs_at_cell.pop(target_cell_id, None)

                if current_proc == "inspection" or target_state.cell_type == CellType.INSPECTION:
                    verdict = target_state.inspection_verdict
                    if target_state.verdict_part_id != tracker.job.part_id:
                        verdict = None
                    if verdict != "PASS":
                        reason = target_state.verdict_reason or verdict or "missing_or_mismatched_verdict"
                        if self._quarantine(tracker, target_cell_id, reason):
                            finished.append(job_id)
                        else:
                            tracker.history.pop()
                        continue

                # Advance only after disposition succeeded; never mark an unremoved part shipped.
                if tracker.route_index + 1 == len(tracker.job.process_route):
                    if not self._final_takeout(target_cell_id, tracker.job.part_id):
                        tracker.history.pop()
                        continue
                tracker.route_index += 1
                if not tracker.is_done:
                    self._issue_next_transfer(tracker)

        for jid in finished:
            self.active_jobs.pop(jid, None)

    # =========================================================================
    # 명령 발행 헬퍼
    # =========================================================================
    def _issue_next_transfer(self, tracker: JobTracker) -> None:
        if tracker.is_done:
            return

        next_proc = tracker.next_process
        target_cell = self.config.process_to_cell.get(next_proc)
        if target_cell is None:
            return

        from_cell = tracker.current_location or self.config.material_cell_id

        key = (tracker.job.job_id, tracker.route_index, from_cell, target_cell)
        if key in self._transferred_keys:
            return
        self._transferred_keys.add(key)

        cmd = MaterialTransferCommand(
            from_cell=from_cell,
            to_cell=target_cell,
            part_id=tracker.job.part_id,
            carrier="AUTO",
            priority=tracker.job.priority,
        )
        self.bus.publish(Topics.MATERIAL_TRANSFER, cmd)
        logger.info("[ORCH] transfer issued: %s %s -> %s (job=%s)",
                    cmd.command_id, from_cell, target_cell, tracker.job.job_id)

    def _issue_dispatch(self, tracker: JobTracker, cell_id: str, process: str) -> None:
        operation = self.config.process_to_operation.get(process, "START")
        recipe = tracker.job.recipe_overrides.get(process, {})
        cmd = DispatchCommand(
            target_cell=cell_id,
            job_id=tracker.job.job_id,
            operation=operation,
            recipe=recipe,
            next_cell=self._lookup_next_cell(tracker),
            priority=tracker.job.priority,
        )
        self.bus.publish(Topics.DISPATCH_COMMAND, cmd)
        logger.info("[ORCH] dispatch issued: %s -> %s op=%s (job=%s)",
                    cmd.command_id, cell_id, operation, tracker.job.job_id)

    def _lookup_next_cell(self, tracker: JobTracker) -> Optional[str]:
        next_idx = tracker.route_index + 1
        if next_idx >= len(tracker.job.process_route):
            return None
        return self.config.process_to_cell.get(tracker.job.process_route[next_idx])

    def _remove_output_part(self, cell_id: str, part_id: str) -> bool:
        cell = self.cell_registry.get(cell_id)
        if cell is None or not cell.output_buffer.occupied or cell.output_buffer.part_id != part_id:
            return False
        return cell.takeout_part() == part_id

    def _quarantine(self, tracker: JobTracker, cell_id: str, reason: str) -> bool:
        """Logical quarantine inventory, not shipment or physical quarantine motion."""
        if not self._remove_output_part(cell_id, tracker.job.part_id):
            return False
        tracker.disposition = "quarantined"
        tracker.disposition_reason = reason
        tracker.completed_at = self._sim_time
        record = {"job_id": tracker.job.job_id, "part_id": tracker.job.part_id,
                  "source_cell": cell_id, "reason": reason, "sim_time": self._sim_time,
                  "history": list(tracker.history), "disposition": "quarantined"}
        self.quarantined_parts[tracker.job.part_id] = record
        self.quarantined_jobs.append(tracker)
        self.bus.publish(Topics.JOB_QUARANTINED, record)
        self.bus.publish(Topics.KPI_LOG, KPIRecord(
            timestamp=time.time(), cell_id=cell_id, metric="job_quarantined_count",
            value=1.0, unit="parts", tags={"job_id": tracker.job.job_id, "reason": reason}))
        logger.warning("[ORCH] quarantined part %s: %s", tracker.job.part_id, reason)
        return True

    def _final_takeout(self, cell_id: str, part_id: str) -> bool:
        """마지막 단계 완료 후 셀의 output_buffer에서 부품을 출하 처리.

        실제 환경이라면 출하 컨베이어/AMR이 가져가는 단계에 해당.
        PoC-1에서는 직접 호출하여 셀이 다시 IDLE이 되도록 한다.
        """
        if not self._remove_output_part(cell_id, part_id):
            logger.warning("[ORCH] final takeout blocked: %s part %s", cell_id, part_id)
            return False
        logger.info("[ORCH] final takeout from %s: part %s shipped", cell_id, part_id)
        return True

    def _publish_job_completed(self, tracker: JobTracker) -> None:
        self.bus.publish(Topics.JOB_COMPLETED, {
            "job_id": tracker.job.job_id,
            "part_id": tracker.job.part_id,
            "started_at": tracker.started_at,
            "completed_at": tracker.completed_at,
            "history": tracker.history,
            "disposition": tracker.disposition,
        })
        # KPI: makespan
        rec = KPIRecord(
            timestamp=time.time(),
            cell_id=None,
            metric="job_makespan_sec",
            value=(tracker.completed_at or self._sim_time) - tracker.started_at,
            unit="sec",
            tags={"job_id": tracker.job.job_id},
        )
        self.bus.publish(Topics.KPI_LOG, rec)
        logger.info("[ORCH] job %s completed in %.1fs (route=%s)",
                    tracker.job.job_id,
                    (tracker.completed_at or self._sim_time) - tracker.started_at,
                    tracker.history)
