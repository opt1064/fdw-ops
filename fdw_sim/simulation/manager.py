"""SimulationManager — 모든 컴포넌트(셀, 오케스트레이터, KPI Logger)를 묶어 step-loop 운영.

두 가지 백엔드를 지원한다:

    * "discrete"  : Isaac Sim 없이 순수 Python 루프로 실행 (Level 1 검증)
    * "isaac"     : Isaac Sim의 World/SimulationContext와 동기화 (Level 2+)

PoC-1은 "discrete" 모드로도 완전한 FDW-OPS 흐름이 검증된다.
Level 2 진입 시 동일 코드에서 mode="isaac"으로 전환하면 된다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional
import logging
import time

from fdw_sim.cells.base.cell_base import DistributedIntelligenceCell
from fdw_sim.cells.material.material_cell import (
    CellLocation,
    MaterialCell,
)
from fdw_sim.kpi.logger import KPILogger
from fdw_sim.messaging.bus import InMemoryBus, MessageBus
from fdw_sim.messaging.schemas import (
    CellState,
    JobSpec,
    KPIRecord,
)
from fdw_sim.orchestrator.orchestrator import (
    FDWOrchestrator,
    OrchestratorConfig,
)

logger = logging.getLogger(__name__)


# =============================================================================
# Config
# =============================================================================
@dataclass
class SimulationConfig:
    """SimulationManager 동작 파라미터."""
    mode: str = "discrete"            # "discrete" | "isaac"
    physics_hz: float = 60.0          # 시뮬레이션 물리 주기
    max_sim_time_sec: float = 300.0   # 최대 시뮬레이션 시간(초)
    realtime: bool = False            # True면 wall-clock 동기화 (디버깅 용)
    log_dir: Path = Path("./fdw_sim/logs")
    run_name: Optional[str] = None

    # Isaac Sim 모드 전용
    headless: bool = True
    stage_units_in_meters: float = 1.0
    isaac_app_kwargs: Dict[str, Any] = field(default_factory=dict)


# =============================================================================
# Manager
# =============================================================================
class SimulationManager:
    """전체 시뮬레이션의 라이프사이클 관리.

    사용 예 (discrete mode):

        bus = InMemoryBus()
        sim = SimulationManager(SimulationConfig(mode="discrete"), bus=bus)

        material = MaterialCell(...)
        welding = WeldingCell(...)
        inspection = InspectionCell(...)

        sim.register_material_cell(material, location=(0,0))
        sim.register_cell(welding, location=(5,0))
        sim.register_cell(inspection, location=(10,0))

        sim.start()
        sim.submit_job(JobSpec(...))
        sim.run_until_done()
        sim.stop()
    """

    def __init__(self, config: SimulationConfig,
                 bus: Optional[MessageBus] = None,
                 orchestrator_config: Optional[OrchestratorConfig] = None) -> None:
        self.config = config
        self.bus = bus or InMemoryBus()

        # 자체 컴포넌트
        self.kpi = KPILogger(log_dir=config.log_dir, run_name=config.run_name)
        self.kpi.attach(self.bus)

        self.orchestrator = FDWOrchestrator(self.bus, orchestrator_config)

        # 셀 레지스트리
        self.cells: Dict[str, DistributedIntelligenceCell] = {}
        self.material_cell: Optional[MaterialCell] = None

        # Isaac Sim 핸들 (mode == "isaac")
        self._isaac_app = None
        self._isaac_world = None

        # 상태
        self._sim_time: float = 0.0
        self._dt: float = 1.0 / max(config.physics_hz, 1.0)
        self._running: bool = False

        # 사용자 정의 step hook
        self._step_hooks: List[Callable[[float, float], None]] = []

    # =========================================================================
    # 셀 등록
    # =========================================================================
    def register_material_cell(self, cell: MaterialCell, location: tuple = (0.0, 0.0)) -> None:
        self.material_cell = cell
        self.cells[cell.cell_id] = cell
        cell.cell_locations[cell.cell_id] = CellLocation(cell.cell_id, location)
        self.orchestrator.register_cell(cell)
        logger.info("[SIM] material cell %s registered @ %s", cell.cell_id, location)

    def register_cell(self, cell: DistributedIntelligenceCell, location: tuple = (0.0, 0.0)) -> None:
        if cell.cell_id in self.cells:
            raise ValueError(f"cell_id duplicated: {cell.cell_id}")
        self.cells[cell.cell_id] = cell
        if self.material_cell is not None:
            self.material_cell.register_cell(cell, CellLocation(cell.cell_id, location))
        self.orchestrator.register_cell(cell)
        logger.info("[SIM] cell %s registered @ %s", cell.cell_id, location)

    def add_step_hook(self, hook: Callable[[float, float], None]) -> None:
        """매 step마다 호출될 함수를 등록 (signature: hook(dt, sim_time))."""
        self._step_hooks.append(hook)

    # =========================================================================
    # 라이프사이클
    # =========================================================================
    def start(self) -> None:
        if self._running:
            return

        if self.config.mode == "isaac":
            self._start_isaac()
        else:
            logger.info("[SIM] starting in DISCRETE mode (physics_hz=%.0f)", self.config.physics_hz)

        # 모든 셀 boot
        for cell in self.cells.values():
            cell.boot()

        self._running = True

    def _start_isaac(self) -> None:
        """Isaac Sim 5.x 백엔드 초기화."""
        # 1) SimulationApp 가장 먼저 생성 (Carbonite 요구사항)
        from isaacsim import SimulationApp  # type: ignore
        app_kwargs = {
            "headless": self.config.headless,
            **self.config.isaac_app_kwargs,
        }
        self._isaac_app = SimulationApp(app_kwargs)
        logger.info("[SIM] Isaac Sim launched (headless=%s)", self.config.headless)

        # 2) 그 다음에 World import / 생성
        from isaacsim.core.api import World  # type: ignore
        self._isaac_world = World(
            stage_units_in_meters=self.config.stage_units_in_meters,
            physics_dt=self._dt,
            rendering_dt=self._dt,
        )
        self._isaac_world.scene.add_default_ground_plane()
        self._isaac_world.reset()

    def stop(self) -> None:
        if not self._running:
            return
        self._running = False
        self.kpi.close()
        if self.config.mode == "isaac" and self._isaac_app is not None:
            self._isaac_app.close()
        self.bus.shutdown()
        logger.info("[SIM] stopped (sim_time=%.1fs)", self._sim_time)

    # =========================================================================
    # Step loop
    # =========================================================================
    def step(self) -> None:
        """단일 시뮬레이션 틱."""
        dt = self._dt

        # 1) Isaac Sim 모드: 물리 step
        if self.config.mode == "isaac" and self._isaac_world is not None:
            self._isaac_world.step(render=not self.config.headless)

        # 2) 모든 셀 step
        for cell in self.cells.values():
            cell.step(dt, self._sim_time)

        # 3) 오케스트레이터 step
        self.orchestrator.step(dt, self._sim_time)

        # 4) 사용자 hook
        for hook in self._step_hooks:
            try:
                hook(dt, self._sim_time)
            except Exception:
                logger.exception("step hook failed")

        self._sim_time += dt

    def run_until_done(self,
                       all_jobs_done: bool = True,
                       extra_idle_sec: float = 2.0) -> None:
        """모든 active job이 완료되거나 max_sim_time에 도달할 때까지 step.

        Args:
            all_jobs_done: True면 active_jobs가 비고 모든 셀이 IDLE이 될 때까지 진행
            extra_idle_sec: 모든 작업 완료 후 추가로 진행할 시간(상태 안정화)
        """
        idle_since: Optional[float] = None
        wall_start = time.time()

        while self._running and self._sim_time < self.config.max_sim_time_sec:
            self.step()

            if all_jobs_done:
                if not self.orchestrator.active_jobs and self._all_cells_quiescent():
                    if idle_since is None:
                        idle_since = self._sim_time
                    elif self._sim_time - idle_since >= extra_idle_sec:
                        break
                else:
                    idle_since = None

            if self.config.realtime:
                target_wall = wall_start + self._sim_time
                lag = target_wall - time.time()
                if lag > 0:
                    time.sleep(lag)

        logger.info("[SIM] run finished (sim_time=%.1fs, completed_jobs=%d)",
                    self._sim_time, len(self.orchestrator.completed_jobs))

    def _all_cells_quiescent(self) -> bool:
        """모든 일반 셀이 IDLE 상태이고, MaterialCell의 transfer_queue/AMR이 비어있는지."""
        for cell in self.cells.values():
            if isinstance(cell, MaterialCell):
                if cell.transfer_queue:
                    return False
                if any(amr.busy for amr in cell.amrs):
                    return False
            else:
                if cell.fsm.state not in (CellState.IDLE,):
                    return False
                if cell.input_buffer.occupied or cell.output_buffer.occupied:
                    return False
        return True

    # =========================================================================
    # Job 투입 헬퍼
    # =========================================================================
    def submit_job(self, job: JobSpec) -> None:
        if self.material_cell is None:
            raise RuntimeError("material cell not registered")
        # 부품을 smart rack에 입고
        self.material_cell.stock_part(job.part_id, job.part_type)
        # 오케스트레이터에 Job 등록
        self.orchestrator.submit_job(job)

    # =========================================================================
    # 디버깅
    # =========================================================================
    @property
    def sim_time(self) -> float:
        return self._sim_time
