"""PoC-1 스모크 테스트.

Isaac Sim 없이 discrete 모드로 전체 흐름이 동작하는지 검증한다.
이 테스트가 통과한다는 것은:
    - 메시지 스키마가 일관적이고
    - 셀 상태머신이 잘 동작하며
    - 오케스트레이터의 라우팅 룰이 작업을 끝까지 진행시킨다
는 의미이다.
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from fdw_sim.cells.base.cell_base import CellConfig
from fdw_sim.cells.inspection.inspection_cell import InspectionCell
from fdw_sim.cells.material.material_cell import MaterialCell
from fdw_sim.cells.welding.welding_cell import WeldingCell
from fdw_sim.messaging.bus import InMemoryBus
from fdw_sim.messaging.schemas import CellType, JobSpec
from fdw_sim.orchestrator.orchestrator import OrchestratorConfig
from fdw_sim.simulation.manager import SimulationConfig, SimulationManager


def build_minimal_sim() -> SimulationManager:
    bus = InMemoryBus()
    sim_cfg = SimulationConfig(mode="discrete", physics_hz=60.0,
                               max_sim_time_sec=300.0,
                               log_dir=Path("/tmp/fdw_sim_test_logs"))
    orch_cfg = OrchestratorConfig(
        material_cell_id="MAT_01",
        process_to_cell={"welding": "WELD_01", "inspection": "INSP_01"},
    )
    sim = SimulationManager(sim_cfg, bus=bus, orchestrator_config=orch_cfg)

    material = MaterialCell(
        CellConfig(cell_id="MAT_01", cell_type=CellType.MATERIAL,
                   input_capacity=8, output_capacity=8),
        bus=bus, num_amrs=2,
    )
    welding = WeldingCell(
        CellConfig(cell_id="WELD_01", cell_type=CellType.WELDING),
        bus=bus, default_cycle_time=5.0,
    )
    inspection = InspectionCell(
        CellConfig(cell_id="INSP_01", cell_type=CellType.INSPECTION),
        bus=bus, default_cycle_time=2.0,
    )

    sim.register_material_cell(material, location=(0.0, 0.0))
    sim.register_cell(welding, location=(2.0, 0.0))
    sim.register_cell(inspection, location=(4.0, 0.0))
    return sim


def test_single_job_completes():
    sim = build_minimal_sim()
    sim.start()
    try:
        sim.submit_job(JobSpec(
            job_id="J1", part_id="P1", part_type="tubular_frame_A",
            process_route=["welding", "inspection"],
            recipe_overrides={"welding": {"speed": 0.1, "power": 1500.0,
                                          "path_length_mm": 200.0}},
        ))
        sim.run_until_done(all_jobs_done=True, extra_idle_sec=1.0)
    finally:
        sim.stop()

    assert len(sim.orchestrator.completed_jobs) == 1
    tr = sim.orchestrator.completed_jobs[0]
    assert tr.job.job_id == "J1"
    assert tr.history == ["welding", "inspection"]


def test_multi_jobs_complete():
    sim = build_minimal_sim()
    sim.start()
    try:
        for i in range(3):
            sim.submit_job(JobSpec(
                job_id=f"J{i}", part_id=f"P{i}", part_type="tubular_frame_A",
                process_route=["welding", "inspection"],
                recipe_overrides={"welding": {"speed": 0.1, "power": 1500.0,
                                              "path_length_mm": 200.0}},
            ))
        sim.run_until_done(all_jobs_done=True, extra_idle_sec=1.0)
    finally:
        sim.stop()

    assert len(sim.orchestrator.completed_jobs) == 3
    assert all(tr.history == ["welding", "inspection"]
               for tr in sim.orchestrator.completed_jobs)


if __name__ == "__main__":
    test_single_job_completes()
    print("[OK] test_single_job_completes")
    test_multi_jobs_complete()
    print("[OK] test_multi_jobs_complete")
    print("\nAll smoke tests passed!")
