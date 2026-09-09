"""Visualization 모듈 import smoke test.

Isaac Sim이 없는 환경에서도 다음을 확인한다:
 1) fdw_sim.visualization 패키지가 import 가능한지
 2) WorkshopVisualizer 인스턴스화 자체는 USD import 없이 동작
    (build_scene 호출 시점에만 USD가 필요)
 3) discrete 모드는 시각화 코드가 추가돼도 영향 없음
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


def test_visualization_module_imports() -> None:
    """패키지 자체 import는 USD 없이도 가능해야 한다."""
    from fdw_sim.visualization import workshop_visualizer  # noqa: F401
    print("[OK] workshop_visualizer module imports without USD")


def test_workshop_visualizer_init_without_usd() -> None:
    """WorkshopVisualizer 인스턴스화는 lazy 이므로 USD 없이도 OK."""
    from fdw_sim.messaging.bus import InMemoryBus
    from fdw_sim.visualization.workshop_visualizer import (
        WorkshopVisualizer, WorkshopVizConfig,
    )
    bus = InMemoryBus()
    viz = WorkshopVisualizer(bus=bus, config=WorkshopVizConfig())
    viz.register_cell("MAT_01", cell_type="material", position=(0, 0, 0))
    viz.register_cell("WELD_01", cell_type="welding", position=(5, 0, 0))
    viz.register_cell("INSP_01", cell_type="inspection", position=(10, 0, 0))
    viz.register_amr("AMR_01")
    # build_scene은 USD가 없으면 실패하므로 호출하지 않음
    assert "MAT_01" in viz.cell_positions
    assert "WELD_01" in viz.cell_positions
    print("[OK] WorkshopVisualizer init without USD")


def test_discrete_still_runs() -> None:
    """시각화 모듈이 추가돼도 discrete 모드는 영향 없음."""
    from fdw_sim.cells.base.cell_base import CellConfig
    from fdw_sim.cells.inspection.inspection_cell import InspectionCell
    from fdw_sim.cells.material.material_cell import MaterialCell
    from fdw_sim.cells.welding.welding_cell import WeldingCell
    from fdw_sim.messaging.bus import InMemoryBus
    from fdw_sim.messaging.schemas import CellType, JobSpec
    from fdw_sim.orchestrator.orchestrator import OrchestratorConfig
    from fdw_sim.simulation.manager import (
        SimulationConfig, SimulationManager,
    )

    bus = InMemoryBus()
    sim = SimulationManager(
        SimulationConfig(mode="discrete", physics_hz=60.0,
                          max_sim_time_sec=120.0,
                          enable_visualization=True),
        bus=bus,
        orchestrator_config=OrchestratorConfig(
            decision_period_sec=0.2,
            material_cell_id="MAT_01",
            process_to_cell={"welding": "WELD_01",
                             "inspection": "INSP_01"},
        ),
    )

    mat = MaterialCell(
        config=CellConfig(cell_id="MAT_01", cell_type=CellType.MATERIAL,
                           input_capacity=8, output_capacity=8),
        bus=bus, num_amrs=2,
    )
    weld = WeldingCell(
        config=CellConfig(cell_id="WELD_01", cell_type=CellType.WELDING,
                           input_capacity=1, output_capacity=1),
        bus=bus, default_cycle_time=10.0,
    )
    insp = InspectionCell(
        config=CellConfig(cell_id="INSP_01", cell_type=CellType.INSPECTION,
                           input_capacity=1, output_capacity=1),
        bus=bus, default_cycle_time=4.0,
    )
    sim.register_material_cell(mat, location=(0, 0))
    sim.register_cell(weld, location=(5, 0))
    sim.register_cell(insp, location=(10, 0))

    sim.start()
    # Discrete 모드에서는 visualizer가 None이어야 함
    assert sim.visualizer is None, "visualizer must be None in discrete mode"

    sim.submit_job(JobSpec(
        job_id="JOB_VIZ_01", part_id="PART_VIZ_01",
        part_type="tubular_frame_A",
        process_route=["welding", "inspection"],
        recipe_overrides={"welding": {"speed": 0.10, "power": 1500.0,
                                       "path_length_mm": 200.0}},
    ))
    sim.run_until_done(all_jobs_done=True, extra_idle_sec=1.0)
    sim.stop()

    assert len(sim.orchestrator.completed_jobs) == 1, \
        f"expected 1 completed job, got {len(sim.orchestrator.completed_jobs)}"
    print("[OK] discrete mode unaffected by visualization module")


if __name__ == "__main__":
    test_visualization_module_imports()
    test_workshop_visualizer_init_without_usd()
    test_discrete_still_runs()
    print("\nAll visualization smoke tests passed!")
