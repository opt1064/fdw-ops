"""Level 2.4 — AMR 실주행 + pick-and-place 시각화 회귀 테스트.

Isaac Sim 없이(discrete 로직 + WorkshopVisualizer 순수 기하 계산만) 확인:
 1) MaterialCell.AMR이 이송 중 total_distance_m/from_cell을 올바르게
    기록하고, 배송 완료 시 last_delivered_* 스냅샷을 남기는지
    (WorkshopVisualizer가 이걸로 "방금 뭘 배송했는지" edge-detect 한다 —
    자세한 이유는 material_cell.py의 AMR 데이터클래스 주석 참고)
 2) WorkshopVisualizer._amr_dock_pos / _amr_current_world_pos의 선형보간이
    기대한 좌표를 내는지
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


def _make_material_cell():
    from fdw_sim.cells.base.cell_base import CellConfig
    from fdw_sim.cells.material.material_cell import CellLocation, MaterialCell
    from fdw_sim.messaging.bus import InMemoryBus
    from fdw_sim.messaging.schemas import CellType

    bus = InMemoryBus()
    mat = MaterialCell(
        CellConfig(cell_id="MATERIAL_CELL_01", cell_type=CellType.MATERIAL),
        bus=bus, num_amrs=1,
    )
    mat.cell_locations["MATERIAL_CELL_01"] = CellLocation("MATERIAL_CELL_01", (0.0, 0.0))
    mat.cell_locations["WELDING_CELL_01"] = CellLocation("WELDING_CELL_01", (10.0, 0.0))
    return mat, bus


def test_amr_records_total_distance_and_from_cell() -> None:
    from fdw_sim.messaging.schemas import MaterialTransferCommand

    mat, _bus = _make_material_cell()
    mat.stock_part("PART_X", "tubular_frame_A")

    cmd = MaterialTransferCommand(
        from_cell="MATERIAL_CELL_01", to_cell="WELDING_CELL_01",
        part_id="PART_X", carrier="AMR_01",
    )
    mat._on_transfer_command(cmd)
    mat._dispatch_amrs()

    amr = mat.amrs[0]
    assert amr.busy is True
    assert amr.payload_part_id == "PART_X"
    assert amr.from_cell == "MATERIAL_CELL_01"
    assert amr.target_cell == "WELDING_CELL_01"
    assert amr.total_distance_m == 10.0
    assert amr.remaining_distance_m == 10.0
    print("[OK] AMR assignment records total_distance_m/from_cell")


def test_amr_delivery_leaves_snapshot_for_visualizer() -> None:
    from fdw_sim.messaging.schemas import MaterialTransferCommand

    mat, _bus = _make_material_cell()
    mat.stock_part("PART_X", "tubular_frame_A")

    class _Sink:
        """receive_part을 항상 성공시키는 더미 타겟 셀."""
        cell_id = "WELDING_CELL_01"

        def receive_part(self, part_id):
            return True

    mat.cell_registry["WELDING_CELL_01"] = _Sink()

    cmd = MaterialTransferCommand(
        from_cell="MATERIAL_CELL_01", to_cell="WELDING_CELL_01",
        part_id="PART_X", carrier="AMR_01",
    )
    mat._on_transfer_command(cmd)
    mat._dispatch_amrs()

    amr = mat.amrs[0]
    # 10m @ 1.0 m/s -> 10초 남았다고 가정하고 한 번에 다 소진
    mat._step_amr(amr, dt=11.0)

    assert amr.busy is False
    assert amr.payload_part_id is None
    assert amr.last_delivered_part_id == "PART_X"
    assert amr.last_delivered_to == "WELDING_CELL_01"
    assert amr.last_delivered_command_id == cmd.command_id
    print("[OK] AMR delivery leaves last_delivered_* snapshot")


def _make_visualizer():
    from fdw_sim.messaging.bus import InMemoryBus
    from fdw_sim.visualization.workshop_visualizer import (
        WorkshopVisualizer, WorkshopVizConfig,
    )
    bus = InMemoryBus()
    viz = WorkshopVisualizer(
        bus=bus,
        config=WorkshopVizConfig(amr_dock_offset=(0.0, -1.0)),
    )
    viz.register_cell("MATERIAL_CELL_01", cell_type="material", position=(0.0, 0.0, 0.0))
    viz.register_cell("WELDING_CELL_01", cell_type="welding", position=(10.0, 0.0, 0.0))
    return viz


def test_amr_dock_pos_applies_offset() -> None:
    viz = _make_visualizer()
    dock = viz._amr_dock_pos("WELDING_CELL_01")
    assert dock == (10.0, -1.0)
    print("[OK] _amr_dock_pos applies configured offset")


def test_amr_current_world_pos_interpolates_linearly() -> None:
    from fdw_sim.cells.material.material_cell import AMR

    viz = _make_visualizer()
    amr = AMR(amr_id="AMR_01", from_cell="MATERIAL_CELL_01",
              target_cell="WELDING_CELL_01",
              total_distance_m=10.0, remaining_distance_m=10.0)

    # 진행률 0% — 출발지 도킹 지점
    pos0 = viz._amr_current_world_pos(amr)
    assert pos0 == (0.0, -1.0, 0.0)

    # 진행률 50%
    amr.remaining_distance_m = 5.0
    pos_half = viz._amr_current_world_pos(amr)
    assert pos_half == (5.0, -1.0, 0.0)

    # 진행률 100% — 도착지 도킹 지점
    amr.remaining_distance_m = 0.0
    pos1 = viz._amr_current_world_pos(amr)
    assert pos1 == (10.0, -1.0, 0.0)
    print("[OK] _amr_current_world_pos interpolates from/to dock linearly")


def test_amr_current_world_pos_none_when_cells_unset() -> None:
    from fdw_sim.cells.material.material_cell import AMR

    viz = _make_visualizer()
    idle_amr = AMR(amr_id="AMR_01")  # from_cell/target_cell 둘 다 None
    assert viz._amr_current_world_pos(idle_amr) is None
    print("[OK] _amr_current_world_pos returns None for an unassigned AMR")


if __name__ == "__main__":
    test_amr_records_total_distance_and_from_cell()
    test_amr_delivery_leaves_snapshot_for_visualizer()
    test_amr_dock_pos_applies_offset()
    test_amr_current_world_pos_interpolates_linearly()
    test_amr_current_world_pos_none_when_cells_unset()
    print("\nAll Level 2.4 AMR pick-and-place tests passed!")
