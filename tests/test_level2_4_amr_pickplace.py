"""Level 2.4 — AMR 실주행 + pick-and-place 시각화 회귀 테스트.

Isaac Sim 없이(discrete 로직 + WorkshopVisualizer 순수 기하 계산만) 확인:
 1) MaterialCell.AMR이 이송 중 total_distance_m/from_cell을 올바르게
    기록하고, 배송 완료 시 last_delivered_* 스냅샷을 남기는지
    (WorkshopVisualizer가 이걸로 "방금 뭘 배송했는지" edge-detect 한다 —
    자세한 이유는 material_cell.py의 AMR 데이터클래스 주석 참고)
 2) WorkshopVisualizer가 MaterialCell의 권위 있는 실제 좌표를 렌더링하는지
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


def _advance_until(mat, predicate):
    for _ in range(2000):
        if predicate():
            return
        amr = mat.amrs[0]
        if amr.awaiting_pickup:
            mat._step_awaiting_pickup(amr, .05)
        else:
            mat._step_amr(amr, .05)
    raise AssertionError("transport failed to reach expected state")


def test_amr_records_total_distance_and_from_cell() -> None:
    import math
    from fdw_sim.messaging.schemas import MaterialTransferCommand

    mat, _bus = _make_material_cell()
    mat.stock_part("PART_X", "tubular_frame_A")
    mat.cell_registry["WELDING_CELL_01"] = _Sink()
    cmd = MaterialTransferCommand("MATERIAL_CELL_01", "WELDING_CELL_01", "PART_X", "AMR_01")
    mat._on_transfer_command(cmd)
    mat._dispatch_amrs()
    amr = mat.amrs[0]
    assert amr.busy and amr.phase == "to_source"
    assert amr.payload_part_id is None
    assert "PART_X" in mat.smart_rack
    assert amr.from_cell == "MATERIAL_CELL_01"
    assert amr.target_cell == "WELDING_CELL_01"
    assert amr.total_distance_m == amr.remaining_distance_m
    assert amr.total_distance_m > 0
    _advance_until(mat, lambda: amr.phase == "to_destination")
    assert amr.payload_part_id == "PART_X"
    assert amr.total_distance_m >= math.dist(mat.dock_positions[cmd.from_cell], mat.dock_positions[cmd.to_cell])
    assert 0 < amr.remaining_distance_m <= amr.total_distance_m

def test_amr_delivery_leaves_snapshot_for_visualizer() -> None:
    from fdw_sim.messaging.schemas import MaterialTransferCommand

    mat, _bus = _make_material_cell()
    mat.stock_part("PART_X", "tubular_frame_A")
    mat.cell_registry["WELDING_CELL_01"] = _Sink()
    cmd = MaterialTransferCommand("MATERIAL_CELL_01", "WELDING_CELL_01", "PART_X", "AMR_01")
    mat._on_transfer_command(cmd)
    mat._dispatch_amrs()
    amr = mat.amrs[0]
    _advance_until(mat, lambda: amr.last_delivered_command_id is not None)
    assert amr.busy and amr.phase == "returning"
    assert amr.payload_part_id is None
    assert amr.last_delivered_part_id == "PART_X"
    assert amr.last_delivered_to == "WELDING_CELL_01"
    assert amr.last_delivered_command_id == cmd.command_id
    _advance_until(mat, lambda: not amr.busy)
    assert amr.position == amr.parking_position
    assert amr.last_delivered_command_id == cmd.command_id

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


def test_amr_current_world_pos_uses_authoritative_pose() -> None:
    from fdw_sim.cells.material.material_cell import AMR
    viz = _make_visualizer()
    amr = AMR(amr_id="AMR_01", position=(3.5, -2.0), from_cell="MATERIAL_CELL_01",
              target_cell="WELDING_CELL_01", total_distance_m=10.0, remaining_distance_m=5.0)
    assert viz._amr_current_world_pos(amr) == (3.5, -2.0, 0.0)
    amr.position = (10.0, -1.0)
    amr.from_cell = amr.target_cell = None
    amr.busy = False
    assert viz._amr_current_world_pos(amr) == (10.0, -1.0, 0.0)


def test_amr_current_world_pos_idle_is_still_rendered() -> None:
    from fdw_sim.cells.material.material_cell import AMR
    viz = _make_visualizer()
    idle_amr = AMR(amr_id="AMR_01", position=(0.0, -3.0))
    assert viz._amr_current_world_pos(idle_amr) == (0.0, -3.0, 0.0)


class _Sink:
    """receive_part을 항상 성공시키는 더미 타겟 셀."""
    cell_id = "WELDING_CELL_01"

    def receive_part(self, part_id):
        return True


def test_amr_waits_at_dock_until_pickup_confirmed() -> None:
    from fdw_sim.messaging.schemas import MaterialTransferCommand

    mat, _bus = _make_material_cell()
    mat.require_pickup_confirmation = True
    mat.stock_part("PART_X", "tubular_frame_A")
    mat.cell_registry["WELDING_CELL_01"] = _Sink()
    cmd = MaterialTransferCommand("MATERIAL_CELL_01", "WELDING_CELL_01", "PART_X", "AMR_01")
    mat._on_transfer_command(cmd)
    mat._dispatch_amrs()
    amr = mat.amrs[0]
    _advance_until(mat, lambda: amr.awaiting_pickup)
    assert amr.busy and amr.payload_part_id == "PART_X"
    assert amr.last_delivered_part_id == "PART_X"
    mat.stock_part("PART_Y", "tubular_frame_A")
    cmd2 = MaterialTransferCommand("MATERIAL_CELL_01", "WELDING_CELL_01", "PART_Y", "AMR_01")
    mat._on_transfer_command(cmd2)
    mat._dispatch_amrs()
    assert amr.awaiting_pickup and amr.payload_part_id == "PART_X"
    assert not mat.confirm_pickup("AMR_01")
    assert mat.confirm_pickup("AMR_01", "PART_X", cmd.command_id)
    assert amr.busy and not amr.awaiting_pickup
    assert amr.payload_part_id is None
    assert mat.dock_occupancy["WELDING_CELL_01"] == amr.amr_id
    _advance_until(mat, lambda: not amr.busy)
    mat._dispatch_amrs()
    assert amr.busy and amr.phase == "to_source"
    assert amr.payload_part_id is None
    _advance_until(mat, lambda: amr.phase == "to_destination")
    assert amr.payload_part_id == "PART_Y"

def test_amr_pickup_wait_times_out_if_never_confirmed() -> None:
    """Timeout reports a fault and retains custody; it never fabricates pickup."""
    from fdw_sim.messaging.schemas import MaterialTransferCommand

    mat, _bus = _make_material_cell()
    mat.require_pickup_confirmation = True
    mat.max_pickup_wait_sec = 5.0
    mat.stock_part("PART_X", "tubular_frame_A")
    mat.cell_registry["WELDING_CELL_01"] = _Sink()
    cmd = MaterialTransferCommand("MATERIAL_CELL_01", "WELDING_CELL_01", "PART_X", "AMR_01")
    mat._on_transfer_command(cmd)
    mat._dispatch_amrs()
    amr = mat.amrs[0]
    _advance_until(mat, lambda: amr.awaiting_pickup)
    mat._step_awaiting_pickup(amr, dt=3.0)
    assert amr.awaiting_pickup and not amr.pickup_blocked
    mat._step_awaiting_pickup(amr, dt=3.0)
    assert amr.awaiting_pickup and amr.busy and amr.pickup_blocked
    assert amr.payload_part_id == "PART_X"
    assert not mat.confirm_pickup("AMR_01", "PART_X", cmd.command_id)

def test_default_require_pickup_confirmation_is_false() -> None:
    """Headless transport needs no robot callback, but still clears its dock."""
    mat, _bus = _make_material_cell()
    assert mat.require_pickup_confirmation is False
    print("[OK] require_pickup_confirmation defaults to False")


def test_attach_material_cell_enables_pickup_confirmation() -> None:
    from fdw_sim.visualization.workshop_visualizer import (
        WorkshopVisualizer, WorkshopVizConfig,
    )
    from fdw_sim.messaging.bus import InMemoryBus

    mat, _bus_mat = _make_material_cell()
    assert mat.require_pickup_confirmation is False

    viz = WorkshopVisualizer(bus=InMemoryBus(), config=WorkshopVizConfig())
    viz.attach_material_cell(mat)
    assert mat.require_pickup_confirmation is True
    print("[OK] attach_material_cell turns on require_pickup_confirmation")


if __name__ == "__main__":
    test_amr_records_total_distance_and_from_cell()
    test_amr_delivery_leaves_snapshot_for_visualizer()
    test_amr_dock_pos_applies_offset()
    test_amr_current_world_pos_uses_authoritative_pose()
    test_amr_current_world_pos_idle_is_still_rendered()
    test_amr_waits_at_dock_until_pickup_confirmed()
    test_amr_pickup_wait_times_out_if_never_confirmed()
    test_default_require_pickup_confirmation_is_false()
    test_attach_material_cell_enables_pickup_confirmation()
    print("\nAll Level 2.4 AMR pick-and-place tests passed!")
