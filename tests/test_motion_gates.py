"""CPU-only command/part keyed placement and process completion regressions.

These tests assert logical safety contracts, not grasp, contact or Isaac motion.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fdw_sim.cells.base.cell_base import CellConfig
from fdw_sim.cells.welding.welding_cell import WeldingCell
from fdw_sim.messaging.bus import InMemoryBus, Topics
from fdw_sim.messaging.schemas import CellState, CellType, DispatchCommand, JobSpec
from fdw_sim.orchestrator.orchestrator import FDWOrchestrator, JobTracker, OrchestratorConfig


def make_cell(motion=True):
    bus = InMemoryBus()
    cell = WeldingCell(CellConfig("WELD", CellType.WELDING), bus)
    cell.boot()
    cell.set_motion_required(motion)
    return bus, cell


def place(cell, part="P", transfer="MOVE_1"):
    assert cell.receive_part(part)
    assert cell.mark_placement_pending(part, transfer)
    assert cell.confirm_placement(part, transfer)


def dispatch(bus, cell, part="P", command="CMD_1", recipe=None):
    cmd = DispatchCommand(cell.cell_id, "JOB_" + part, "START_WELDING",
                          recipe=recipe or {}, command_id=command, part_id=part)
    bus.publish(Topics.DISPATCH_COMMAND, cmd)
    return cmd


def test_robot_input_is_not_ready_until_matching_transfer_places_it():
    bus, cell = make_cell()
    assert cell.receive_part("P")
    assert not cell.is_part_ready("P")
    assert cell.fsm.state == CellState.IDLE
    cmd = dispatch(bus, cell)
    assert cell.current_command is None
    assert not cell.on_command(cmd)  # Direct calls cannot bypass placement.
    assert cell.mark_placement_pending("P", "MOVE_1")
    assert not cell.mark_placement_pending("P", "MOVE_2")
    assert not cell.confirm_placement("OTHER", "MOVE_1")
    assert not cell.confirm_placement("P", "MOVE_OLD")
    assert not cell.is_part_ready("P")
    assert cell.confirm_placement("P", "MOVE_1")
    assert cell.is_part_ready("P")
    assert not cell.confirm_placement("P", "MOVE_1")  # Duplicate result.
    bus.publish(Topics.DISPATCH_COMMAND, cmd)
    assert cell.current_command is cmd
    assert cell.fsm.state == CellState.PROCESSING


def test_logical_mode_keeps_clock_only_operation_explicit():
    bus, cell = make_cell(motion=False)
    assert not cell.motion_required
    assert cell.receive_part("P")
    dispatch(bus, cell)
    cell.step(5, 5)
    assert cell.output_buffer.part_id == "P"
    assert cell.fsm.state == CellState.BLOCKED
    assert not cell.confirm_process_motion("CMD_1", "P")


def test_process_timer_alone_never_releases_robot_part():
    bus, cell = make_cell()
    place(cell)
    dispatch(bus, cell)
    cell.step(6, 6)
    assert cell.progress == 1.0
    assert cell.fsm.state == CellState.PROCESSING
    assert cell.input_buffer.part_id == "P"
    assert not cell.output_buffer.occupied
    assert cell.takeout_part() is None
    quality_kpis = len(cell.kpi_buffer)
    cell.step(1, 7)
    assert len(cell.kpi_buffer) == quality_kpis  # No repeated quality work.
    assert not cell.confirm_process_motion("CMD_OLD", "P")
    assert not cell.confirm_process_motion("CMD_1", "OTHER")
    assert cell.confirm_process_motion("CMD_1", "P")
    assert cell.fsm.state == CellState.BLOCKED
    assert cell.output_buffer.part_id == "P"
    assert not cell.confirm_process_motion("CMD_1", "P")


def test_motion_ack_before_recipe_end_does_not_finish_process_early():
    bus, cell = make_cell()
    place(cell)
    dispatch(bus, cell)
    assert cell.confirm_process_motion("CMD_1", "P")
    cell.step(4, 4)
    assert not cell.output_buffer.occupied
    assert cell.input_buffer.part_id == "P"
    cell.step(1, 5)
    assert cell.output_buffer.part_id == "P"


def test_placement_failure_is_sticky_and_preserves_custody():
    bus, cell = make_cell()
    assert cell.receive_part("P")
    assert cell.mark_placement_pending("P", "MOVE_1")
    assert cell.confirm_placement("P", "MOVE_1", success=False, reason="controller failed")
    assert cell.fsm.state == CellState.FAULT
    assert cell.health.alarm_code == "placement_failed"
    assert not cell.confirm_placement("P", "MOVE_1")
    dispatch(bus, cell)
    cell.step(100, 100)
    assert cell.current_command is None
    assert cell.input_buffer.part_id == "P"
    assert not cell.output_buffer.occupied


@pytest.mark.parametrize("failure", ["controller", "timeout"])
def test_process_failure_or_timeout_never_counts_as_success(failure):
    bus, cell = make_cell()
    place(cell)
    dispatch(bus, cell)
    cell.max_process_motion_wait_sec = 2
    if failure == "controller":
        assert cell.confirm_process_motion("CMD_1", "P", success=False, reason="unreachable")
    else:
        cell.step(7, 7)
    assert cell.fsm.state == CellState.FAULT
    assert not cell.confirm_process_motion("CMD_1", "P")
    assert cell.input_buffer.part_id == "P"
    assert not cell.output_buffer.occupied
    assert not any(k.metric == "cycle_time" for k in cell.kpi_buffer)
    with pytest.raises(RuntimeError, match="motion mode"):
        cell.set_motion_required(False)


def test_recipe_and_visual_motion_share_one_duration_and_snapshot():
    bus, cell = make_cell()
    place(cell)
    dispatch(bus, cell, recipe={"speed": .01, "path_length_mm": 200})
    spec = cell.get_process_motion_spec()
    assert spec["command_id"] == "CMD_1"
    assert spec["part_id"] == "P"
    assert spec["travel_time_sec"] == cell.cycle_time_estimate == 20
    assert spec["recipe"]["speed"] == .01
    spec["recipe"]["speed"] = 999
    assert cell.active_recipe["speed"] == .01
    assert spec["motion_required"] and not spec["motion_complete"]


def test_duplicate_dispatch_does_not_reset_progress_or_replace_live_job():
    bus, cell = make_cell()
    place(cell)
    cmd = dispatch(bus, cell)
    cell.step(1, 1)
    old_progress = cell.progress
    bus.publish(Topics.DISPATCH_COMMAND, cmd)
    dispatch(bus, cell, command="CMD_OTHER")
    assert cell.current_command is cmd
    assert cell.progress == old_progress
    assert cell.confirm_process_motion("CMD_1", "P")
    cell.step(4, 5)
    assert cell.takeout_part() == "P"
    place(cell, transfer="MOVE_2")
    bus.publish(Topics.DISPATCH_COMMAND, cmd)
    assert cell.current_command is None
    dispatch(bus, cell, command="CMD_2")
    assert not cell.confirm_process_motion("CMD_1", "P")
    assert cell.current_command.command_id == "CMD_2"


def test_wrong_part_dispatch_does_not_start_or_consume_command_id():
    bus, cell = make_cell()
    place(cell)
    dispatch(bus, cell, part="OTHER")
    assert cell.current_command is None
    dispatch(bus, cell)
    assert cell.current_command.part_id == "P"


@pytest.mark.parametrize("missing_part", [None, ""])
def test_replayed_unidentified_start_cannot_seize_later_physical_part(missing_part):
    bus, cell = make_cell()
    stale = DispatchCommand("WELD", "OLD_JOB", "START_WELDING",
                            command_id="DELAYED_CMD", part_id=missing_part)
    bus.publish(Topics.DISPATCH_COMMAND, stale)  # Initially rejected: no part.
    assert cell.current_command is None

    place(cell, "OLD_PART", "MOVE_OLD")
    dispatch(bus, cell, part="OLD_PART", command="CMD_OLD")
    assert cell.confirm_process_motion("CMD_OLD", "OLD_PART")
    cell.step(5, 5)
    assert cell.takeout_part() == "OLD_PART"

    place(cell, "NEW_PART", "MOVE_NEW")
    bus.publish(Topics.DISPATCH_COMMAND, stale)
    assert not cell.on_command(stale)  # Direct calls cannot bypass identity.
    assert cell.current_command is None
    assert cell.input_buffer.part_id == "NEW_PART"
    assert cell.fsm.state == CellState.READY
    assert cell.get_process_motion_spec() is None

    dispatch(bus, cell, part="NEW_PART", command="CMD_NEW")
    assert cell.current_command.job_id == "JOB_NEW_PART"
    assert cell.current_command.part_id == "NEW_PART"


def test_legacy_unidentified_start_is_supported_only_in_logical_mode():
    bus, cell = make_cell(motion=False)
    assert cell.receive_part("P")
    legacy = DispatchCommand("WELD", "JOB_P", "START_WELDING")
    bus.publish(Topics.DISPATCH_COMMAND, legacy)
    assert cell.current_command is legacy
    cell.step(5, 5)
    assert cell.output_buffer.part_id == "P"


def test_status_snapshots_expose_readiness_and_exact_process_identity():
    bus, cell = make_cell()
    statuses = []
    bus.subscribe(Topics.CELL_STATUS, statuses.append)
    cell.receive_part("P")
    cell.mark_placement_pending("P", "MOVE_1")
    cell.publish_status()
    pending = statuses[-1]
    assert not pending.placement_ready
    assert pending.placement_transfer_command_id == "MOVE_1"
    cell.confirm_placement("P", "MOVE_1")
    dispatch(bus, cell)
    cell.publish_status()
    active = statuses[-1]
    assert active.current_command_id == "CMD_1"
    assert active.current_part_id == "P"
    assert active.current_job_id == "JOB_P"
    assert active.motion_required and active.placement_ready
    assert not pending.placement_ready


def test_orchestrator_waits_for_placement_and_rechecks_stale_ready_cache():
    bus, cell = make_cell(motion=False)
    orch = FDWOrchestrator(bus, OrchestratorConfig(process_to_cell={"welding": "WELD"}))
    orch.register_cell(cell)
    tracker = JobTracker(JobSpec("J", "P", "test", ["welding"]))
    orch.active_jobs["J"] = tracker
    commands = []
    bus.subscribe(Topics.DISPATCH_COMMAND, commands.append)
    cell.receive_part("P")
    cell.publish_status()  # READY, but already obsolete when decision runs.
    assert cell.mark_placement_pending("P", "MOVE_1")
    orch._tick_decisions()
    assert not commands
    assert not orch._dispatched_jobs_at_cell
    cell.publish_status()  # Explicit pending placement also blocks dispatch.
    orch._tick_decisions()
    assert not commands
    assert cell.confirm_placement("P", "MOVE_1")
    cell.publish_status()
    orch._tick_decisions()
    assert len(commands) == 1
    assert commands[0].part_id == "P"


@pytest.mark.parametrize("recipe", [{"speed": float("nan")}, {"speed": 0},
                                    {"path_length_mm": -1}, {"power": "invalid"}])
def test_invalid_recipe_does_not_start_motion(recipe):
    bus, cell = make_cell()
    place(cell)
    dispatch(bus, cell, recipe=recipe)
    assert cell.current_command is None
    assert cell.get_process_motion_spec() is None
    assert cell.input_buffer.part_id == "P"


def test_queued_placement_remains_unready_after_old_output_taken():
    _, cell = make_cell()
    cell.output_buffer.occupied = True
    cell.output_buffer.part_id = "OLD"
    cell.fsm.transition(CellState.READY)
    cell.fsm.transition(CellState.BLOCKED)
    assert cell.receive_part("P")
    assert cell.mark_placement_pending("P", "MOVE_1")
    assert cell.takeout_part() == "OLD"
    assert cell.fsm.state == CellState.IDLE
    assert not cell.is_part_ready("P")
    assert cell.confirm_placement("P", "MOVE_1")
    assert cell.is_part_ready("P")
