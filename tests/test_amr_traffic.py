"""CPU-only continuous-pose, swept-footprint, custody and progress regressions."""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fdw_sim.cells.base.cell_base import CellConfig
from fdw_sim.cells.inspection.inspection_cell import InspectionCell
from fdw_sim.cells.material.material_cell import CellLocation, MaterialCell
from fdw_sim.cells.material.traffic import clear_route, point_segment_distance, segment_is_clear
from fdw_sim.messaging.bus import InMemoryBus
from fdw_sim.messaging.schemas import CellType, MaterialTransferCommand


class Sink:
    def __init__(self, cell_id):
        self.cell_id = cell_id
        self.received = []

    def receive_part(self, part_id):
        self.received.append(part_id)
        return True


def fleet(num_amrs=2, confirm=False):
    bus = InMemoryBus()
    material = MaterialCell(CellConfig("MAT", CellType.MATERIAL), bus, num_amrs=num_amrs)
    for name, position in (("WEST", (-3.0, 1.0)), ("EAST", (5.0, 0.0))):
        material.register_cell(Sink(name), CellLocation(name, position))
    material.require_pickup_confirmation = confirm
    material.configure_transport_geometry({"MAT": (0.0, 0.0), "WEST": (-3.0, 1.0), "EAST": (5.0, 0.0)})
    return material


def queue(material, part, target="EAST", source="MAT"):
    if source == "MAT":
        material.stock_part(part, "test")
    command = MaterialTransferCommand(source, target, part, "AUTO")
    material._on_transfer_command(command)
    return command


def run_until(material, predicate, *, dt=.1, bound=2000):
    for step in range(bound):
        if predicate():
            return step * dt
        material.step(dt, step * dt)
    pytest.fail("transport did not reach expected state within a bounded run")


def assert_clear(material):
    for i, first in enumerate(material.amrs):
        for second in material.amrs[i + 1:]:
            separation = first.footprint_radius + second.footprint_radius + material.clearance_m
            assert math.dist(first.position, second.position) >= separation - 1e-8


def test_spawn_private_waiting_bays_cover_rotating_footprints():
    material = fleet(4)
    assert_clear(material)
    assert all(amr.position == amr.parking_position for amr in material.amrs)
    assert len({amr.position for amr in material.amrs}) == 4
    assert all(amr.position[1] < material.aisle_y for amr in material.amrs)
    with pytest.raises(ValueError, match="overlap"):
        material.configure_transport_geometry({"MAT": (0, 0)}, parking_positions=[(0, -5)] * 4)


def test_assignment_does_not_teleport_or_take_part_before_source_arrival():
    material = fleet()
    command = queue(material, "P")
    before = [amr.position for amr in material.amrs]
    material._dispatch_amrs()
    owner = next(amr for amr in material.amrs if amr.busy)
    assert [amr.position for amr in material.amrs] == before
    assert owner.phase == "to_source"
    assert owner.payload_part_id is None
    assert "P" in material.smart_rack and "P" not in material.in_transit
    run_until(material, lambda: owner.phase == "to_destination", dt=.01)
    assert "P" not in material.smart_rack
    assert owner.payload_part_id == command.part_id
    assert material.in_transit["P"] is owner
    assert math.dist(owner.position, material.dock_positions["MAT"]) <= .011


def test_position_and_heading_are_continuous_and_speed_bounded():
    material = fleet()
    queue(material, "P", "WEST")
    dt = .07
    saw_empty = saw_loaded = saw_return = False
    for index in range(1800):
        before = [(amr.position, amr.heading) for amr in material.amrs]
        material.step(dt, index * dt)
        for amr, (position, heading) in zip(material.amrs, before):
            assert math.dist(amr.position, position) <= amr.speed_mps * dt + 1e-8
            assert abs(amr.heading - heading) <= amr.turn_speed_radps * dt + 1e-8
            saw_empty |= amr.phase == "to_source"
            saw_loaded |= amr.phase == "to_destination"
            saw_return |= amr.phase == "returning"
        assert_clear(material)
        if material.cell_registry["WEST"].received and all(not amr.busy for amr in material.amrs):
            break
    else:
        pytest.fail("route failed to finish")
    assert saw_empty and saw_loaded and saw_return
    assert material.aisle_owner is None and not material.dock_occupancy


def test_only_one_owner_and_whole_route_avoids_parked_footprints():
    material = fleet()
    for i in range(4):
        queue(material, f"P{i}", "WEST" if i % 2 else "EAST")
    used = set()
    # Large steps exercise endpoint-crossing/tunneling protection as well.
    for index in range(300):
        material.step(.71, index * .71)
        busy = [amr for amr in material.amrs if amr.busy]
        assert len(busy) <= 1
        if busy:
            owner = busy[0]
            used.add(owner.amr_id)
            assert material.aisle_owner == owner.amr_id
            points = [owner.position] + owner.waypoints
            for start, end in zip(points, points[1:]):
                assert segment_is_clear(start, end, material._obstacles(owner))
        assert_clear(material)
        if not material.transfer_queue and not busy:
            break
    else:
        pytest.fail("two-AMR queued jobs deadlocked")
    assert used == {amr.amr_id for amr in material.amrs}
    assert sorted(material.cell_registry["WEST"].received + material.cell_registry["EAST"].received) == [f"P{i}" for i in range(4)]


def test_pickup_is_keyed_and_dock_is_held_until_footprint_departure():
    material = fleet(confirm=True)
    command = queue(material, "P")
    next_command = queue(material, "Q")
    run_until(material, lambda: any(amr.awaiting_pickup for amr in material.amrs))
    owner = next(amr for amr in material.amrs if amr.awaiting_pickup)
    dock = material.dock_positions["EAST"]
    assert owner.position == dock
    assert material.dock_occupancy["EAST"] == owner.amr_id
    assert owner.payload_part_id == "P" and material.in_transit["P"] is owner
    assert not material.confirm_pickup(owner.amr_id)
    assert not material.confirm_pickup(owner.amr_id, "Q", command.command_id)
    assert not material.confirm_pickup(owner.amr_id, "P", next_command.command_id)
    assert material.confirm_pickup(owner.amr_id, "P", command.command_id)
    assert not material.confirm_pickup(owner.amr_id, "P", command.command_id)
    assert owner.busy and owner.phase == "returning"
    assert material.dock_occupancy["EAST"] == owner.amr_id
    assert material.aisle_owner == owner.amr_id
    material._dispatch_amrs()
    assert all(not amr.busy for amr in material.amrs if amr is not owner)
    run_until(material, lambda: "EAST" not in material.dock_occupancy)
    assert math.dist(owner.position, dock) >= 2 * owner.footprint_radius + material.clearance_m
    run_until(material, lambda: not owner.busy)
    assert owner.position == owner.parking_position


def test_pickup_timeout_fails_closed_and_late_confirmation_cannot_release():
    material = fleet(confirm=True)
    material.max_pickup_wait_sec = .5
    command = queue(material, "P")
    queue(material, "Q")
    run_until(material, lambda: any(amr.awaiting_pickup for amr in material.amrs))
    owner = next(amr for amr in material.amrs if amr.awaiting_pickup)
    position = owner.position
    material._step_awaiting_pickup(owner, 20)
    assert owner.busy and owner.awaiting_pickup and owner.pickup_blocked
    assert owner.blocked_reason == "pickup confirmation timeout"
    assert not material.confirm_pickup(owner.amr_id, "P", command.command_id)
    for step in range(20):
        material.step(1, step)
    assert owner.position == position
    assert material.aisle_owner == owner.amr_id
    assert material.dock_occupancy["EAST"] == owner.amr_id
    assert material.cell_registry["EAST"].received == ["P"]
    assert "Q" in material.smart_rack and owner.payload_part_id == "P"


def test_material_pickup_confirmation_releases_exact_target_placement():
    material = fleet(confirm=True)
    target = InspectionCell(CellConfig("INSP", CellType.INSPECTION), material.bus)
    target.boot()
    material.register_cell(target, CellLocation("INSP", (3, 2)))
    docks = dict(material.dock_positions, INSP=(3, 2))
    material.configure_transport_geometry(docks)
    command = queue(material, "P", "INSP")
    run_until(material, lambda: any(amr.awaiting_pickup for amr in material.amrs))
    amr = next(amr for amr in material.amrs if amr.awaiting_pickup)
    assert not target.is_part_ready("P")
    assert material.confirm_pickup(amr.amr_id, "P", command.command_id)
    assert target.is_part_ready("P")
    assert not material.in_transit


def test_swept_segment_detects_tunneling_and_routes_around_parked_vehicle():
    obstacles = [((0.0, 0.0), 1.6)]
    assert math.dist((-5, 0), (0, 0)) > 1.6
    assert math.dist((5, 0), (0, 0)) > 1.6
    assert not segment_is_clear((-5, 0), (5, 0), obstacles)
    route = clear_route((-5, 0), (5, 0), obstacles)
    assert route and len(route) > 1
    for start, end in zip([(-5, 0)] + route, route):
        assert point_segment_distance((0, 0), start, end) >= 1.6


def test_invalid_time_and_mid_transport_geometry_change_are_rejected():
    material = fleet()
    for dt in (-1, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            material.step(dt, 0)
    queue(material, "P")
    material._dispatch_amrs()
    with pytest.raises(RuntimeError, match="after dispatch"):
        material.configure_transport_geometry({"MAT": (100, 100)})


@pytest.mark.parametrize("field,bad_value", [
    (field, value) for field in ("speed_mps", "turn_speed_radps")
    for value in (float("nan"), float("inf"), -float("inf"), 0.0, -1.0)
] + [
    ("position", (float("nan"), 0.0)),
    ("position", (0.0, float("inf"))),
    ("position", (-float("inf"), 0.0)),
    ("heading", float("nan")),
    ("heading", float("inf")),
    ("heading", -float("inf")),
])
def test_invalid_kinematics_at_assignment_cannot_pickup_or_teleport(field, bad_value):
    material = fleet()
    command = queue(material, "P")
    owner = material.amrs[0]
    old_heading = owner.heading
    old_position = owner.position
    setattr(owner, field, bad_value)
    material._dispatch_amrs()
    assert owner.busy and owner.phase == "blocked"
    assert owner.transfer_command_id == command.command_id
    assert material.aisle_owner == owner.amr_id
    assert "invalid" in owner.blocked_reason
    for index in range(3):
        material.step(.01, index * .01)
    assert material.cell_registry["EAST"].received == []
    assert "P" in material.smart_rack and owner.payload_part_id is None
    assert owner.last_delivered_command_id is None
    if field != "position":
        assert owner.position == old_position
    if field != "heading":
        assert owner.heading == old_heading


@pytest.mark.parametrize("phase", ["to_source", "to_destination", "returning"])
@pytest.mark.parametrize("field,bad_value", [
    (field, value) for field in ("speed_mps", "turn_speed_radps")
    for value in (float("nan"), float("inf"), -float("inf"), 0.0, -1.0)
] + [
    ("position", (float("nan"), 0.0)),
    ("position", (0.0, float("inf"))),
    ("position", (-float("inf"), 0.0)),
    ("heading", float("nan")),
    ("heading", float("inf")),
    ("heading", -float("inf")),
])
def test_invalid_kinematics_mid_trip_hold_custody_and_reservations(phase, field, bad_value):
    material = fleet()
    queue(material, "P")
    queue(material, "Q")
    material._dispatch_amrs()
    owner = next(amr for amr in material.amrs if amr.busy)
    run_until(material, lambda: owner.phase == phase, dt=.05)
    old_position, old_heading = owner.position, owner.heading
    old_payload = owner.payload_part_id
    held_docks = dict(material.dock_occupancy)
    old_received = list(material.cell_registry["EAST"].received)
    old_transit = dict(material.in_transit)
    setattr(owner, field, bad_value)
    material._step_amr(owner, .01)
    assert owner.busy and owner.phase == "blocked"
    assert "invalid" in owner.blocked_reason
    assert owner.payload_part_id == old_payload
    assert material.in_transit == old_transit
    assert material.dock_occupancy == held_docks
    assert material.aisle_owner == owner.amr_id
    for index in range(3):
        material.step(1000, index * 1000)
    assert material.cell_registry["EAST"].received == old_received
    assert "Q" in material.smart_rack
    assert all(not vehicle.busy for vehicle in material.amrs if vehicle is not owner)
    if field != "position":
        assert owner.position == old_position
    if field != "heading":
        assert owner.heading == old_heading


def test_default_geometry_refresh_cannot_hide_invalid_pose_before_assignment():
    material = MaterialCell(CellConfig("MAT", CellType.MATERIAL), InMemoryBus(), num_amrs=1)
    material.register_cell(Sink("EAST"), CellLocation("EAST", (5, 0)))
    queue(material, "P")
    material.amrs[0].position = (float("nan"), 0)
    material._dispatch_amrs()
    assert material.amrs[0].phase == "blocked"
    assert "invalid position" in material.amrs[0].blocked_reason
    assert "P" in material.smart_rack


def test_invalid_parked_vehicle_pose_blocks_active_vehicle_before_swept_math():
    material = fleet()
    queue(material, "P")
    material._dispatch_amrs()
    owner = material.amrs[0]
    material.amrs[1].position = (float("nan"), float("nan"))
    before = owner.position
    material._step_amr(owner, 100)
    assert owner.position == before
    assert owner.phase == "blocked" and material.aisle_owner == owner.amr_id
    assert "AMR_02" in owner.blocked_reason


def test_invalid_rate_in_delivery_callback_cannot_teleport_return_in_same_tick():
    material = fleet()
    queue(material, "P")
    material._dispatch_amrs()
    owner = material.amrs[0]
    original_receive = material.cell_registry["EAST"].receive_part

    def receive_and_invalidate_rate(part_id):
        owner.speed_mps = float("inf")
        return original_receive(part_id)

    material.cell_registry["EAST"].receive_part = receive_and_invalidate_rate
    material._step_amr(owner, 1000)
    assert material.cell_registry["EAST"].received == ["P"]
    assert owner.position == material.dock_positions["EAST"]
    assert owner.phase == "blocked"
    assert material.dock_occupancy["EAST"] == owner.amr_id
    assert material.aisle_owner == owner.amr_id


@pytest.mark.parametrize("destination", ["MAT", "UNREGISTERED"])
def test_destination_dock_without_receiver_keeps_stock_and_command(destination):
    material = fleet()
    material.configure_transport_geometry(dict(material.dock_positions, UNREGISTERED=(7, 2)))
    command = queue(material, "P", destination)
    before = [amr.position for amr in material.amrs]
    for index in range(10):
        material.step(10, index * 10)
    assert material.transfer_queue == [command]
    assert "P" in material.smart_rack and not material.in_transit
    assert all(not amr.busy and amr.last_delivered_command_id is None for amr in material.amrs)
    assert [amr.position for amr in material.amrs] == before
    assert material.aisle_owner is None


def test_receiver_disappearing_during_loaded_trip_blocks_without_losing_custody():
    material = fleet()
    command = queue(material, "P")
    material._dispatch_amrs()
    owner = material.amrs[0]
    run_until(material, lambda: owner.phase == "to_destination")
    receiver = material.cell_registry.pop("EAST")
    material._step_amr(owner, 1000)
    assert owner.busy and owner.phase == "blocked"
    assert owner.blocked_reason == "destination receiver unavailable"
    assert owner.position == material.dock_positions["EAST"]
    assert owner.payload_part_id == command.part_id
    assert material.in_transit["P"] is owner
    assert owner.last_delivered_command_id is None
    assert command.command_id not in material._completed_commands
    assert receiver.received == []
    assert material.dock_occupancy["EAST"] == owner.amr_id
    assert material.aisle_owner == owner.amr_id


def test_registered_object_without_callable_receiver_is_not_a_destination():
    material = fleet()
    material.cell_registry["EAST"] = object()
    command = queue(material, "P")
    material._dispatch_amrs()
    assert material.transfer_queue == [command]
    assert "P" in material.smart_rack and not material.in_transit
