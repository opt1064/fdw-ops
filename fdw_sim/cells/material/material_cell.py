"""Material custody and conservative, continuous AMR transport.

The fleet shares one reserved aisle. An owner travels empty to its source,
loads there, delivers, waits for keyed placement confirmation when enabled,
and clears the dock by returning to its private bay before releasing the
reservation. This deliberately trades throughput for provable AMR separation.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Dict, List, Optional
import logging
import math

from fdw_sim.cells.base.cell_base import CellConfig, DistributedIntelligenceCell
from fdw_sim.cells.material.traffic import (
    StaticObstacle, blocked_obstacle_ids, clear_route, segment_is_clear,
)
from fdw_sim.messaging.bus import MessageBus, Topics
from fdw_sim.messaging.schemas import DispatchCommand, MaterialTransferCommand, QualityPrediction

logger = logging.getLogger(__name__)


@dataclass
class AMR:
    """Authoritative planar pose and transport/custody state of one vehicle."""
    amr_id: str
    speed_mps: float = 1.0
    position: tuple = (0.0, 0.0)
    heading: float = math.pi / 2             # radians, continuous/unwrapped
    turn_speed_radps: float = math.pi / 2
    footprint_size: tuple = (1.2, 0.8)
    # Reserve the maximum permitted carried load even on empty approach/return.
    payload_footprint_size: tuple = (0.0, 0.0)
    payload_offset: tuple = (0.0, 0.0)
    parking_position: tuple = (0.0, 0.0)
    busy: bool = False
    phase: str = "parked"
    payload_part_id: Optional[str] = None
    payload_quality: Optional[QualityPrediction] = None
    from_cell: Optional[str] = None
    target_cell: Optional[str] = None
    remaining_distance_m: float = 0.0
    transfer_command_id: Optional[str] = None
    total_distance_m: float = 0.0
    last_delivered_part_id: Optional[str] = None
    last_delivered_to: Optional[str] = None
    last_delivered_command_id: Optional[str] = None
    awaiting_pickup: bool = False
    pickup_wait_elapsed: float = 0.0
    pickup_blocked: bool = False
    blocked_reason: Optional[str] = None
    held_docks: set = field(default_factory=set)
    waypoints: list = field(default_factory=list)
    command: Optional[MaterialTransferCommand] = field(default=None, repr=False)

    @property
    def footprint_radius(self) -> float:
        chassis = math.hypot(*self.footprint_size) / 2.0
        load = math.hypot(*(abs(self.payload_offset[i]) + self.payload_footprint_size[i] / 2
                            for i in range(2)))
        return max(chassis, load)


@dataclass
class CellLocation:
    cell_id: str
    position: tuple


class MaterialCell(DistributedIntelligenceCell):
    """Material cell with exact-part custody and fail-closed dock pickup."""

    def __init__(self, config: CellConfig, bus: MessageBus,
                 cell_locations: Optional[Dict[str, CellLocation]] = None,
                 num_amrs: int = 2) -> None:
        super().__init__(config, bus)
        self.smart_rack: Dict[str, dict] = {}
        self.cell_locations = cell_locations or {}
        self.amrs = [AMR(amr_id=f"AMR_{i+1:02d}") for i in range(num_amrs)]
        self.transfer_queue: List[MaterialTransferCommand] = []
        self.require_pickup_confirmation = False
        self.max_pickup_wait_sec = 15.0
        self.in_transit: Dict[str, AMR] = {}
        self.cell_registry: Dict[str, DistributedIntelligenceCell] = {}
        self.dock_positions: dict[str, tuple] = {}
        self.dock_occupancy: dict[str, str] = {}
        self.aisle_owner: Optional[str] = None
        self.aisle_y = 0.0
        self.clearance_m = 0.15
        self.static_obstacles: tuple[StaticObstacle, ...] = ()
        self.navigation_bounds = None
        self.navigation_boundary = None
        self.navigation_source = "logical-unmapped"
        self.payload_geometry_issues: dict[str, str] = {}
        self.validated_payloads: set[str] = set()
        self.require_payload_geometry_validation = False
        self._geometry_explicit = False
        self._transport_started = False
        self._next_amr_index = 0
        self._completed_commands: set[str] = set()
        self.configure_transport_geometry(self._logical_docks())
        self._geometry_explicit = False
        self.bus.subscribe(Topics.MATERIAL_TRANSFER, self._on_transfer_command)

    def _logical_docks(self) -> dict[str, tuple]:
        docks = {cell_id: tuple(location.position[:2])
                 for cell_id, location in self.cell_locations.items()}
        docks.setdefault(self.cell_id, (0.0, 0.0))
        return docks

    def configure_transport_geometry(self, docks: dict[str, tuple], *,
                                     parking_positions=None,
                                     footprint_size=(1.2, 0.8),
                                     clearance=0.15, static_obstacles=(),
                                     navigation_bounds=None, navigation_boundary=None,
                                     navigation_source="logical-unmapped",
                                     payload_footprint_size=(0.0, 0.0),
                                     payload_offset=(0.0, 0.0)) -> None:
        """Set spawn/docks before first transport; never relocate a live fleet.

        Automatic waiting bays are one vehicle diameter below the shared aisle,
        which is itself below every dock. Each bay has an independent approach.
        A visualizer must render these same poses instead of inventing spawns.
        """
        if self._transport_started or any(amr.busy for amr in self.amrs):
            raise RuntimeError("transport geometry cannot change after dispatch")
        if (len(footprint_size) != 2 or any(not math.isfinite(v) or v <= 0 for v in footprint_size)
                or not math.isfinite(clearance) or clearance < 0):
            raise ValueError("invalid AMR footprint or clearance")
        converted = {key: tuple(float(v) for v in value[:2]) for key, value in docks.items()}
        converted.setdefault(self.cell_id, self._logical_docks()[self.cell_id])
        if any(len(point) != 2 or not all(math.isfinite(v) for v in point) for point in converted.values()):
            raise ValueError("dock positions must be finite XY coordinates")
        if (len(payload_footprint_size) != 2 or len(payload_offset) != 2
                or not all(math.isfinite(v) and v >= 0 for v in payload_footprint_size)
                or not all(math.isfinite(v) for v in payload_offset)):
            raise ValueError("invalid payload envelope")
        obstacles = tuple(static_obstacles)
        if any(not isinstance(obs, StaticObstacle) for obs in obstacles):
            raise ValueError("invalid static navigation map: expected StaticObstacle entries")
        radius = max(math.hypot(*footprint_size) / 2,
                     math.hypot(*(abs(payload_offset[i]) + payload_footprint_size[i] / 2
                                  for i in range(2))))
        inflated = [obs.inflated(radius + clearance) for obs in obstacles]
        bounds = self._inset_bounds(navigation_bounds, radius + clearance)
        # Validate the complete map and every endpoint before mutating any pose.
        for name, point in converted.items():
            blockers = blocked_obstacle_ids(point, point, inflated, bounds=bounds,
                convex_boundary=navigation_boundary, boundary_margin=radius + clearance)
            if blockers:
                raise ValueError(f"dock {name} obstructed by {', '.join(blockers)}")
        gap = 2 * radius + clearance + 0.05
        minimum_x = min(point[0] for point in converted.values())
        minimum_y = min(point[1] for point in converted.values())
        aisle_y = minimum_y - gap
        if parking_positions is None:
            positions = [(minimum_x + i * gap, aisle_y - gap) for i in range(len(self.amrs))]
        elif isinstance(parking_positions, dict):
            positions = [tuple(parking_positions[amr.amr_id][:2]) for amr in self.amrs]
        else:
            positions = [tuple(point[:2]) for point in parking_positions]
        if len(positions) != len(self.amrs):
            raise ValueError("one private parking bay is required per AMR")
        separation = 2 * radius + clearance
        for index, point in enumerate(positions):
            if len(point) != 2 or not all(math.isfinite(v) for v in point):
                raise ValueError("parking positions must be finite XY coordinates")
            blockers = blocked_obstacle_ids(point, point, inflated, bounds=bounds,
                convex_boundary=navigation_boundary, boundary_margin=radius + clearance)
            if blockers:
                raise ValueError(f"parking AMR_{index+1:02d} obstructed by {', '.join(blockers)}")
            if any(math.dist(point, other) < separation for other in positions[:index]):
                raise ValueError("AMR parking footprints overlap")
            if any(math.dist(point, dock) < separation for dock in converted.values()):
                raise ValueError("parking footprint intrudes on a dock")
        self.static_obstacles = obstacles
        self.navigation_bounds = navigation_bounds
        self.navigation_boundary = navigation_boundary
        self.navigation_source = navigation_source
        self.require_payload_geometry_validation = navigation_source == "usd"
        self.dock_positions = converted
        self.aisle_y = aisle_y
        self.clearance_m = clearance
        self._geometry_explicit = True
        for amr, point in zip(self.amrs, positions):
            amr.position = amr.parking_position = point
            amr.heading = math.pi / 2
            amr.footprint_size = tuple(footprint_size)
            amr.payload_footprint_size = tuple(payload_footprint_size)
            amr.payload_offset = tuple(payload_offset)

    @staticmethod
    def _inset_bounds(bounds, radius):
        if bounds is None:
            return None
        if len(bounds) != 4 or not all(math.isfinite(v) for v in bounds):
            raise ValueError("invalid navigation bounds")
        inset = (bounds[0] + radius, bounds[1] - radius,
                 bounds[2] + radius, bounds[3] - radius)
        if inset[0] >= inset[1] or inset[2] >= inset[3]:
            raise ValueError("navigation bounds too small for loaded AMR envelope")
        return inset

    def _route_bounds(self, amr):
        return self._inset_bounds(self.navigation_bounds, amr.footprint_radius + self.clearance_m)

    def _boundary_kwargs(self, amr):
        return {"convex_boundary": self.navigation_boundary,
                "boundary_margin": amr.footprint_radius + self.clearance_m}

    @staticmethod
    def _kinematic_issue(amr: AMR) -> Optional[str]:
        """Reject invalid telemetry/configuration before it reaches arithmetic."""
        try:
            for name in ("speed_mps", "turn_speed_radps"):
                value = getattr(amr, name)
                if not math.isfinite(value) or value <= 0:
                    return f"invalid {name}: expected a finite positive rate"
            if len(amr.position) != 2 or not all(math.isfinite(v) for v in amr.position):
                return "invalid position: expected finite XY coordinates"
            if not math.isfinite(amr.heading):
                return "invalid heading: expected finite radians"
            if (len(amr.footprint_size) != 2
                    or not all(math.isfinite(v) and v > 0 for v in amr.footprint_size)):
                return "invalid footprint: expected finite positive dimensions"
            if (len(amr.payload_footprint_size) != 2 or len(amr.payload_offset) != 2
                    or not all(math.isfinite(v) and v >= 0 for v in amr.payload_footprint_size)
                    or not all(math.isfinite(v) for v in amr.payload_offset)):
                return "invalid payload envelope"
        except (TypeError, ValueError, OverflowError):
            return "invalid AMR kinematics: expected numeric pose, rates and footprint"
        return None

    def _payload_issue(self, amr: AMR) -> Optional[str]:
        part_id = amr.payload_part_id or (amr.command.part_id if amr.command else None)
        issue = self.payload_geometry_issues.get(part_id)
        if issue:
            return issue
        if (part_id and self.require_payload_geometry_validation
                and part_id not in self.validated_payloads):
            return f"unmeasured payload {part_id}: successful USD geometry validation required"
        return None

    def _fleet_issue(self) -> Optional[str]:
        for vehicle in self.amrs:
            issue = self._kinematic_issue(vehicle)
            if issue:
                return f"{vehicle.amr_id}: {issue}"
        return None

    def _ensure_geometry(self) -> None:
        # Configuration refresh must not quietly repair bad live telemetry or
        # hide an invalid pose/rate from assignment's fail-closed validation.
        if not self._geometry_explicit and not self._transport_started and not self._fleet_issue():
            self.configure_transport_geometry(self._logical_docks())
            self._geometry_explicit = False

    def configure(self) -> None:
        self._ensure_geometry()
        logger.info("[%s] configured with %d AMRs", self.cell_id, len(self.amrs))

    def register_cell(self, cell: DistributedIntelligenceCell, location: CellLocation) -> None:
        self.cell_registry[cell.cell_id] = cell
        self.cell_locations[cell.cell_id] = location
        self._ensure_geometry()

    def stock_part(self, part_id: str, part_type: str) -> None:
        self.smart_rack[part_id] = {"part_type": part_type}
        self.log_kpi("rack_stock_count", float(len(self.smart_rack)), unit="parts")

    def on_command(self, command: DispatchCommand) -> bool:
        if command.operation != "RESET":
            return False
        # Reset is not a way to discard a held load or teleport a vehicle home.
        if any(amr.busy for amr in self.amrs):
            logger.warning("[%s] cannot RESET while transport or pickup is active", self.cell_id)
            return False
        self.transfer_queue.clear()
        return True

    def _on_transfer_command(self, cmd: MaterialTransferCommand) -> None:
        if cmd.from_cell != self.cell_id and cmd.from_cell not in self.cell_registry:
            return
        if (cmd.command_id in self._completed_commands
                or any(item.command_id == cmd.command_id for item in self.transfer_queue)
                or any(amr.transfer_command_id == cmd.command_id for amr in self.amrs)):
            return
        self.transfer_queue.append(cmd)

    def step(self, dt: float, sim_time: float) -> None:
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("dt must be finite and nonnegative")
        self._dispatch_amrs()
        for amr in self.amrs:
            if amr.awaiting_pickup:
                self._step_awaiting_pickup(amr, dt)
            elif amr.busy:
                self._step_amr(amr, dt)
        super().step(dt, sim_time)

    def _source_ready(self, cmd: MaterialTransferCommand) -> bool:
        if cmd.from_cell == self.cell_id:
            return cmd.part_id in self.smart_rack
        source = self.cell_registry.get(cmd.from_cell)
        return bool(source and source.output_buffer.occupied and source.output_buffer.part_id == cmd.part_id)

    @staticmethod
    def _has_receiver(target) -> bool:
        return target is not None and (
            callable(getattr(target, "receive_part_with_quality", None))
            or callable(getattr(target, "receive_part", None)))

    def _target_available(self, cmd: MaterialTransferCommand) -> bool:
        target = self.cell_registry.get(cmd.to_cell)
        buffer = getattr(target, "input_buffer", None)
        return self._has_receiver(target) and not (buffer is not None and buffer.occupied)

    def _dispatch_amrs(self) -> None:
        self._ensure_geometry()
        if self.aisle_owner is not None or not self.amrs:
            return
        # Inspect the whole queue. A full destination must not head-of-line block
        # the output transfer that will free that destination.
        for _ in range(len(self.transfer_queue)):
            cmd = self.transfer_queue.pop(0)
            for offset in range(len(self.amrs)):
                index = (self._next_amr_index + offset) % len(self.amrs)
                amr = self.amrs[index]
                if not amr.busy:
                    if self._assign_amr(amr, cmd):
                        self._next_amr_index = (index + 1) % len(self.amrs)
                        return
                    break

    def _assign_amr(self, amr: AMR, cmd: MaterialTransferCommand) -> bool:
        self._ensure_geometry()
        if (amr.busy or self.aisle_owner is not None or not self._target_available(cmd)
                or not self._source_ready(cmd) or cmd.from_cell not in self.dock_positions
                or cmd.to_cell not in self.dock_positions):
            if not any(item.command_id == cmd.command_id for item in self.transfer_queue):
                self.transfer_queue.append(cmd)
            return False
        self.aisle_owner = amr.amr_id
        self._transport_started = True
        amr.busy = True
        amr.command = cmd
        amr.from_cell, amr.target_cell = cmd.from_cell, cmd.to_cell
        amr.transfer_command_id = cmd.command_id
        amr.pickup_blocked = False
        amr.blocked_reason = None
        issue = self._fleet_issue() or self._payload_issue(amr)
        if issue:
            self._block(amr, issue)
            return True
        self._set_route(amr, self.dock_positions[cmd.from_cell], "to_source")
        return True

    def _obstacles(self, amr: AMR) -> list:
        return ([obs.inflated(amr.footprint_radius + self.clearance_m)
                 for obs in self.static_obstacles] +
                [(other.position, amr.footprint_radius + other.footprint_radius + self.clearance_m)
                 for other in self.amrs if other is not amr])

    def _clearance_issue(self, amr, start, end):
        try:
            ids = blocked_obstacle_ids(start, end, self._obstacles(amr),
                                       bounds=self._route_bounds(amr),
                                       **self._boundary_kwargs(amr))
            return "swept envelope obstructed by " + ", ".join(ids) if ids else None
        except (ValueError, TypeError, OverflowError, AttributeError) as exc:
            return f"invalid navigation map: {exc}"

    def _set_route(self, amr: AMR, destination: tuple, phase: str) -> None:
        # A preferred fixed aisle may itself cross a rack. Plan in free space;
        # one fleet reservation still serializes traffic and preserves custody.
        try:
            obstacles = self._obstacles(amr)
            bounds = self._route_bounds(amr)
            route = clear_route(amr.position, destination, obstacles, bounds=bounds, **self._boundary_kwargs(amr))
            if route is None:
                ids = blocked_obstacle_ids(amr.position, destination, obstacles, bounds=bounds, **self._boundary_kwargs(amr))
                self._block(amr, "no footprint-clear route; obstacles=" + ", ".join(ids or ["disconnected free space"]))
                return
        except (ValueError, TypeError, OverflowError, AttributeError) as exc:
            self._block(amr, f"invalid navigation map: {exc}")
            return
        amr.phase = phase
        amr.waypoints = route
        amr.total_distance_m = amr.remaining_distance_m = sum(
            math.dist(a, b) for a, b in zip([amr.position] + route, route))

    def _block(self, amr: AMR, reason: str) -> None:
        amr.blocked_reason = reason
        amr.phase = "blocked"
        amr.busy = True
        if amr.awaiting_pickup:
            amr.pickup_blocked = True
        logger.error("[%s] %s transport blocked: %s", self.cell_id, amr.amr_id, reason)

    def _hold_dock(self, amr: AMR, cell_id: str) -> None:
        owner = self.dock_occupancy.get(cell_id)
        if owner is not None and owner != amr.amr_id:
            self._block(amr, "dock already occupied")
            return
        self.dock_occupancy[cell_id] = amr.amr_id
        amr.held_docks.add(cell_id)

    def _release_departed_docks(self, amr: AMR) -> None:
        for cell_id in list(amr.held_docks):
            # Keep the whole dock exclusion disk reserved through departure.
            if math.dist(amr.position, self.dock_positions[cell_id]) >= 2 * amr.footprint_radius + self.clearance_m:
                self.dock_occupancy.pop(cell_id, None)
                amr.held_docks.remove(cell_id)

    def _step_amr(self, amr: AMR, dt: float) -> None:
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("dt must be finite and nonnegative")
        if not amr.busy or amr.awaiting_pickup or amr.phase == "blocked":
            return
        if self.aisle_owner != amr.amr_id:
            self._block(amr, "missing shared-aisle reservation")
            return
        issue = self._fleet_issue() or self._payload_issue(amr)
        if issue:
            self._block(amr, issue)
            return
        try:
            valid_route = all(len(point) == 2 and all(math.isfinite(v) for v in point)
                              for point in amr.waypoints)
        except (TypeError, ValueError, OverflowError):
            valid_route = False
        if not valid_route:
            self._block(amr, "invalid route waypoint")
            return
        remaining_time = dt
        # A step may finish multiple legs, but never consumes more time/distance
        # than available. Pickup confirmation always creates an event boundary.
        while amr.busy and not amr.awaiting_pickup and amr.phase != "blocked":
            # Source/target callbacks may run between route legs in this tick;
            # recheck before each movement as well as on entry.
            issue = self._fleet_issue() or self._payload_issue(amr)
            if issue:
                self._block(amr, issue)
                return
            issue = self._clearance_issue(amr, amr.position, amr.position)
            if issue:
                self._block(amr, issue)
                return
            if not amr.waypoints:
                if not self._arrive(amr):
                    return
                continue
            waypoint = amr.waypoints[0]
            distance = math.dist(amr.position, waypoint)
            if not math.isfinite(distance):
                self._block(amr, "invalid route distance")
                return
            if distance <= 1e-9:
                amr.waypoints.pop(0)
                continue
            if remaining_time <= 1e-12:
                return
            issue = self._clearance_issue(amr, amr.position, waypoint)
            if issue:
                self._block(amr, issue)
                return
            desired = math.atan2(waypoint[1] - amr.position[1], waypoint[0] - amr.position[0])
            angle = (desired - amr.heading + math.pi) % (2 * math.pi) - math.pi
            turn_time = abs(angle) / amr.turn_speed_radps
            if turn_time > 1e-9:
                consumed = min(remaining_time, turn_time)
                amr.heading += math.copysign(consumed * amr.turn_speed_radps, angle)
                remaining_time -= consumed
                if consumed < turn_time - 1e-9:
                    return
            # Rotation uses the same circumscribed disk. Validate the full
            # translation segment, not only this tick's endpoint.
            travel = min(distance, amr.speed_mps * remaining_time)
            ratio = travel / distance
            amr.position = (amr.position[0] + (waypoint[0] - amr.position[0]) * ratio,
                            amr.position[1] + (waypoint[1] - amr.position[1]) * ratio)
            amr.remaining_distance_m = max(0.0, amr.remaining_distance_m - travel)
            remaining_time -= travel / amr.speed_mps
            self._release_departed_docks(amr)
            if travel >= distance - 1e-9:
                amr.position = waypoint
                amr.waypoints.pop(0)
            else:
                return

    def _arrive(self, amr: AMR) -> bool:
        if amr.phase == "to_source":
            self._hold_dock(amr, amr.from_cell)
            if amr.phase == "blocked":
                return False
            cmd = amr.command
            if not self._source_ready(cmd):
                self._block(amr, "reserved source part no longer available")
                return False
            quality = None
            if cmd.from_cell == self.cell_id:
                del self.smart_rack[cmd.part_id]
                taken = cmd.part_id
            else:
                source = self.cell_registry[cmd.from_cell]
                source_quality = getattr(source, "quality", None)
                quality = replace(source_quality) if isinstance(source_quality, QualityPrediction) else None
                taken = source.takeout_part()
            if taken != cmd.part_id:
                self._block(amr, "source pickup identity mismatch")
                return False
            amr.payload_part_id, amr.payload_quality = taken, quality
            self.in_transit[taken] = amr
            self._set_route(amr, self.dock_positions[cmd.to_cell], "to_destination")
            return True
        if amr.phase == "to_destination":
            self._hold_dock(amr, amr.target_cell)
            if amr.phase == "blocked":
                return False
            return self._deliver(amr)
        if amr.phase == "returning":
            self._release_departed_docks(amr)
            if amr.held_docks:
                self._block(amr, "parking bay does not clear dock footprint")
                return False
            amr.busy = False
            amr.phase = "parked"
            amr.command = None
            amr.from_cell = amr.target_cell = amr.transfer_command_id = None
            self.aisle_owner = None
            return False
        return False

    def _deliver(self, amr: AMR) -> bool:
        target = self.cell_registry.get(amr.target_cell)
        if not self._has_receiver(target):
            self._block(amr, "destination receiver unavailable")
            return False                     # never discard a load at an unknown dock
        receive_with_quality = getattr(target, "receive_part_with_quality", None)
        ok = (receive_with_quality(amr.payload_part_id, amr.payload_quality)
              if callable(receive_with_quality) else target.receive_part(amr.payload_part_id))
        if not ok:
            return False                     # keep load and dock; retry in place
        amr.last_delivered_part_id = amr.payload_part_id
        amr.last_delivered_to = amr.target_cell
        amr.last_delivered_command_id = amr.transfer_command_id
        self.log_kpi("amr_delivery_count", 1.0, tags={"amr_id": amr.amr_id})
        if self.require_pickup_confirmation:
            amr.awaiting_pickup = True
            amr.phase = "awaiting_pickup"
            amr.pickup_wait_elapsed = 0.0
            mark_pending = getattr(target, "mark_placement_pending", None)
            if mark_pending is not None and not mark_pending(amr.payload_part_id, amr.transfer_command_id):
                amr.pickup_blocked = True
                amr.blocked_reason = "target refused placement reservation"
            return False
        return self._finish_pickup(amr)

    def _finish_pickup(self, amr: AMR) -> bool:
        issue = self._payload_issue(amr)
        if issue:
            self._block(amr, issue)
            return False
        self.in_transit.pop(amr.payload_part_id, None)
        self._completed_commands.add(amr.transfer_command_id)
        amr.payload_part_id = None
        amr.payload_quality = None
        amr.awaiting_pickup = False
        amr.pickup_wait_elapsed = 0.0
        amr.pickup_blocked = False
        amr.blocked_reason = None
        # Do not release the aisle/dock here: the footprint is still there.
        self._set_route(amr, amr.parking_position, "returning")
        return True

    def _step_awaiting_pickup(self, amr: AMR, dt: float) -> None:
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("dt must be finite and nonnegative")
        if not amr.awaiting_pickup:
            return
        issue = self._payload_issue(amr)
        if issue:
            self._block(amr, issue)
            return
        amr.pickup_wait_elapsed += dt
        if amr.pickup_wait_elapsed >= self.max_pickup_wait_sec and not amr.pickup_blocked:
            amr.pickup_blocked = True
            amr.blocked_reason = "pickup confirmation timeout"
            target = self.cell_registry.get(amr.target_cell)
            fail_placement = getattr(target, "confirm_placement", None)
            if fail_placement is not None:
                fail_placement(amr.payload_part_id, amr.transfer_command_id, success=False)
            logger.error("[%s] %s pickup confirmation timeout; load and dock retained",
                         self.cell_id, amr.amr_id)

    def confirm_pickup(self, amr_id: str, part_id: Optional[str] = None,
                       transfer_command_id: Optional[str] = None) -> bool:
        """Release physical custody only for this exact, successful placement.

        A timeout/failure needs explicit recovery; a late success callback is
        rejected rather than silently resuming a faulted process.
        """
        for amr in self.amrs:
            if (amr.amr_id == amr_id and amr.awaiting_pickup and not amr.pickup_blocked
                    and part_id is not None and part_id == amr.payload_part_id
                    and transfer_command_id is not None and transfer_command_id == amr.transfer_command_id):
                issue = self._payload_issue(amr)
                if issue:
                    self._block(amr, issue)
                    return False
                target = self.cell_registry.get(amr.target_cell)
                confirm_placement = getattr(target, "confirm_placement", None)
                if confirm_placement is not None and not confirm_placement(
                        part_id, transfer_command_id, success=True):
                    return False
                return self._finish_pickup(amr)
        return False

    def step_processing(self, dt: float) -> None:  # pragma: no cover
        pass
