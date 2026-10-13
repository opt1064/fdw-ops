"""CPU-only static-map sweep and visibility routing regressions.

The rack fixtures match SceneBuilder.add_steel_shelving geometry, including the
0.02 m support overhang. These checks require neither USD nor an Isaac process.
"""
from __future__ import annotations

from dataclasses import FrozenInstanceError
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fdw_sim.cells.material.traffic import (
    MAX_ROUTE_OBSTACLES,
    StaticObstacle,
    blocked_obstacle_ids,
    clear_route,
    point_segment_distance,
    segment_is_clear,
)


def rack_bank():
    return [StaticObstacle(f"MaterialRack_{index}", 3.98, 12.02,
                           2.98 + 2 * index, 4.02 + 2 * index)
            for index in range(3)]


def assert_clear_route(start, end, route, obstacles, *, bounds=None):
    assert route is not None
    assert route[-1] == end
    for first, second in zip([start] + route, route):
        assert segment_is_clear(first, second, obstacles, bounds=bounds)
        assert segment_is_clear(second, first, obstacles, bounds=bounds)
        assert blocked_obstacle_ids(first, second, obstacles, bounds=bounds) == []


def test_static_obstacle_is_immutable_and_inflates_every_side():
    rack = StaticObstacle("rack", -4, 2, -1, 3)
    assert rack.bounds == (-4.0, 2.0, -1.0, 3.0)
    assert rack.inflated(.75) == StaticObstacle("rack", -4.75, 2.75, -1.75, 3.75)
    assert rack.inflated(0) == rack
    with pytest.raises(FrozenInstanceError):
        rack.xmin = -5


@pytest.mark.parametrize("start,end", [
    ((-10, 0), (10, 0)),       # Endpoint-only validation would miss this sweep.
    ((-10, 1), (10, 1)),       # Collinear edge contact.
    ((-2, 0), (0, 2)),         # A single corner contact.
    ((1, -2), (1, 2)),         # Vertical edge contact / parallel slab.
    ((0, 0), (0, 0)),          # Already inside / in-place turn.
    ((-1, 1), (-1, 1)),        # Corner contact with zero translation.
    ((-2, 0), (-1, 0)),        # Endpoint contact.
])
def test_closed_static_box_blocks_the_entire_sweep_and_contacts(start, end):
    obstacles = [StaticObstacle("rack", -1, 1, -1, 1)]
    assert not segment_is_clear(start, end, obstacles)
    assert not segment_is_clear(end, start, obstacles)
    assert blocked_obstacle_ids(start, end, obstacles) == ["rack"]


@pytest.mark.parametrize("start,end", [
    ((-10, 1.00001), (10, 1.00001)),
    ((1.00001, -10), (1.00001, 10)),
    ((-2, 0), (0, 2.0001)),
    ((2, 0), (2, 0)),
    ((-2, 2), (-1.00001, 2)),
])
def test_near_misses_and_zero_translation_outside_are_clear(start, end):
    assert segment_is_clear(start, end, [StaticObstacle("rack", -1, 1, -1, 1)])


def test_exact_three_rack_survey_routes_around_bank_and_not_through_gaps():
    # The historical straight lane crosses the bottom rack's centerline.
    radius = math.hypot(1.2, .8) / 2 + .15
    obstacles = [rack.inflated(radius) for rack in rack_bank()]
    start, end = (2.0, 3.5), (14.0, 3.5)
    assert blocked_obstacle_ids(start, end, obstacles) == ["MaterialRack_0"]
    route = clear_route(start, end, obstacles, bounds=(0, 20, 0, 14))
    assert len(route) > 1
    assert any(point[1] < 2.98 - radius for point in route)
    assert_clear_route(start, end, route, obstacles, bounds=(0, 20, 0, 14))
    # The 0.96 m apparent rack gap cannot fit a 1.44 m rotating chassis.
    assert clear_route((8, 4.5), (14, 4.5), obstacles) is None


def test_rectangular_load_rotation_envelope_requires_a_wider_detour():
    empty_radius = math.hypot(1.2, .8) / 2 + .05
    loaded_radius = max(empty_radius, math.hypot(2.4, .8) / 2 + .05)
    lower = StaticObstacle("lower_rack", -2, 2, -4, -1)
    upper = StaticObstacle("upper_rack", -2, 2, 1, 4)
    empty_obstacles = [rack.inflated(empty_radius) for rack in (lower, upper)]
    loaded_obstacles = [rack.inflated(loaded_radius) for rack in (lower, upper)]
    start, end = (-4.0, 0.0), (4.0, 0.0)
    assert clear_route(start, end, empty_obstacles) == [end]
    assert not segment_is_clear(start, end, loaded_obstacles)
    loaded_route = clear_route(start, end, loaded_obstacles)
    assert len(loaded_route) > 1
    assert_clear_route(start, end, loaded_route, loaded_obstacles)
    # That load route becomes impossible if the outer walls close the detour.
    assert clear_route(start, end, loaded_obstacles, bounds=(-5, 5, -4.5, 4.5)) is None


def test_inflated_box_covers_corner_sweep_and_every_in_place_heading():
    rack = StaticObstacle("rack", 0, 1, 0, 1)
    radius = math.hypot(1.6, .8) / 2
    obstacle = rack.inflated(radius)
    # Both coordinates outside the raw rack still enter its conservative corner
    # exclusion, protecting the long chassis corner during an in-place turn.
    corner = (-radius / 2, -radius / 2)
    assert segment_is_clear(corner, corner, [rack])
    assert not segment_is_clear(corner, corner, [obstacle])
    safe_center = (-radius - .001, -radius - .001)
    assert segment_is_clear(safe_center, safe_center, [obstacle])
    for step in range(360):
        theta = math.radians(step)
        for x, y in ((-.8, -.4), (-.8, .4), (.8, -.4), (.8, .4)):
            rotated = (safe_center[0] + x * math.cos(theta) - y * math.sin(theta),
                       safe_center[1] + x * math.sin(theta) + y * math.cos(theta))
            assert not (rack.xmin <= rotated[0] <= rack.xmax
                        and rack.ymin <= rotated[1] <= rack.ymax)


def test_mixed_static_and_parked_vehicle_obstacles_keep_legacy_disk_api():
    obstacles = [StaticObstacle("rack", -1, 1, -1, 1), ((3.0, 0.0), 1.2)]
    start, end = (-4.0, 0.0), (6.0, 0.0)
    assert blocked_obstacle_ids(start, end, obstacles) == ["rack", "disk:1"]
    route = clear_route(start, end, obstacles)
    assert_clear_route(start, end, route, obstacles)
    assert clear_route(start, end, list(reversed(obstacles))) == route
    disks = [((0.0, 0.0), 1.6)]
    assert segment_is_clear((-5, 1.6), (5, 1.6), disks)  # Existing tangent rule.
    assert not segment_is_clear((-5, 0), (5, 0), disks)
    disk_route = clear_route((-5, 0), (5, 0), disks)
    for first, second in zip([(-5, 0)] + disk_route, disk_route):
        assert point_segment_distance((0, 0), first, second) >= 1.6
    assert segment_is_clear((0, 0), (0, 0), [((0, 0), 0)])


def test_wall_spanning_navigable_bounds_has_no_route_or_fallback():
    obstacles = [StaticObstacle("wall", 4, 6, -1, 11)]
    start, end = (2, 5), (8, 5)
    assert clear_route(start, end, obstacles, bounds=(0, 10, 0, 10)) is None
    assert_clear_route(start, end, clear_route(start, end, obstacles), obstacles)


def test_enclosed_free_endpoint_is_unreachable_without_explicit_bounds():
    obstacles = [StaticObstacle("left", -2, -1, -2, 2),
                 StaticObstacle("right", 1, 2, -2, 2),
                 StaticObstacle("bottom", -2, 2, -2, -1),
                 StaticObstacle("top", -2, 2, 1, 2)]
    assert segment_is_clear((0, 0), (0, 0), obstacles)
    assert segment_is_clear((4, 0), (4, 0), obstacles)
    assert clear_route((0, 0), (4, 0), obstacles) is None


def test_empty_route_blocked_endpoints_and_boundary_diagnostics():
    obstacles = [StaticObstacle("rack", 0, 1, 0, 1)]
    assert clear_route((-2, 0), (-2, 0), obstacles) == []
    assert clear_route((-2, 0), (0, 0), obstacles) is None
    assert clear_route((0, 0), (-2, 0), obstacles) is None
    bounds = (-3, 3, -3, 3)
    assert not segment_is_clear((-4, 2), (2, 2), obstacles, bounds=bounds)
    assert blocked_obstacle_ids((-4, 2), (2, 2), obstacles, bounds=bounds) == ["map_boundary"]
    assert clear_route((-3, 2), (2, 2), obstacles, bounds=bounds) is None
    assert clear_route((-2, 2), (2, 2), obstacles, bounds=bounds) == [(2, 2)]


@pytest.mark.parametrize("bounds", [
    (0, 0, 0, 1), (2, 1, 0, 1), (0, 1, 1, 0),
    (0, float("nan"), 0, 1), (0, 1, float("inf"), 2),
    (-1e308, 1e308, 0, 1),
])
def test_invalid_static_or_navigation_bounds_raise(bounds):
    with pytest.raises(ValueError):
        StaticObstacle("bad", *bounds)
    with pytest.raises(ValueError):
        clear_route((0, 0), (1, 0), [], bounds=bounds)
    with pytest.raises(ValueError):
        segment_is_clear((0, 0), (1, 0), [], bounds=bounds)


@pytest.mark.parametrize("radius", [-1, float("nan"), float("inf"), "1", None])
def test_invalid_inflation_radius_cannot_disable_clearance(radius):
    with pytest.raises(ValueError):
        StaticObstacle("rack", 0, 1, 0, 1).inflated(radius)


@pytest.mark.parametrize("obstacle_id", ["", "  ", None, 42])
def test_missing_static_obstacle_identity_is_rejected(obstacle_id):
    with pytest.raises(ValueError):
        StaticObstacle(obstacle_id, 0, 1, 0, 1)


@pytest.mark.parametrize("obstacles", [
    [((float("nan"), 0), 1)], [((0, 0), float("nan"))],
    [((0, 0), float("inf"))], [((0, 0), -1)],
    [((0,), 1)], [None], None,
    # Validate even an unused malformed obstacle after a definite first blocker.
    [StaticObstacle("blocking_rack", -1, 1, -1, 1), ((0, 0), float("nan"))],
])
def test_invalid_mixed_map_never_silently_allows_or_ignores_motion(obstacles):
    for operation in (segment_is_clear, clear_route, blocked_obstacle_ids):
        with pytest.raises(ValueError):
            operation((-2, 0), (2, 0), obstacles)


@pytest.mark.parametrize("point", [(float("nan"), 0), (0, float("inf")),
                                   (None, 0), (0,), ("0", 0)])
def test_invalid_pose_never_becomes_a_clear_route(point):
    for operation in (segment_is_clear, clear_route, blocked_obstacle_ids):
        with pytest.raises(ValueError):
            operation(point, (1, 0), [])
        with pytest.raises(ValueError):
            operation((1, 0), point, [])


def test_inflation_overflow_and_excessive_maps_fail_explicitly():
    with pytest.raises(ValueError):
        StaticObstacle("far", 1e308, 1.7e308, 0, 1).inflated(1e308)
    obstacles = [StaticObstacle(f"rack-{index}", index, index + .5, 0, 1)
                 for index in range(MAX_ROUTE_OBSTACLES + 1)]
    with pytest.raises(ValueError, match="obstacle limit"):
        clear_route((-1, -1), (1, -1), obstacles)
    with pytest.raises(ValueError, match="obstacle limit"):
        segment_is_clear((-1, -1), (1, -1), obstacles)


def test_hundred_group_survey_has_a_deterministic_fully_checked_route():
    obstacles = [StaticObstacle(f"rack-{x}-{y}", x * 3, x * 3 + 1,
                                y * 3, y * 3 + 1).inflated(.5)
                 for x in range(10) for y in range(10)]
    start, end, bounds = (-2, -2), (31, 31), (-4, 33, -4, 33)
    route = clear_route(start, end, obstacles, bounds=bounds)
    assert 1 < len(route) <= 4 * len(obstacles) + 1
    assert route == clear_route(start, end, list(reversed(obstacles)), bounds=bounds)
    assert_clear_route(start, end, route, obstacles, bounds=bounds)


ROTATED_FLOOR = ((0.0, 2.0), (2.0, 0.0), (4.0, 2.0), (2.0, 4.0))


def test_rotated_floor_excludes_empty_world_aabb_corners():
    start, end = (.25, .25), (2, 2)
    assert segment_is_clear(start, end, [], bounds=(0, 4, 0, 4))
    assert not segment_is_clear(start, end, [], bounds=(0, 4, 0, 4),
                                convex_boundary=ROTATED_FLOOR)
    assert blocked_obstacle_ids(start, end, [], bounds=(0, 4, 0, 4),
                                convex_boundary=ROTATED_FLOOR) == ["map_boundary"]
    assert clear_route(start, end, [], convex_boundary=ROTATED_FLOOR) is None


@pytest.mark.parametrize("point", [(0, 2), (1, 1), (3, 3), (2, 0), (2, 4)])
def test_rotated_floor_closed_edge_and_vertex_contact_is_blocked(point):
    assert not segment_is_clear(point, (2, 2), [], convex_boundary=ROTATED_FLOOR)
    assert clear_route((2, 2), point, [], convex_boundary=ROTATED_FLOOR) is None


def test_visibility_detour_stays_inside_rotated_floor_with_swept_margin():
    obstacle = StaticObstacle("rack", 1.8, 2.2, 1.8, 2.2)
    start, end, margin = (1, 2), (3, 2), .25
    route = clear_route(start, end, [obstacle], convex_boundary=ROTATED_FLOOR,
                        boundary_margin=margin)
    assert route and len(route) > 1 and route[-1] == end
    for first, second in zip([start] + route, route):
        assert segment_is_clear(first, second, [obstacle],
                                convex_boundary=ROTATED_FLOOR, boundary_margin=margin)
        assert blocked_obstacle_ids(first, second, [obstacle],
                                    convex_boundary=ROTATED_FLOOR, boundary_margin=margin) == []
        # Independent floor-frame test, including rotating envelope circle:
        # |x - 2| + |y - 2| < 2 describes this 45-degree rotated square.
        for step in range(11):
            center = tuple(first[i] + (second[i] - first[i]) * step / 10 for i in (0, 1))
            for theta in range(0, 360, 15):
                x = center[0] + margin * math.cos(math.radians(theta))
                y = center[1] + margin * math.sin(math.radians(theta))
                assert abs(x - 2) + abs(y - 2) < 2
    assert route == clear_route(start, end, [obstacle],
                                convex_boundary=tuple(reversed(ROTATED_FLOOR)),
                                boundary_margin=margin)
    assert route == clear_route(start, end, [obstacle],
                                convex_boundary=ROTATED_FLOOR + (ROTATED_FLOOR[0],),
                                boundary_margin=margin)


def test_boundary_margin_uses_perpendicular_radius_and_closes_narrow_floor():
    near_wall, middle = (2, 1), (2, 2)
    assert segment_is_clear(near_wall, middle, [], convex_boundary=ROTATED_FLOOR,
                            boundary_margin=.7)
    assert not segment_is_clear(near_wall, middle, [], convex_boundary=ROTATED_FLOOR,
                                boundary_margin=.71)
    assert blocked_obstacle_ids(near_wall, middle, [], convex_boundary=ROTATED_FLOOR,
                                boundary_margin=.71) == ["map_boundary"]
    assert clear_route(middle, middle, [], convex_boundary=ROTATED_FLOOR,
                        boundary_margin=2) is None
    # Exact contact of the envelope with an axis-aligned boundary is blocked.
    square = ((0, 0), (4, 0), (4, 4), (0, 4))
    assert not segment_is_clear((1, 2), (2, 2), [], convex_boundary=square,
                                boundary_margin=1)
    assert segment_is_clear((1.00001, 2), (2, 2), [], convex_boundary=square,
                            boundary_margin=1)


def test_rotated_floor_prevents_escape_around_a_spanning_wall():
    obstacles = [StaticObstacle("wall", 1.8, 2.2, -1, 5)]
    assert clear_route((1, 2), (3, 2), obstacles, convex_boundary=ROTATED_FLOOR) is None


@pytest.mark.parametrize("polygon", [
    (), ((0, 0),), ((0, 0), (1, 1)),
    ((0, 0), (1, 0), (2, 0)),  # Collinear triangle.
    ((0, 0), (1, 0), (2, 0), (2, 2), (0, 2)),  # Redundant collinear vertex.
    ((0, 0), (2, 0), (1, 1), (2, 2), (0, 2)),  # Concavity.
    ((0, 0), (2, 2), (0, 2), (2, 0)),  # Bow-tie self intersection.
    ((0, 0), (2, 0), (2, 2), (2, 0), (0, 2)),  # Repeated interior vertex.
    ((0, 0), (2, 0), (float("nan"), 2), (0, 2)),
    ((0, 0), (2, 0), (2, float("inf")), (0, 2)),
    ((0, 0), (2, 0), (2,)), 7,
])
def test_invalid_convex_boundaries_fail_closed_before_direct_route(polygon):
    for operation in (segment_is_clear, clear_route, blocked_obstacle_ids):
        with pytest.raises(ValueError):
            operation((.5, .5), (1, 1), [], convex_boundary=polygon)


def test_self_intersecting_star_with_consistent_local_turns_is_rejected():
    outer = [(math.cos(2 * math.pi * index / 5), math.sin(2 * math.pi * index / 5))
             for index in range(5)]
    star = [outer[index] for index in (0, 2, 4, 1, 3)]
    with pytest.raises(ValueError, match="strictly convex"):
        clear_route((0, 0), (.1, .1), [], convex_boundary=star)


@pytest.mark.parametrize("margin", [-1, float("nan"), float("inf"), "1", None])
def test_invalid_boundary_margin_does_not_silently_disable_envelope(margin):
    for operation in (segment_is_clear, clear_route, blocked_obstacle_ids):
        with pytest.raises(ValueError):
            operation((1, 2), (2, 2), [], convex_boundary=ROTATED_FLOOR,
                      boundary_margin=margin)


def test_rotated_wall_contact_is_conservative_against_float_roundoff():
    # At 9 degrees this edge midpoint otherwise has a tiny positive signed
    # distance after floating-point transformation, despite lying on the wall.
    for angle in (9, 17, 31, 91, 217):
        theta = math.radians(angle)
        floor = tuple((12 + x * math.cos(theta) - y * math.sin(theta),
                       15 + x * math.sin(theta) + y * math.cos(theta))
                      for x, y in ((-2, -1), (2, -1), (2, 1), (-2, 1)))
        midpoint = tuple((floor[0][index] + floor[1][index]) / 2 for index in (0, 1))
        assert not segment_is_clear(midpoint, (12, 15), [], convex_boundary=floor)
        inward = (midpoint[0] + 1e-8 * -math.sin(theta),
                  midpoint[1] + 1e-8 * math.cos(theta))
        assert segment_is_clear(inward, (12, 15), [], convex_boundary=floor)
