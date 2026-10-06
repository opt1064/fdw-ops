"""Conservative planar AMR geometry, independent of Isaac Sim.

An exclusion disk covers a rotating vehicle (and its carried load). Expanding
fixed geometry's XY boxes by that disk's radius conservatively covers the whole
swept footprint, including turns and corner crossings. Every route edge uses an
analytic intersection check; checking or sampling only its endpoints is unsafe.
"""
from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import heapq
import math
from typing import Iterable

Point = tuple[float, float]
Bounds = tuple[float, float, float, float]  # xmin, xmax, ymin, ymax
# A malformed/unbounded survey must fail explicitly rather than constructing an
# arbitrarily expensive all-pairs graph. Group scene meshes into equipment boxes.
MAX_ROUTE_OBSTACLES = 256
MAX_ROUTE_NODES = 4 * MAX_ROUTE_OBSTACLES + 2
MAX_BOUNDARY_VERTICES = 256
_CORNER_PADDING = 1e-4
_DISK_TOLERANCE = 1e-9  # Preserve the existing parked-AMR disk contact semantics.


def _finite_number(value, description: str) -> float:
    try:
        if isinstance(value, (bool, str, bytes)):
            raise ValueError
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{description} must be finite numeric geometry") from exc
    if not math.isfinite(result):
        raise ValueError(f"{description} must be finite numeric geometry")
    return result


def _point(value, description: str) -> Point:
    try:
        if len(value) != 2:
            raise ValueError
        return (_finite_number(value[0], description),
                _finite_number(value[1], description))
    except (TypeError, IndexError, ValueError) as exc:
        raise ValueError(f"{description} must be finite XY coordinates") from exc


def _bounds(value, description: str) -> Bounds:
    try:
        if len(value) != 4:
            raise ValueError
        result = tuple(_finite_number(v, description) for v in value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{description} must contain four finite bounds") from exc
    xmin, xmax, ymin, ymax = result
    if (xmin >= xmax or ymin >= ymax
            or not math.isfinite(xmax - xmin) or not math.isfinite(ymax - ymin)):
        raise ValueError(f"{description} must have finite positive width and height")
    return result


@dataclass(frozen=True)
class StaticObstacle:
    """A fixed world-XY AABB, or its already-inflated center exclusion box.

    Negative coordinates are valid; width and height must be strictly positive.
    The ID is retained in blocked-route diagnostics and through inflation.
    """

    obstacle_id: str
    xmin: float
    xmax: float
    ymin: float
    ymax: float

    def __post_init__(self) -> None:
        if not isinstance(self.obstacle_id, str) or not self.obstacle_id.strip():
            raise ValueError("static obstacle requires a nonempty obstacle_id")
        values = _bounds((self.xmin, self.xmax, self.ymin, self.ymax),
                         f"static obstacle {self.obstacle_id}")
        for name, value in zip(("xmin", "xmax", "ymin", "ymax"), values):
            object.__setattr__(self, name, value)

    @property
    def bounds(self) -> Bounds:
        return self.xmin, self.xmax, self.ymin, self.ymax

    def inflated(self, radius: float) -> StaticObstacle:
        """Conservatively exclude a circular swept envelope around this box.

        The box expansion also excludes the rounded Minkowski-sum corners, so
        a circumscribed vehicle/load disk cannot clip a rack while turning.
        """
        radius = _finite_number(radius, "inflation radius")
        if radius < 0:
            raise ValueError("inflation radius must be nonnegative")
        return StaticObstacle(self.obstacle_id, self.xmin - radius,
                              self.xmax + radius, self.ymin - radius,
                              self.ymax + radius)


Obstacle = StaticObstacle | tuple[Point, float]


@dataclass(frozen=True)
class _Exclusion:
    obstacle_id: str
    bounds: Bounds
    disk: tuple[Point, float] | None = None


def _exclusions(obstacles: Iterable[Obstacle]) -> tuple[_Exclusion, ...]:
    """Validate the *entire* map before allowing any clear/early-exit result."""
    result = []
    try:
        iterator = iter(obstacles)
    except TypeError as exc:
        raise ValueError("obstacles must be an iterable of boxes or disks") from exc
    for index, obstacle in enumerate(iterator):
        if index >= MAX_ROUTE_OBSTACLES:
            raise ValueError(f"navigation map exceeds {MAX_ROUTE_OBSTACLES} obstacle limit")
        if isinstance(obstacle, StaticObstacle):
            result.append(_Exclusion(obstacle.obstacle_id, obstacle.bounds))
            continue
        try:
            center, radius = obstacle
            center = _point(center, f"disk:{index} center")
            radius = _finite_number(radius, f"disk:{index} radius")
            if radius < 0:
                raise ValueError("disk radius must be nonnegative")
        except (TypeError, ValueError) as exc:
            raise ValueError(f"invalid obstacle at index {index}: {exc}") from exc
        disk_bounds = (center[0] - radius, center[0] + radius,
                       center[1] - radius, center[1] + radius)
        if not all(math.isfinite(value) for value in disk_bounds):
            raise ValueError(f"disk:{index} bounds must be finite")
        result.append(_Exclusion(f"disk:{index}", disk_bounds, (center, radius)))
    return tuple(result)


def _segment_delta(start: Point, end: Point) -> tuple[float, float, float]:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    if not math.isfinite(length):
        raise ValueError("segment length exceeds finite navigation geometry")
    return dx, dy, length


def _point_segment_distance(point: Point, start: Point, end: Point) -> float:
    dx, dy, length = _segment_delta(start, end)
    if length <= 1e-9:
        return math.dist(point, start)
    # Normalize before multiplying to avoid the squared-length overflow of the
    # usual dot/dot formula. Finite inputs can still overflow on subtraction.
    px, py = point[0] - start[0], point[1] - start[1]
    if not math.isfinite(px) or not math.isfinite(py):
        raise ValueError("point separation exceeds finite navigation geometry")
    t = max(0.0, min(1.0, (px / length) * (dx / length)
                          + (py / length) * (dy / length)))
    return math.dist(point, (start[0] + t * dx, start[1] + t * dy))


def point_segment_distance(point: Point, start: Point, end: Point) -> float:
    return _point_segment_distance(_point(point, "point"),
                                   _point(start, "start"), _point(end, "end"))


def _inside_bounds(point: Point, bounds: Bounds) -> bool:
    # bounds describe the usable *center* region, already inset by the caller's
    # envelope. Touching a hard map boundary is excluded like obstacle contact.
    xmin, xmax, ymin, ymax = bounds
    return xmin < point[0] < xmax and ymin < point[1] < ymax


@dataclass(frozen=True)
class _BoundaryPlane:
    origin: Point
    inward_normal: Point

    def distance(self, point: Point) -> float:
        dx, dy = point[0] - self.origin[0], point[1] - self.origin[1]
        distance = dx * self.inward_normal[0] + dy * self.inward_normal[1]
        if not math.isfinite(distance):
            raise ValueError("convex boundary distance exceeds finite geometry")
        return distance

    def contains(self, point: Point, margin: float) -> bool:
        # A transformed point on a wall can acquire a tiny positive distance
        # through subtraction/normalization roundoff. Reserve a few coordinate
        # ULPs so closed contact cannot become an accidentally clear endpoint.
        tolerance = 16 * max(math.ulp(value) for value in (*self.origin, *point, margin))
        return self.distance(point) > margin + tolerance


@dataclass(frozen=True)
class _ConvexBoundary:
    planes: tuple[_BoundaryPlane, ...]
    margin: float

    def contains(self, point: Point) -> bool:
        # The center must be more than one envelope radius from *every* wall.
        # Every half-plane is convex, so this also covers the full swept edge.
        return all(plane.contains(point, self.margin) for plane in self.planes)


def _convex_boundary(vertices: Iterable[Point] | None, margin: float) -> _ConvexBoundary | None:
    margin = _finite_number(margin, "boundary margin")
    if margin < 0:
        raise ValueError("boundary margin must be nonnegative")
    if vertices is None:
        return None
    points = []
    try:
        for index, vertex in enumerate(vertices):
            if index > MAX_BOUNDARY_VERTICES:
                raise ValueError(f"convex boundary exceeds {MAX_BOUNDARY_VERTICES} vertex limit")
            points.append(_point(vertex, "convex boundary vertex"))
    except TypeError as exc:
        raise ValueError("convex boundary must be an iterable of finite XY vertices") from exc
    # Both common polygon conventions are accepted: an unclosed vertex ring or
    # one terminal copy of its first point. Other duplicate vertices are invalid.
    if len(points) > 1 and points[-1] == points[0]:
        points.pop()
    if not 3 <= len(points) <= MAX_BOUNDARY_VERTICES:
        raise ValueError(f"convex boundary requires 3..{MAX_BOUNDARY_VERTICES} vertices")
    if len(set(points)) != len(points):
        raise ValueError("convex boundary has duplicate vertices")

    def plane(first, second):
        dx, dy, length = _segment_delta(first, second)
        if length == 0:
            raise ValueError("convex boundary has a zero-length edge")
        return _BoundaryPlane(first, (-dy / length, dx / length))

    direction = plane(points[0], points[1]).distance(points[2])
    if direction < 0:
        points.reverse()
    planes = []
    for index, first in enumerate(points):
        next_index = (index + 1) % len(points)
        edge = plane(first, points[next_index])
        # Testing every other vertex rules out self-intersecting stars as well
        # as concavity, collinear vertices, and backtracking. Local turn signs
        # alone are insufficient to establish a valid convex boundary.
        if any(edge.distance(point) <= 0 for other, point in enumerate(points)
               if other not in (index, next_index)):
            raise ValueError("convex boundary must be strictly convex and non-self-intersecting")
        planes.append(edge)
    return _ConvexBoundary(tuple(planes), margin)


def _inside_region(point: Point, bounds: Bounds | None,
                   boundary: _ConvexBoundary | None) -> bool:
    return ((bounds is None or _inside_bounds(point, bounds))
            and (boundary is None or boundary.contains(point)))


def _segment_hits_box(start: Point, end: Point, bounds: Bounds) -> bool:
    """Closed AABB slab intersection, including corner/edge contact."""
    entry, exit_ = 0.0, 1.0
    for origin, destination, low, high in (
            (start[0], end[0], bounds[0], bounds[1]),
            (start[1], end[1], bounds[2], bounds[3])):
        delta = destination - origin
        if delta == 0:
            if origin < low or origin > high:
                return False
            continue
        first, last = (low - origin) / delta, (high - origin) / delta
        if first > last:
            first, last = last, first
        entry, exit_ = max(entry, first), min(exit_, last)
        if entry > exit_:
            return False
    return True


def _hits(start: Point, end: Point, obstacle: _Exclusion) -> bool:
    if not _segment_hits_box(start, end, obstacle.bounds):
        return False
    if obstacle.disk is None:
        return True
    center, radius = obstacle.disk
    return _point_segment_distance(center, start, end) < radius - _DISK_TOLERANCE


class _Visibility:
    """Per-route immutable obstacle index; exact predicates after broad phase."""

    def __init__(self, obstacles: tuple[_Exclusion, ...], bounds: Bounds | None,
                 boundary: _ConvexBoundary | None):
        self.bounds = bounds
        self.boundary = boundary
        self.obstacles = sorted(obstacles, key=lambda item: (item.bounds[0], item.obstacle_id))
        self.xmins = [item.bounds[0] for item in self.obstacles]

    def is_clear(self, start: Point, end: Point) -> bool:
        if not (_inside_region(start, self.bounds, self.boundary)
                and _inside_region(end, self.bounds, self.boundary)):
            return False
        xmin, xmax = min(start[0], end[0]), max(start[0], end[0])
        ymin, ymax = min(start[1], end[1]), max(start[1], end[1])
        # A rectangle is convex, so in-bounds endpoints imply the full edge is
        # in bounds. The same XY broad phase prunes most equipment groups.
        for index in range(bisect_right(self.xmins, xmax)):
            obstacle = self.obstacles[index]
            box = obstacle.bounds
            if box[1] < xmin or box[3] < ymin or box[2] > ymax:
                continue
            if _hits(start, end, obstacle):
                return False
        return True


def _validated_geometry(start: Point, end: Point, obstacles: Iterable[Obstacle],
                        bounds: Bounds | None, convex_boundary: Iterable[Point] | None,
                        boundary_margin: float):
    start, end = _point(start, "start"), _point(end, "end")
    _segment_delta(start, end)
    exclusions = _exclusions(obstacles)
    region = None if bounds is None else _bounds(bounds, "navigation bounds")
    boundary = _convex_boundary(convex_boundary, boundary_margin)
    return start, end, exclusions, region, boundary


def segment_is_clear(start: Point, end: Point, obstacles: Iterable[Obstacle], *,
                     bounds: Bounds | None = None,
                     convex_boundary: Iterable[Point] | None = None,
                     boundary_margin: float = 0.0) -> bool:
    """Check a whole swept center segment against boxes and legacy disks.

    Boxes must already be inflated by the vehicle/load envelope; disk radii
    likewise represent the combined exclusion radius. Invalid data raises
    ValueError, and a valid but obstructed/out-of-bounds segment returns False.
    A convex_boundary is a raw factory-floor polygon in either winding order;
    boundary_margin insets every edge by the vehicle/load radius and clearance.
    """
    start, end, exclusions, region, boundary = _validated_geometry(
        start, end, obstacles, bounds, convex_boundary, boundary_margin)
    return _Visibility(exclusions, region, boundary).is_clear(start, end)


def blocked_obstacle_ids(start: Point, end: Point, obstacles: Iterable[Obstacle], *,
                         bounds: Bounds | None = None,
                         convex_boundary: Iterable[Point] | None = None,
                         boundary_margin: float = 0.0) -> list[str]:
    """Return blocker IDs in map order (legacy disks use ``disk:<index>``)."""
    start, end, exclusions, region, boundary = _validated_geometry(
        start, end, obstacles, bounds, convex_boundary, boundary_margin)
    blocked = []
    if not (_inside_region(start, region, boundary) and _inside_region(end, region, boundary)):
        blocked.append("map_boundary")
    for obstacle in exclusions:
        if _hits(start, end, obstacle) and obstacle.obstacle_id not in blocked:
            blocked.append(obstacle.obstacle_id)
    return blocked


def _outside(value: float, sign: int) -> float:
    # Ensure padding remains representable even for large surveyed coordinates.
    offset = max(_CORNER_PADDING, 4 * math.ulp(value))
    result = value + sign * offset
    if not math.isfinite(result):
        raise ValueError("obstacle corner exceeds finite navigation geometry")
    return result


def clear_route(start: Point, end: Point, obstacles: Iterable[Obstacle], *,
                bounds: Bounds | None = None,
                convex_boundary: Iterable[Point] | None = None,
                boundary_margin: float = 0.0) -> list[Point] | None:
    """Find a deterministic, bounded visibility route; exclude the start point.

    Fixed boxes must already be inflated by the full rotating vehicle/load
    radius. Vertices sit strictly outside box and disk-bounding-square corners;
    every edge is analytically checked against the entire mixed obstacle map.
    A Euclidean A* search and cached symmetric edge checks avoid repeatedly
    testing all node pairs. Invalid or oversized maps raise ValueError; a
    blocked endpoint or disconnected map returns None, never a straight fallback.
    Optional bounds are the already-inset rectangular navigable center region.
    Optional convex_boundary is the raw (possibly rotated) floor polygon, with
    boundary_margin applied as a perpendicular inset of every boundary edge.
    """
    start, end, exclusions, region, boundary = _validated_geometry(
        start, end, obstacles, bounds, convex_boundary, boundary_margin)
    visibility = _Visibility(exclusions, region, boundary)
    if not visibility.is_clear(start, start) or not visibility.is_clear(end, end):
        return None
    if visibility.is_clear(start, end):
        return [end] if math.dist(start, end) > 1e-9 else []

    nodes = [start, end]
    seen = {start, end}
    # Canonical box order makes route geometry deterministic independently of
    # input map ordering. Equal-cost alternatives use stable vertex indices.
    for obstacle in sorted(exclusions, key=lambda item: (item.bounds, item.obstacle_id)):
        xmin, xmax, ymin, ymax = obstacle.bounds
        for x, y in ((_outside(xmin, -1), _outside(ymin, -1)),
                     (_outside(xmin, -1), _outside(ymax, 1)),
                     (_outside(xmax, 1), _outside(ymin, -1)),
                     (_outside(xmax, 1), _outside(ymax, 1))):
            point = (x, y)
            if point not in seen and visibility.is_clear(point, point):
                nodes.append(point)
                seen.add(point)
                if len(nodes) > MAX_ROUTE_NODES:
                    raise ValueError(f"navigation graph exceeds {MAX_ROUTE_NODES} node limit")

    heuristic = [math.dist(point, end) for point in nodes]
    if not all(math.isfinite(value) for value in heuristic):
        raise ValueError("navigation graph exceeds finite distances")
    queue = [(heuristic[0], 0.0, 0)]
    distance = {0: 0.0}
    previous = {}
    edge_clear = {}
    settled = set()
    while queue:
        _, cost, current = heapq.heappop(queue)
        if current in settled or cost > distance[current] + 1e-9:
            continue
        if current == 1:
            route = []
            while current:
                route.append(nodes[current])
                current = previous[current]
            return list(reversed(route))
        settled.add(current)
        for neighbor, point in enumerate(nodes):
            if neighbor in settled:
                continue
            candidate = cost + math.dist(nodes[current], point)
            if candidate >= distance.get(neighbor, math.inf) - 1e-9:
                continue
            key = (min(current, neighbor), max(current, neighbor))
            if key not in edge_clear:
                edge_clear[key] = visibility.is_clear(nodes[current], point)
            if not edge_clear[key]:
                continue
            if not math.isfinite(candidate):
                raise ValueError("navigation route exceeds finite distance")
            distance[neighbor] = candidate
            previous[neighbor] = current
            heapq.heappush(queue, (candidate + heuristic[neighbor], candidate, neighbor))
    return None
