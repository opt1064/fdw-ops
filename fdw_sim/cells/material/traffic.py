"""Conservative planar AMR geometry, independent of Isaac Sim.

A circumscribed disk covers each rectangular chassis at every heading, including
in-place turns. Segment checks therefore cover the entire swept footprint, not
just sampled center positions. This model handles AMR-to-AMR clearance only;
static equipment/fence clearance needs a surveyed navigation map.
"""
from __future__ import annotations

import heapq
import math
from typing import Iterable

Point = tuple[float, float]


def point_segment_distance(point: Point, start: Point, end: Point) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length2 = dx * dx + dy * dy
    if length2 <= 1e-18:
        return math.dist(point, start)
    t = max(0.0, min(1.0, ((point[0] - start[0]) * dx +
                          (point[1] - start[1]) * dy) / length2))
    return math.dist(point, (start[0] + t * dx, start[1] + t * dy))


def segment_is_clear(start: Point, end: Point,
                     obstacles: Iterable[tuple[Point, float]]) -> bool:
    return all(point_segment_distance(center, start, end) >= radius - 1e-9
               for center, radius in obstacles)


def clear_route(start: Point, end: Point,
                obstacles: list[tuple[Point, float]]) -> list[Point] | None:
    """Find a deterministic visibility route around inflated parked vehicles.

    Corners lie outside each exclusion disk. Every candidate edge is checked
    against every disk, so even a very large simulation timestep cannot tunnel
    through a parked vehicle. Return waypoints excluding the starting position.
    """
    if not segment_is_clear(start, start, obstacles) or not segment_is_clear(end, end, obstacles):
        return None
    if segment_is_clear(start, end, obstacles):
        return [end] if math.dist(start, end) > 1e-9 else []
    nodes = [start, end]
    for center, radius in obstacles:
        offset = radius + 1e-4
        for xsign, ysign in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
            point = (center[0] + xsign * offset, center[1] + ysign * offset)
            if segment_is_clear(point, point, obstacles):
                nodes.append(point)
    queue = [(0.0, 0)]
    distance = {0: 0.0}
    previous = {}
    while queue:
        cost, current = heapq.heappop(queue)
        if cost > distance[current] + 1e-9:
            continue
        if current == 1:
            route = []
            while current:
                route.append(nodes[current])
                current = previous[current]
            return list(reversed(route))
        for neighbor, point in enumerate(nodes):
            if neighbor == current or not segment_is_clear(nodes[current], point, obstacles):
                continue
            candidate = cost + math.dist(nodes[current], point)
            if candidate < distance.get(neighbor, math.inf) - 1e-9:
                distance[neighbor] = candidate
                previous[neighbor] = current
                heapq.heappush(queue, (candidate, neighbor))
    return None
