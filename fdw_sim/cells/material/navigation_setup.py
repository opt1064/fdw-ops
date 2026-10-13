"""Bounded startup-only service-dock and parking selection in a measured map.

These are logical transfer positions, not proof of manipulator reach. No rack,
cell, or live vehicle is moved to make a route possible.
"""
from __future__ import annotations

import math
from fdw_sim.cells.material.traffic import clear_route, blocked_obstacle_ids


def select_transport_positions(docks, cell_centers, obstacles, bounds, radius,
                               clearance, num_amrs, *, max_dock_shift=4.0, boundary_polygon=None):
    """Return validated dock/bay poses; raise with obstacle IDs if none exist.

    Candidate docks are within max_dock_shift meters of their requested pose.
    Selection is deterministic and bounded. A conservative visibility route
    must connect every dock and each bay while all other vehicles are parked.
    """
    from fdw_sim.cells.material.material_cell import MaterialCell
    inflated = [obs.inflated(radius + clearance) for obs in obstacles]
    inset = MaterialCell._inset_bounds(bounds, radius + clearance)
    boundary = {"convex_boundary": boundary_polygon, "boundary_margin": radius + clearance}
    selected = {}
    for name, desired in docks.items():
        if not blocked_obstacle_ids(desired, desired, inflated, bounds=inset, **boundary):
            selected[name] = desired
            continue
        # A bounded quarter-meter lattice around the requested dock. Prefer
        # minimum displacement, then shorter cell distance and stable XY order.
        steps = int(max_dock_shift / .25)
        candidates = [(desired[0] + dx * .25, desired[1] + dy * .25)
                      for dx in range(-steps, steps + 1)
                      for dy in range(-steps, steps + 1)
                      if math.hypot(dx * .25, dy * .25) <= max_dock_shift]
        candidates.sort(key=lambda point: (math.dist(point, desired),
                                            math.dist(point, cell_centers[name]), point))
        selected[name] = next((p for p in candidates if not
            blocked_obstacle_ids(p, p, inflated, bounds=inset, **boundary)), None)
        if selected[name] is None:
            ids = blocked_obstacle_ids(desired, desired, inflated, bounds=inset, **boundary)
            raise ValueError(f"no valid service dock {name}; obstacles={','.join(ids)}")
    separation = 2 * radius + clearance + .05
    x0, x1, y0, y1 = inset
    # Keep a numerical gap from boundaries. The sampled bays only choose spawn
    # positions; route collision checks are analytic, never sampled.
    candidates = [(x0 + .01 + ix * separation, y0 + .01 + iy * separation)
                  for iy in range(min(100, int((y1-y0) / separation) + 1))
                  for ix in range(min(100, int((x1-x0) / separation) + 1))]
    candidates = [p for p in candidates if not
                  blocked_obstacle_ids(p, p, inflated, bounds=inset, **boundary)
                  and all(math.dist(p, d) >= separation for d in selected.values())]
    bays = []
    for candidate in candidates:
        if any(math.dist(candidate, bay) < separation for bay in bays):
            continue
        trial = bays + [candidate]
        # Check both new bay reachability and existing bay egress before taking
        # a spot that could seal a narrow passage.
        if all(all(clear_route(bay, dock, inflated +
                    [(other, 2 * radius + clearance) for other in trial if other != bay],
                    bounds=inset, **boundary) is not None for dock in selected.values()) for bay in trial):
            bays.append(candidate)
            if len(bays) == num_amrs:
                return selected, bays
    if num_amrs == 0:
        return selected, []
    ids = ','.join(obs.obstacle_id for obs in obstacles)
    raise ValueError(f"no connected parking/dock route for {num_amrs} AMRs; obstacles={ids}")
