"""Static AMR navigation geometry from the geometry that is actually displayed.

No Isaac or USD import occurs at module import time. USD extraction is strict:
missing/unmeasurable geometry raises instead of silently substituting a hand-drawn
map. Bounds are conservative world-axis-aligned XY envelopes, in metres. This is
geometric route checking, not PhysX contact or articulated-robot validation.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from fdw_sim.cells.material.traffic import StaticObstacle

Bounds = tuple[float, float, float, float]
WORKSHOP_BOUNDS: Bounds = (0.0, 36.1, 0.0, 12.4)
# Shared with SceneBuilder.add_workshop_layout, including the shelving posts.
MATERIAL_RACK_LAYOUT = (
    (4.0, 3.0, 8.0, 1.0, 2.2),
    (4.0, 5.0, 8.0, 1.0, 2.2),
    (4.0, 7.0, 8.0, 1.0, 2.2),
)
SHELVING_POST_SIZE = 0.04
CHARGING_STATION_POSITIONS = ((4.5, 10.9, 0.0), (6.5, 10.9, 0.0))
CANTILEVER_POSITION = (14.5, 2.0, 0.0)
FORMING_MACHINE_POSITION = (27.5, 2.5, 0.0)
PIPE_BENDER_POSITION = (33.0, 2.5, 0.0)
LINEAR_UNIT_POSITION = (34.6, 10.2, 0.0)
CRANE_PILLAR_X_SOUTH = (2.0, 10.0, 19.5, 30.5)
CRANE_PILLAR_X_NORTH = (2.0, 10.0, 19.5, 33.0)


@dataclass(frozen=True)
class StaticNavigationMap:
    obstacles: tuple[StaticObstacle, ...]
    bounds: Bounds
    source: str
    units: str = "meters"
    # Evidence for grouped assemblies (including sibling stored pipe bundles).
    source_paths: tuple[tuple[str, tuple[str, ...]], ...] = ()
    floor_height_m: float = 0.0
    boundary_polygon: tuple[tuple[float, float], ...] = ()

    def __post_init__(self) -> None:
        values = tuple(float(v) for v in self.bounds)
        if (len(values) != 4 or not all(math.isfinite(v) for v in values)
                or values[0] >= values[1] or values[2] >= values[3]):
            raise ValueError("Static navigation bounds must be finite and nonempty")
        if not math.isfinite(self.floor_height_m):
            raise ValueError("Navigation floor height must be finite")
        if self.units != "meters":
            raise ValueError("Static navigation maps must use meters")
        object.__setattr__(self, "bounds", values)
        object.__setattr__(self, "obstacles", tuple(self.obstacles))


def _finite_xyz(point: Iterable[float]) -> tuple[float, float, float]:
    values = tuple(float(v) for v in point)
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError("Expected a finite XYZ point")
    return values


def stage_meters_per_unit(stage) -> float:
    from pxr import UsdGeom

    value = float(UsdGeom.GetStageMetersPerUnit(stage))
    if not math.isfinite(value) or value <= 0:
        raise ValueError("USD stage metersPerUnit must be finite and positive")
    return value


def _world_transform(stage, prim_path):
    from pxr import UsdGeom

    prim = stage.GetPrimAtPath(str(prim_path))
    if not prim or not prim.IsValid():
        raise ValueError(f"Cannot transform missing USD prim: {prim_path}")
    return UsdGeom.XformCache().GetLocalToWorldTransform(prim)


def local_point_to_world_meters(stage, prim_path, point) -> tuple[float, float, float]:
    from pxr import Gf

    scale = stage_meters_per_unit(stage)
    result = _world_transform(stage, prim_path).Transform(Gf.Vec3d(*_finite_xyz(point)))
    return _finite_xyz(float(v) * scale for v in result)


def world_meters_to_local_point(stage, prim_path, point) -> tuple[float, float, float]:
    from pxr import Gf

    scale = stage_meters_per_unit(stage)
    transform = _world_transform(stage, prim_path)
    determinant = float(transform.GetDeterminant())
    if not math.isfinite(determinant) or abs(determinant) <= 1e-15:
        raise ValueError(f"USD parent has a singular transform: {prim_path}")
    result = transform.GetInverse().Transform(Gf.Vec3d(*(v / scale for v in _finite_xyz(point))))
    return _finite_xyz(result)


def planar_parent_yaw(stage, prim_path) -> float:
    """Validate a moving frame and return its world yaw in radians.

    Static obstacle extraction permits arbitrary affine transforms. Moving AMRs
    require a horizontal, orientation-preserving, uniformly scaled XY frame:
    otherwise a footprint measured at one heading need not bound later turns.
    Z scaling may differ because it does not change the planar swept envelope.
    """
    from pxr import Gf, UsdGeom

    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
        raise ValueError("AMR navigation requires a Z-up USD stage")
    matrix = _world_transform(stage, prim_path)
    x = _finite_xyz(matrix.TransformDir(Gf.Vec3d(1, 0, 0)))
    y = _finite_xyz(matrix.TransformDir(Gf.Vec3d(0, 1, 0)))
    z = _finite_xyz(matrix.TransformDir(Gf.Vec3d(0, 0, 1)))
    sx, sy = math.hypot(x[0], x[1]), math.hypot(y[0], y[1])
    tolerance = max(sx, sy, abs(z[2]), 1.0) * 1e-7
    dot = x[0] * y[0] + x[1] * y[1]
    if (min(sx, sy, z[2]) <= 1e-12 or abs(sx - sy) > tolerance
            or abs(dot) > tolerance * max(sx, sy)
            or max(abs(x[2]), abs(y[2]), abs(z[0]), abs(z[1])) > tolerance
            or x[0] * y[1] - x[1] * y[0] <= 0):
        raise ValueError("AMR parent must have a planar, positive uniform XY scale; "
                         "tilt, shear, reflection, and nonuniform XY scale are unsupported")
    return math.atan2(x[1], x[0])


def _excluded(parts: tuple[str, ...]) -> bool:
    if not parts:
        return False
    # These scopes are authored separately from fixed equipment by SceneBuilder.
    if parts[0] in {"AMRs", "Parts", "Ground", "Zones", "Lighting", "Looks", "Materials", "NavigationMarkers"}:
        return True
    if any(part in {"Looks", "Materials"} for part in parts):
        return True
    if parts[:2] in {("Layout", "LaneMarkings"), ("Environment", "Roof"),
                     ("Environment", "CeilingLights")}:
        return True
    # The charging pad and paint must remain drivable; its bumpers/tower must not.
    if parts[-1] in {"Parking_Pad", "Guide_Line_L", "Guide_Line_R"}:
        return True
    return False


def _assembly_path(root: str, parts: tuple[str, ...]) -> str:
    """Group storage/equipment; never fill the spaces between walls or posts."""
    if parts[0] == "Layout" and len(parts) >= 2:
        name = parts[1]
        if name == "CantileverRack" or name.startswith("PipeBundle_"):
            return f"{root}/Layout/CantileverRack"
        if name != "ServerRoom":
            return f"{root}/Layout/{name}"
    if parts[0] == "Cells" and len(parts) >= 3:
        # Keep attachments distinct, rather than sweeping an entire cell aisle.
        return f"{root}/{'/'.join(parts[:3])}"
    if parts[0] == "UnimplementedCells" and len(parts) >= 2:
        return f"{root}/{'/'.join(parts[:2])}"
    return f"{root}/{'/'.join(parts)}"


def extract_static_navigation_map(stage, root_path: str = "/World/FDW", *,
                                  bounds: Bounds = WORKSHOP_BOUNDS,
                                  robot_height_m: float = 1.5) -> StaticNavigationMap:
    """Measure visible fixed meshes after references/transforms are composed.

    All included boundables are measured in world space before projection. In
    particular this does not use a prim's local translation or its USD extent
    directly. AABB unions include post widths, rotations, scales, nested asset
    transforms, and the stored pipes authored as siblings of CantileverRack.
    The height slab skips ceiling equipment but keeps its ground-level columns.
    """
    from pxr import Usd, UsdGeom, UsdShade

    if stage is None:
        raise ValueError("Cannot extract static navigation geometry without a USD stage")
    root_path = str(root_path).rstrip("/")
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        raise ValueError(f"Missing navigation root: {root_path}")
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
        raise ValueError("Static XY navigation requires a Z-up USD stage")
    if not math.isfinite(robot_height_m) or robot_height_m <= 0:
        raise ValueError("AMR height must be finite and positive")
    scale = stage_meters_per_unit(stage)
    xmin, xmax, ymin, ymax = bounds
    corners = [local_point_to_world_meters(stage, root_path, (x, y, 0.0))
               for x, y in ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax))]
    polygon = tuple((p[0], p[1]) for p in corners)
    area2 = sum(a[0]*b[1] - b[0]*a[1] for a, b in zip(polygon, polygon[1:] + polygon[:1]))
    if abs(area2) <= 1e-12:
        raise ValueError("Transformed workshop floor has a degenerate XY boundary")
    if area2 < 0:
        polygon = tuple(reversed(polygon))
    world_bounds = (min(p[0] for p in corners), max(p[0] for p in corners),
                    min(p[1] for p in corners), max(p[1] for p in corners))
    floor_z = min(p[2] for p in corners)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                             [UsdGeom.Tokens.default_, UsdGeom.Tokens.render,
                              UsdGeom.Tokens.proxy], useExtentsHint=False)
    envelopes: dict[str, list[float]] = {}
    sources: dict[str, list[str]] = {}
    # Include instance proxies: referenced static equipment may be instanced.
    # Default USD traversal filters out unloaded payload roots entirely. Include
    # those roots so an unloaded fixed asset cannot become free space.
    predicate = Usd.PrimIsActive & Usd.PrimIsDefined & ~Usd.PrimIsAbstract
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies(predicate)):
        path = str(prim.GetPath())
        relative = path[len(root_path):].strip("/")
        parts = tuple(relative.split("/")) if relative else ()
        if not parts or _excluded(parts):
            continue
        # Material/shader references carry appearance, not collision geometry.
        # They can be nested inside equipment assemblies in imported assets.
        if prim.IsA(UsdShade.Material) or prim.IsA(UsdShade.Shader) or prim.IsA(UsdShade.NodeGraph):
            continue
        imageable = UsdGeom.Imageable(prim)
        if imageable and (imageable.ComputeVisibility() == UsdGeom.Tokens.invisible
                          or imageable.ComputePurpose() == UsdGeom.Tokens.guide):
            continue
        # Partial composition is unsafe even if an unresolved reference has a
        # locally authored empty Xform child (or a second reference did resolve).
        errors = prim.GetPrimIndex().localErrors
        if errors:
            raise ValueError(f"Static USD geometry has composition errors: {path}: "
                             + "; ".join(str(error) for error in errors))
        if prim.HasAuthoredReferences() or prim.HasAuthoredPayloads():
            if not prim.IsLoaded():
                raise ValueError(f"Static USD payload is not loaded: {path}")
            if not any(child.IsA(UsdGeom.Boundable)
                       for child in Usd.PrimRange(prim, Usd.TraverseInstanceProxies())):
                raise ValueError(f"Static USD reference/payload has no measurable geometry: {path}")
        if not prim.IsA(UsdGeom.Boundable):
            continue
        aligned = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if aligned.IsEmpty():
            raise ValueError(f"Static USD geometry has empty bounds: {path}")
        low = _finite_xyz(float(v) * scale for v in aligned.GetMin())
        high = _finite_xyz(float(v) * scale for v in aligned.GetMax())
        if high[2] < floor_z or low[2] > floor_z + robot_height_m:
            continue
        if high[0] <= low[0] or high[1] <= low[1]:
            raise ValueError(f"Static USD geometry has degenerate XY bounds: {path}")
        group = _assembly_path(root_path, parts)
        rectangle = [low[0], high[0], low[1], high[1]]
        if group in envelopes:
            old = envelopes[group]
            rectangle = [min(old[0], rectangle[0]), max(old[1], rectangle[1]),
                         min(old[2], rectangle[2]), max(old[3], rectangle[3])]
        envelopes[group] = rectangle
        sources.setdefault(group, []).append(path)
    obstacles = tuple(StaticObstacle(name, *envelopes[name]) for name in sorted(envelopes))
    return StaticNavigationMap(obstacles, world_bounds, "usd",
                               source_paths=tuple((name, tuple(sources[name]))
                                                  for name in sorted(sources)),
                               floor_height_m=floor_z, boundary_polygon=polygon)


def default_static_navigation_map(*, root_path: str = "/World/FDW",
                                  include_environment: bool = True,
                                  cell_specs: Iterable[dict] = ()) -> StaticNavigationMap:
    """Headless fixture for the built-in workshop layout, in world metres.

    This is an explicit schematic fixture, never a fallback for failed live USD
    extraction. Real asset references and custom layout changes require the live
    stage map. Placement and rack dimensions share SceneBuilder constants.
    """
    from fdw_sim.visualization.scene_builder import WORKSHOP_ZONES, FORMING_GATE_WIDTH_M

    obstacles: list[StaticObstacle] = []
    root = root_path.rstrip("/")

    def box(path: str, x: float, y: float, half_x: float, half_y: float) -> None:
        obstacles.append(StaticObstacle(f"{root}/{path}", x - half_x, x + half_x,
                                         y - half_y, y + half_y))

    for i, (x0, y0, width, depth, _height) in enumerate(MATERIAL_RACK_LAYOUT):
        post = SHELVING_POST_SIZE / 2
        box(f"Layout/MaterialRack_{i}", x0 + width/2, y0 + depth/2,
            width/2 + post, depth/2 + post)
    x, y, _ = CANTILEVER_POSITION
    box("Layout/CantileverRack", x + 1.65, y, 1.725, 1.0)
    for i, (x, y, _z) in enumerate(CHARGING_STATION_POSITIONS):
        box(f"Layout/AMR_Charging_Station_{i}", x - 0.425, y, 0.325, 0.5)
    sx, sy, sw, sd, *_ = WORKSHOP_ZONES["server_room"]
    box("Layout/ServerRoom/wall_e", sx + sw - .05, sy + sd/2, .05, sd/2)
    box("Layout/ServerRoom/wall_s", sx + sw/2, sy + .05, sw/2, .05)
    x, y, _ = FORMING_MACHINE_POSITION
    box("Layout/MetalForming_Machine", x - .125, y + .25, 1.375, .85)
    x, y, _ = PIPE_BENDER_POSITION
    box("Layout/CNC_Pipe_Bender", x - .275, y + .05, 2.925, .85)
    x, y, _ = LINEAR_UNIT_POSITION
    box("Layout/Robot_Linear_Unit", x, y - .0625, 1.2, .7125)
    x, y, w, d, *_ = WORKSHOP_ZONES["additive_cell"]
    box("UnimplementedCells/additive_cell", x + w/2, y + d/2, w*.35, d*.35)
    if include_environment:
        xmin, xmax, ymin, ymax = WORKSHOP_BOUNDS
        box("Environment/Walls/North", (xmin+xmax)/2, ymax, (xmax-xmin)/2+.2, .2)
        box("Environment/Walls/East", xmax, (ymin+ymax)/2, .2, (ymax-ymin)/2+.2)
        box("Environment/Walls/West", xmin, (ymin+ymax)/2, .2, (ymax-ymin)/2+.2)
        fx, _, fw, *_ = WORKSHOP_ZONES["forming_cell"]
        gate_center, gate_half = fx + fw/2, FORMING_GATE_WIDTH_M/2 + .5
        i = 0
        x = xmin + 6.0
        while x < xmax - 3.0:
            for y in (ymin+.5, ymax-.5):
                if not gate_center-gate_half <= x <= gate_center+gate_half:
                    box(f"Environment/Pillars/Pillar_{i:02d}", x, y, .2, .2)
                    i += 1
            x += 6.0
        for side, xs, y in (("S", CRANE_PILLAR_X_SOUTH, ymin+1),
                             ("N", CRANE_PILLAR_X_NORTH, ymax-1)):
            for i, x in enumerate(xs):
                box(f"Environment/OverheadCrane/Pillar_Concrete_{side}_{i:02d}", x, y, .22, .22)
        box("Environment/OverheadCrane/Hook", 22.0, (ymin+ymax)/2 + 1.0, .08, .08)
    for cell in cell_specs:
        x, y, *_ = cell["position"]
        width, depth, *_ = cell.get("size", (2.0, 2.0, .8))
        box(f"Cells/{cell['cell_id']}/Workbench", x, y, width/2, depth/2)
    xmin, xmax, ymin, ymax = WORKSHOP_BOUNDS
    return StaticNavigationMap(tuple(obstacles), WORKSHOP_BOUNDS, "layout_fixture",
        boundary_polygon=((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)))
