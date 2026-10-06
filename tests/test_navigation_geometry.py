"""CPU geometry regressions, including real OpenUSD when usd-core is installed.

No Isaac, GPU, contacts, or real-robot reachability is exercised here.
"""
from __future__ import annotations

import math
from pathlib import Path
import subprocess
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fdw_sim.visualization.navigation_geometry import (
    StaticNavigationMap, default_static_navigation_map, extract_static_navigation_map,
    local_point_to_world_meters, world_meters_to_local_point, planar_parent_yaw,
)
from fdw_sim.visualization.scene_builder import SceneBuilder, SceneConfig


def _usd():
    return pytest.importorskip("pxr")


def _builder(*, meters=1.0, root="/World/FDW"):
    _usd()
    from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf
    scene = SceneBuilder.__new__(SceneBuilder)
    scene.config = SceneConfig(root_prim_path=root, add_default_lighting=False)
    scene._Usd, scene._UsdGeom, scene._UsdShade = Usd, UsdGeom, UsdShade
    scene._UsdLux, scene._Sdf, scene._Gf = UsdLux, Sdf, Gf
    scene._stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(scene._stage, meters)
    UsdGeom.SetStageUpAxis(scene._stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(scene._stage, root)
    scene._material_cache = {}
    scene._cell_prims, scene._amr_prims, scene._part_prims = {}, {}, {}
    # This test exercises built-in geometry only, without network asset loading.
    scene.add_usd_reference = lambda *args, **kwargs: None
    return scene


def _rect(obstacle):
    return (obstacle.xmin, obstacle.xmax, obstacle.ymin, obstacle.ymax)


def _by_suffix(navmap, suffix):
    matches = [o for o in navmap.obstacles if o.obstacle_id.endswith(suffix)]
    assert len(matches) == 1, [o.obstacle_id for o in navmap.obstacles]
    return matches[0]


def test_module_import_does_not_require_usd_or_isaac():
    code = """
import sys
sys.modules['pxr'] = None
sys.modules['omni'] = None
from fdw_sim.visualization.navigation_geometry import default_static_navigation_map
from fdw_sim.visualization.scene_builder import SceneBuilder
assert default_static_navigation_map().source == 'layout_fixture'
"""
    subprocess.run([sys.executable, "-c", code], check=True,
                   cwd=Path(__file__).resolve().parents[1])


def test_headless_fixture_uses_exact_rack_post_extents():
    navmap = default_static_navigation_map()
    for i in range(3):
        assert _rect(_by_suffix(navmap, f"MaterialRack_{i}")) == pytest.approx(
            (3.98, 12.02, 2.98 + 2*i, 4.02 + 2*i))
    assert navmap.bounds == (0.0, 36.1, 0.0, 12.4)
    assert not any("Fences" in o.obstacle_id for o in navmap.obstacles)


def test_real_scene_shelving_and_loaded_pipe_siblings_are_measured():
    scene = _builder()
    scene.add_workshop_layout()
    result = scene.get_static_navigation_map()
    for i in range(3):
        assert _rect(_by_suffix(result, f"MaterialRack_{i}")) == pytest.approx(
            (3.98, 12.02, 2.98 + 2*i, 4.02 + 2*i), abs=1e-6)
    rack = _by_suffix(result, "CantileverRack")
    assert _rect(rack) == pytest.approx((14.425, 17.875, 1.0, 3.0), abs=1e-6)
    paths = dict(result.source_paths)[rack.obstacle_id]
    assert any("/PipeBundle_L1_L/" in path for path in paths)
    assert any("/PipeBundle_L1_R/" in path for path in paths)
    assert any("/CantileverRack/" in path for path in paths)
    assert not any("PipeBundle_" in o.obstacle_id for o in result.obstacles)


def test_longer_sibling_pipe_is_included_beyond_rack_supports():
    scene = _builder()
    scene.add_loaded_cantilever_rack(pipe_length=8.0, loaded_levels=[1])
    result = scene.get_static_navigation_map()
    rack = _by_suffix(result, "CantileverRack")
    assert _rect(rack) == pytest.approx((12.15, 20.15, 1.0, 3.0), abs=1e-6)


def test_real_geometry_excludes_floors_markings_amrs_parts_and_ceiling():
    scene = _builder()
    scene.add_ground_plane()
    scene.add_workshop_layout()
    scene.add_ceiling_lights()
    scene.add_roof()
    scene.add_factory_walls()
    scene.add_structural_pillars()
    scene.add_overhead_crane()
    scene.add_amr("A", (15, 7, 0))
    scene.add_part("P", (16, 7, 0))
    scene.add_cell_workbench("M", (18, 7, 0))
    result = scene.get_static_navigation_map()
    ids = [o.obstacle_id for o in result.obstacles]
    for excluded in ("Ground", "/Zones/", "LaneMarkings", "/AMRs/", "/Parts/",
                     "Roof", "CeilingLights", "Rail_South", "Girder_", "Parking_Pad", "Guide_Line_"):
        assert not any(excluded in name for name in ids), excluded
    for present in ("Walls/North", "Walls/East", "Walls/West", "Pillars/Pillar_00",
                    "OverheadCrane/Pillar_Concrete_S_00", "MetalForming_Machine",
                    "CNC_Pipe_Bender", "Robot_Linear_Unit", "ServerRoom/wall_e",
                    "ServerRoom/wall_s", "AMR_Charging_Station_0", "Cells/M/Workbench",
                    "UnimplementedCells/additive_cell"):
        assert any(name.endswith(present) for name in ids), present
    # Open space between independent server walls is not filled with one AABB.
    assert not any(name.endswith("/ServerRoom") for name in ids)
    station = _by_suffix(result, "AMR_Charging_Station_0")
    assert _rect(station) == pytest.approx((3.75, 4.4, 10.4, 11.4), abs=1e-6)


def test_fence_gate_is_not_filled_by_assembly_envelope():
    scene = _builder()
    scene.add_safety_fence("F", 0, 0, 10, 5, gate_w=4)
    navmap = scene.get_static_navigation_map()
    assert len(navmap.obstacles) == 5
    assert not any(o.xmin < 5 < o.xmax and o.ymin < .1 < o.ymax for o in navmap.obstacles)


def test_nested_world_transform_rotation_scale_and_stage_units():
    scene = _builder(meters=.01)
    from pxr import UsdGeom
    parent = UsdGeom.Xform.Define(scene._stage, "/World/FDW/Layout/Rotated")
    parent.AddTranslateOp().Set((1000, 2000, 0))
    parent.AddRotateZOp().Set(90)
    parent.AddScaleOp().Set((2, 3, 1))
    child = UsdGeom.Cube.Define(scene._stage, "/World/FDW/Layout/Rotated/Nested/Body")
    child.CreateSizeAttr(2)
    child.AddTranslateOp().Set((100, 0, 50))
    child.AddScaleOp().Set((50, 25, 50))
    rect = _rect(_by_suffix(scene.get_static_navigation_map(), "/Rotated"))
    assert rect == pytest.approx((9.25, 10.75, 21.0, 23.0), abs=1e-6)
    world = local_point_to_world_meters(scene._stage, str(parent.GetPath()), (100, 0, 0))
    assert world == pytest.approx((10, 22, 0))
    assert world_meters_to_local_point(scene._stage, str(parent.GetPath()), world) == pytest.approx((100, 0, 0))


def test_transformed_root_bounds_are_in_world_meters():
    scene = _builder(meters=.5)
    from pxr import UsdGeom
    root = UsdGeom.Xformable(scene._stage.GetPrimAtPath(scene.config.root_prim_path))
    root.AddTranslateOp().Set((20, 40, 0))
    root.AddRotateZOp().Set(90)
    root.AddScaleOp().Set((2, 2, 2))
    scene.add_steel_shelving()
    navmap = scene.get_static_navigation_map()
    assert navmap.bounds == pytest.approx((-2.4, 10, 20, 56.1), abs=1e-6)


def test_hidden_and_guide_shapes_do_not_block_routes():
    scene = _builder()
    from pxr import UsdGeom
    hidden = UsdGeom.Xform.Define(scene._stage, "/World/FDW/Layout/Hidden")
    UsdGeom.Imageable(hidden).MakeInvisible()
    UsdGeom.Cube.Define(scene._stage, "/World/FDW/Layout/Hidden/Body")
    guide = UsdGeom.Cube.Define(scene._stage, "/World/FDW/Layout/Guide")
    guide.CreatePurposeAttr(UsdGeom.Tokens.guide)
    scene.add_steel_shelving(name="Visible")
    result = scene.get_static_navigation_map()
    assert len(result.obstacles) == 1
    assert result.obstacles[0].obstacle_id.endswith("/Visible")


def test_missing_root_bad_units_invalid_mesh_and_unresolved_reference_fail_closed():
    scene = _builder()
    from pxr import UsdGeom
    with pytest.raises(ValueError, match="Missing navigation root"):
        extract_static_navigation_map(scene._stage, "/Missing")
    UsdGeom.SetStageMetersPerUnit(scene._stage, -1)
    with pytest.raises(ValueError, match="metersPerUnit"):
        scene.get_static_navigation_map()
    UsdGeom.SetStageMetersPerUnit(scene._stage, 1)
    UsdGeom.Mesh.Define(scene._stage, "/World/FDW/Layout/Empty")
    with pytest.raises(ValueError, match="empty bounds"):
        scene.get_static_navigation_map()
    scene._stage.RemovePrim("/World/FDW/Layout/Empty")
    root = UsdGeom.Xform.Define(scene._stage, "/World/FDW/Layout/MissingAsset")
    root.GetPrim().GetReferences().AddReference("/missing/fdw-static-test.usda")
    with pytest.raises(ValueError, match="composition errors"):
        scene.get_static_navigation_map()


def test_instance_proxy_geometry_is_measured():
    scene = _builder()
    from pxr import UsdGeom, Sdf
    proto = UsdGeom.Xform.Define(scene._stage, "/Prototype")
    body = UsdGeom.Cube.Define(scene._stage, "/Prototype/Body")
    body.CreateSizeAttr(2)
    instance = UsdGeom.Xform.Define(scene._stage, "/World/FDW/Layout/Instance")
    instance.GetPrim().GetReferences().AddInternalReference(Sdf.Path("/Prototype"))
    instance.GetPrim().SetInstanceable(True)
    instance.AddTranslateOp().Set((5, 5, 1))
    assert _rect(_by_suffix(scene.get_static_navigation_map(), "/Instance")) == pytest.approx((4, 6, 4, 6))


def test_move_amr_and_payload_use_world_meters_with_rotated_scaled_parent():
    scene = _builder(meters=.01)
    from pxr import UsdGeom, Gf
    root = UsdGeom.Xformable(scene._stage.GetPrimAtPath(scene.config.root_prim_path))
    root.AddTranslateOp().Set((1000, 2000, 0))
    root.AddRotateZOp().Set(90)
    root.AddScaleOp().Set((2, 2, 2))
    scene.add_amr("A", (0, 0, 0))
    scene.move_amr("A", (12, 23, 0), heading=0.0)
    path = scene._amr_prims["A"]
    world = UsdGeom.XformCache().GetLocalToWorldTransform(scene._stage.GetPrimAtPath(path))
    assert tuple(v*.01 for v in world.ExtractTranslation()) == pytest.approx((12, 23, 0))
    assert tuple(world.TransformDir(Gf.Vec3d(1, 0, 0))) == pytest.approx((2, 0, 0), abs=1e-6)
    scene.add_part("P", (0, 0, 0))
    scene.move_part_world("P", (12, 23, .75))
    assert scene.local_point_to_world_meters((0, 0, 0), scene._part_prims["P"]) == pytest.approx((12, 23, .75))


def test_footprint_uses_composed_origin_including_offset_and_centimeter_units():
    scene = _builder(meters=.01)
    from pxr import UsdGeom
    root = UsdGeom.Xform.Define(scene._stage, "/World/FDW/AMRs/A")
    root.AddTranslateOp().Set((1000, 2000, 0))
    root.AddRotateZOp().Set(90)
    mesh = UsdGeom.Cube.Define(scene._stage, "/World/FDW/AMRs/A/Body")
    mesh.CreateSizeAttr(2)
    mesh.AddTranslateOp().Set((-50, 0, 50))
    mesh.AddScaleOp().Set((150, 100, 50))
    scene._amr_prims['A'] = str(root.GetPath())
    # Origin-centred local 4m x 2m envelope, turned 90 degrees.
    assert scene.get_amr_footprint('A', (99999, 99999, 0)) == pytest.approx((2, 4), abs=1e-6)
    scene.add_part("P", (0, 0, 0), size=10)
    assert scene.get_part_footprint("P") == pytest.approx((.07, .35), abs=1e-6)


@pytest.mark.parametrize("scale,rotation", [((2, 1, 1), 0), ((-1, 1, 1), 0), ((1, 1, 1), 30)])
def test_unsupported_moving_parent_transform_fails_without_mutating_pose(scale, rotation):
    scene = _builder()
    from pxr import UsdGeom
    root = UsdGeom.Xformable(scene._stage.GetPrimAtPath(scene.config.root_prim_path))
    root.AddRotateXOp().Set(rotation)
    root.AddScaleOp().Set(scale)
    scene.add_amr("A", (2, 3, 0))
    before = scene.local_point_to_world_meters((0, 0, 0), scene._amr_prims["A"])
    with pytest.raises(ValueError, match="AMR parent"):
        scene.move_amr("A", (10, 10, 0))
    assert scene.local_point_to_world_meters((0, 0, 0), scene._amr_prims["A"]) == before


def test_world_cell_origin_and_invalid_map_contract():
    scene = _builder(meters=.01)
    scene.add_cell_workbench("M", (800, 600, 0))
    assert scene.get_cell_world_position("M") == pytest.approx((8, 6, 0))
    with pytest.raises(ValueError):
        StaticNavigationMap((), (0, math.nan, 0, 1), "test")
    with pytest.raises(ValueError):
        StaticNavigationMap((), (0, 1, 0, 1), "test", units="centimeters")


def test_boundary_polygon_and_floor_height_follow_transformed_root():
    scene = _builder(meters=.01)
    from pxr import UsdGeom
    root = UsdGeom.Xformable(scene._stage.GetPrimAtPath(scene.config.root_prim_path))
    root.AddTranslateOp().Set((1000, 2000, 500))
    root.AddRotateZOp().Set(30)
    root.AddScaleOp().Set((100, 100, 100))
    scene.add_steel_shelving()
    result = scene.get_static_navigation_map()
    expected = tuple(scene.local_point_to_world_meters(p)[:2] for p in
                     ((0, 0, 0), (36.1, 0, 0), (36.1, 12.4, 0), (0, 12.4, 0)))
    assert result.floor_height_m == 5
    for actual, desired in zip(result.boundary_polygon, expected):
        assert actual == pytest.approx(desired)
    assert sum(a[0]*b[1]-b[0]*a[1] for a, b in zip(
        result.boundary_polygon, result.boundary_polygon[1:]+result.boundary_polygon[:1])) > 0


def test_navigation_markers_have_world_poses_and_never_become_obstacles():
    scene = _builder(meters=.5)
    from pxr import UsdGeom
    root = UsdGeom.Xformable(scene._stage.GetPrimAtPath(scene.config.root_prim_path))
    root.AddTranslateOp().Set((20, 40, 2))
    root.AddRotateZOp().Set(90)
    root.AddScaleOp().Set((2, 2, 2))
    scene.add_steel_shelving()
    before = scene.get_static_navigation_map()
    path = scene.add_navigation_markers({'M': (6, 25)}, [(7, 30), (5, 32)])
    assert scene.local_point_to_world_meters((0, 0, 0), f'{path}/Dock_00') == pytest.approx((6, 25, 1.035))
    assert scene.local_point_to_world_meters((0, 0, 0), f'{path}/Bay_01') == pytest.approx((5, 32, 1.035))
    assert scene.get_static_navigation_map().obstacles == before.obstacles
    scene.add_navigation_markers({'M': (6, 25)}, [(7, 30)])
    assert not scene._stage.GetPrimAtPath(f'{path}/Bay_01')


def test_tall_amr_and_offset_part_height_and_overhead_slab():
    scene = _builder(meters=.01)
    from pxr import UsdGeom
    body = UsdGeom.Cube.Define(scene._stage, '/World/FDW/AMRs/Tall')
    body.CreateSizeAttr(2)
    body.AddTranslateOp().Set((0, 0, 0))
    body.AddScaleOp().Set((50, 50, 240))
    scene._amr_prims['Tall'] = str(body.GetPath())
    assert scene.get_amr_height('Tall') == pytest.approx(2.4)
    part = UsdGeom.Xform.Define(scene._stage, '/World/FDW/Parts/P')
    child = UsdGeom.Cube.Define(scene._stage, '/World/FDW/Parts/P/OffsetGeometry')
    child.AddTranslateOp().Set((0, 0, 30))
    child.AddScaleOp().Set((10, 10, 20))
    scene._part_prims['P'] = str(part.GetPath())
    assert scene.get_part_height('P') == pytest.approx(1.0)
    beam = UsdGeom.Cube.Define(scene._stage, '/World/FDW/Layout/OverheadBeam')
    beam.AddTranslateOp().Set((1000, 1000, 225))
    beam.AddScaleOp().Set((100, 100, 25))
    assert not scene.get_static_navigation_map(robot_height_m=1.5).obstacles
    assert len(scene.get_static_navigation_map(robot_height_m=2.4).obstacles) == 1


def test_heading_is_outer_planar_rotation_before_fixed_tilted_asset_axes():
    scene = _builder()
    from pxr import UsdGeom
    root = UsdGeom.Xform.Define(scene._stage, '/World/FDW/AMRs/TiltedAsset')
    root.AddTranslateOp().Set((5, 5, 0))
    root.AddRotateXOp().Set(90)
    body = UsdGeom.Cube.Define(scene._stage, '/World/FDW/AMRs/TiltedAsset/Body')
    body.CreateSizeAttr(2)
    body.AddScaleOp().Set((2, .2, .2))
    scene._amr_prims['A'] = str(root.GetPath())
    scene.move_amr('A', (5, 5, 0), math.pi/2)
    first = scene.get_amr_footprint('A')
    assert first == pytest.approx((.4, 4), abs=1e-6)
    radius = math.hypot(*first)/2
    for heading in (0, math.pi/2, math.pi, 3*math.pi/2, .31):
        scene.move_amr('A', (5, 5, 0), heading)
        # Actual farthest transformed asset corner remains in the startup disk.
        world = UsdGeom.XformCache().GetLocalToWorldTransform(body.GetPrim())
        from pxr import Gf
        for x in (-1, 1):
            for y in (-1, 1):
                for z in (-1, 1):
                    corner = world.Transform(Gf.Vec3d(x, y, z))
                    assert math.hypot(corner[0]-5, corner[1]-5) <= radius + 1e-6
        assert scene.local_point_to_world_meters((0,0,0), str(root.GetPath())) == pytest.approx((5,5,0))
    names = [op.GetOpName() for op in root.GetOrderedXformOps()]
    assert names.index('xformOp:rotateZ:fdwHeading') < names.index('xformOp:rotateX')


@pytest.mark.parametrize('kind', ['matrix', 'pivot', 'reset'])
def test_unsupported_amr_root_stack_rejected_before_pose_mutation(kind):
    scene = _builder()
    from pxr import UsdGeom, Gf
    scene.add_amr('A', (5,5,0))
    root = UsdGeom.Xformable(scene._stage.GetPrimAtPath(scene._amr_prims['A']))
    if kind == 'matrix':
        root.AddTransformOp().Set(Gf.Matrix4d(1).SetTranslate(Gf.Vec3d(2,0,0)))
    elif kind == 'pivot':
        root.AddTranslateOp(opSuffix='pivot').Set((2,0,0))
    else:
        root.SetResetXformStack(True)
    before = scene.local_point_to_world_meters((0,0,0), scene._amr_prims['A'])
    with pytest.raises(ValueError, match='unsupported reset, matrix, or pivot'):
        scene.move_amr('A', (10,10,0), 1.0)
    assert scene.local_point_to_world_meters((0,0,0), scene._amr_prims['A']) == before


@pytest.mark.parametrize('arc,child', [('payload', False), ('reference', True)])
def test_unresolved_static_composition_cannot_disguise_itself_as_empty_geometry(arc, child):
    scene = _builder()
    from pxr import UsdGeom
    root = UsdGeom.Xform.Define(scene._stage, '/World/FDW/Layout/BadAsset')
    if arc == 'payload':
        root.GetPrim().GetPayloads().AddPayload('/missing/fdw-static-test.usda')
    else:
        root.GetPrim().GetReferences().AddReference('/missing/fdw-static-test.usda')
    if child:
        UsdGeom.Xform.Define(scene._stage, '/World/FDW/Layout/BadAsset/EmptyChild')
    with pytest.raises(ValueError, match='composition errors'):
        scene.get_static_navigation_map()


def test_empty_or_unloaded_valid_reference_payload_fails_closed(tmp_path):
    scene = _builder()
    from pxr import Usd, UsdGeom
    asset = Usd.Stage.CreateNew(str(tmp_path / 'asset.usda'))
    root = UsdGeom.Xform.Define(asset, '/Asset')
    UsdGeom.Xform.Define(asset, '/Asset/EmptyChild')
    asset.SetDefaultPrim(root.GetPrim())
    asset.GetRootLayer().Save()
    ref = UsdGeom.Xform.Define(scene._stage, '/World/FDW/Layout/EmptyAsset')
    ref.GetPrim().GetReferences().AddReference(str(tmp_path / 'asset.usda'))
    with pytest.raises(ValueError, match='no measurable geometry'):
        scene.get_static_navigation_map()
    scene._stage.RemovePrim('/World/FDW/Layout/EmptyAsset')
    UsdGeom.Cube.Define(asset, '/Asset/Body')
    asset.GetRootLayer().Save()
    payload = UsdGeom.Xform.Define(scene._stage, '/World/FDW/Layout/UnloadedAsset')
    payload.GetPrim().GetPayloads().AddPayload(str(tmp_path / 'asset.usda'))
    scene._stage.Unload(str(payload.GetPath()))
    with pytest.raises(ValueError, match='payload is not loaded'):
        scene.get_static_navigation_map()


def test_unresolved_dynamic_asset_is_outside_static_map_scope():
    scene = _builder()
    from pxr import UsdGeom
    for path in ('/World/FDW/AMRs/A', '/World/FDW/Parts/P'):
        prim = UsdGeom.Xform.Define(scene._stage, path).GetPrim()
        prim.GetReferences().AddReference('/missing/dynamic-test.usda')
    scene.add_steel_shelving()
    assert len(scene.get_static_navigation_map().obstacles) == 1


def test_amr_descendant_reset_is_rejected_before_moving_any_geometry():
    scene = _builder()
    from pxr import UsdGeom
    scene.add_amr('A', (5, 5, 0))
    body_path = scene._amr_prims['A'] + '/Body'
    body = UsdGeom.Xformable(scene._stage.GetPrimAtPath(body_path))
    body.SetResetXformStack(True)
    before = scene.local_point_to_world_meters((0,0,0), scene._amr_prims['A'])
    with pytest.raises(ValueError, match='descendant resets'):
        scene.move_amr('A', (10,10,0))
    assert scene.local_point_to_world_meters((0,0,0), scene._amr_prims['A']) == before


def _complete_workshop_fixture():
    scene = _builder()
    scene.add_ground_plane()
    scene.add_workshop_layout()
    scene.add_factory_walls()
    scene.add_structural_pillars(skip_x_ranges=[(28.2, 33.0)])
    scene.add_overhead_crane()
    scene.add_ceiling_lights()
    centers = {'MAT': (10.3,6.2), 'WELD': (25.1,10.75), 'INSP': (34.6,6.45)}
    for name, point in centers.items():
        scene.add_cell_workbench(name, (*point, 0), size=(1.8,1.8,.8))
    return scene, centers


def test_complete_actual_workshop_matches_shared_fixture_bounds():
    scene, _ = _complete_workshop_fixture()
    navmap = scene.get_static_navigation_map()
    assert len(navmap.obstacles) == 41
    actual = {o.obstacle_id: o.bounds for o in navmap.obstacles}
    for obstacle in default_static_navigation_map().obstacles:
        assert actual[obstacle.obstacle_id] == pytest.approx(obstacle.bounds, abs=1e-6)


def test_complete_actual_workshop_two_amrs_finish_repeated_transport():
    from fdw_sim.cells.material.material_cell import MaterialCell, CellLocation
    from fdw_sim.cells.material.navigation_setup import select_transport_positions
    from fdw_sim.cells.material.traffic import segment_is_clear
    from fdw_sim.cells.base.cell_base import CellConfig
    from fdw_sim.messaging.bus import InMemoryBus
    from fdw_sim.messaging.schemas import CellType, MaterialTransferCommand
    scene, centers = _complete_workshop_fixture()
    navmap = scene.get_static_navigation_map()
    wanted = {name: (p[0], p[1]-1.3) for name, p in centers.items()}
    docks, bays = select_transport_positions(wanted, centers, navmap.obstacles, navmap.bounds,
        math.hypot(1.2,.8)/2, .15, 2, boundary_polygon=navmap.boundary_polygon)
    assert docks == {'MAT': (13.05,4.9), 'WELD': (25.1,8.95), 'INSP': (34.6,4.65)}
    material = MaterialCell(CellConfig('MAT', CellType.MATERIAL), InMemoryBus(), num_amrs=2)
    received = []
    class Sink:
        def __init__(self, name): self.cell_id = name
        def receive_part(self, part): received.append(part); return True
    for name in ('WELD', 'INSP'):
        material.register_cell(Sink(name), CellLocation(name, centers[name]))
    material.configure_transport_geometry(docks, parking_positions=bays,
        static_obstacles=navmap.obstacles, navigation_bounds=navmap.bounds,
        navigation_boundary=navmap.boundary_polygon, navigation_source=navmap.source,
        payload_footprint_size=(1.2,.8))
    for i in range(4):
        part = f'P{i}'
        material.stock_part(part, 'test')
        scene.add_part(part, (*centers['MAT'], 1.0))
        measured = scene.get_part_footprint(part)
        height = scene.get_part_height(part)
        assert measured is not None and measured[0] <= 1.2 and measured[1] <= .8
        assert height is not None and height <= 1.0
        material.validated_payloads.add(part)
        material._on_transfer_command(MaterialTransferCommand('MAT', 'WELD' if i%2 else 'INSP', part, 'AUTO'))
    used = set()
    for tick in range(1500):
        for amr in material.amrs:
            points = [amr.position] + amr.waypoints
            for start, end in zip(points, points[1:]):
                assert segment_is_clear(start, end, material._obstacles(amr),
                    bounds=material._route_bounds(amr), convex_boundary=navmap.boundary_polygon,
                    boundary_margin=amr.footprint_radius + material.clearance_m)
            if amr.busy: used.add(amr.amr_id)
        material.step(.73, tick*.73)
        assert not any(amr.phase == 'blocked' for amr in material.amrs)
        if not material.transfer_queue and not any(amr.busy for amr in material.amrs):
            break
    else:
        pytest.fail('complete real-USD workshop routing deadlocked')
    assert sorted(received) == ['P0','P1','P2','P3']
    assert used == {'AMR_01','AMR_02'}
    assert all(amr.position == amr.parking_position for amr in material.amrs)


def test_complete_actual_workshop_oversized_envelope_fails_closed():
    from fdw_sim.cells.material.navigation_setup import select_transport_positions
    scene, centers = _complete_workshop_fixture()
    navmap = scene.get_static_navigation_map()
    wanted = {name: (p[0],p[1]-1.3) for name,p in centers.items()}
    with pytest.raises(ValueError, match='no connected parking/dock route'):
        select_transport_positions(wanted, centers, navmap.obstacles, navmap.bounds,
            math.hypot(2.4,1.0)/2, .15, 2, boundary_polygon=navmap.boundary_polygon)


def test_referenced_material_nested_in_machine_is_not_a_geometry_assembly():
    scene = _builder()
    from pxr import UsdGeom, UsdShade
    UsdShade.Material.Define(scene._stage, '/SourceMaterial')
    machine = '/World/FDW/Layout/Machine'
    UsdGeom.Cube.Define(scene._stage, machine + '/Body')
    for suffix in ('Looks/Material', 'UnusuallyNamedAppearance'):
        material = UsdShade.Material.Define(scene._stage, machine + '/' + suffix)
        material.GetPrim().GetReferences().AddInternalReference('/SourceMaterial')
    assert len(scene.get_static_navigation_map().obstacles) == 1


def test_untyped_unloaded_payload_cannot_lose_its_geometry_requirement(tmp_path):
    scene = _builder()
    from pxr import Usd, UsdGeom
    asset = Usd.Stage.CreateNew(str(tmp_path / 'payload.usda'))
    root = UsdGeom.Xform.Define(asset, '/Asset')
    UsdGeom.Cube.Define(asset, '/Asset/Body')
    asset.SetDefaultPrim(root.GetPrim())
    asset.GetRootLayer().Save()
    wrapper = scene._stage.DefinePrim('/World/FDW/Layout/Untyped')
    wrapper.GetPayloads().AddPayload(str(tmp_path / 'payload.usda'))
    scene._stage.Unload(str(wrapper.GetPath()))
    assert not wrapper.GetTypeName()
    with pytest.raises(ValueError, match='payload is not loaded'):
        scene.get_static_navigation_map()
