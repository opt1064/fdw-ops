"""Real-USD regressions for nonphysical welding sparks in the startup survey.

CPU-only: the startup test substitutes the Isaac context and external asset
loading, but uses the real scene builder, visualizer, emitter, and navigation
extraction. It does not validate rendering, PhysX, or robot articulation.
"""
from __future__ import annotations

from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fdw_sim.cells.material.traffic import segment_is_clear
from fdw_sim.visualization.scene_builder import SceneBuilder, SceneConfig
from fdw_sim.visualization.spark_emitter import SparkEmitterConfig, WeldingSparkEmitter


ROLE_ATTRIBUTE = "fdw:navigation:role"
VFX_ROLE = "nonphysical_vfx"
CELL_ID = "WELDING_CELL_01"
CELL_PATH = f"/World/FDW/Cells/{CELL_ID}"


@pytest.fixture
def usd_stage(monkeypatch):
    pytest.importorskip("pxr")
    from pxr import Usd

    stage = Usd.Stage.CreateInMemory()
    omni = ModuleType("omni")
    omni.__path__ = []
    omni.usd = ModuleType("omni.usd")
    omni.usd.get_context = lambda: SimpleNamespace(get_stage=lambda: stage)
    monkeypatch.setitem(sys.modules, "omni", omni)
    monkeypatch.setitem(sys.modules, "omni.usd", omni.usd)
    return stage


@pytest.fixture
def scene(usd_stage):
    builder = SceneBuilder(SceneConfig(add_default_lighting=False))
    builder.add_cell_workbench(CELL_ID, (10, 6, 0), size=(2, 2, 0.8))
    return builder


def _source_paths(navmap):
    return {path for _, paths in navmap.source_paths for path in paths}


def _assert_sparks_excluded(navmap, emitter):
    path = emitter.get_instancer_path()
    assert not any(o.obstacle_id == path or o.obstacle_id.startswith(path + "/")
                   for o in navmap.obstacles)
    assert not any(p == path or p.startswith(path + "/") for p in _source_paths(navmap))


@pytest.mark.parametrize("state", ["empty", "populated", "cleared"])
def test_real_emitter_lifecycle_is_excluded_without_changing_fixed_collision(scene, state):
    from pxr import Sdf, UsdGeom

    baseline = scene.get_static_navigation_map()
    emitter = WeldingSparkEmitter(scene._stage, CELL_PATH,
                                  SparkEmitterConfig(spark_rate=40.0))
    prim = scene._stage.GetPrimAtPath(emitter.get_instancer_path())
    role = prim.GetAttribute(ROLE_ATTRIBUTE)
    assert prim.IsA(UsdGeom.PointInstancer)
    assert role and role.HasAuthoredValueOpinion()
    assert role.GetTypeName() == Sdf.ValueTypeNames.Token
    assert role.Get() == VFX_ROLE
    assert emitter.alive_count() == 0
    assert len(UsdGeom.PointInstancer(prim).GetPositionsAttr().Get()) == 0
    if state != "empty":
        emitter.set_tcp_position((6.0, 6.0, 0.5))
        emitter.set_active(True)
        emitter.update(0.1)
        assert emitter.alive_count() == 4
        assert len(UsdGeom.PointInstancer(prim).GetPositionsAttr().Get()) == 4
    if state == "cleared":
        emitter.set_active(False)
        emitter.clear()
        assert emitter.alive_count() == 0
        assert len(UsdGeom.PointInstancer(prim).GetPositionsAttr().Get()) == 0

    navmap = scene.get_static_navigation_map()
    _assert_sparks_excluded(navmap, emitter)
    assert scene._stage.GetPrimAtPath(emitter.get_instancer_path() + "/Proto/Sphere")
    assert navmap == baseline
    assert any(o.obstacle_id == CELL_PATH + "/Workbench" for o in navmap.obstacles)
    # The fixed bench still blocks its crossing after every emitter state.
    assert not segment_is_clear((8, 6), (12, 6), navmap.obstacles)


@pytest.mark.parametrize("label", [None, "static_geometry"])
def test_empty_point_instancer_without_explicit_vfx_role_fails_closed(scene, label):
    from pxr import Sdf

    emitter = WeldingSparkEmitter(scene._stage, CELL_PATH)
    prim = scene._stage.GetPrimAtPath(emitter.get_instancer_path())
    prim.RemoveProperty(ROLE_ATTRIBUTE)
    if label is not None:
        prim.CreateAttribute(ROLE_ATTRIBUTE, Sdf.ValueTypeNames.Token).Set(label)
    with pytest.raises(ValueError, match=r"empty bounds: .*WELDING_CELL_01/Sparks"):
        scene.get_static_navigation_map()


def test_populated_unlabeled_point_instancer_is_still_measured(scene):
    emitter = WeldingSparkEmitter(scene._stage, CELL_PATH)
    prim = scene._stage.GetPrimAtPath(emitter.get_instancer_path())
    prim.RemoveProperty(ROLE_ATTRIBUTE)
    emitter.set_tcp_position((6, 6, 0.5))
    emitter.set_active(True)
    emitter.update(0.1)
    navmap = scene.get_static_navigation_map()
    assert emitter.get_instancer_path() in _source_paths(navmap)
    assert any(o.obstacle_id == emitter.get_instancer_path() for o in navmap.obstacles)


def test_empty_real_static_mesh_still_fails_with_labeled_emitter_present(scene):
    from pxr import UsdGeom

    WeldingSparkEmitter(scene._stage, CELL_PATH)
    UsdGeom.Mesh.Define(scene._stage, "/World/FDW/Layout/BrokenFixedEquipment")
    with pytest.raises(ValueError, match="empty bounds: .*BrokenFixedEquipment"):
        scene.get_static_navigation_map()


@pytest.mark.parametrize("label_target", [None, "cube", "parent"])
def test_real_geometry_named_sparks_is_not_excluded_even_with_vfx_label(scene, label_target):
    from pxr import Sdf, UsdGeom

    # Neither a name match nor a label on a non-PointInstancer grants exclusion.
    path = "/World/FDW/Layout/Sparks"
    parent = UsdGeom.Xform.Define(scene._stage, path)
    cube = UsdGeom.Cube.Define(scene._stage, path + "/Body")
    cube.CreateSizeAttr(2)
    cube.AddTranslateOp().Set((5, 5, 1))
    if label_target is not None:
        target = cube if label_target == "cube" else parent
        target.GetPrim().CreateAttribute(ROLE_ATTRIBUTE, Sdf.ValueTypeNames.Token).Set(VFX_ROLE)
    navmap = scene.get_static_navigation_map()
    assert path + "/Body" in _source_paths(navmap)
    assert next(o.bounds for o in navmap.obstacles if o.obstacle_id == path) == (4, 6, 4, 6)
    assert not segment_is_clear((3, 5), (7, 5), navmap.obstacles)


def test_default_workshop_startup_collects_map_after_real_empty_emitter(usd_stage, monkeypatch):
    from pxr import UsdGeom
    from fdw_sim.cells.base.cell_base import CellConfig
    from fdw_sim.cells.material.material_cell import MaterialCell
    from fdw_sim.messaging.bus import InMemoryBus
    from fdw_sim.messaging.schemas import CellType
    from fdw_sim.visualization.workshop_visualizer import WorkshopVisualizer, WorkshopVizConfig

    # Keep default workshop/environment geometry. Replace only asset I/O and
    # articulation loading, then let the real _spawn_robot_arm attach its VFX.
    monkeypatch.setattr(SceneBuilder, "add_usd_reference", lambda *a, **kw: None)
    for method in ("add_amr_usd", "add_smart_rack_real", "add_inspection_camera_real"):
        monkeypatch.setattr(SceneBuilder, method, lambda *a, **kw: None)

    def load_robot(builder, cell_id, **kwargs):
        builder.add_robot_arm_placeholder(cell_id)
        return object()

    monkeypatch.setattr(SceneBuilder, "add_real_robot_arm", load_robot)
    monkeypatch.setenv("FDW_DISABLE_STAGE_DIAGNOSE", "1")
    bus = InMemoryBus()
    material = MaterialCell(CellConfig("MATERIAL_CELL_01", CellType.MATERIAL), bus,
                            num_amrs=2)
    viz = WorkshopVisualizer(bus, WorkshopVizConfig(
        use_real_robot=True, enable_ik=False, skip_auto_camera=True))
    for cell_id, cell_type, position in (
        ("MATERIAL_CELL_01", "material", (10.3, 6.2, 0)),
        (CELL_ID, "welding", (25.1, 10.75, 0)),
        ("INSPECTION_CELL_01", "inspection", (34.6, 6.45, 0)),
    ):
        viz.register_cell(cell_id, cell_type, position)
    viz.attach_material_cell(material)
    for amr in material.amrs:
        viz.register_amr(amr.amr_id, (*amr.position, 0), heading=amr.heading)

    collected = []
    extract = SceneBuilder.get_static_navigation_map

    def collect_after_emitter(builder, **kwargs):
        emitter = viz._robots[CELL_ID]["sparks"]
        assert isinstance(emitter, WeldingSparkEmitter)
        assert emitter.get_instancer_path() == CELL_PATH + "/Sparks"
        assert emitter.alive_count() == 0
        instancer = UsdGeom.PointInstancer(usd_stage.GetPrimAtPath(emitter.get_instancer_path()))
        assert len(instancer.GetPositionsAttr().Get()) == 0
        result = extract(builder, **kwargs)
        _assert_sparks_excluded(result, emitter)
        collected.append(result)
        return result

    monkeypatch.setattr(SceneBuilder, "get_static_navigation_map", collect_after_emitter)
    viz.build_scene()
    assert len(collected) == 1
    assert collected[0].source == "usd"
    assert material.navigation_source == "usd"
    assert material.static_obstacles == collected[0].obstacles
    for suffix in ("Layout/MaterialRack_0", "Environment/Walls/North",
                   f"Cells/{CELL_ID}/Workbench", f"Cells/{CELL_ID}/RobotArm"):
        assert any(o.obstacle_id.endswith(suffix) for o in collected[0].obstacles), suffix
    assert set(viz._navigation_docks) == set(viz.cell_positions)
    assert not material._transport_started
