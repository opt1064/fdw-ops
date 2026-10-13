"""Exercise startup with a strict GroundPlane wrapper and real USD visibility.

CPU regression only: no Isaac renderer or contact simulation is exercised.
"""
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fdw_sim.simulation.manager import SimulationConfig, SimulationManager


@pytest.mark.parametrize("visualization", [True, False])
def test_startup_hides_ground_without_wrapper_visibility_method(monkeypatch, visualization):
    pytest.importorskip("pxr")
    from pxr import Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.CreateInMemory()
    events = []
    ground_path = "/World/defaultGroundPlane"

    class Scene:
        def add_default_ground_plane(self, *, prim_path):
            assert prim_path == ground_path
            UsdGeom.Xform.Define(stage, prim_path)
            collider = UsdGeom.Plane.Define(stage, prim_path + "/CollisionPlane")
            UsdPhysics.CollisionAPI.Apply(collider.GetPrim()).CreateCollisionEnabledAttr(True)
            UsdGeom.Mesh.Define(stage, prim_path + "/Mesh")
            events.append("ground")
            # Match the reported API: no set_visibility on the GroundPlane wrapper.
            return SimpleNamespace(prim_path=prim_path)

    class World:
        def __init__(self, **kwargs):
            self.scene = Scene()

        def reset(self):
            events.append("reset")

    def module(name, **attrs):
        result = ModuleType(name)
        result.__dict__.update(attrs)
        return result

    api = module("isaacsim.core.api", World=World)
    core = module("isaacsim.core", api=api)
    isaac = module("isaacsim", core=core, SimulationApp=lambda kwargs: object())
    usd = module("omni.usd", get_context=lambda: SimpleNamespace(get_stage=lambda: stage))
    for name, value in {
        "isaaclab": None, "isaaclab.app": None,
        "isaacsim": isaac, "isaacsim.core": core, "isaacsim.core.api": api,
        "omni": module("omni", usd=usd), "omni.usd": usd,
    }.items():
        monkeypatch.setitem(sys.modules, name, value)

    sim = SimulationManager.__new__(SimulationManager)
    sim.config = SimulationConfig(enable_visualization=visualization)
    sim._dt = 1 / 60
    sim._pre_app_rtx_guard = lambda: None
    sim._build_app_launch_args = lambda: {}
    sim._resolve_isaaclab_experience_file = lambda: None
    sim._apply_rtx_settings = lambda: None
    sim._build_visualization = lambda: events.append("visualization")
    sim._start_isaac()

    expected = UsdGeom.Tokens.invisible if visualization else UsdGeom.Tokens.inherited
    assert UsdGeom.Imageable(stage.GetPrimAtPath(ground_path + "/Mesh")).ComputeVisibility() == expected
    collider = stage.GetPrimAtPath(ground_path + "/CollisionPlane")
    assert collider.HasAPI(UsdPhysics.CollisionAPI)
    assert UsdPhysics.CollisionAPI(collider).GetCollisionEnabledAttr().Get() is True
    assert events == (["ground", "reset", "visualization"] if visualization else ["ground", "reset"])
