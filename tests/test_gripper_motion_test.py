"""Control-loop and composed USD regression checks, not actual Isaac validation."""
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fdw_sim.visualization.gripper_motion_test import (
    run_gripper_cycle, inspect_motion_prerequisites, articulation_root_paths,
    bind_live_gripper, compose_gripper_experiment, require_physics_scene,
    ensure_physx_attachment,
)


class Plant:
    def __init__(self):
        self.q = 0.0
        self.target = 0.0
        self.commands = []
        self.paused = False
        self.steps = 0
        self.stuck = False
        self.right_stuck = False

    def read(self):
        return self.q

    def command(self, target):
        self.target = target
        self.commands.append(target)

    def pads(self):
        inward = 0.08*self.q
        return [[-0.07+inward, 0, 0], [0.07 if self.right_stuck else 0.07-inward, 0, 0]]

    def step(self):
        self.steps += 1
        if not self.stuck:
            self.q += 0.6*(self.target-self.q)

    def stop(self):
        self.paused = True


def run(plant, **overrides):
    args = dict(read_joint=plant.read, command=plant.command, read_pads=plant.pads,
                step=plant.step, stop=plant.stop, limits=(0, math.pi/4), dt=0.05)
    args.update(overrides)
    return run_gripper_cycle(**args)


def test_bilateral_close_return_twice_has_independent_measured_evidence():
    plant = Plant()
    result = run(plant)
    assert result["status"] == "PASSED_UNLOADED_GRIPPER_CYCLE"
    assert result["passed_legs"] == 4 and result["returned_to_start"] and plant.paused
    assert all(v > 0.001 for row in result["legs"] if row["leg"] == "partial_close" for v in row["inward_displacements_m"])
    assert result["samples"] and not result["physical_grasp_verified"]
    assert max(plant.commands) == pytest.approx(math.radians(10))
    assert max(abs(np.diff(plant.commands))) <= math.radians(0.126)
    json.dumps(result, allow_nan=False)


def test_primary_joint_motion_alone_cannot_pass_when_right_finger_stays_still():
    plant = Plant()
    plant.right_stuck = True
    result = run(plant)
    assert result["status"] == "FAILED" and result["passed_legs"] == 0
    assert "both fingers" in result["errors"][0] and plant.paused
    assert plant.commands[-1] == plant.q


def test_frozen_joint_feedback_times_out_and_pauses():
    plant = Plant()
    plant.stuck = True
    result = run(plant)
    assert result["status"] == "FAILED" and plant.paused
    assert result["passed_legs"] == 0


def test_static_usd_geometry_cannot_pass_even_if_joint_feedback_tracks():
    plant = Plant()
    result = run(plant, read_pads=lambda: [[-.07, 0, 0], [.07, 0, 0]])
    assert "both fingers" in result["errors"][0] and result["status"] == "FAILED"


def test_interrupt_holds_measured_joint_without_automatic_return():
    plant = Plant()
    result = run(plant, is_running=lambda: plant.steps < 20)
    assert result["status"] == "FAILED" and plant.paused and plant.q > 0
    assert plant.commands[-1] == plant.q


@pytest.mark.parametrize("options", [
    {"limits": (0, math.radians(5))}, {"dt": float("nan")},
    {"read_joint": lambda: float("nan")}, {"read_pads": lambda: np.full((2, 3), np.nan)},
])
def test_invalid_preflight_sends_no_commands(options):
    plant = Plant()
    result = run(plant, **options)
    assert result["status"] == "FAILED" and not plant.commands and plant.paused


def test_constraint_roundoff_does_not_produce_out_of_limit_commands():
    plant = Plant()
    plant.q = -0.00001
    result = run(plant)
    assert result["status"] == "PASSED_UNLOADED_GRIPPER_CYCLE"
    assert result["open_target_rad"] == 0
    assert min(plant.commands) >= 0


def test_finger_geometry_stuck_closed_cannot_pass_return_leg():
    plant = Plant()
    def pads():
        q = plant.q if plant.steps < 115 else math.radians(10)
        return [[-0.07+0.08*q, 0, 0], [0.07-0.08*q, 0, 0]]
    result = run(plant, read_pads=pads)
    assert result["status"] == "FAILED" and result["passed_legs"] == 1
    assert "return" in result["errors"][0]


@pytest.fixture
def usd_model():
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/Robotiq_2F_140/robotiq_base_link")
    for side in ("left", "right"):
        UsdGeom.Cube.Define(stage, f"/Robotiq_2F_140/{side}_inner_finger/Fingertip_01")
    primary = UsdPhysics.RevoluteJoint.Define(stage, "/Robotiq_2F_140/finger_joint")
    primary.CreateLowerLimitAttr(0)
    primary.CreateUpperLimitAttr(45)
    UsdPhysics.DriveAPI.Apply(primary.GetPrim(), "angular").CreateStiffnessAttr(0.1125)
    follower = UsdPhysics.RevoluteJoint.Define(stage, "/Robotiq_2F_140/right_outer_knuckle_joint").GetPrim()
    follower.CreateRelationship("physxMimicJoint:rotX:referenceJoint").SetTargets([primary.GetPath()])
    follower.CreateAttribute("physxMimicJoint:rotX:gearing", Sdf.ValueTypeNames.Float).Set(-1)
    joint = UsdPhysics.FixedJoint.Define(stage, "/FixedJoint")
    joint.CreateBody1Rel().SetTargets(["/Robotiq_2F_140/robotiq_base_link"])
    return stage


def test_prerequisites_read_live_relationships_without_mutating_asset(usd_model):
    before = usd_model.GetRootLayer().ExportToString()
    result = inspect_motion_prerequisites(usd_model)
    assert result["mimic_gearing"] == -1
    assert result["limit_rad"] == pytest.approx([0, math.pi/4])
    assert len(result["pad_paths"]) == 2
    assert usd_model.GetRootLayer().ExportToString() == before


@pytest.mark.parametrize("fault", ["mimic", "gearing", "follower_drive", "anchor"])
def test_bad_mimic_or_mount_configuration_blocks_test(usd_model, fault):
    from pxr import UsdPhysics
    follower = usd_model.GetPrimAtPath("/Robotiq_2F_140/right_outer_knuckle_joint")
    if fault == "mimic":
        follower.GetRelationship("physxMimicJoint:rotX:referenceJoint").SetTargets([])
    elif fault == "gearing":
        follower.GetAttribute("physxMimicJoint:rotX:gearing").Set(1)
    elif fault == "follower_drive":
        UsdPhysics.DriveAPI.Apply(follower, "angular").CreateStiffnessAttr(1)
    else:
        usd_model.RemovePrim("/FixedJoint")
    with pytest.raises(ValueError):
        inspect_motion_prerequisites(usd_model)


def test_articulation_paths_are_retraversed_after_instance_proxy_expires():
    from pxr import Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/Template")
    UsdGeom.Scope.Define(stage, "/Template/Looks")
    instance = UsdGeom.Xform.Define(stage, "/Gripper").GetPrim()
    instance.GetReferences().AddInternalReference("/Template")
    instance.SetInstanceable(True)
    old_proxy = stage.GetPrimAtPath("/Gripper/Looks")
    assert old_proxy.IsInstanceProxy()
    UsdPhysics.ArticulationRootAPI.Apply(UsdPhysics.FixedJoint.Define(stage, "/FixedJoint").GetPrim())
    instance.SetInstanceable(False)
    with pytest.raises(RuntimeError, match="expired"):
        old_proxy.GetTypeName()
    assert articulation_root_paths(stage) == ["/FixedJoint"]
    assert stage.GetPrimAtPath("/Gripper/Looks").GetTypeName() == "Scope"


def test_binding_creates_fresh_tensor_view_only_after_final_play_and_physics_step(usd_model):
    events = []
    class World:
        def play(self):
            events.append("play")
        def step(self, *, render):
            assert render
            events.append("step")
    class View:
        def set_subspace_roots(self, root):
            assert root == "/"
            events.append("subspace")
    fresh = View()
    def make_view(backend):
        assert backend == "numpy" and events == ["play", "step", "attach"]
        events.append("new_view")
        return fresh
    def prepare_attachment():
        assert events == ["play", "step"]
        events.append("attach")
        return {"usd_stage_id": 42, "physx_stage_id_after": 42}
    class Art:
        dof_names = ["right_outer_knuckle_joint", "finger_joint"]
        def __init__(self, **kwargs):
            assert not kwargs["reset_xform_properties"]
            events.append("new_art")
        def initialize(self, *, physics_sim_view):
            assert physics_sim_view is fresh
            events.append("initialize")
        def get_joint_positions(self):
            assert events[-1] == "initialize"
            events.append("read")
            return np.array([0.011, 0.010])
        def apply_action(self, *args):
            pytest.fail("binding must not issue drive commands")
    art, names, metadata = bind_live_gripper(lambda: usd_model, World(), Art, make_view,
                                            prepare_attachment=prepare_attachment)
    assert metadata["commanded_joint_index"] == 1
    assert metadata["fresh_physics_view_after_final_play"]
    assert metadata["usd_stage_id"] == metadata["physx_stage_id_after"] == 42
    assert events == ["play", "step", "attach", "new_view", "subspace", "new_art", "initialize", "read"]


def test_fresh_binding_rejects_missing_runtime_feedback(usd_model):
    from types import SimpleNamespace
    world = SimpleNamespace(play=lambda: None, step=lambda **kw: None)
    view = SimpleNamespace(set_subspace_roots=lambda root: None)
    art = SimpleNamespace(initialize=lambda **kw: None, dof_names=["finger_joint"], get_joint_positions=lambda: None)
    with pytest.raises(ValueError, match="finite runtime feedback"):
        bind_live_gripper(lambda: usd_model, world, lambda **kw: art, lambda backend: view)


def test_anonymous_root_preserves_scene_and_source_asset_when_session_is_cleared(usd_model):
    from pxr import Usd, UsdGeom, UsdPhysics
    source = usd_model
    source.SetDefaultPrim(source.GetPrimAtPath("/Robotiq_2F_140"))
    before = source.GetRootLayer().ExportToString()
    stage = Usd.Stage.CreateInMemory()
    metadata = compose_gripper_experiment(stage, source.GetRootLayer().identifier,
                                         meters=0.01, up_axis="Z", default_prim_path="/Robotiq_2F_140")
    scene = UsdPhysics.Scene.Define(stage, "/World/GripperPhysicsScene")
    scene.CreateGravityMagnitudeAttr(0)
    UsdGeom.Camera.Define(stage, "/World/GripperTest/Camera")
    stage.SetEditTarget(stage.GetSessionLayer())
    UsdGeom.Scope.Define(stage, "/Temporary")
    stage.GetSessionLayer().Clear()
    require_physics_scene(stage, "/World/GripperPhysicsScene", "after session clear")
    assert stage.GetPrimAtPath("/World/GripperTest/Camera")
    assert stage.GetPrimAtPath("/FixedJoint")  # Include the source's top-level world anchor too.
    assert UsdGeom.GetStageMetersPerUnit(stage) == 0.01
    assert stage.GetDefaultPrim().GetPath() == source.GetDefaultPrim().GetPath()
    assert source.GetRootLayer().ExportToString() == before
    assert metadata["experiment_root_is_anonymous"]


def test_session_only_scene_is_lost_and_guard_rejects_it(usd_model):
    from pxr import UsdPhysics
    usd_model.SetEditTarget(usd_model.GetSessionLayer())
    UsdPhysics.Scene.Define(usd_model, "/SessionPhysicsScene")
    usd_model.GetSessionLayer().Clear()
    with pytest.raises(ValueError, match="physics scene missing"):
        require_physics_scene(usd_model, "/SessionPhysicsScene", "after reset")


def test_missing_scene_blocks_tensor_creation_before_final_play(usd_model):
    from types import SimpleNamespace
    def forbidden(*a, **kw):
        pytest.fail("must reject missing scene before play or tensor binding")
    world = SimpleNamespace(play=forbidden, step=forbidden)
    with pytest.raises(ValueError, match="before final play"):
        bind_live_gripper(lambda: usd_model, world, forbidden, forbidden, physics_scene_path="/DeletedScene")


def test_experiment_refuses_to_edit_file_backed_source(tmp_path):
    from pxr import Usd
    stage = Usd.Stage.CreateNew(str(tmp_path / "source.usda"))
    with pytest.raises(ValueError, match="anonymous root"):
        compose_gripper_experiment(stage, "other.usda", meters=1, up_axis="Z", default_prim_path=None)


@pytest.mark.parametrize("initial", [0, -1, 2**64-1])
def test_detached_physx_stage_is_explicitly_attached_and_verified(initial):
    class Interface:
        attached = initial
        def get_attached_stage(self):
            return self.attached
        def attach_stage(self, stage_id):
            self.attached = stage_id
            return True
        def flush_changes(self):
            assert self.attached == 42
    result = ensure_physx_attachment(Interface(), 42)
    assert result["physx_stage_id_after"] == result["usd_stage_id"] == 42
    assert result["physx_stage_attached_explicitly"]


def test_valid_physx_attachment_is_not_detached_or_replaced():
    from types import SimpleNamespace
    def forbidden(*args):
        pytest.fail("must not replace an already valid physics attachment")
    interface = SimpleNamespace(get_attached_stage=lambda: 42, attach_stage=forbidden, flush_changes=lambda: None)
    assert not ensure_physx_attachment(interface, 42)["physx_stage_attached_explicitly"]


def test_other_attached_stage_is_rejected_before_replacing_it():
    from types import SimpleNamespace
    def forbidden(*args):
        pytest.fail("must not detach another stage")
    interface = SimpleNamespace(get_attached_stage=lambda: 15, attach_stage=forbidden)
    with pytest.raises(ValueError, match="another stage"):
        ensure_physx_attachment(interface, 42)


@pytest.mark.parametrize("attach_success", [False, True])
def test_failed_or_mismatched_attachment_cannot_proceed(attach_success):
    from types import SimpleNamespace
    interface = SimpleNamespace(get_attached_stage=lambda: 0, attach_stage=lambda stage_id: attach_success,
                                flush_changes=lambda: None)
    with pytest.raises(RuntimeError, match="attach|mismatch"):
        ensure_physx_attachment(interface, 42)
