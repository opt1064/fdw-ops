"""USD assembly/feedback regression checks; not real Isaac mounting/contact validation."""
import json
import math
from types import SimpleNamespace

import numpy as np
import pytest
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from fdw_sim.visualization.gripper_motion_test import compose_gripper_experiment, articulation_root_paths, bind_live_gripper
from fdw_sim.visualization.gripper_mount_test import (
    mount_fanuc_gripper, mount_frame_observation, StationaryArmGuard, check_mount_unit_compatibility,
)


@pytest.fixture
def assembly():
    arm = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(arm, 1)
    root = UsdGeom.Xform.Define(arm, "/Robot")
    arm.SetDefaultPrim(root.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    for i, name in enumerate(["base", *[f"L{j}" for j in range(1, 7)]]):
        body = UsdGeom.Xform.Define(arm, "/Robot/"+name)
        body.AddTranslateOp().Set((i*0.1, 0, 0))
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
        UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr(2)
    for j in range(1, 7):
        joint = UsdPhysics.RevoluteJoint.Define(arm, f"/Robot/J{j}")
        joint.CreateBody0Rel().SetTargets(["/Robot/base" if j == 1 else f"/Robot/L{j-1}"])
        joint.CreateBody1Rel().SetTargets([f"/Robot/L{j}"])
        UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular").CreateStiffnessAttr(500)
    flange = UsdGeom.Xform.Define(arm, "/Robot/L6/flange")
    flange.AddTranslateOp().Set((0, 0, 0.03))
    flange.AddRotateYOp().Set(90)
    gripper = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(gripper, 1)
    root = UsdGeom.Xform.Define(gripper, "/Robotiq_2F_140")
    root.AddRotateYOp().Set(90)
    gripper.SetDefaultPrim(root.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    base = UsdGeom.Xform.Define(gripper, "/Robotiq_2F_140/robotiq_base_link")
    base.AddTranslateOp().Set((0, 0.02, 0.01))
    base.AddRotateXOp().Set(45)
    UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
    UsdPhysics.MassAPI.Apply(base.GetPrim()).CreateMassAttr(1.2)
    tcp = UsdGeom.Xform.Define(gripper, "/Robotiq_2F_140/robotiq_base_link/grasp_frame")
    tcp.AddTranslateOp().Set((0, 0, 0.18))
    anchor = UsdPhysics.FixedJoint.Define(gripper, "/FixedJoint")
    anchor.CreateBody1Rel().SetTargets([base.GetPath()])
    stage = Usd.Stage.CreateInMemory()
    compose_gripper_experiment(stage, gripper.GetRootLayer().identifier, meters=1, up_axis="Z",
                               default_prim_path="/Robotiq_2F_140")
    setup = {"base_path": str(base.GetPath()), "anchor_path": "/FixedJoint"}
    # Keep the anonymous source stages/layers alive for references throughout each test.
    return stage, arm, gripper, setup


def test_direct_mount_aligns_frames_preserves_sources_and_builds_single_tree(assembly):
    stage, arm, gripper, setup = assembly
    arm_before, gripper_before = arm.GetRootLayer().ExportToString(), gripper.GetRootLayer().ExportToString()
    mount = mount_fanuc_gripper(stage, arm.GetRootLayer().identifier, setup)
    observation = mount_frame_observation(stage, mount)
    assert observation["flange_to_base_error"]["position_error_m"] < 1e-6
    assert observation["flange_to_base_error"]["orientation_error_rad"] < 1e-6
    np.testing.assert_allclose(np.array(observation["flange_to_tcp_candidate_matrix_m"])[:3, 3], [0, 0, .18], atol=1e-6)
    assert not observation["tcp_calibration_verified"] and not mount["mount_calibration_verified"]
    assert not mount["tool_changer_present"]
    joint = UsdPhysics.FixedJoint(stage.GetPrimAtPath(mount["mount_joint_path"]))
    assert list(map(str, joint.GetBody0Rel().GetTargets())) == ["/World/Fanuc/L6"]
    assert list(map(str, joint.GetBody1Rel().GetTargets())) == [setup["base_path"]]
    assert joint.GetJointEnabledAttr().Get() and not joint.GetExcludeFromArticulationAttr().Get()
    assert not joint.GetCollisionEnabledAttr().Get()
    assert not UsdPhysics.FixedJoint(stage.GetPrimAtPath("/FixedJoint")).GetJointEnabledAttr().Get()
    assert articulation_root_paths(stage) == ["/World/Fanuc"]
    assert arm.GetRootLayer().ExportToString() == arm_before
    assert gripper.GetRootLayer().ExportToString() == gripper_before
    assert UsdPhysics.MassAPI(stage.GetPrimAtPath(setup["base_path"])).GetMassAttr().Get() == pytest.approx(1.2)
    json.dumps(mount, allow_nan=False)
    json.dumps(observation, allow_nan=False)


def test_mount_joint_local_frames_coincide_in_world(assembly):
    stage, arm, _, setup = assembly
    mount = mount_fanuc_gripper(stage, arm.GetRootLayer().identifier, setup)
    joint = UsdPhysics.FixedJoint(stage.GetPrimAtPath(mount["mount_joint_path"]))
    cache = UsdGeom.XformCache()
    frames = []
    for i in (0, 1):
        body = stage.GetPrimAtPath(getattr(joint, f"GetBody{i}Rel")().GetTargets()[0])
        pos = getattr(joint, f"GetLocalPos{i}Attr")().Get()
        quat = getattr(joint, f"GetLocalRot{i}Attr")().Get()
        local = Gf.Matrix4d(1)
        local.SetRotate(Gf.Quatd(quat))
        local.SetTranslateOnly(Gf.Vec3d(pos))
        frames.append(local*cache.GetLocalToWorldTransform(body))
    np.testing.assert_allclose(np.array(frames[0]), np.array(frames[1]), atol=1e-6)


def test_real_robotiq_unit_roundoff_allows_mount_without_scaling_or_source_edits(assembly):
    stage, arm, gripper, setup = assembly
    actual_robotiq_meters = 0.9999999776482582
    UsdGeom.SetStageMetersPerUnit(gripper, actual_robotiq_meters)
    UsdGeom.SetStageMetersPerUnit(stage, actual_robotiq_meters)
    before = gripper.GetRootLayer().ExportToString()
    mount = mount_fanuc_gripper(stage, arm.GetRootLayer().identifier, setup)
    units = mount["unit_compatibility"]
    assert units["source_meters_per_unit"] == 1
    assert units["stage_meters_per_unit"] == actual_robotiq_meters
    assert units["relative_difference"] == pytest.approx(2.2351741790771484e-8)
    assert not units["unit_rescaling_applied"]
    observation = mount_frame_observation(stage, mount)
    assert observation["flange_to_base_error"]["position_error_m"] < 1e-6
    assert observation["flange_to_base_error"]["orientation_error_rad"] < 1e-6
    assert UsdGeom.GetStageMetersPerUnit(stage) == actual_robotiq_meters
    assert gripper.GetRootLayer().ExportToString() == before


@pytest.mark.parametrize("source,stage", [(1, 1), (1, 0.9999999776482582), (.01, .0100000001), (1e-8, 1e-8*(1-2e-8))])
def test_unit_comparison_accepts_only_relative_roundoff(source, stage):
    report = check_mount_unit_compatibility(source, stage)
    assert report["relative_difference"] <= report["relative_tolerance"]
    assert not report["unit_rescaling_applied"]


@pytest.mark.parametrize("source,stage", [(1, .01), (.01, 1), (1, 1.000001), (1e-8, 2e-8), (0, 1), (-1, 1),
                                           (float("nan"), 1), (1, float("inf"))])
def test_unit_comparison_rejects_true_mismatch_and_invalid_metadata(source, stage):
    with pytest.raises(ValueError):
        check_mount_unit_compatibility(source, stage)


def test_mount_survives_clearing_session_layer(assembly):
    stage, arm, _, setup = assembly
    mount = mount_fanuc_gripper(stage, arm.GetRootLayer().identifier, setup)
    stage.GetSessionLayer().Clear()
    assert stage.GetPrimAtPath(mount["mount_joint_path"])
    assert not UsdPhysics.FixedJoint(stage.GetPrimAtPath("/FixedJoint")).GetJointEnabledAttr().Get()


def test_existing_arm_world_anchor_is_preserved(assembly):
    stage, arm, _, setup = assembly
    joint = UsdPhysics.FixedJoint.Define(arm, "/Robot/Anchor")
    joint.CreateBody1Rel().SetTargets(["/Robot/base"])
    mount = mount_fanuc_gripper(stage, arm.GetRootLayer().identifier, setup)
    assert mount["arm_world_anchors"] == ["/World/Fanuc/Anchor"]
    assert not stage.GetPrimAtPath("/World/FanucWorldAnchor")


def test_fanuc_fixed_root_joint_schema_is_preserved_and_binding_is_scoped(assembly):
    stage, arm, _, setup = assembly
    arm.GetDefaultPrim().RemoveAPI(UsdPhysics.ArticulationRootAPI)
    anchor = UsdPhysics.FixedJoint.Define(arm, "/Robot/root_joint")
    anchor.CreateBody1Rel().SetTargets(["/Robot/base"])
    UsdPhysics.ArticulationRootAPI.Apply(anchor.GetPrim())
    mount = mount_fanuc_gripper(stage, arm.GetRootLayer().identifier, setup)
    assert mount["arm_articulation_path"] == "/World/Fanuc/root_joint"
    attempted = []
    class Art:
        dof_names = ["finger_joint", *[f"J{i}" for i in range(1, 7)]]
        def __init__(self, prim_path, **kwargs):
            attempted.append(prim_path)
            self.path = prim_path
        def initialize(self, **kwargs):
            if self.path != mount["arm_articulation_path"]:
                raise ValueError("requires exact articulation root joint")
        def get_joint_positions(self): return np.zeros(7)
    view = SimpleNamespace(set_subspace_roots=lambda path: None)
    world = SimpleNamespace(play=lambda: None, step=lambda **kwargs: None)
    _, _, metadata = bind_live_gripper(lambda: stage, world, Art, lambda backend: view,
                                      articulation_path=mount["robot_path"])
    assert attempted == ["/World/Fanuc", "/World/Fanuc/root_joint"]
    assert metadata["live_articulation_path"] == mount["arm_articulation_path"]


@pytest.mark.parametrize("fault", ["units", "flange", "grasp", "joint_name", "external_body", "root", "base_reset"])
def test_invalid_assembly_blocks_before_runtime(assembly, fault):
    stage, arm, gripper, setup = assembly
    if fault == "units":
        UsdGeom.SetStageMetersPerUnit(arm, .01)
    elif fault == "flange":
        arm.RemovePrim("/Robot/L6/flange")
    elif fault == "grasp":
        gripper.RemovePrim("/Robotiq_2F_140/robotiq_base_link/grasp_frame")
    elif fault == "joint_name":
        arm.RemovePrim("/Robot/J6")
    elif fault == "external_body":
        UsdPhysics.Joint(arm.GetPrimAtPath("/Robot/J6")).GetBody0Rel().SetTargets(["/OtherBody"])
    elif fault == "root":
        arm.GetPrimAtPath("/Robot").RemoveAPI(UsdPhysics.ArticulationRootAPI)
    else:
        UsdGeom.Xformable(stage.GetPrimAtPath(setup["base_path"])).SetResetXformStack(True)
    with pytest.raises(ValueError):
        mount_fanuc_gripper(stage, arm.GetRootLayer().identifier, setup)


@pytest.fixture
def arm_view():
    names = ["finger_joint", "J6", "right_outer_knuckle_joint", "J1", "J4", "J2", "J5", "J3"]
    class View:
        shared_metatype = SimpleNamespace(dof_names=names)
        q = np.array([[.1, .2, .3, .4, .5, .6, .7, .8]], dtype=np.float32)
        targets = np.ones((1, 8), dtype=np.float32)*.9
        velocities = np.ones((1, 8), dtype=np.float32)*.1
        stiffness = np.ones((1, 8), dtype=np.float32)*10
        def get_dof_positions(self): return self.q
        def get_dof_position_targets(self): return self.targets
        def get_dof_velocity_targets(self): return self.velocities
        def get_dof_stiffnesses(self): return self.stiffness
        def set_dof_position_targets(self, data, indices): self.targets = data.copy()
        def set_dof_velocity_targets(self, data, indices): self.velocities = data.copy()
    return View()


def test_arm_hold_uses_named_joint_mapping_and_preserves_gripper_targets(arm_view):
    guard = StationaryArmGuard(arm_view)
    guard.hold()
    np.testing.assert_array_equal(arm_view.targets[0, guard.indices], arm_view.q[0, guard.indices])
    np.testing.assert_array_equal(arm_view.targets[0, [0, 2]], np.array([.9, .9], dtype=np.float32))
    assert np.all(arm_view.velocities[0, guard.indices] == 0)
    np.testing.assert_array_equal(arm_view.velocities[0, [0, 2]], np.array([.1, .1], dtype=np.float32))
    guard.check()
    assert guard.diagnostics()["hold_commands_issued"]
    assert not guard.diagnostics()["arm_production_motion_verified"]


def test_arm_drift_aborts(arm_view):
    guard = StationaryArmGuard(arm_view)
    arm_view.q[0, guard.indices[3]] += math.radians(.6)
    with pytest.raises(RuntimeError, match="joint drift"):
        guard.check()
    assert guard.observed_drift_deg > .5


def test_nonfinite_arm_feedback_and_missing_hold_drive_block(arm_view):
    arm_view.stiffness[0, 1] = 0
    with pytest.raises(ValueError, match="positive stiffness"):
        StationaryArmGuard(arm_view)
    arm_view.stiffness[0, 1] = 10
    guard = StationaryArmGuard(arm_view)
    arm_view.q[0, 1] = np.nan
    with pytest.raises(ValueError, match="finite merged arm"):
        guard.check()
