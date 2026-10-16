"""Analytical scene/decision fixtures; these tests do not prove an Isaac physical transfer."""
import math

import numpy as np
import pytest
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from fdw_sim.visualization.grasp_transfer_test import (
    plan_transfer, create_target_support, audit_target_clearance,
    gripper_collision_world_points, plan_empty_return,
)
from fdw_sim.visualization.physical_grasp_test import ContactEvidence, run_contact_grasp
from fdw_sim.visualization.mounted_arm_motion_test import MovingArmController
from test_mounted_arm_motion_test import Plant
from test_physical_grasp_test import emit


class Model:
    limits = np.tile([-math.pi, math.pi], (6, 1))
    def predict(self, q):
        pose = np.eye(4)
        pose[:3, 3] = [0, .9*q[0], .5+.8*q[1]+.35*q[4]]
        return pose, pose


def fixture():
    pose = np.eye(4); pose[2, 3] = .5
    return dict(initial_object_world_matrix_m=pose.tolist(), dimensions_base_m=[.02, .11, .015],
                support_dimensions_world_m=[.013, .0715, .01], support_top_world_z_m=.492,
                block_path="/World/GraspTest/Block", support_path="/World/GraspTest/Support",
                meters_per_unit=1., kilograms_per_unit=1., scope="CPU fake feedback")


def poses():
    lift = np.zeros(6); lift[4] = math.radians(2)
    return np.zeros(6), lift


def test_transfer_plan_requires_real_distance_separate_support_and_clearance():
    start, lift = poses()
    plan = plan_transfer(Model(), start, lift, fixture())
    assert .10 <= plan["horizontal_distance_m"] <= .15
    assert abs(plan["j1_delta_deg"]) == 8
    assert plan["target_support_dimensions_m"][1] == .02
    assert np.array(plan["down_rad"])[4] == 0
    assert plan["sweep_samples"] == 41
    model = Model(); model.limits = np.tile([-.01, .01], (6, 1))
    with pytest.raises(ValueError, match="no limit-safe"):
        plan_transfer(model, start, lift, fixture())
    with pytest.raises(ValueError, match="no limit-safe"):
        plan_transfer(Model(), start, start, fixture())
    f = fixture(); f["dimensions_base_m"][1] = .4
    with pytest.raises(ValueError, match="separate support"):
        plan_transfer(Model(), start, lift, f)


def test_target_is_static_rotated_support_and_never_changes_block():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    f = fixture()
    source = UsdGeom.Cube.Define(stage, f["support_path"])
    source.CreateSizeAttr(1)
    source.AddTranslateOp().Set(Gf.Vec3d(0, 0, .487))
    source.AddScaleOp().Set(Gf.Vec3f(.013, .0715, .01))
    UsdPhysics.CollisionAPI.Apply(source.GetPrim())
    block = UsdGeom.Cube.Define(stage, f["block_path"])
    block.AddTranslateOp().Set(Gf.Vec3d(0, 0, .5))
    start, lift = poses()
    plan = plan_transfer(Model(), start, lift, f)
    create_target_support(stage, f, plan)
    target = stage.GetPrimAtPath(f["target_support_path"])
    assert target.HasAPI(UsdPhysics.CollisionAPI) and not target.HasAPI(UsdPhysics.RigidBodyAPI)
    assert block.GetPrim().GetAttribute("xformOp:translate").Get() == Gf.Vec3d(0, 0, .5)
    assert not any(p.IsA(UsdPhysics.Joint) for p in stage.Traverse())
    matrix = UsdGeom.XformCache().GetLocalToWorldTransform(target)
    assert matrix.ExtractTranslation()[1] == pytest.approx(np.array(plan["target_object_world_matrix_m"])[1, 3])
    with pytest.raises(ValueError, match="already exists"):
        create_target_support(stage, f, plan)


def test_contacts_distinguish_source_from_target_and_clear_each_step():
    f = fixture(); f["target_support_path"] = "/World/GraspTest/TargetSupport"
    evidence = ContactEvidence(f, ["/L", "/R"], 9, .01, str, "LOST")
    emit(evidence, other=f["target_support_path"])
    assert evidence.read()["target_support"] == 1
    assert evidence.read()["support"] == 0
    evidence.begin_step()
    assert evidence.read()["target_support"] == 0


def test_target_clearance_audit_rejects_hidden_robot_overlap_and_missing_geometry():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    target = UsdGeom.Cube.Define(stage, "/Target")
    target.CreateSizeAttr(.02)
    f = {"target_support_path": "/Target", "meters_per_unit": 1.}
    collider = UsdGeom.Cube.Define(stage, "/Robot/Collider")
    collider.CreateSizeAttr(.02)
    move = collider.AddTranslateOp()
    move.Set(Gf.Vec3d(.1, 0, 0))
    UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
    collider.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    audit = audit_target_clearance(stage, f, ["/Robot"])
    assert audit["checked_colliders"] == 1
    assert audit["minimum_aabb_distance_m"] == pytest.approx(.08)
    move.Set(Gf.Vec3d(.01, 0, 0))
    with pytest.raises(RuntimeError, match="overlaps"):
        audit_target_clearance(stage, f, ["/Robot"])
    UsdPhysics.CollisionAPI(collider.GetPrim()).CreateCollisionEnabledAttr(False)
    with pytest.raises(ValueError, match="no robot collision"):
        audit_target_clearance(stage, f, ["/Robot"])


def test_transfer_controller_expands_only_explicit_j1_envelope():
    plant = Plant()
    default = MovingArmController(plant, 2)
    command = np.zeros(6); command[0] = math.radians(8)
    with pytest.raises(ValueError, match="envelope"): default.set_target(command)
    transfer = MovingArmController(plant, 2, transfer_j1_deg=8)
    transfer.set_target(command)
    command[1] = math.radians(1)
    with pytest.raises(ValueError, match="envelope"): transfer.set_target(command)
    with pytest.raises(ValueError, match="transfer J1"): MovingArmController(plant, 2, transfer_j1_deg=9)
    command[1] = math.radians(3)
    transfer.allow_empty_return_j2()
    transfer.set_target(command)
    command[1] = math.radians(9)
    with pytest.raises(ValueError, match="envelope"): transfer.set_target(command)


def run_fake(fault=None, empty_return=True):
    f = fixture(); start, lift = poses()
    f["transfer"] = plan_transfer(Model(), start, lift, f)
    f["target_support_path"] = "/World/GraspTest/TargetSupport"
    goal_y = np.array(f["transfer"]["target_object_world_matrix_m"])[1, 3]
    s = dict(phase=None, grip=.07, arm=start.copy(), paused=False)
    def color(phase): s["phase"] = phase
    def command(grip, arm): s.update(grip=grip, arm=arm.copy())
    def observe():
        phase, arm, grip = s["phase"], s["arm"], s["grip"]
        released = phase in ("OPEN_RELEASE", "RETREAT_EMPTY", "RAISE_FOR_RETURN", "RETURN_ABOVE_SOURCE", "PARK_EMPTY")
        y, rise = (.9*arm[0], .35*arm[4]) if not released else (goal_y, 0)
        if fault == "frozen_object": y = 0
        if fault == "falls_during_transfer" and phase == "TRANSFER": rise = 0
        if fault == "moves_during_retreat" and phase == "RETREAT_EMPTY": y += .01
        pose = np.eye(4); pose[:3, 3] = [0, y, .4995+rise]
        relative = np.eye(4); relative[2, 3] = .4995
        if fault == "slip" and phase == "TRANSFER": relative[1, 3] = .01
        both = not released and phase != "SETTLE_ON_SUPPORT" and grip > .12
        forces = dict(left=float(both), right=float(both),
                      support=.4905 if abs(y) < .003 and rise < .001 else 0.,
                      target_support=.4905 if abs(y-goal_y) < .003 and rise < .001 else 0.)
        if fault == "no_target_contact": forces["target_support"] = 0
        if fault == "source_contact_only" and released:
            forces.update(target_support=0, support=.4905)
        if fault == "one_finger" and phase == "TRANSFER": forces["right"] = 0
        if fault == "retained_finger" and released: forces["left"] = .1
        if fault == "finger_recontacts_during_return" and phase == "RETURN_ABOVE_SOURCE": forces["left"] = .129
        clearance = .8*arm[1]+.35*arm[4]-.02
        if fault == "insufficient_return_height": clearance = -.001
        return dict(object_world_matrix_m=pose.tolist(), object_relative_base_matrix_m=relative.tolist(),
                    object_speed_m_s=0., support_clearance_m=rise, target_support_clearance_m=rise,
                    gripper_clearance_above_placed_object_m=clearance,
                    contacts=forces, gripper_rad=grip, gripper_speed_rad_s=0., arm_target_error_rad=0., arm_speed_rad_s=0.)
    def stop(): s["paused"] = True
    def prepare_return():
        s["planner_phase"] = s["phase"]
        high = np.array(f["transfer"]["across_rad"]); high[1] = math.radians(3)
        home = high.copy(); home[0] = 0
        return high, home, {"source": "explicit synthetic planner fixture"}
    result = run_contact_grasp(command, observe, lambda: None, stop, color, f, start, lift,
                               .07, .24, dt=1/120, empty_return_planner=prepare_return if empty_return else None)
    return result, s


def test_transfer_requires_12_phases_and_target_survives_empty_return():
    r, s = run_fake()
    assert r["status"] == "PASSED_SIMULATED_GRASP_TRANSFER_PLACE", r["errors"]
    assert r["passed_phases"] == 12 and r["transfer_verified"] and r["target_placement_verified"]
    assert not r["returned_to_start"] and r["empty_gripper_parked"] and r["returned_above_source"] and s["paused"]
    assert r["phases"][4]["measured_horizontal_distance_m"] > .10
    assert r["phases"][7]["measured_setdown_descent_m"] >= .005
    assert r["phases"][9]["phase"] == "RAISE_FOR_RETURN" and s["planner_phase"] == "RETREAT_EMPTY"


@pytest.mark.parametrize("fault", ["frozen_object", "falls_during_transfer", "moves_during_retreat",
                                   "slip", "no_target_contact", "source_contact_only", "one_finger", "retained_finger",
                                   "finger_recontacts_during_return", "insufficient_return_height"])
def test_missing_transfer_or_placement_evidence_fails_and_pauses(fault):
    r, s = run_fake(fault)
    assert r["status"] == "FAILED" and r["errors"]
    assert not r["transfer_verified"] and not r["target_placement_verified"] and s["paused"]


def test_missing_return_planner_never_starts_transfer_trial():
    r, s = run_fake(empty_return=False)
    assert r["status"] == "FAILED" and s["paused"]
    assert not r["motion_commands_issued"] and "clearance planner" in r["errors"][0]


def test_return_planner_clears_whole_geometry_instead_of_tcp_only():
    model = Model(); f = fixture(); start, lift = poses()
    f["transfer"] = plan_transfer(model, start, lift, f)
    across = np.array(f["transfer"]["across_rad"])
    tcp = model.predict(across)[1]
    # Synthetic open-gripper envelope has a bottom 30mm below TCP.
    local = np.array([[x, y, z, 1.] for x in (-.02, .02) for y in (-.08, .08) for z in (-.03, .02)])
    world = (tcp @ local.T).T[:, :3]
    object_pose = np.array(f["transfer"]["target_object_world_matrix_m"])
    high, home, plan = plan_empty_return(model, world, across, start, across, f, object_pose)
    assert 0 < abs(math.degrees(high[1])) <= 8
    assert plan["minimum_predicted_gripper_clearance_m"] >= .008
    assert plan["collision_corner_count"] == 8 and home[0] == 0
    np.testing.assert_allclose(high[[0, 2, 3, 4, 5]], across[[0, 2, 3, 4, 5]])
    model.limits = np.tile([-.001, .001], (6, 1))
    with pytest.raises(ValueError, match="no authored-limit-safe"):
        plan_empty_return(model, world, across, start, across, f, object_pose)


def test_measured_gripper_bounds_include_hidden_geometry_and_fail_when_empty():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    shape = UsdGeom.Cube.Define(stage, "/Grip/HiddenCollider")
    shape.CreateSizeAttr(1)
    shape.AddTranslateOp().Set(Gf.Vec3d(0, 0, .4))
    shape.AddRotateYOp().Set(90)
    shape.AddScaleOp().Set(Gf.Vec3f(.02, .04, .06))
    shape.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    points, paths = gripper_collision_world_points(stage, {"gripper_root_path": "/Grip"})
    assert points.shape == (8, 3) and paths == ["/Grip/HiddenCollider"]
    assert points[:, 2].min() == pytest.approx(.39)
    UsdPhysics.CollisionAPI(shape.GetPrim()).CreateCollisionEnabledAttr(False)
    with pytest.raises(ValueError, match="measured gripper collision"):
        gripper_collision_world_points(stage, {"gripper_root_path": "/Grip"})
