"""Decision/geometry tests with explicit fake feedback, not PhysX grasp validation."""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from fdw_sim.visualization.physical_grasp_test import (
    create_grasp_fixture, pad_bounds_in_base, ContactEvidence, LoadedTestBlock,
    run_contact_grasp, plan_lift,
)


def geometry():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, "Z")
    base = UsdGeom.Xform.Define(stage, "/Grip/Base")
    base.AddTranslateOp().Set(Gf.Vec3d(0, 0, .8))
    base.AddRotateYOp().Set(90)
    paths = []
    for name, y in (("Left", -.075), ("Right", .075)):
        p = UsdGeom.Cube.Define(stage, "/Grip/Base/"+name)
        p.CreateSizeAttr(1)
        p.AddTranslateOp().Set(Gf.Vec3d(0, y, .18))
        p.AddScaleOp().Set(Gf.Vec3f(.02, .01, .04))
        paths.append(str(p.GetPath()))
    return stage, {"gripper_base_path": "/Grip/Base"}, paths


def test_fixture_physics_and_initial_clearance_without_attachment():
    stage, mount, paths = geometry()
    before = stage.GetRootLayer().ExportToString()
    bounds = pad_bounds_in_base(stage, mount, paths)
    assert bounds.shape == (2, 2, 3)
    f = create_grasp_fixture(stage, mount, paths[::-1])  # Labels do not assume +/-Y ordering.
    assert f["measured_open_clearance_m"] == pytest.approx(.14)
    assert f["dimensions_base_m"][1] == pytest.approx(.119)
    block = stage.GetPrimAtPath(f["block_path"])
    support = stage.GetPrimAtPath(f["support_path"])
    assert block.HasAPI(UsdPhysics.RigidBodyAPI)
    assert not UsdPhysics.RigidBodyAPI(block).GetKinematicEnabledAttr().Get()
    assert UsdPhysics.MassAPI(block).GetMassAttr().Get() == pytest.approx(.05)
    assert block.HasAPI(UsdPhysics.CollisionAPI) and support.HasAPI(UsdPhysics.CollisionAPI)
    assert not support.HasAPI(UsdPhysics.RigidBodyAPI)
    assert "PhysxContactReportAPI" in block.GetMetadata("apiSchemas").GetAddedOrExplicitItems()
    assert not any(p.IsA(UsdPhysics.Joint) for p in stage.Traverse())
    assert stage.GetPrimAtPath("/Grip/Base/Left").GetAttribute("xformOp:translate").Get()[1] == -.075
    with pytest.raises(ValueError, match="already exists"):
        create_grasp_fixture(stage, mount, paths)
    assert before != stage.GetRootLayer().ExportToString()


def test_fixture_rejects_wrong_up_axis_and_unsupported_base():
    stage, mount, paths = geometry()
    UsdGeom.SetStageUpAxis(stage, "Y")
    with pytest.raises(ValueError, match="Z up"):
        create_grasp_fixture(stage, mount, paths)
    UsdGeom.SetStageUpAxis(stage, "Z")
    stage.GetPrimAtPath("/Grip/Base").GetAttribute("xformOp:rotateY").Set(0)
    with pytest.raises(ValueError, match="horizontal"):
        create_grasp_fixture(stage, mount, paths)


def fixture():
    return dict(block_path="/World/GraspTest/Block", support_path="/World/GraspTest/Support",
                meters_per_unit=1., kilograms_per_unit=1., mass_kg=.05,
                object_downward_load_n=.4905, dimensions_base_m=[.02, .1, .02],
                support_top_world_z_m=.49, scope="synthetic fake feedback for CPU decision tests")


def evidence():
    return ContactEvidence(fixture(), ["/Robotiq_2F_140/L/Pad", "/Robotiq_2F_140/R/Pad"], 9, .01, str, "LOST")


def emit(e, other="/Robotiq_2F_140/L/Pad", impulse=(0, .01, 0), separation=0, **overrides):
    h = dict(stage_id=9, type="PERSIST", collider0=fixture()["block_path"], collider1=other,
             contact_data_offset=0, num_contact_data=1)
    h.update(overrides)
    e.callback([NS(**h)], [NS(separation=separation, impulse=impulse, normal=(0, 1, 0))])


def test_contacts_are_per_step_and_filter_stage_lost_and_nonblock_pairs():
    e = evidence()
    emit(e)
    assert e.read()["left"] == pytest.approx(1)
    e.begin_step()
    emit(e, stage_id=10)
    emit(e, type="LOST")
    emit(e, collider0="/Other")
    assert e.read()["left"] == 0
    emit(e, other="/Robotiq_2F_140/R/Pad/Shape", impulse=.002)
    assert e.read()["right"] == pytest.approx(.2)


@pytest.mark.parametrize("kwargs", [dict(separation=-.003), dict(impulse=float("nan")),
                                     dict(contact_data_offset=4), dict(impulse=(0, 1, 0)),
                                     dict(other="/Robotiq_2F_140/Knuckle")])
def test_bad_contact_feedback_latches_failure(kwargs):
    e = evidence()
    emit(e, **kwargs)
    with pytest.raises(RuntimeError):
        e.read()
    # Parser errors remain latched; force envelope failures recur if the contact recurs.
    if e.error:
        e.begin_step()
        with pytest.raises(RuntimeError): e.read()


class FakeRigid:
    count = 1
    def get_masses(self): return np.array([[.05]])
    def get_transforms(self): return np.array([[0, 0, .5, 0, 0, 0, 1.]])
    def get_velocities(self): return np.zeros((1, 6))
    def apply_forces(self, values, indices, global_):
        self.applied = values.copy(), indices.copy(), global_


def test_native_block_uses_feedback_and_force_only_no_pose_setter():
    view = FakeRigid()
    owner = NS(create_rigid_body_view=lambda path: view)
    block = LoadedTestBlock(owner, fixture())
    assert block.owner is owner
    block.load()
    assert view.applied[0][0, 2] == pytest.approx(-.4905)
    assert view.applied[2] is True
    m = block.measure()
    assert np.array(m["object_world_matrix_m"])[2, 3] == .5
    assert m["support_clearance_m"] == pytest.approx(0)
    view.get_masses = lambda: np.array([[1.]])
    with pytest.raises(ValueError, match="mass"):
        LoadedTestBlock(owner, fixture())
    view.get_transforms = lambda: np.full((1, 7), np.nan)
    with pytest.raises(ValueError, match="feedback"):
        block.measure()


class FakeModel:
    limits = np.tile([-math.pi, math.pi], (6, 1))
    def predict(self, q):
        pose = np.eye(4)
        pose[2, 3] = q[4]
        return pose, pose
    def diagnostics(self): return {"fake": True}


def test_lift_plan_selects_upward_motion_and_rejects_limits_or_insufficient_rise():
    m = FakeModel()
    target, plan = plan_lift(m, np.zeros(6))
    assert target[4] == pytest.approx(math.radians(2))
    assert plan["predicted_tcp_rise_m"] > .008
    m.limits = np.tile([-.001, .001], (6, 1))
    with pytest.raises(ValueError, match="no authored"):
        plan_lift(m, np.zeros(6))


def fake_run(fault=None):
    """Fixture drives fake observations by phase; this tests gating, never contact physics."""
    state = {"phase": None, "grip": .07, "arm": np.zeros(6), "paused": False, "steps": 0}
    def color(phase): state["phase"] = phase
    def command(grip, arm): state.update(grip=grip, arm=arm.copy())
    def step(): state["steps"] += 1
    def observe():
        phase, grip = state["phase"], state["grip"]
        raised = max(0, state["arm"][4])*.35
        both = phase in ("CLOSE_CONTACT", "LIFT", "HOLD_UNSUPPORTED") and grip > .12
        if fault == "no_contact": both = False
        lift = raised if phase in ("LIFT", "HOLD_UNSUPPORTED") else 0
        if fault == "support_hold": lift = 0
        if fault == "attached_release" and phase == "OPEN_RELEASE": lift = .012
        pose = np.eye(4); pose[2, 3] = .5+lift
        relative = np.eye(4); relative[2, 3] = .5
        if fault == "slip" and phase == "HOLD_UNSUPPORTED": relative[0, 3] = .01
        contacts = {"left": 1. if both else 0., "right": 1. if both else 0., "support": .4905 if lift < .001 else 0.}
        if fault == "one_finger": contacts["right"] = 0
        return dict(object_world_matrix_m=pose.tolist(), object_relative_base_matrix_m=relative.tolist(),
                    contacts=contacts, support_clearance_m=lift, object_speed_m_s=0.,
                    gripper_rad=grip, arm_target_error_rad=0., arm_speed_rad_s=0.)
    def stop(): state["paused"] = True
    target = np.zeros(6); target[4] = math.radians(2)
    result = run_contact_grasp(command, observe, step, stop, color, fixture(), np.zeros(6), target,
                               .07, .24, dt=1/120, is_running=lambda: fault != "closed_viewer")
    return result, state


def test_six_phase_gate_requires_unsupported_hold_and_release_then_pauses():
    r, s = fake_run()
    assert r["status"] == "PASSED_SIMULATED_CONTACT_GRASP_RELEASE", r["errors"]
    assert r["passed_phases"] == 6 and r["physical_grasp_verified"] and r["release_verified"]
    assert r["phases"][3]["bilateral_hold_sec"] == pytest.approx(3)
    assert s["paused"]


@pytest.mark.parametrize("fault", ["no_contact", "one_finger", "support_hold", "slip", "attached_release", "closed_viewer"])
def test_missing_physical_evidence_never_passes(fault):
    r, s = fake_run(fault)
    assert r["status"] == "FAILED" and r["errors"]
    assert not r["physical_grasp_verified"] and not r["release_verified"]
    assert s["paused"] and r["passed_phases"] < 6
