"""Analytical/feedback fixtures, not Isaac PhysX or hardware validation."""
import json
import math
from types import SimpleNamespace

import numpy as np
import pytest
from pxr import Usd, UsdGeom, UsdPhysics, Gf

from fdw_sim.visualization.mounted_arm_motion_test import UsdMountedArmModel, MovingArmController, run_mounted_arm_motion


def rotation_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])


def analytical_tcp(q):
    result = np.eye(4)
    result[:3, :3] = rotation_z(q[0]+q[5])
    result[:3, 3] = [.8+.2*math.sin(q[0]), .1+.15*math.sin(q[4]), .7]
    return result


class Plant:
    dt = 1/120
    def __init__(self):
        self.shared_metatype = SimpleNamespace(dof_names=[*[f"J{i}" for i in range(1, 7)], "finger_joint", "follower"])
        self.q = np.zeros((1, 8), dtype=np.float32)
        self.q[0, 6:] = math.radians(4)
        self.targets = self.q.copy()
        self.velocities = np.zeros_like(self.q)
        self.steps = 0
        self.command_count = 0
        self.stuck = False
        self.pose_frozen = False
        self.tcp_error = 0.
        self.mount_error = 0.
        self.grip_drift = False
        self.pads_stuck = False
        self.stopped = False
    def get_dof_positions(self): return self.q
    def get_dof_position_targets(self): return self.targets
    def get_dof_velocity_targets(self): return np.zeros_like(self.q)
    def get_dof_stiffnesses(self): return np.ones_like(self.q)*100
    def get_dof_velocities(self): return self.velocities
    def set_dof_position_targets(self, values, indices): self.targets = values.copy()
    def set_dof_velocity_targets(self, values, indices): pass
    def step(self):
        old = self.q.copy()
        self.steps += 1
        if not self.stuck:
            self.q += .8*(self.targets-self.q)
        self.velocities = (self.q-old)/self.dt
    def pads(self):
        shift = .003 if self.pads_stuck and self.steps else 0
        return [[-.075+shift, 0, .18], [.075, 0, .18]]
    def grip(self):
        return float(self.q[0, 6]) + (math.radians(.6) if self.grip_drift and self.steps else 0)
    def observe(self):
        q = np.zeros(6) if self.pose_frozen else self.q[0, :6]
        pose = analytical_tcp(q)
        pose[0, 3] += self.tcp_error
        return {"tcp_candidate_world_matrix_m": pose.tolist(),
                "flange_to_base_error": {"position_error_m": self.mount_error if self.steps else 0.,
                                          "orientation_error_rad": 0.}}
    def stop(self): self.stopped = True


class Model:
    limits = np.array([[-math.pi, math.pi]]*6)
    def predict(self, q):
        return np.eye(4), analytical_tcp(q)
    def diagnostics(self): return {"prediction_source": "analytical_test_fixture"}


def run(plant, **overrides):
    controller = MovingArmController(plant, 1)
    def hold(grip):
        plant.command_count += 1
        controller.hold()
    args = dict(controller=controller, model=Model(), command_gripper_hold=hold, read_gripper=plant.grip,
                read_pads=plant.pads, observe=plant.observe, step=plant.step, stop=plant.stop, dt=plant.dt)
    args.update(overrides)
    return run_mounted_arm_motion(**args)


def test_three_joints_plus_minus_return_have_12_measured_passed_legs():
    plant = Plant()
    result = run(plant)
    assert result["status"] == "PASSED_MOUNTED_ARM_TCP_TEST"
    assert result["passed_legs"] == result["planned_legs"] == 12
    assert result["returned_to_start"] and plant.stopped
    assert result["samples"] and result["initial_tcp"] and result["final_tcp"]
    assert max(np.max(np.abs(s["command_arm_rad"])) for s in result["samples"]) <= math.radians(1)+1e-7
    assert result["max_tcp_position_error_m"] == pytest.approx(0)
    assert not result["tcp_calibration_verified"] and not result["collision_verified"]
    json.dumps(result, allow_nan=False)


def test_joint_target_writes_preserve_gripper_and_passive_targets():
    plant = Plant()
    controller = MovingArmController(plant, 1)
    before = plant.targets[0, 6:].copy()
    controller.set_target(np.array([.01, 0, 0, 0, 0, -.01]))
    controller.hold()
    np.testing.assert_array_equal(plant.targets[0, 6:], before)
    assert plant.targets[0, 0] == pytest.approx(.01)


@pytest.mark.parametrize("fault", ["joint_stuck", "tcp_frozen", "mount_drift", "grip_drift", "pad_drift"])
def test_false_motion_or_mount_gripper_faults_fail_and_pause(fault):
    plant = Plant()
    if fault == "joint_stuck": plant.stuck = True
    if fault == "tcp_frozen": plant.pose_frozen = True
    if fault == "mount_drift": plant.mount_error = .003
    if fault == "grip_drift": plant.grip_drift = True
    if fault == "pad_drift": plant.pads_stuck = True
    result = run(plant)
    assert result["status"] == "FAILED" and result["passed_legs"] < 12
    assert result["errors"] and plant.stopped
    assert result["motion_commands_issued"] and "abort_hold_arm_rad" in result


def test_initial_fk_error_blocks_all_arm_commands():
    plant = Plant()
    plant.tcp_error = .003
    result = run(plant)
    assert result["status"] == "FAILED" and not result["motion_commands_issued"]
    assert plant.command_count == plant.steps == 0
    assert plant.stopped


def test_target_outside_authored_limits_blocks_all_arm_commands():
    plant = Plant()
    model = Model()
    model.limits = np.array([[-.005, .005]]*6)
    result = run(plant, model=model)
    assert result["status"] == "FAILED" and plant.command_count == plant.steps == 0
    assert "authored limits" in result["errors"][0]


def test_viewer_close_holds_measured_arm_and_pauses():
    plant = Plant()
    result = run(plant, is_running=lambda: plant.steps < 10)
    assert result["status"] == "FAILED" and plant.stopped
    assert "abort_hold_arm_rad" in result and "viewer closed" in result["errors"][0]


@pytest.mark.parametrize("joint,angle", [(0, 2), (3, .6)])
def test_selected_and_unselected_excursion_limits_are_enforced(joint, angle):
    plant = Plant()
    controller = MovingArmController(plant, 1)
    plant.q[0, joint] = math.radians(angle)
    with pytest.raises(RuntimeError, match="envelope"):
        controller.check()


def test_nonfinite_feedback_is_rejected():
    plant = Plant()
    controller = MovingArmController(plant, 1)
    plant.q[0, 3] = np.nan
    with pytest.raises(ValueError, match="finite"):
        controller.check()


@pytest.mark.parametrize("amplitude", [0, 3, float("nan")])
def test_invalid_amplitude_is_rejected_before_commands(amplitude):
    plant = Plant()
    result = run(plant, amplitude_deg=amplitude)
    assert result["status"] == "FAILED" and plant.steps == 0


@pytest.fixture
def usd_chain():
    # Analytic serial chain: six local-Z revolute joints, each parent offset 0.1 m.
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    for i in range(7):
        body = UsdGeom.Xform.Define(stage, f"/Arm/L{i}")
        body.AddTranslateOp().Set((i*.1, 0, 0))
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    for i in range(1, 7):
        joint = UsdPhysics.RevoluteJoint.Define(stage, f"/Arm/J{i}")
        joint.CreateBody0Rel().SetTargets([f"/Arm/L{i-1}"])
        joint.CreateBody1Rel().SetTargets([f"/Arm/L{i}"])
        joint.CreateAxisAttr("Z")
        joint.CreateLocalPos0Attr((.1, 0, 0))
        joint.CreateLocalPos1Attr((0, 0, 0))
        joint.CreateLowerLimitAttr(-90)
        joint.CreateUpperLimitAttr(90)
    UsdGeom.Xform.Define(stage, "/Arm/L6/flange").AddTranslateOp().Set((.05, 0, 0))
    UsdGeom.Xform.Define(stage, "/Tool/Base").AddTranslateOp().Set((.65, 0, 0))
    UsdGeom.Xform.Define(stage, "/Tool/Base/TCP").AddTranslateOp().Set((.15, 0, 0))
    mount = {"robot_path": "/Arm", "arm_base_path": "/Arm/L0", "terminal_body_path": "/Arm/L6",
             "flange_path": "/Arm/L6/flange", "gripper_base_path": "/Tool/Base", "tcp_candidate_path": "/Tool/Base/TCP",
             "desired_flange_to_base_matrix_m": np.eye(4).tolist()}
    return stage, mount


def test_usd_fk_matches_analytic_chain_and_does_not_read_moving_body_poses(usd_chain):
    stage, mount = usd_chain
    model = UsdMountedArmModel(stage, mount)
    q = np.zeros(6)
    q[0] = math.pi/6
    flange, tcp = model.predict(q)
    expected = np.array([.1, 0, 0])+rotation_z(q[0]) @ np.array([.7, 0, 0])
    np.testing.assert_allclose(tcp[:3, 3], expected, atol=1e-7)
    np.testing.assert_allclose(tcp[:3, :3], rotation_z(q[0]), atol=1e-7)
    # Mutate USD body transforms after snapshot: prediction must remain independent of live transforms.
    stage.GetPrimAtPath("/Arm/L6").GetAttribute("xformOp:translate").Set((5, 5, 5))
    np.testing.assert_allclose(model.predict(q)[1], tcp)
    assert not model.diagnostics()["tcp_calibration_verified"]


@pytest.mark.parametrize("fault", ["limits", "axis", "graph", "disabled"])
def test_bad_usd_joint_model_blocks_before_motion(usd_chain, fault):
    stage, mount = usd_chain
    joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath("/Arm/J3"))
    if fault == "limits": joint.CreateUpperLimitAttr(float("inf"))
    if fault == "axis": joint.CreateAxisAttr("BAD")
    if fault == "graph": joint.GetBody0Rel().SetTargets(["/Arm/L0"])
    if fault == "disabled": joint.CreateJointEnabledAttr(False)
    with pytest.raises(ValueError):
        UsdMountedArmModel(stage, mount)


def test_usd_local_rotations_and_nonzero_child_joint_frame_are_respected(usd_chain):
    stage, mount = usd_chain
    joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath("/Arm/J1"))
    # Rotate both joint axes equally by 180 degrees about X: positive local-Z is negative world-Z.
    flip = Gf.Quatf(0, Gf.Vec3f(1, 0, 0))
    joint.CreateLocalRot0Attr(flip)
    joint.CreateLocalRot1Attr(flip)
    model = UsdMountedArmModel(stage, mount)
    q = np.array([math.pi/6, 0, 0, 0, 0, 0])
    _, tcp = model.predict(q)
    expected = np.array([.1, 0, 0])+rotation_z(-q[0]) @ np.array([.7, 0, 0])
    np.testing.assert_allclose(tcp[:3, 3], expected, atol=1e-7)
