"""Control-loop and composed USD regression checks, not actual Isaac validation."""
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fdw_sim.visualization.gripper_motion_test import run_gripper_cycle, inspect_motion_prerequisites


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
