"""Real composed USD checks, with no Isaac or gripper actuation claims."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fdw_sim.visualization.tool_diagnostics import collect_tool_diagnostics, load_tool_configuration

CONFIG = Path(__file__).resolve().parents[1] / "fdw_sim/config/fanuc_tooling.yaml"


@pytest.fixture
def tooling():
    from pxr import Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    root = UsdGeom.Xform.Define(stage, "/Robot")
    root.AddTranslateOp().Set((100, 200, 0))
    body = UsdGeom.Xform.Define(stage, "/Robot/J6_link")
    body.AddRotateZOp().Set(90)
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    flange = UsdGeom.Xform.Define(stage, "/Robot/J6_link/flange")
    flange.AddTranslateOp().Set((16, 0, 0))
    UsdGeom.Xform.Define(stage, "/Robot/J6_link/flange/ee_link")
    tcp = UsdGeom.Xform.Define(stage, "/Robot/J6_link/flange/torch_tcp")
    tcp.AddTranslateOp().Set((0, 0, 25))
    spec = SimpleNamespace(end_effector_frame="ee_link")
    feedback = {"joint_feedback": {"names": [f"J{i}" for i in range(1, 7)], "positions": [0]*6}}
    return stage, spec, load_tool_configuration(CONFIG), feedback


def collect(tooling):
    stage, spec, config, feedback = tooling
    return collect_tool_diagnostics(stage, "/Robot", spec, config, feedback)


def test_passive_reference_frame_is_not_a_calibrated_tool_tcp(tooling):
    stage, _, _, _ = tooling
    before = stage.GetRootLayer().ExportToString()
    report = collect(tooling)
    assert stage.GetRootLayer().ExportToString() == before
    assert report["status"] == "NEEDS_TOOL_CONFIGURATION"
    assert report["flange_to_reference_end"]["translation_m"] == pytest.approx([0, 0, 0])
    assert report["flange_to_reference_end"]["orientation_wxyz"] == pytest.approx([1, 0, 0, 0])
    assert not report["tcp_calibration_verified"] and not report["motion_commands_issued"]
    assert "tcp_frame" in report["tool_profiles"]["gripper"]["missing"]
    json.dumps(report, allow_nan=False)


def test_measured_tool_offset_uses_flange_axes_and_meters(tooling):
    from pxr import Gf
    tooling[0].GetPrimAtPath("/Robot/J6_link/flange/torch_tcp").GetAttribute("xformOp:translate").Set(Gf.Vec3d(25, 0, 0))
    tooling[2]["tools"]["welding_torch"]["tcp_frame"] = "torch_tcp"
    report = collect(tooling)
    offset = report["tool_profiles"]["welding_torch"]["observed_flange_to_tcp"]
    assert offset["translation_m"] == pytest.approx([0.25, 0, 0])
    assert not offset["calibration_verified"]


def test_duplicate_mount_frames_fail_without_selecting_candidate(tooling):
    from pxr import UsdGeom
    UsdGeom.Xform.Define(tooling[0], "/Robot/other/flange")
    report = collect(tooling)
    assert report["status"] == "FAILED"
    assert len(report["mounting_frame_paths"]) == 2
    assert "flange_to_reference_end" not in report


def test_sibling_gripper_attachment_collision_and_drive_evidence(tooling):
    from pxr import Sdf, UsdGeom, UsdPhysics
    stage = tooling[0]
    root = UsdGeom.Xform.Define(stage, "/AttachedGripper")
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    root.GetPrim().SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit(["PhysicsRigidBodyAPI", "PhysxContactReportAPI"]))
    root.GetPrim().CreateAttribute("physxContactReport:threshold", Sdf.ValueTypeNames.Float).Set(0.25)
    joint = UsdPhysics.FixedJoint.Define(stage, "/MountJoint")
    joint.CreateBody0Rel().SetTargets(["/Robot/J6_link"])
    joint.CreateBody1Rel().SetTargets(["/AttachedGripper"])
    finger = UsdGeom.Cube.Define(stage, "/AttachedGripper/finger")
    collision = UsdPhysics.CollisionAPI.Apply(finger.GetPrim())
    collision.CreateCollisionEnabledAttr(False)
    slider = UsdPhysics.PrismaticJoint.Define(stage, "/AttachedGripper/finger_joint")
    drive = UsdPhysics.DriveAPI.Apply(slider.GetPrim(), "linear")
    drive.CreateStiffnessAttr(123)
    report = collect(tooling)
    assert "/AttachedGripper" in report["fixed_attachment_bodies"]
    assert report["fixed_attachment_joints"] == ["/MountJoint"]
    assert report["colliders"][0]["collision_enabled"] is False
    evidence = report["colliders"][0]["ancestor_material_contact_evidence"][0]
    assert evidence["contact_report_attributes"]["physxContactReport:threshold"]["value"] == 0.25
    assert report["movable_joints"][0]["joint_attributes"]["drive:linear:physics:stiffness"]["value"] == 123
    assert report["movable_joints"][0]["measured_position"] is None
    assert not report["physical_grasp_verified"]


def test_disabled_attachment_is_not_counted(tooling):
    from pxr import UsdGeom, UsdPhysics
    stage = tooling[0]
    UsdGeom.Xform.Define(stage, "/UnattachedGripper")
    joint = UsdPhysics.FixedJoint.Define(stage, "/DisabledMount")
    joint.CreateBody0Rel().SetTargets(["/Robot/J6_link"])
    joint.CreateBody1Rel().SetTargets(["/UnattachedGripper"])
    joint.CreateJointEnabledAttr(False)
    report = collect(tooling)
    assert "/UnattachedGripper" not in report["fixed_attachment_bodies"]
    assert not report["fixed_attachment_joints"]


def test_sibling_finger_bodies_and_external_joint_are_in_scope(tooling):
    from pxr import UsdGeom, UsdPhysics
    stage = tooling[0]
    UsdGeom.Xform.Define(stage, "/GripperBase")
    UsdGeom.Xform.Define(stage, "/FingerBody")
    mount = UsdPhysics.FixedJoint.Define(stage, "/Mount")
    mount.CreateBody0Rel().SetTargets(["/Robot/J6_link"])
    mount.CreateBody1Rel().SetTargets(["/GripperBase"])
    finger = UsdPhysics.PrismaticJoint.Define(stage, "/GripperJoints/finger_slide")
    finger.CreateBody0Rel().SetTargets(["/GripperBase"])
    finger.CreateBody1Rel().SetTargets(["/FingerBody"])
    UsdPhysics.CollisionAPI.Apply(UsdGeom.Cube.Define(stage, "/FingerBody/pad").GetPrim())
    report = collect(tooling)
    assert "/FingerBody" in report["terminal_attachment_bodies"]
    assert report["movable_joints"][0]["path"] == "/GripperJoints/finger_slide"
    assert report["colliders"][0]["path"] == "/FingerBody/pad"


def test_arm_colliders_and_tool_names_do_not_prove_gripper_presence(tooling):
    from pxr import UsdGeom, UsdPhysics
    UsdPhysics.CollisionAPI.Apply(UsdGeom.Cube.Define(tooling[0], "/Robot/J6_link/tool_mesh").GetPrim())
    report = collect(tooling)
    assert report["tool_candidates"] and report["colliders"]
    assert not report["surface_gripper_candidates"] and not report["physical_grasp_verified"]
    assert report["status"] == "NEEDS_TOOL_CONFIGURATION"


@pytest.mark.parametrize("offset", [
    {"translation_m": [0, 0], "orientation_wxyz": [1, 0, 0, 0]},
    {"translation_m": [0, 0, 0], "orientation_wxyz": [0, 0, 0, 0]},
    {"translation_m": [0, 0, float("nan")], "orientation_wxyz": [1, 0, 0, 0]},
])
def test_malformed_tcp_config_fails_instead_of_substituting_zero(tmp_path, offset):
    config = load_tool_configuration(CONFIG)
    config["tools"]["gripper"]["flange_to_tcp"] = offset
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match="TCP offset"):
        load_tool_configuration(path)


def test_passive_tool_cli_rejects_drive_option_before_boot(monkeypatch):
    from scripts import run_poc1_level2_2 as cli
    monkeypatch.setattr(sys, "argv", ["runner", "--tool-diagnostics", "--show-robot-motion"])
    with pytest.raises(SystemExit) as exc:
        cli.main()
    assert exc.value.code == 2
