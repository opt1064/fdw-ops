"""Real USD passive-inventory checks; no robot motion or physics validation."""
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fdw_sim.visualization.robot_diagnostics import collect_robot_diagnostics


@pytest.fixture
def robot():
    pytest.importorskip("pxr")
    from pxr import Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    parent = UsdGeom.Xform.Define(stage, "/Cell")
    parent.AddTranslateOp().Set((100, 200, 0))
    root = UsdGeom.Xform.Define(stage, "/Cell/RobotArm")
    root.AddTranslateOp().Set((20, 0, 80))
    link = UsdGeom.Xform.Define(stage, "/Cell/RobotArm/tool0")
    link.AddTranslateOp().Set((0, 0, 30))
    UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
    for i in range(1, 7):
        joint = UsdPhysics.RevoluteJoint.Define(stage, f"/Cell/RobotArm/J{i}")
        joint.CreateAxisAttr("Z")
        joint.CreateLowerLimitAttr(-90)
        joint.CreateUpperLimitAttr(90)
        joint.CreateBody1Rel().SetTargets([link.GetPath()])
    spec = SimpleNamespace(name="fanuc_crx10ia", end_effector_frame="tool0")

    class Feedback:
        dof_names = [f"J{i}" for i in range(1, 7)]

        def get_joint_positions(self):
            return [0.1] * 6

        def apply_action(self, *args):
            raise AssertionError("diagnostic must never send actions")

    return stage, spec, Feedback()


def test_passive_inventory_composes_parent_pose_and_converts_units(robot):
    stage, spec, feedback = robot
    before = stage.GetRootLayer().ExportToString()
    report = collect_robot_diagnostics(stage, "/Cell/RobotArm", spec, feedback)
    assert stage.GetRootLayer().ExportToString() == before
    assert report["base_pose"]["position_m"] == pytest.approx([1.2, 2, 0.8])
    assert report["configured_tcp"]["pose"]["position_m"] == pytest.approx([1.2, 2, 1.1])
    assert report["joint_feedback"]["status"] == "available"
    assert not report["joint_feedback"]["names_not_found_in_usd_joints"]
    assert len(report["usd_joints"]) == 6
    assert all(j["limit_units"] == "degrees" and j["lowerLimit"] == -90 for j in report["usd_joints"])
    assert report["configured_tcp"]["calibration_verified"] is False
    assert report["physical_grasp_verified"] is False


def test_missing_tcp_does_not_select_a_candidate(robot):
    stage, spec, feedback = robot
    spec.end_effector_frame = "unknown_tcp"
    report = collect_robot_diagnostics(stage, "/Cell/RobotArm", spec, feedback)
    assert report["configured_tcp"]["status"] == "missing"
    assert report["configured_tcp"]["pose"] is None
    assert report["tcp_candidates"]


def test_duplicate_tcp_names_are_ambiguous(robot):
    from pxr import UsdGeom
    stage, spec, feedback = robot
    UsdGeom.Xform.Define(stage, "/Cell/RobotArm/Other/tool0")
    report = collect_robot_diagnostics(stage, "/Cell/RobotArm", spec, feedback)
    assert report["configured_tcp"]["status"] == "ambiguous"
    assert report["configured_tcp"]["pose"] is None


@pytest.mark.parametrize("positions", [None, [0.1], [float("nan")] * 6])
def test_missing_or_invalid_feedback_is_reported(robot, positions):
    stage, spec, feedback = robot
    feedback.get_joint_positions = lambda: positions
    report = collect_robot_diagnostics(stage, "/Cell/RobotArm", spec, feedback)
    assert report["joint_feedback"]["status"] == "unavailable"
    assert report["errors"]


def test_empty_robot_fails_instead_of_reporting_placeholder_success(robot):
    stage, spec, feedback = robot
    with pytest.raises(RuntimeError, match="missing or empty"):
        collect_robot_diagnostics(stage, "/Missing", spec, feedback)


def test_observed_fanuc_nested_ee_link_is_found_without_claiming_calibration(robot):
    from pxr import UsdGeom
    from fdw_sim.visualization.robot_loader import ROBOT_CATALOG
    stage, _, feedback = robot
    path = "/Cell/RobotArm/J6_link/flange/ee_link"
    UsdGeom.Xform.Define(stage, path)
    report = collect_robot_diagnostics(stage, "/Cell/RobotArm", ROBOT_CATALOG["fanuc_crx10ia"], feedback)
    assert report["configured_tcp"]["matching_paths"] == [path]
    assert report["configured_tcp"]["status"] == "unique_link_found"
    assert report["configured_tcp"]["calibration_verified"] is False


def test_motion_file_discovery_never_initializes_or_validates_backend(tmp_path, monkeypatch):
    from fdw_sim.visualization.robot_diagnostics import discover_motion_configuration
    from fdw_sim.visualization.robot_loader import ROBOT_CATALOG
    from fdw_sim.visualization.rmpflow_controller import RMPflowController
    paths = {}
    for suffix in ("URDF", "ROBOT_DESCRIPTION", "RMPFLOW_YAML"):
        path = tmp_path / suffix
        path.write_text("unvalidated test file", encoding="utf-8")
        monkeypatch.setenv(f"FDW_FANUC_CRX10IA_{suffix}", str(path))
        paths[suffix] = str(path)

    def forbid_backend(self):
        raise AssertionError("passive diagnosis must not initialize motion")

    monkeypatch.setattr(RMPflowController, "_ensure_backend", forbid_backend)
    result = discover_motion_configuration(ROBOT_CATALOG["fanuc_crx10ia"])
    assert result["files"] == {"urdf": paths["URDF"],
                               "robot_description": paths["ROBOT_DESCRIPTION"],
                               "rmpflow": paths["RMPFLOW_YAML"]}
    assert result["all_three_files_found"] is True
    assert result["backend_initialized"] is False
    assert result["model_frame_matching_verified"] is False
