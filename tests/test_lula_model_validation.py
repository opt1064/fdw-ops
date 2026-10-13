"""Independent analytic FK and real USD joint anchors; actual Lula is not available on CPU host."""
import math
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from test_robot_model_export import model
from fdw_sim.visualization.lula_model_validation import (
    make_pose_samples, prepare_and_validate_lula, usd_joint_forward_kinematics,
)


@pytest.fixture
def anchored_model(model):
    from pxr import Gf, UsdGeom, UsdPhysics
    path, stage, diagnostics = model
    parent = UsdGeom.Xform.Define(stage, "/Robot/J1_link")
    parent.AddTranslateOp().Set((0, 0, 0.2))
    parent.AddRotateZOp().Set(45)
    for name, body0, body1, axis, offset in (
        ("J1", "robot_base", "J1_link", "Z", (0, 0, 0.2)),
        ("J2", "J1_link", "J2_link", "Y", (0.5, 0, 0)),
    ):
        joint = UsdPhysics.RevoluteJoint.Define(stage, f"/Robot/joints/{name}")
        joint.CreateBody0Rel().SetTargets([f"/Robot/{body0}"])
        joint.CreateBody1Rel().SetTargets([f"/Robot/{body1}"])
        joint.CreateAxisAttr(axis)
        joint.CreateLocalPos0Attr(Gf.Vec3f(*offset))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot0Attr(Gf.Quatf(1))
        joint.CreateLocalRot1Attr(Gf.Quatf(1))
    exported = {"urdf_path": str(path), "comparison": {"status": "PASSED_SINGLE_POSE"},
                "source_diagnostics": diagnostics}
    return stage, exported


class AnalyticSolver:
    """Strict FK-only double with reversed solver joint order and analytic geometry."""
    def __init__(self, *, robot_description_path, urdf_path):
        self.description = yaml.safe_load(Path(robot_description_path).read_text())
        self.base = np.zeros(3)

    def get_joint_names(self):
        return ["J2", "J1"]

    def get_all_frame_names(self):
        return ["robot_base", "J1_link", "J2_link", "ee_link"]

    def set_robot_base_pose(self, position, orientation):
        assert orientation == pytest.approx([1, 0, 0, 0])
        self.base = np.array(position)

    def compute_forward_kinematics(self, frame, q):
        assert frame == "ee_link"
        b, a = q
        c, s, cb, sb = math.cos(a), math.sin(a), math.cos(b), math.sin(b)
        position = self.base + np.array([c*(0.5+0.2*cb), s*(0.5+0.2*cb), 0.2-0.2*sb])
        rotation = np.array([[c*cb, -s, c*sb], [s*cb, c, s*sb], [-sb, 0, cb]])
        return position, rotation


def test_three_independent_models_match_multiple_nonzero_poses_without_stage_edits(anchored_model):
    stage, exported = anchored_model
    before = stage.GetRootLayer().ExportToString()
    report = prepare_and_validate_lula(stage, "/Robot", exported, solver_factory=AnalyticSolver)
    assert report["status"] == "PASSED_COMPUTED_MULTI_POSE", report
    assert report["checked_poses"] == report["passed_poses"] == 12
    assert report["lula_joint_names"] == ["J2", "J1"]
    assert stage.GetRootLayer().ExportToString() == before
    description = yaml.safe_load(Path(report["robot_description_path"]).read_text())
    assert description["collision_spheres"] == []
    assert "acceleration_limits" not in description and "jerk_limits" not in description
    assert report["collision_checked"] is False
    assert report["motion_commands_issued"] is False
    assert report["physical_grasp_verified"] is False
    with pytest.raises(FileExistsError):
        prepare_and_validate_lula(stage, "/Robot", exported, solver_factory=AnalyticSolver)


def test_wrong_usd_axis_fails_at_multiple_poses(anchored_model):
    stage, exported = anchored_model
    stage.GetPrimAtPath("/Robot/joints/J2").GetAttribute("physics:axis").Set("X")
    report = prepare_and_validate_lula(stage, "/Robot", exported, solver_factory=AnalyticSolver)
    assert report["status"] == "FAILED"
    assert report["passed_poses"] < report["checked_poses"]


def test_lula_load_failure_is_saved_without_fallback(anchored_model):
    stage, exported = anchored_model
    def unavailable(**kwargs):
        raise RuntimeError("Lula configuration rejected")
    report = prepare_and_validate_lula(stage, "/Robot", exported, solver_factory=unavailable)
    assert report["status"] == "FAILED"
    assert report["checked_poses"] == 0
    assert "Lula configuration rejected" in report["errors"][0]
    assert (Path(exported["urdf_path"]).parent / "lula_model_validation.json").is_file()


def test_sample_poses_respect_limits_without_clamping():
    samples = make_pose_samples(["J1"], [0.09], {"J1": (-0.1, 0.1)})
    assert all(-0.1 <= q[0] <= 0.1 for _, q in samples)
    assert "J1_+15deg" not in [label for label, q in samples]
    with pytest.raises(ValueError, match="outside"):
        make_pose_samples(["J1"], [0.2], {"J1": (-0.1, 0.1)})


def test_usd_anchor_fk_uses_child_frame_inverse(anchored_model):
    from pxr import Gf
    stage, exported = anchored_model
    # This changes the physical child anchor; it must change the computed pose.
    stage.GetPrimAtPath("/Robot/joints/J2").GetAttribute("physics:localRot1").Set(
        Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), 180).GetQuat()))
    pose = usd_joint_forward_kinematics(stage, "/Robot", "robot_base",
        "/Robot/J2_link/flange/ee_link", {"J1": 0, "J2": 0})
    assert pose[:3, :3] == pytest.approx(np.diag([1, -1, -1]), abs=1e-6)
