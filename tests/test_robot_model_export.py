"""URDF-vs-USD checks at nonzero joint angles, without Isaac or motor commands."""
from pathlib import Path
from types import SimpleNamespace
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fdw_sim.visualization.robot_model_export import (
    add_fixed_end_frame, compare_urdf_to_usd, export_robot_model, urdf_forward_kinematics,
)

URDF = '''<robot name="test">
<link name="robot_base"/><link name="J1_link"/><link name="J2_link"/><link name="ee_link"/>
<joint name="J1" type="revolute"><parent link="robot_base"/><child link="J1_link"/>
<origin xyz="0 0 0.2"/><axis xyz="0 0 1"/><limit lower="-3.141592653589793" upper="3.141592653589793"/></joint>
<joint name="J2" type="revolute"><parent link="J1_link"/><child link="J2_link"/>
<origin xyz="0.5 0 0"/><axis xyz="0 1 0"/><limit lower="-1.570796326794897" upper="1.570796326794897"/></joint>
<joint name="flange" type="fixed"><parent link="J2_link"/><child link="ee_link"/><origin xyz="0.2 0 0"/></joint>
</robot>'''


@pytest.fixture
def model(tmp_path):
    pytest.importorskip("pxr")
    from pxr import Gf, Usd, UsdGeom
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Robot")
    root.AddTranslateOp().Set((25, 10, 0.85))
    UsdGeom.Xform.Define(stage, "/Robot/robot_base")
    link = UsdGeom.Xform.Define(stage, "/Robot/J2_link")
    link.AddTranslateOp().Set((0.5 / math.sqrt(2), 0.5 / math.sqrt(2), 0.2))
    z = Gf.Rotation(Gf.Vec3d(0, 0, 1), 45).GetQuat()
    y = Gf.Rotation(Gf.Vec3d(0, 1, 0), -30).GetQuat()
    link.AddOrientOp().Set(Gf.Quatf(z * y))
    ee = UsdGeom.Xform.Define(stage, "/Robot/J2_link/flange/ee_link")
    ee.AddTranslateOp().Set((0.2, 0, 0))
    path = tmp_path / "robot.urdf"
    path.write_text(URDF, encoding="utf-8")
    diagnostics = {
        "meters_per_unit": 1.0,
        "joint_feedback": {"status": "available", "names": ["J1", "J2"],
                           "positions": [math.pi / 4, -math.pi / 6]},
        "configured_tcp": {"frame": "ee_link", "status": "unique_link_found",
                           "matching_paths": [str(ee.GetPath())]},
        "usd_joints": [
            {"name": "J1", "type": "PhysicsRevoluteJoint", "lowerLimit": -180, "upperLimit": 180,
             "body0": ["/Robot/robot_base"], "body1": ["/Robot/J1_link"]},
            {"name": "J2", "type": "PhysicsRevoluteJoint", "lowerLimit": -90, "upperLimit": 90,
             "body0": ["/Robot/J1_link"], "body1": ["/Robot/J2_link"]},
        ],
    }
    return path, stage, diagnostics


def test_matches_usd_at_nonzero_joint_angles(model):
    path, stage, diagnostics = model
    report = compare_urdf_to_usd(path, stage, "/Robot", diagnostics)
    assert report["status"] == "PASSED_SINGLE_POSE", report
    assert report["position_error_m"] < 1e-6
    assert report["orientation_error_rad"] < 1e-6
    assert report["full_kinematics_verified"] is False


@pytest.mark.parametrize("change", ["axis", "limit", "parent", "frame", "offset"])
def test_mismatch_never_passes(model, change):
    path, stage, diagnostics = model
    xml = ET.parse(path)
    root = xml.getroot()
    first = root.find("joint")
    if change == "axis":
        first.find("axis").set("xyz", "1 0 0")
    elif change == "limit":
        first.find("limit").set("lower", "-1")
    elif change == "parent":
        first.find("parent").set("link", "J2_link")
    elif change == "frame":
        root.find("link[@name='ee_link']").set("name", "wrong_link")
    else:
        root.find("joint[@name='flange']/origin").set("xyz", "0.25 0 0")
    xml.write(path)
    report = compare_urdf_to_usd(path, stage, "/Robot", diagnostics)
    assert report["status"] == "FAILED"
    assert report["errors"]


def test_fk_handles_rpy_and_prismatic_translation():
    robot = ET.fromstring('''<robot><link name="base"/><link name="tip"/>
    <joint name="slide" type="prismatic"><parent link="base"/><child link="tip"/>
    <origin xyz="1 0 0" rpy="0 0 1.570796326794897"/><axis xyz="1 0 0"/></joint></robot>''')
    _, fk = urdf_forward_kinematics(robot, {"slide": 0.2})
    assert fk["tip"][:3, 3] == pytest.approx([1, 0.2, 0])


def test_export_uses_scoped_api_and_fresh_output_without_source_changes(model, tmp_path):
    path, stage, diagnostics = model
    before = stage.GetRootLayer().ExportToString()

    class Converter:
        def __init__(self, *, stage, root_prim_path, mesh_dir_name, mesh_path_prefix):
            assert root_prim_path == "/Robot"
            assert mesh_dir_name == "meshes" and mesh_path_prefix == "./"

        def convert(self, output_path):
            Path(output_path).write_text(URDF, encoding="utf-8")
            return output_path

    target = tmp_path / "export"
    report = export_robot_model(stage, "/Robot", SimpleNamespace(name="test"), diagnostics,
                                target, converter_factory=Converter)
    assert report["comparison"]["status"] == "PASSED_SINGLE_POSE"
    assert (target / "model_comparison.json").is_file()
    assert stage.GetRootLayer().ExportToString() == before
    with pytest.raises(FileExistsError):
        export_robot_model(stage, "/Robot", SimpleNamespace(name="test"), diagnostics,
                           target, converter_factory=Converter)


def test_export_failure_preserves_evidence(model, tmp_path):
    path, stage, diagnostics = model

    def unavailable_converter(**kwargs):
        raise ImportError("export API unavailable")

    target = tmp_path / "failed_export"
    with pytest.raises(RuntimeError, match="details saved"):
        export_robot_model(stage, "/Robot", SimpleNamespace(name="test"), diagnostics,
                           target, converter_factory=unavailable_converter)
    assert "export API unavailable" in (target / "export_error.json").read_text()


def _remove_exporter_site_frame(path):
    tree = ET.parse(path)
    root = tree.getroot()
    root.remove(root.find("link[@name='ee_link']"))
    root.remove(root.find("joint[@name='flange']"))
    tree.write(path)


@pytest.mark.parametrize("pitch", [0, 35, 90, -90])
def test_omitted_end_frame_is_derived_from_usd_even_with_rotation(model, pitch):
    from pxr import Gf, UsdGeom
    path, stage, diagnostics = model
    _remove_exporter_site_frame(path)
    raw = path.read_bytes()
    end = UsdGeom.Xform(stage.GetPrimAtPath(diagnostics["configured_tcp"]["matching_paths"][0]))
    q = (Gf.Rotation(Gf.Vec3d(0, 0, 1), 30).GetQuat()
         * Gf.Rotation(Gf.Vec3d(0, 1, 0), pitch).GetQuat()
         * Gf.Rotation(Gf.Vec3d(1, 0, 0), 20).GetQuat())
    end.AddOrientOp().Set(Gf.Quatf(q))
    before = stage.GetRootLayer().ExportToString()
    derived, evidence = add_fixed_end_frame(path, stage, "/Robot", diagnostics)
    assert path.read_bytes() == raw
    assert stage.GetRootLayer().ExportToString() == before
    assert derived != path
    assert evidence["status"] == "ADDED_FROM_USD"
    assert evidence["xyz_m"] == pytest.approx([0.2, 0, 0], abs=1e-6)
    assert evidence["calibration_verified"] is False
    assert ET.parse(derived).getroot().find("link[@name='ee_link']/inertial") is None
    comparison = compare_urdf_to_usd(derived, stage, "/Robot", diagnostics)
    assert comparison["status"] == "PASSED_SINGLE_POSE", comparison


def test_omitted_rigid_body_is_not_replaced_with_fixed_frame(model):
    from pxr import UsdPhysics
    path, stage, diagnostics = model
    _remove_exporter_site_frame(path)
    end = stage.GetPrimAtPath(diagnostics["configured_tcp"]["matching_paths"][0])
    UsdPhysics.RigidBodyAPI.Apply(end)
    with pytest.raises(ValueError, match="omitted rigid body"):
        add_fixed_end_frame(path, stage, "/Robot", diagnostics)


def test_derived_frame_converts_stage_length_units(model):
    from pxr import UsdGeom
    path, stage, diagnostics = model
    _remove_exporter_site_frame(path)
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    diagnostics["meters_per_unit"] = 0.01
    _, evidence = add_fixed_end_frame(path, stage, "/Robot", diagnostics)
    assert evidence["xyz_m"] == pytest.approx([0.002, 0, 0], abs=1e-8)


def test_export_workflow_preserves_raw_and_compares_derived_model(model, tmp_path):
    path, stage, diagnostics = model

    class PhysicsOnlyConverter:
        def __init__(self, **kwargs):
            pass

        def convert(self, output_path):
            Path(output_path).write_text(URDF, encoding="utf-8")
            _remove_exporter_site_frame(output_path)
            return output_path

    report = export_robot_model(stage, "/Robot", SimpleNamespace(name="test"), diagnostics,
                               tmp_path / "physics_only", converter_factory=PhysicsOnlyConverter)
    assert report["raw_urdf_path"] != report["urdf_path"]
    assert "ee_link" not in Path(report["raw_urdf_path"]).read_text()
    assert report["fixed_end_frame"]["status"] == "ADDED_FROM_USD"
    assert report["comparison"]["status"] == "PASSED_SINGLE_POSE"
