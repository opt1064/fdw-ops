"""Read-only asset checks, not hardware or Isaac drive validation."""
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fdw_sim.visualization.gripper_asset_inventory import inspect_gripper_stage


@pytest.fixture
def asset():
    from pxr import Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    body = UsdGeom.Xform.Define(stage, "/Gripper")
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    finger = UsdGeom.Cube.Define(stage, "/Gripper/finger")
    UsdPhysics.CollisionAPI.Apply(finger.GetPrim())
    joint = UsdPhysics.PrismaticJoint.Define(stage, "/Gripper/joint")
    joint.CreateLowerLimitAttr(0)
    joint.CreateUpperLimitAttr(7)
    return stage


def test_asset_inventory_preserves_source_and_does_not_claim_grasp(asset):
    before = asset.GetRootLayer().ExportToString()
    result = inspect_gripper_stage(asset)
    assert result["status"] == "INVENTORIED_NOT_PHYSICS_VALIDATED"
    assert result["meters_per_unit"] == 0.01 and result["movable_joint_count"] == 1
    assert not result["motion_commands_issued"] and not result["mounted_on_fanuc"]
    assert not result["physical_grasp_verified"] and not result["tcp_calibration_verified"]
    assert asset.GetRootLayer().ExportToString() == before
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("missing", ["body", "joint", "collision"])
def test_incomplete_assets_fail_instead_of_substituting_another_model(asset, missing):
    from pxr import UsdPhysics
    if missing == "body":
        asset.GetPrimAtPath("/Gripper").RemoveAPI(UsdPhysics.RigidBodyAPI)
    elif missing == "joint":
        asset.RemovePrim("/Gripper/joint")
    else:
        UsdPhysics.CollisionAPI(asset.GetPrimAtPath("/Gripper/finger")).GetCollisionEnabledAttr().Set(False)
    assert inspect_gripper_stage(asset)["status"] == "FAILED"


def test_unopened_asset_is_rejected():
    with pytest.raises(ValueError, match="could not be opened"):
        inspect_gripper_stage(None)
