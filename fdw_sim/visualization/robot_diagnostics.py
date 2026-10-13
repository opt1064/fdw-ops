"""Read-only robot inventory. Presence of a link is not TCP/grasp calibration."""
from __future__ import annotations

import math
import os
from pathlib import Path


def collect_robot_diagnostics(stage, robot_path, spec, articulation=None):
    """Inspect composed USD and available feedback without issuing robot commands."""
    from pxr import Usd, UsdGeom, UsdPhysics

    root = stage.GetPrimAtPath(robot_path)
    if not root or not list(root.GetChildren()):
        raise RuntimeError(f"Robot USD is missing or empty: {robot_path}")
    meters = UsdGeom.GetStageMetersPerUnit(stage)
    cache = UsdGeom.XformCache()
    errors = []

    def pose(prim):
        if not prim.IsA(UsdGeom.Xformable):
            return None
        matrix = cache.GetLocalToWorldTransform(prim)
        quat = matrix.ExtractRotationQuat()
        return {"position_m": [float(v) * meters for v in matrix.ExtractTranslation()],
                "orientation_wxyz": [float(quat.GetReal()), *map(float, quat.GetImaginary())],
                "source": "composed_usd", "calibration_verified": False}

    joints, bodies, candidates, configured = [], [], [], []
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        path, name = str(prim.GetPath()), prim.GetName()
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            bodies.append(path)
        if prim.IsA(UsdPhysics.Joint):
            joint = UsdPhysics.Joint(prim)
            entry = {"name": name, "path": path, "type": prim.GetTypeName(),
                     "body0": list(map(str, joint.GetBody0Rel().GetTargets())),
                     "body1": list(map(str, joint.GetBody1Rel().GetTargets()))}
            for key in ("axis", "lowerLimit", "upperLimit"):
                attr = prim.GetAttribute("physics:" + key)
                value = attr.Get() if attr and attr.HasAuthoredValueOpinion() else None
                entry[key] = str(value) if isinstance(value, float) and not math.isfinite(value) else value
            entry["limit_units"] = ("degrees" if prim.IsA(UsdPhysics.RevoluteJoint)
                                    else "stage_length_units" if prim.IsA(UsdPhysics.PrismaticJoint) else None)
            joints.append(entry)
        if prim.IsA(UsdGeom.Xformable) and not prim.IsA(UsdGeom.Gprim):
            if any(token in name.lower() for token in ("tool", "flange", "tcp", "j6_link", "ee_link")):
                candidates.append({"path": path, "pose": pose(prim)})
            if name == spec.end_effector_frame:
                configured.append(prim)

    feedback = {"status": "unavailable", "names": None, "positions": None,
                "revolute_position_units": "radians"}
    if articulation is not None:
        try:
            names = list(articulation.dof_names)
            values = articulation.get_joint_positions()
            positions = list(map(float, values)) if values is not None else []
            if not names or len(names) != len(positions) or not all(map(math.isfinite, positions)):
                raise ValueError("joint names and finite positions must have the same nonzero length")
            feedback.update(status="available", names=names, positions=positions)
        except Exception as exc:
            errors.append(f"joint_feedback: {type(exc).__name__}: {exc}")
    else:
        errors.append("joint_feedback: articulation wrapper unavailable")

    movable = [j for j in joints if j["type"] in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint")]
    usd_names = {j["name"] for j in movable}
    if not movable:
        errors.append("no USD revolute/prismatic joints found; check robot asset loading")
    feedback["names_not_found_in_usd_joints"] = (
        sorted(set(feedback["names"]) - usd_names) if feedback["names"] else None)
    tcp_status = "unique_link_found" if len(configured) == 1 else "missing" if not configured else "ambiguous"
    if tcp_status != "unique_link_found":
        errors.append(f"configured TCP '{spec.end_effector_frame}': {tcp_status}")
    config_files = {}
    for suffix in ("URDF", "ROBOT_DESCRIPTION", "RMPFLOW_YAML"):
        key = f"FDW_{spec.name.upper()}_{suffix}"
        value = os.environ.get(key)
        config_files[key] = {"path": value, "exists": Path(value).is_file() if value else False}

    return {
        "robot": spec.name, "robot_path": robot_path,
        "diagnostic_type": "passive_inventory_no_motion_commands",
        "meters_per_unit": meters, "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
        "base_pose": pose(root), "joint_feedback": feedback,
        "usd_joints": joints, "rigid_body_paths": bodies,
        "configured_tcp": {"frame": spec.end_effector_frame, "status": tcp_status,
                           "matching_paths": [str(p.GetPath()) for p in configured],
                           "pose": pose(configured[0]) if len(configured) == 1 else None,
                           "calibration_verified": False},
        "tcp_candidates": candidates, "explicit_motion_config_files": config_files,
        "errors": errors,
        "physical_grasp_verified": False,
        "next_step": "Confirm joint mapping, TCP calibration, and matching kinematics before empty-hand motion.",
    }
