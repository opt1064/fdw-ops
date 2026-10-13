"""Passive multi-pose FK comparison: USD joint anchors, URDF, and Lula.

No simulation articulation commands, home commands, or collision certificates.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from fdw_sim.visualization.robot_model_export import (
    _gf_rigid_matrix, _rotation, _usd_rigid_matrix, urdf_forward_kinematics,
)


def make_pose_samples(names, current, limits):
    if len(names) != len(current) or len(set(names)) != len(names):
        raise ValueError("joint names and feedback must be unique and complete")
    current = np.asarray(current, dtype=float)
    if not np.isfinite(current).all():
        raise ValueError("joint feedback must be finite")

    def in_limits(q):
        return all(limits[n][0] <= float(q[i]) <= limits[n][1] for i, n in enumerate(names))

    if not in_limits(current):
        raise ValueError("current joints are outside URDF limits")
    samples = [("current", current.copy())]
    if in_limits(np.zeros(len(names))):
        samples.append(("zero", np.zeros(len(names))))
    for i, name in enumerate(names):
        for degrees in (-15, -5, 5, 15):
            q = current.copy()
            q[i] += math.radians(degrees)
            if in_limits(q):
                samples.append((f"{name}_{degrees:+d}deg", q))
    for sign in (-1, 1):
        q = current + np.array([sign * (-1)**i * math.radians(10) for i in range(len(names))])
        if in_limits(q):
            samples.append((f"coupled_{sign:+d}", q))
    return samples


def _unique_prim(stage, robot_path, name):
    from pxr import Usd, UsdGeom
    matches = [p for p in Usd.PrimRange(stage.GetPrimAtPath(robot_path), Usd.TraverseInstanceProxies())
               if p.GetName() == name and p.IsA(UsdGeom.Xformable)]
    if len(matches) != 1:
        raise ValueError(f"no unique USD frame for {name}")
    return matches[0]


def usd_joint_forward_kinematics(stage, robot_path, root_name, end_path, q):
    """Independent USD anchor equation: parent * local0 * motion(q) * inv(local1)."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    cache = UsdGeom.XformCache()
    meters = UsdGeom.GetStageMetersPerUnit(stage)
    root = _unique_prim(stage, robot_path, root_name)
    body_frames = {str(root.GetPath()): _usd_rigid_matrix(cache, root, meters)}
    joints = []
    used_names = set()
    for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_path), Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        parents, children = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
        if not parents:  # World anchoring root joint is not part of the robot's FK tree.
            continue
        if len(parents) != 1 or len(children) != 1:
            raise ValueError("USD FK requires single parent/child joints")
        if joint.GetPrim().GetName() in used_names:
            raise ValueError("duplicate USD joint names")
        used_names.add(joint.GetPrim().GetName())
        joints.append((joint, str(parents[0]), str(children[0])))

    def local_frame(joint, index):
        pos = joint.GetLocalPos0Attr().Get() if index == 0 else joint.GetLocalPos1Attr().Get()
        rot = joint.GetLocalRot0Attr().Get() if index == 0 else joint.GetLocalRot1Attr().Get()
        value = Gf.Matrix4d(1)
        value.SetRotateOnly(Gf.Rotation(Gf.Quatd(rot)))
        value.SetTranslateOnly(Gf.Vec3d(pos))
        return _gf_rigid_matrix(value, meters)

    while joints:
        remaining = []
        for joint, parent, child in joints:
            if parent not in body_frames:
                remaining.append((joint, parent, child))
                continue
            if child in body_frames:
                raise ValueError("USD FK cannot resolve a closed loop or duplicate child")
            motion = np.eye(4)
            prim = joint.GetPrim()
            if prim.IsA(UsdPhysics.RevoluteJoint):
                name = prim.GetName()
                axis = prim.GetAttribute("physics:axis").Get()
                vector = {"X": [1, 0, 0], "Y": [0, 1, 0], "Z": [0, 0, 1]}.get(str(axis))
                if vector is None or name not in q or not math.isfinite(q[name]):
                    raise ValueError(f"missing axis/feedback: {name}")
                motion[:3, :3] = _rotation(vector, q[name])
            elif not prim.IsA(UsdPhysics.FixedJoint):
                raise ValueError("this FANUC validation supports revolute/fixed USD joints only")
            body_frames[child] = body_frames[parent] @ local_frame(joint, 0) @ motion @ np.linalg.inv(local_frame(joint, 1))
        if len(remaining) == len(joints):
            raise ValueError("disconnected or cyclic USD joint graph")
        joints = remaining
    end = stage.GetPrimAtPath(end_path)
    if not end or not end.GetPath().HasPrefix(stage.GetPrimAtPath(robot_path).GetPath()):
        raise ValueError("USD end frame is missing or outside this robot")
    ancestor = end
    while ancestor and str(ancestor.GetPath()) not in body_frames:
        if ancestor.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError("end-frame rigid body is absent from the USD FK tree")
        ancestor = ancestor.GetParent()
    if not ancestor:
        raise ValueError("end frame has no body ancestor in the USD FK tree")
    # The end-frame offset is rigid and comes from USD, not an exporter-frame fit.
    offset = np.linalg.inv(_usd_rigid_matrix(cache, ancestor, meters)) @ _usd_rigid_matrix(cache, end, meters)
    return body_frames[str(ancestor.GetPath())] @ offset


def _pose_error(first, second):
    for matrix in (first, second):
        if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
                or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5)
                or not math.isclose(float(np.linalg.det(matrix[:3, :3])), 1, abs_tol=1e-5)):
            raise ValueError("FK returned a nonfinite or nonrigid pose")
    position = float(np.linalg.norm(first[:3, 3] - second[:3, 3]))
    cosine = (np.trace(first[:3, :3].T @ second[:3, :3]) - 1) / 2
    return {"position_error_m": position,
            "orientation_error_rad": math.acos(float(np.clip(cosine, -1, 1)))}


def add_authored_velocity_limits(urdf_path, stage, diagnostics):
    """Recover omitted PhysX velocities from authored USD only, retaining input URDF."""
    from pxr import UsdPhysics
    tree = ET.parse(urdf_path)
    entries = {j["name"]: j for j in diagnostics["usd_joints"]}
    additions = []
    for joint in tree.getroot().findall("joint"):
        if joint.get("type") == "fixed":
            continue
        name, limit = joint.get("name"), joint.find("limit")
        if limit is None:
            raise ValueError(f"URDF joint limit is missing: {name}")
        existing = limit.get("velocity")
        if existing is not None:
            velocity = float(existing)
            if not math.isfinite(velocity) or velocity <= 0:
                raise ValueError(f"invalid existing URDF velocity: {name}")
            continue
        entry = entries.get(name)
        prim = stage.GetPrimAtPath(entry["path"]) if entry and entry.get("path") else None
        if not prim or not prim.IsA(UsdPhysics.RevoluteJoint):
            raise ValueError(f"no unique revolute USD joint for velocity recovery: {name}")
        attr = prim.GetAttribute("physxJoint:maxJointVelocity")
        if not attr or not attr.HasAuthoredValueOpinion():
            raise ValueError(f"authored physxJoint:maxJointVelocity is missing for {name}; no guessed limit used")
        source = float(attr.Get())
        if not math.isfinite(source) or source <= 0:
            raise ValueError(f"invalid authored PhysX velocity: {name}")
        velocity = math.radians(source)
        limit.set("velocity", format(velocity, ".17g"))
        additions.append({"joint": name, "source_prim_path": str(prim.GetPath()),
                          "source_attribute": "physxJoint:maxJointVelocity",
                          "source_degrees_per_second": source, "urdf_radians_per_second": velocity})
    if not additions:
        return Path(urdf_path), {"status": "NOT_NEEDED", "added": []}
    output = Path(urdf_path).with_name(Path(urdf_path).stem + "_with_velocity_limits.urdf")
    if output.exists():
        raise FileExistsError(output)
    ET.indent(tree)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return output, {"status": "ADDED_FROM_AUTHORED_USD", "added": additions,
                    "manufacturer_limits_verified": False, "source_urdf_preserved": True}


def prepare_and_validate_lula(stage, robot_path, exported, *, solver_factory=None):
    """Save FK-only YAML, load actual Lula, compare models at bounded sample poses."""
    from pxr import UsdGeom
    directory = Path(exported["urdf_path"]).parent
    output = directory / "lula_model_validation.json"
    if output.exists():
        raise FileExistsError("validation report already exists; use a fresh model export directory")
    report = {"status": "FAILED", "errors": [], "pose_results": [],
              "scope": "computed multi-pose model consistency; robot was not moved",
              "collision_checked": False, "physical_grasp_verified": False,
              "full_kinematics_verified": False, "motion_commands_issued": False,
              "position_tolerance_m": 0.002, "orientation_tolerance_rad": 0.01}
    try:
        if exported["comparison"]["status"] != "PASSED_SINGLE_POSE":
            raise ValueError("single-pose model comparison must pass first")
        diagnostics = exported["source_diagnostics"]
        report["usd_joint_anchors"] = diagnostics["usd_joints"]
        feedback = diagnostics["joint_feedback"]
        if feedback["status"] != "available":
            raise ValueError("joint feedback is required")
        names, current = feedback["names"], feedback["positions"]
        lula_urdf_path, velocity_evidence = add_authored_velocity_limits(exported["urdf_path"], stage, diagnostics)
        report.update(source_urdf_path=exported["urdf_path"], urdf_path=str(lula_urdf_path),
                      velocity_limit_recovery=velocity_evidence)
        robot = ET.parse(lula_urdf_path).getroot()
        active = {j.get("name"): j for j in robot.findall("joint") if j.get("type") != "fixed"}
        if set(names) != set(active) or any(j.get("type") != "revolute" for j in active.values()):
            raise ValueError("URDF and feedback must contain the same revolute joints")
        limits = {name: (float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
                  for name, j in active.items()}
        samples = make_pose_samples(names, current, limits)
        root, _ = urdf_forward_kinematics(robot, dict(zip(names, current)))
        frame = diagnostics["configured_tcp"]["frame"]
        description = {"api_version": 1.0, "cspace": list(names), "root_link": root,
                       "default_q": list(map(float, current)), "cspace_to_urdf_rules": [],
                       "collision_spheres": []}
        yaml_path = directory / "fanuc_robot_description_fk_only.yaml"
        if yaml_path.exists():
            raise FileExistsError("use a fresh model export directory for each validation")
        yaml_path.write_text("# FK validation only. No collision/trajectory certification or home command.\n"
                             + yaml.safe_dump(description, sort_keys=False), encoding="utf-8")
        report.update(robot_description_path=str(yaml_path),
                      joint_names=list(names), root_link=root, end_link=frame, planned_poses=len(samples))
        if solver_factory is None:
            from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver
            solver_factory = LulaKinematicsSolver
        solver = solver_factory(robot_description_path=str(yaml_path), urdf_path=str(lula_urdf_path))
        solver_names = list(solver.get_joint_names())
        if len(set(solver_names)) != len(names) or set(solver_names) != set(names):
            raise ValueError("Lula joint mapping differs from USD/URDF")
        if frame not in solver.get_all_frame_names():
            raise ValueError("Lula is missing the end-link frame")
        base_prim = _unique_prim(stage, robot_path, root)
        cache = UsdGeom.XformCache()
        base_transform = cache.GetLocalToWorldTransform(base_prim)
        base_quat = base_transform.ExtractRotationQuat()
        meters = UsdGeom.GetStageMetersPerUnit(stage)
        base_world = _usd_rigid_matrix(cache, base_prim, meters)
        solver.set_robot_base_pose(np.array(base_transform.ExtractTranslation()),
                                  np.array([base_quat.GetReal(), *base_quat.GetImaginary()]))
        report["lula_joint_names"] = solver_names
        for label, values in samples:
            q = dict(zip(names, map(float, values)))
            _, fk = urdf_forward_kinematics(robot, q)
            urdf_pose = base_world @ fk[frame]
            usd_pose = usd_joint_forward_kinematics(stage, robot_path, root,
                                                   diagnostics["configured_tcp"]["matching_paths"][0], q)
            position, rotation = solver.compute_forward_kinematics(frame, np.array([q[n] for n in solver_names]))
            lula_pose = np.eye(4)
            lula_pose[:3, 3] = np.asarray(position, dtype=float).reshape(3) * meters
            lula_pose[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
            pairs = {"usd_vs_urdf": _pose_error(usd_pose, urdf_pose),
                     "urdf_vs_lula": _pose_error(urdf_pose, lula_pose),
                     "usd_vs_lula": _pose_error(usd_pose, lula_pose)}
            passed = all(e["position_error_m"] <= 0.002 and e["orientation_error_rad"] <= 0.01
                         for e in pairs.values())
            report["pose_results"].append({"pose": label, "joints_rad": q, "passed": passed,
                                           "comparisons": pairs})
        if not report["pose_results"] or not all(row["passed"] for row in report["pose_results"]):
            raise ValueError("one or more computed sample poses differ between USD/URDF/Lula")
        report["status"] = "PASSED_COMPUTED_MULTI_POSE"
    except Exception as exc:
        report["errors"].append(f"{type(exc).__name__}: {exc}")
    report["checked_poses"] = len(report["pose_results"])
    report["passed_poses"] = sum(row["passed"] for row in report["pose_results"])
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    return report
