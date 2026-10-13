"""Export a loaded robot and check its URDF against USD, without motion commands."""
from __future__ import annotations

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def _vector(text, default):
    values = np.array([float(v) for v in (text or default).split()])
    if values.shape != (3,) or not np.isfinite(values).all():
        raise ValueError("URDF vectors must contain three finite values")
    return values


def _rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if not math.isfinite(angle) or not math.isfinite(float(norm)) or norm < 1e-12:
        raise ValueError("invalid joint axis or angle")
    x, y, z = axis / norm
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


def _joint_transform(joint, q):
    origin = joint.find("origin")
    xyz = _vector(origin.get("xyz") if origin is not None else None, "0 0 0")
    rpy = _vector(origin.get("rpy") if origin is not None else None, "0 0 0")
    transform = np.eye(4)
    transform[:3, :3] = (_rotation([0, 0, 1], rpy[2]) @ _rotation([0, 1, 0], rpy[1])
                         @ _rotation([1, 0, 0], rpy[0]))
    transform[:3, 3] = xyz
    motion = np.eye(4)
    kind = joint.get("type")
    if kind != "fixed":
        name = joint.get("name")
        if name not in q:
            raise ValueError(f"joint feedback missing for {name}")
        axis_node = joint.find("axis")
        axis = _vector(axis_node.get("xyz") if axis_node is not None else None, "1 0 0")
        if kind in ("revolute", "continuous"):
            motion[:3, :3] = _rotation(axis, q[name])
        elif kind == "prismatic":
            if np.linalg.norm(axis) < 1e-12:
                raise ValueError("zero prismatic axis")
            motion[:3, 3] = axis / np.linalg.norm(axis) * q[name]
        else:
            raise ValueError(f"unsupported joint type {kind}")
    return transform @ motion


def urdf_forward_kinematics(robot, q):
    links = [node.get("name") for node in robot.findall("link")]
    joints = robot.findall("joint")
    if not links or None in links or len(set(links)) != len(links):
        raise ValueError("missing or duplicate URDF links")
    parents, children = {}, {}
    names = set()
    for joint in joints:
        name = joint.get("name")
        if not name or name in names:
            raise ValueError("missing or duplicate URDF joints")
        names.add(name)
        parent, child = joint.find("parent"), joint.find("child")
        if parent is None or child is None:
            raise ValueError(f"missing parent/child: {name}")
        p, c = parent.get("link"), child.get("link")
        if p not in links or c not in links or c in parents:
            raise ValueError(f"invalid or multiply parented link: {name}")
        parents[c] = p
        children.setdefault(p, []).append((c, joint))
    roots = set(links) - set(parents)
    if len(roots) != 1:
        raise ValueError("URDF must have one root link")
    root = roots.pop()
    transforms = {root: np.eye(4)}
    queue = [root]
    while queue:
        parent = queue.pop()
        for child, joint in children.get(parent, []):
            if child in transforms:
                raise ValueError("URDF joint graph contains a cycle")
            transforms[child] = transforms[parent] @ _joint_transform(joint, q)
            queue.append(child)
    if set(transforms) != set(links):
        raise ValueError("URDF has disconnected or cyclic links")
    return root, transforms


def compare_urdf_to_usd(urdf_path, stage, robot_path, diagnostics):
    """A single-pose consistency check, not full kinematic/contact certification."""
    from pxr import Usd, UsdGeom

    errors = []
    result = {"status": "FAILED", "errors": errors,
              "scope": "joint topology/limits and one measured-pose end-link FK",
              "full_kinematics_verified": False, "physical_grasp_verified": False}
    try:
        robot = ET.parse(urdf_path).getroot()
        if robot.tag != "robot":
            raise ValueError("URDF root must be robot")
        feedback = diagnostics["joint_feedback"]
        if feedback["status"] != "available" or diagnostics["configured_tcp"]["status"] != "unique_link_found":
            raise ValueError("finite joint feedback and a unique end link are required")
        q = dict(zip(feedback["names"], feedback["positions"]))
        if len(q) != len(feedback["names"]) or not all(map(math.isfinite, q.values())):
            raise ValueError("duplicate or invalid feedback")
        usd_joints = {j["name"]: j for j in diagnostics["usd_joints"]
                      if j["type"] in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint")}
        urdf_joints = {j.get("name"): j for j in robot.findall("joint") if j.get("type") != "fixed"}
        if set(urdf_joints) != set(usd_joints) or set(q) != set(usd_joints):
            errors.append("movable joint names differ between USD, URDF, and feedback")
        for name in set(urdf_joints) & set(usd_joints):
            joint, source = urdf_joints[name], usd_joints[name]
            kind = "revolute" if source["type"] == "PhysicsRevoluteJoint" else "prismatic"
            if joint.get("type") != kind:
                errors.append(f"joint type mismatch: {name}")
            limit = joint.find("limit")
            factor = math.pi / 180 if kind == "revolute" else diagnostics["meters_per_unit"]
            for xml_key, usd_key in (("lower", "lowerLimit"), ("upper", "upperLimit")):
                if (limit is None or limit.get(xml_key) is None or source[usd_key] is None
                        or not math.isclose(float(limit.get(xml_key)), float(source[usd_key]) * factor,
                                            abs_tol=1e-4, rel_tol=1e-5)):
                    errors.append(f"joint {xml_key} limit mismatch: {name}")
            for tag, source_key in (("parent", "body0"), ("child", "body1")):
                node = joint.find(tag)
                expected = source[source_key]
                if len(expected) != 1 or node is None or node.get("link") != expected[0].rsplit("/", 1)[-1]:
                    errors.append(f"joint {tag} link mismatch: {name}")
        root, fk = urdf_forward_kinematics(robot, q)
        frame = diagnostics["configured_tcp"]["frame"]
        if frame not in fk:
            raise ValueError(f"URDF end link missing: {frame}")
        prims = list(Usd.PrimRange(stage.GetPrimAtPath(robot_path), Usd.TraverseInstanceProxies()))
        roots = [p for p in prims if p.GetName() == root and p.IsA(UsdGeom.Xformable)]
        if len(roots) != 1:
            raise ValueError(f"URDF root link has no unique USD match: {root}")
        cache = UsdGeom.XformCache()

        def matrix(prim):
            transform = cache.GetLocalToWorldTransform(prim)
            value = np.asarray([[transform[i][j] for j in range(4)] for i in range(4)]).T.copy()
            if not np.isfinite(value).all() or not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-5):
                raise ValueError("scaled/nonfinite robot frame cannot be compared as a rigid URDF pose")
            value[:3, 3] *= diagnostics["meters_per_unit"]
            return value

        predicted = matrix(roots[0]) @ fk[frame]
        actual = matrix(stage.GetPrimAtPath(diagnostics["configured_tcp"]["matching_paths"][0]))
        distance = float(np.linalg.norm(predicted[:3, 3] - actual[:3, 3]))
        cosine = (np.trace(predicted[:3, :3].T @ actual[:3, :3]) - 1) / 2
        angle = math.acos(float(np.clip(cosine, -1, 1)))
        result.update(root_link=root, end_link=frame, joint_positions=q,
                      position_error_m=distance, orientation_error_rad=angle,
                      position_tolerance_m=0.002, orientation_tolerance_rad=0.01)
        if distance > 0.002 or angle > 0.01:
            errors.append("one-pose forward kinematics differs from measured USD end-link pose")
        if not errors:
            result["status"] = "PASSED_SINGLE_POSE"
    except Exception as exc:
        errors.append(f"{type(exc).__name__}: {exc}")
    return result


def export_robot_model(stage, robot_path, spec, diagnostics, output_dir, *, converter_factory=None):
    """Write only to a fresh output directory; never alter source assets or controller settings."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=False)
    urdf_path = output_dir / f"{spec.name}.urdf"
    try:
        if converter_factory is None:
            from isaacsim.asset.exporter.urdf import UsdToUrdfConverter
            converter_factory = UsdToUrdfConverter
        converter = converter_factory(stage=stage, root_prim_path=robot_path,
                                      mesh_dir_name="meshes", mesh_path_prefix="./")
        converter.convert(str(urdf_path))
        if not urdf_path.is_file():
            raise RuntimeError(f"URDF exporter did not create {urdf_path}")
    except Exception as exc:
        error_path = output_dir / "export_error.json"
        error_path.write_text(json.dumps({"status": "EXPORT_FAILED", "robot_path": robot_path,
            "error": f"{type(exc).__name__}: {exc}", "source_diagnostics": diagnostics},
            ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        raise RuntimeError(f"Robot export failed; details saved to {error_path}") from exc
    check = compare_urdf_to_usd(urdf_path, stage, robot_path, diagnostics)
    report = {"urdf_path": str(urdf_path), "source_diagnostics": diagnostics, "comparison": check,
              "controller_configuration_changed": False,
              "next_step": "Review URDF and one-pose comparison before generating a matching Lula description."}
    (output_dir / "model_comparison.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    return report
