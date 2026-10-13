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


def _usd_rigid_matrix(cache, prim, meters):
    return _gf_rigid_matrix(cache.GetLocalToWorldTransform(prim), meters)


def _gf_rigid_matrix(transform, meters):
    value = np.asarray([[transform[i][j] for j in range(4)] for i in range(4)]).T.copy()
    rotation = value[:3, :3]
    if (not np.isfinite(value).all()
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
            or not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-5)):
        raise ValueError("scaled/nonfinite/reflected robot frame cannot be compared as a rigid URDF pose")
    value[:3, 3] *= meters
    return value


def exporter_link_frames(stage, robot_path):
    """Read the installed exporter's own link-frame convention, not body frames."""
    from pxr import UsdGeom
    from isaacsim.asset.exporter.urdf.converter.robot_finder import find_robot
    from isaacsim.asset.exporter.urdf.converter.urdf_frames import build_urdf_frames
    description = find_robot(stage, robot_path)
    frames, _ = build_urdf_frames(description)
    robot_world = UsdGeom.XformCache().GetLocalToWorldTransform(description.root_prim)
    meters = UsdGeom.GetStageMetersPerUnit(stage)
    return {path: _gf_rigid_matrix(frame * robot_world, meters) for path, frame in frames.items()}


def add_fixed_end_frame(urdf_path, stage, robot_path, diagnostics, *, link_frames=None):
    """Preserve raw export; derive a missing nonphysical end frame from USD only."""
    from pxr import Usd, UsdGeom, UsdPhysics

    tree = ET.parse(urdf_path)
    robot = tree.getroot()
    frame = diagnostics["configured_tcp"]["frame"]
    links = {node.get("name") for node in robot.findall("link")}
    if frame in links:
        return Path(urdf_path), {"status": "NOT_NEEDED"}
    configured = diagnostics["configured_tcp"]
    if configured["status"] != "unique_link_found" or len(configured["matching_paths"]) != 1:
        raise ValueError("cannot add a missing or ambiguous USD end frame")
    root = stage.GetPrimAtPath(robot_path)
    end = stage.GetPrimAtPath(configured["matching_paths"][0])
    if (not end or not end.GetPath().HasPrefix(root.GetPath())
            or not end.IsA(UsdGeom.Xformable)):
        raise ValueError("end frame must be an Xformable inside this robot")
    parent = end
    while parent and parent != root and parent.GetName() not in links:
        if parent.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError("cannot replace an omitted rigid body with a fixed virtual frame")
        parent = parent.GetParent()
    if not parent or parent.GetName() not in links:
        raise ValueError("no exported ancestor link found for USD end frame")
    matches = [p for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies())
               if p.GetName() == parent.GetName() and p.IsA(UsdGeom.Xformable)]
    if len(matches) != 1:
        raise ValueError("exported ancestor name is ambiguous in USD")
    cache = UsdGeom.XformCache()
    meters = diagnostics["meters_per_unit"]
    parent_path = str(parent.GetPath())
    if link_frames is None or parent_path not in link_frames:
        raise ValueError("official exporter link frame is required; USD body frame is not a substitute")
    parent_frame = np.asarray(link_frames[parent_path], dtype=float)
    if (parent_frame.shape != (4, 4) or not np.isfinite(parent_frame).all()
            or not np.allclose(parent_frame[3], [0, 0, 0, 1])
            or not np.allclose(parent_frame[:3, :3].T @ parent_frame[:3, :3], np.eye(3), atol=1e-5)
            or not math.isclose(float(np.linalg.det(parent_frame[:3, :3])), 1.0, abs_tol=1e-5)):
        raise ValueError("invalid exporter link frame")
    relative = (np.linalg.inv(parent_frame)
                @ _usd_rigid_matrix(cache, end, meters))
    rotation = relative[:3, :3]
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1, 1)))
    if abs(math.cos(pitch)) > 1e-8:
        roll, yaw = math.atan2(rotation[2, 1], rotation[2, 2]), math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll, yaw = 0.0, math.atan2(-rotation[0, 1], rotation[1, 1])
    rpy = [roll, pitch, yaw]
    joint_name = f"fdw_{frame}_fixed"
    if any(j.get("name") == joint_name for j in robot.findall("joint")):
        raise ValueError(f"derived fixed joint name already exists: {joint_name}")
    ET.SubElement(robot, "link", name=frame)
    joint = ET.SubElement(robot, "joint", name=joint_name, type="fixed")
    ET.SubElement(joint, "parent", link=parent.GetName())
    ET.SubElement(joint, "child", link=frame)
    ET.SubElement(joint, "origin", xyz=" ".join(format(v, ".17g") for v in relative[:3, 3]),
                  rpy=" ".join(format(v, ".17g") for v in rpy))
    output = Path(urdf_path).with_name(Path(urdf_path).stem + "_with_end_frame.urdf")
    if output.exists():
        raise FileExistsError(output)
    ET.indent(tree)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    body_frame = _usd_rigid_matrix(cache, parent, meters)
    return output, {"status": "ADDED_FROM_USD", "parent_link": parent.GetName(),
                    "source_parent_path": str(parent.GetPath()), "source_end_path": str(end.GetPath()),
                    "xyz_m": relative[:3, 3].tolist(), "rpy_rad": rpy,
                    "parent_frame_source": "installed_exporter.build_urdf_frames",
                    "exporter_parent_world_matrix_m": parent_frame.tolist(),
                    "usd_body_parent_world_matrix_m": body_frame.tolist(),
                    "calibration_verified": False, "inertial_properties_added": False}


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

        predicted = _usd_rigid_matrix(cache, roots[0], diagnostics["meters_per_unit"]) @ fk[frame]
        actual = _usd_rigid_matrix(cache, stage.GetPrimAtPath(diagnostics["configured_tcp"]["matching_paths"][0]),
                                   diagnostics["meters_per_unit"])
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


def export_robot_model(stage, robot_path, spec, diagnostics, output_dir, *, converter_factory=None,
                       frame_provider=None):
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
    raw_urdf_path = urdf_path
    try:
        frame = diagnostics["configured_tcp"]["frame"]
        needs_frame = not any(link.get("name") == frame for link in ET.parse(raw_urdf_path).getroot().findall("link"))
        frames = (frame_provider or exporter_link_frames)(stage, robot_path) if needs_frame else None
        urdf_path, end_frame = add_fixed_end_frame(raw_urdf_path, stage, robot_path, diagnostics,
                                                 link_frames=frames)
        check = compare_urdf_to_usd(urdf_path, stage, robot_path, diagnostics)
    except Exception as exc:
        end_frame = {"status": "FAILED", "error": f"{type(exc).__name__}: {exc}"}
        check = {"status": "FAILED", "errors": [end_frame["error"]],
                 "full_kinematics_verified": False, "physical_grasp_verified": False}
    report = {"raw_urdf_path": str(raw_urdf_path), "urdf_path": str(urdf_path),
              "fixed_end_frame": end_frame, "source_diagnostics": diagnostics, "comparison": check,
              "controller_configuration_changed": False,
              "next_step": "Review URDF and one-pose comparison before generating a matching Lula description."}
    (output_dir / "model_comparison.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    return report
