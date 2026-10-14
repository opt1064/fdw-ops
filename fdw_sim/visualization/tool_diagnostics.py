"""Passive tool/changer inventory. Never select tools or certify TCP/contact."""
from __future__ import annotations

import math
import json
from pathlib import Path

import numpy as np
import yaml

from fdw_sim.visualization.robot_model_export import _usd_rigid_matrix


def load_tool_configuration(path):
    config = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(config, dict) or config.get("schema_version") != 1:
        raise ValueError("tool configuration schema_version must be 1")
    tools = config.get("tools")
    if not isinstance(tools, dict) or set(tools) != {"welding_torch", "gripper"}:
        raise ValueError("separate welding_torch and gripper profiles are required")
    if config.get("active_tool") not in (None, *tools):
        raise ValueError("active_tool must be null or an existing tool profile")
    for name, tool in tools.items():
        if not isinstance(tool, dict) or tool.get("kind") != name:
            raise ValueError(f"invalid tool profile: {name}")
        offset = tool.get("flange_to_tcp")
        if offset is not None:
            if not isinstance(offset, dict):
                raise ValueError(f"invalid TCP offset: {name}")
            pos = np.asarray(offset.get("translation_m"), dtype=float)
            quat = np.asarray(offset.get("orientation_wxyz"), dtype=float)
            if (pos.shape != (3,) or quat.shape != (4,) or not np.isfinite(pos).all()
                    or not np.isfinite(quat).all() or not math.isclose(float(quat @ quat), 1, abs_tol=1e-5)):
                raise ValueError(f"TCP offset requires finite meters and a unit WXYZ quaternion: {name}")
    try:
        json.dumps(config, allow_nan=False)
    except (ValueError, TypeError) as exc:
        raise ValueError("tool configuration must contain finite JSON-compatible values") from exc
    return config


def collect_tool_diagnostics(stage, robot_path, spec, config, robot_diagnostics):
    """Read composed frames, attachment graph, drives and collision/contact attributes."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    root = stage.GetPrimAtPath(robot_path)
    if not root or not root.IsActive():
        raise ValueError("robot root missing or inactive")
    meters = UsdGeom.GetStageMetersPerUnit(stage)
    cache = UsdGeom.XformCache()
    robot_prims = list(Usd.PrimRange(root, Usd.TraverseInstanceProxies()))
    stage_prims = list(Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()))
    errors, pending = [], []
    report = {"status": "NEEDS_TOOL_CONFIGURATION", "scope": "passive composed-USD tool inventory",
              "motion_commands_issued": False, "tcp_calibration_verified": False,
              "tool_switch_verified": False, "physical_grasp_verified": False,
              "errors": errors, "pending": pending, "robot_path": robot_path,
              "configuration": config, "meters_per_unit": meters}

    def safe_value(value):
        if value is None or isinstance(value, (str, bool, int)):
            return value
        if isinstance(value, float):
            return value if math.isfinite(value) else str(value)
        try:
            return [safe_value(v) for v in value]
        except TypeError:
            return str(value)

    def attributes(prim, prefixes):
        return {attr.GetName(): {"value": safe_value(attr.Get()),
                                "authored": attr.HasAuthoredValueOpinion()}
                for attr in prim.GetAttributes() if attr.GetName().startswith(prefixes)}

    def schemas(prim):
        # CPU OpenUSD may not register PhysX/Isaac plugin schemas; retain USD tokens.
        opinion = prim.GetMetadata("apiSchemas")
        tokens = (list(opinion.explicitItems) + list(opinion.prependedItems) + list(opinion.appendedItems)) if opinion else []
        return sorted(set(prim.GetAppliedSchemas()) | set(tokens))

    def find_frame(identifier):
        if not identifier:
            return None, []
        if identifier.startswith("/"):
            candidate = stage.GetPrimAtPath(identifier)
            matches = [candidate] if candidate and candidate.IsA(UsdGeom.Xformable) else []
        else:
            matches = [p for p in robot_prims if p.GetName() == identifier and p.IsA(UsdGeom.Xformable)]
        return (matches[0] if len(matches) == 1 else None), [str(p.GetPath()) for p in matches]

    def relative(mount, tip):
        matrix = np.linalg.inv(_usd_rigid_matrix(cache, mount, meters)) @ _usd_rigid_matrix(cache, tip, meters)
        rotation = Gf.Matrix3d(*matrix[:3, :3].T.flatten().tolist()).ExtractRotation().GetQuat()
        return {"translation_m": matrix[:3, 3].tolist(),
                "orientation_wxyz": [rotation.GetReal(), *rotation.GetImaginary()],
                "source": "composed_usd_relative_transform", "calibration_verified": False}

    mount, mount_paths = find_frame(config.get("mounting_frame"))
    reference, reference_paths = find_frame(config.get("reference_end_frame"))
    report["mounting_frame_paths"] = mount_paths
    report["reference_end_frame_paths"] = reference_paths
    if mount is None:
        errors.append("mounting frame missing or ambiguous")
    if reference is None:
        errors.append("reference end frame missing or ambiguous")
    if mount is not None and reference is not None:
        try:
            report["flange_to_reference_end"] = relative(mount, reference)
        except ValueError as exc:
            errors.append(str(exc))

    terminal = mount
    while terminal and not terminal.HasAPI(UsdPhysics.RigidBodyAPI):
        terminal = terminal.GetParent()
    # Also inspect tools mounted beside RobotArm through explicit fixed joints.
    connected = {str(terminal.GetPath())} if terminal and not terminal.IsPseudoRoot() else set()
    fixed = []
    for prim in stage_prims:
        if prim.IsA(UsdPhysics.FixedJoint):
            joint = UsdPhysics.Joint(prim)
            b0, b1 = list(joint.GetBody0Rel().GetTargets()), list(joint.GetBody1Rel().GetTargets())
            if len(b0) == len(b1) == 1:
                fixed.append((prim, str(b0[0]), str(b1[0])))
    changed = True
    while changed:
        changed = False
        for prim, b0, b1 in fixed:
            enabled = prim.GetAttribute("physics:jointEnabled").Get()
            if enabled is not False and (b0 in connected or b1 in connected):
                before = len(connected)
                connected.update((b0, b1))
                changed |= len(connected) != before
    report["fixed_attachment_bodies"] = sorted(connected)
    report["fixed_attachment_joints"] = [str(p.GetPath()) for p, b0, b1 in fixed
                                          if b0 in connected and b1 in connected
                                          and p.GetAttribute("physics:jointEnabled").Get() is not False]
    robot_sdf = Sdf.Path(robot_path)
    # Follow outward movable joints too, for sibling finger bodies in mounted tools.
    attachment_bodies = set(connected)
    outward_joints = set()
    changed = True
    while changed:
        changed = False
        for prim in stage_prims:
            if not prim.IsA(UsdPhysics.Joint) or prim.GetAttribute("physics:jointEnabled").Get() is False:
                continue
            joint = UsdPhysics.Joint(prim)
            b0, b1 = list(map(str, joint.GetBody0Rel().GetTargets())), list(map(str, joint.GetBody1Rel().GetTargets()))
            if len(b0) == len(b1) == 1 and b0[0] in attachment_bodies:
                outward_joints.add(str(prim.GetPath()))
                before = len(attachment_bodies)
                attachment_bodies.add(b1[0])
                changed |= len(attachment_bodies) != before
    roots = [robot_sdf, *[Sdf.Path(p) for p in attachment_bodies]]
    scoped = [p for p in stage_prims if any(p.GetPath().HasPrefix(r) for r in roots)
              or str(p.GetPath()) in outward_joints]
    report["terminal_attachment_bodies"] = sorted(attachment_bodies)
    report["inventory_scope_roots"] = list(map(str, roots))
    report["discovery_note"] = "Names identify candidates only. Scope includes robot subtree, fixed-connected terminal bodies and outward joint children; unnamed/unattached external tools require explicit model selection."
    feedback = robot_diagnostics.get("joint_feedback", {})
    names = feedback.get("names") or []
    positions = feedback.get("positions") or []
    feedback_map = dict(zip(names, positions))
    joints, colliders, candidates, surfaces = [], [], [], []
    for prim in scoped:
        path = str(prim.GetPath())
        prim_schemas = schemas(prim)
        if any(token in prim.GetName().lower() for token in ("grip", "finger", "jaw", "suction", "vacuum", "torch", "tool")):
            candidates.append({"path": path, "type": prim.GetTypeName(), "candidate_only": True})
        if "gripper" in prim.GetTypeName().lower() or any("gripper" in s.lower() for s in prim_schemas):
            surfaces.append({"path": path, "type": prim.GetTypeName(), "schemas": prim_schemas,
                             "attributes": attributes(prim, ("surfaceGripper:", "isaac:", "physx:"))})
        if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
            joint = UsdPhysics.Joint(prim)
            joints.append({"name": prim.GetName(), "path": path, "type": prim.GetTypeName(),
                           "body0": list(map(str, joint.GetBody0Rel().GetTargets())),
                           "body1": list(map(str, joint.GetBody1Rel().GetTargets())),
                           "joint_attributes": attributes(prim, ("physics:", "physxJoint:", "drive:", "physxMimicJoint:")),
                           "measured_position": feedback_map.get(prim.GetName()),
                           "position_units": "radians" if prim.IsA(UsdPhysics.RevoluteJoint) else "stage_units",
                           "is_catalog_arm_joint": prim.GetName() in {f"J{i}" for i in range(1, 7)},
                           "gripper_membership_verified": False})
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            ancestors, body = [], prim
            while body and not body.IsPseudoRoot():
                bindings = [{"relationship": rel.GetName(), "targets": list(map(str, rel.GetTargets()))}
                            for rel in body.GetRelationships() if rel.GetName().startswith("material:binding")]
                for binding in bindings:
                    binding["target_material_attributes"] = {
                        target: attributes(stage.GetPrimAtPath(target), ("physics:", "physxMaterial:"))
                        for target in binding["targets"] if stage.GetPrimAtPath(target)}
                contact_attributes = attributes(body, ("physxContactReport:",))
                if bindings or contact_attributes or any("ContactReport" in schema for schema in schemas(body)):
                    ancestors.append({"path": str(body.GetPath()), "material_bindings": bindings,
                                      "contact_report_attributes": contact_attributes,
                                      "schemas": schemas(body)})
                body = body.GetParent()
            colliders.append({"path": path, "collision_enabled": UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get(),
                              "attributes": attributes(prim, ("physics:", "physxCollision:", "physxContactReport:")),
                              "ancestor_material_contact_evidence": ancestors,
                              "is_terminal_scope": any(prim.GetPath().HasPrefix(Sdf.Path(p)) for p in attachment_bodies),
                              "gripper_contact_surface_verified": False})
    report.update(tool_candidates=candidates, movable_joints=joints, colliders=colliders,
                  surface_gripper_candidates=surfaces, live_joint_names=names)
    profiles = {}
    for name, tool in config["tools"].items():
        tcp, paths = find_frame(tool.get("tcp_frame"))
        item = {"tcp_paths": paths, "tcp_calibration_verified": False, "missing": []}
        for key in ("asset_path", "root_prim_path", "tcp_frame", "flange_to_tcp", "calibration_evidence"):
            if tool.get(key) is None:
                item["missing"].append(key)
        if tcp is not None and mount is not None:
            try:
                item["observed_flange_to_tcp"] = relative(mount, tcp)
            except ValueError as exc:
                errors.append(f"{name}: {exc}")
        elif tool.get("tcp_frame"):
            item["missing"].append("unique_live_tcp_frame")
        if tool.get("root_prim_path"):
            item["configured_root_prim_exists"] = bool(stage.GetPrimAtPath(tool["root_prim_path"]))
            if not item["configured_root_prim_exists"]:
                item["missing"].append("live_tool_root")
        if name == "gripper":
            for key in ("gripper_type", "contact_feedback_source"):
                if tool.get(key) is None:
                    item["missing"].append(key)
            if tool.get("gripper_type") == "parallel":
                for key in ("joint_names", "opened_positions", "closed_positions"):
                    if not tool.get(key):
                        item["missing"].append(key)
            item["configured_joint_names_not_in_feedback"] = sorted(set(tool.get("joint_names", []))-set(names))
            if item["configured_joint_names_not_in_feedback"]:
                item["missing"].append("configured_joint_feedback")
        profiles[name] = item
        pending.extend(f"{name}: {key}" for key in item["missing"])
    report["tool_profiles"] = profiles
    if config.get("active_tool") is None:
        pending.append("active tool identity and mounted/locked state")
    for key in ("asset_path", "mounting_joint_path", "lock_feedback_source"):
        if not config.get("tool_changer", {}).get(key):
            pending.append(f"tool changer: {key}")
    pending.extend(["tool-specific TCP calibration against measured geometry",
                    "gripper opening/closing and contact/retention validation",
                    "tool exchange, lock acknowledgement and TCP switching validation"])
    report["status"] = "FAILED" if errors else "NEEDS_TOOL_CONFIGURATION"
    return report
