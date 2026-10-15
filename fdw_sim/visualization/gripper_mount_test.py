"""Temporary direct-frame FANUC/Robotiq assembly; never a calibrated tool profile."""
import math

import numpy as np

from fdw_sim.visualization.robot_model_export import _usd_rigid_matrix


def check_mount_unit_compatibility(source_meters, stage_meters):
    """Accept float32 metadata roundoff only; never rescale geometry or physics."""
    source_meters, stage_meters = float(source_meters), float(stage_meters)
    if not all(math.isfinite(v) and v > 0 for v in (source_meters, stage_meters)):
        raise ValueError("mount unit metadata must be positive and finite")
    # Isaac's Robotiq asset encodes 0.9999999776482582 instead of exactly 1 m/unit.
    # One float32 relative epsilon covers this export precision, at any unit magnitude.
    tolerance = float(np.finfo(np.float32).eps)
    relative_difference = abs(source_meters-stage_meters)/max(source_meters, stage_meters)
    if not math.isclose(source_meters, stage_meters, rel_tol=tolerance, abs_tol=0):
        raise ValueError(f"FANUC/gripper units differ: source={source_meters}, stage={stage_meters}; "
                         "refusing implicit physics scaling")
    return {"source_meters_per_unit": source_meters, "stage_meters_per_unit": stage_meters,
            "relative_difference": relative_difference, "relative_tolerance": tolerance,
            "comparison": "float32_metadata_precision", "unit_rescaling_applied": False}


def _unique_frame(stage, root_path, name):
    from pxr import Usd, UsdGeom
    root = stage.GetPrimAtPath(root_path)
    if not root:
        raise ValueError(f"missing assembly root: {root_path}")
    found = [p for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies())
             if p.GetName() == name and p.IsA(UsdGeom.Xformable)]
    if len(found) != 1:
        raise ValueError(f"missing/ambiguous assembly frame: {root_path}/{name}")
    return found[0]


def _local_joint_frame(joint, side, matrix, meters):
    from pxr import Gf
    rotation = Gf.Matrix3d(*matrix[:3, :3].T.flatten().tolist()).ExtractRotation().GetQuat()
    getattr(joint, f"CreateLocalPos{side}Attr")().Set(Gf.Vec3f(*(matrix[:3, 3]/meters)))
    getattr(joint, f"CreateLocalRot{side}Attr")().Set(Gf.Quatf(float(rotation.GetReal()), Gf.Vec3f(*rotation.GetImaginary())))


def mount_fanuc_gripper(stage, robot_source, gripper_setup):
    """Merge articulations using one fixed joint; all edits stay on the anonymous root."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
    if not stage.GetRootLayer().anonymous or stage.GetEditTarget().GetLayer() != stage.GetRootLayer():
        raise ValueError("mount experiment requires anonymous root edit target")
    source = Usd.Stage.Open(robot_source)
    if not source or not source.GetDefaultPrim():
        raise ValueError("FANUC asset must provide a default prim")
    meters = UsdGeom.GetStageMetersPerUnit(stage)
    unit_compatibility = check_mount_unit_compatibility(UsdGeom.GetStageMetersPerUnit(source), meters)
    arm_path = "/World/Fanuc"
    if stage.GetPrimAtPath(arm_path) or stage.GetPrimAtPath("/World/GripperMount"):
        raise ValueError("mount experiment already contains an arm/mount")
    arm = UsdGeom.Xform.Define(stage, arm_path).GetPrim()
    arm.GetReferences().AddReference(robot_source)
    flange = _unique_frame(stage, arm_path, "flange")
    ee = _unique_frame(stage, arm_path, "ee_link")
    if not ee.GetPath().HasPrefix(flange.GetPath()):
        raise ValueError("FANUC ee_link must be a fixed child frame of flange")
    terminal = flange
    while terminal and not terminal.IsPseudoRoot() and not terminal.HasAPI(UsdPhysics.RigidBodyAPI):
        terminal = terminal.GetParent()
    if not terminal or terminal.IsPseudoRoot() or not terminal.GetPath().HasPrefix(Sdf.Path(arm_path)):
        raise ValueError("FANUC flange has no rigid-body ancestor")
    base = stage.GetPrimAtPath(gripper_setup["base_path"])
    if not base or not base.HasAPI(UsdPhysics.RigidBodyAPI):
        raise ValueError("Robotiq base must be an actual rigid body")
    gripper = base
    while gripper.GetParent() and not gripper.GetParent().IsPseudoRoot():
        gripper = gripper.GetParent()
    root_path = str(gripper.GetPath())
    grasp = _unique_frame(stage, root_path, "grasp_frame")
    # Verify the FANUC joint graph before changing the gripper anchor.
    arm_prims = list(Usd.PrimRange(arm, Usd.TraverseInstanceProxies()))
    body_paths = {str(p.GetPath()) for p in arm_prims if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    child_bodies, arm_world_anchors = set(), []
    movable_names = []
    for p in arm_prims:
        if not p.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(p)
        if joint.GetJointEnabledAttr().Get() is False:
            continue
        b0, b1 = list(map(str, joint.GetBody0Rel().GetTargets())), list(map(str, joint.GetBody1Rel().GetTargets()))
        if len(b1) != 1 or b1[0] not in body_paths or len(b0) > 1 or (b0 and b0[0] not in body_paths):
            raise ValueError("FANUC joint bodies escape referenced arm or are ambiguous")
        if b0:
            child_bodies.add(b1[0])
        elif p.IsA(UsdPhysics.FixedJoint):
            arm_world_anchors.append(str(p.GetPath()))
        else:
            raise ValueError("unexpected FANUC movable world joint")
        if p.IsA(UsdPhysics.RevoluteJoint):
            movable_names.append(p.GetName())
    if sorted(movable_names) != [f"J{i}" for i in range(1, 7)]:
        raise ValueError("expected exact FANUC J1-J6 revolute joint inventory")
    roots = body_paths-child_bodies
    if len(roots) != 1 or len(arm_world_anchors) > 1:
        raise ValueError("FANUC root body/world anchor ambiguous")
    arm_base_path = roots.pop()
    cache = UsdGeom.XformCache()
    flange_world = _usd_rigid_matrix(cache, flange, meters)
    ee_world = _usd_rigid_matrix(cache, ee, meters)
    terminal_world = _usd_rigid_matrix(cache, terminal, meters)
    base_world = _usd_rigid_matrix(cache, base, meters)
    grasp_world = _usd_rigid_matrix(cache, grasp, meters)
    root_world = _usd_rigid_matrix(cache, gripper, meters)
    approach = (np.linalg.inv(base_world) @ grasp_world)[:3, 3]
    if np.linalg.norm(approach) < 1e-6 or not np.allclose(approach/np.linalg.norm(approach), [0, 0, 1], atol=1e-5):
        raise ValueError("Robotiq base-to-grasp approach must match the asset's +Z convention")
    # Source FANUC flange/ee_link extends along +X (J6->flange offset is +X).
    # Robotiq fingers extend along base +Z. Map +Z->EE +X, +Y->EE +Y, +X->EE -Z.
    ee_to_base_rotation = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
    desired_base = flange_world.copy()  # Keep mounting position at the flange, not at an assumed TCP.
    desired_base[:3, :3] = ee_world[:3, :3] @ ee_to_base_rotation
    flange_to_base = np.linalg.inv(flange_world) @ desired_base
    new_root = desired_base @ np.linalg.inv(base_world) @ root_world
    # Gripper is a top-level prim: local == world. Preserve every internal body/joint transform.
    stage_matrix = new_root.copy()
    stage_matrix[:3, 3] /= meters
    xform = UsdGeom.Xformable(gripper)
    xform.ClearXformOpOrder()
    xform.AddTransformOp(opSuffix="fdwMount").Set(Gf.Matrix4d(*stage_matrix.T.flatten().tolist()))
    placed_base = _usd_rigid_matrix(UsdGeom.XformCache(), base, meters)
    if not np.allclose(placed_base, desired_base, atol=1e-6, rtol=0):
        raise ValueError("gripper base failed frame alignment (reset transforms/scales unsupported)")
    if not arm_world_anchors:
        # A world anchor outside the source default prim is not imported by a reference.
        # Fix the actual root body's current pose, without choosing a new home pose.
        anchor = UsdPhysics.FixedJoint.Define(stage, "/World/FanucWorldAnchor")
        anchor.CreateBody1Rel().SetTargets([arm_base_path])
        _local_joint_frame(anchor, 0, _usd_rigid_matrix(cache, stage.GetPrimAtPath(arm_base_path), meters), meters)
        _local_joint_frame(anchor, 1, np.eye(4), meters)
        arm_world_anchors.append(str(anchor.GetPath()))
    old_anchor = UsdPhysics.FixedJoint(stage.GetPrimAtPath(gripper_setup["anchor_path"]))
    if (not old_anchor or old_anchor.GetBody0Rel().GetTargets()
            or list(map(str, old_anchor.GetBody1Rel().GetTargets())) != [str(base.GetPath())]):
        raise ValueError("expected original Robotiq world anchor")
    old_anchor.CreateJointEnabledAttr(False)
    removed_roots = []
    for p in Usd.PrimRange(gripper, Usd.TraverseInstanceProxies()):
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            if p.IsInstanceProxy():
                raise ValueError("cannot override instance-proxy articulation root")
            p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            removed_roots.append(str(p.GetPath()))
    if not removed_roots:
        raise ValueError("Robotiq articulation root missing before merge")
    mount = UsdPhysics.FixedJoint.Define(stage, "/World/GripperMount")
    mount.CreateBody0Rel().SetTargets([terminal.GetPath()])
    mount.CreateBody1Rel().SetTargets([base.GetPath()])
    _local_joint_frame(mount, 0, np.linalg.inv(terminal_world) @ desired_base, meters)
    _local_joint_frame(mount, 1, np.eye(4), meters)
    mount.CreateJointEnabledAttr(True)
    mount.CreateExcludeFromArticulationAttr(False)
    mount.CreateCollisionEnabledAttr(False)  # Connected terminal/base pair only; retain other collisions.
    arm_roots = [str(p.GetPath()) for p in arm_prims if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(arm_roots) != 1:
        raise ValueError("expected exactly one FANUC articulation root for merged arm/gripper")
    collision_setting = stage.GetPrimAtPath(arm_roots[0]).GetAttribute("physxArticulation:enabledSelfCollisions")
    return {"mode": "temporary_direct_frame_mount", "robot_source_usd": robot_source,
            "unit_compatibility": unit_compatibility,
            "robot_path": arm_path, "arm_base_path": arm_base_path, "arm_articulation_path": arm_roots[0],
            "flange_path": str(flange.GetPath()), "ee_path": str(ee.GetPath()),
            "terminal_body_path": str(terminal.GetPath()),
            "gripper_root_path": root_path, "gripper_base_path": str(base.GetPath()),
            "tcp_candidate_path": str(grasp.GetPath()), "mount_joint_path": str(mount.GetPath()),
            "disabled_gripper_world_anchor": gripper_setup["anchor_path"],
            "removed_gripper_articulation_roots": removed_roots, "arm_world_anchors": arm_world_anchors,
            "merged_self_collision_setting": collision_setting.Get() if collision_setting else None,
            "self_collision_policy": "inherit FANUC articulation setting; contact behavior unverified",
            "mount_convention": "base origin at flange; Robotiq +Z aligned to EE +X and +Y to EE +Y; no adapter/changer offset",
            "desired_flange_to_base_matrix_m": flange_to_base.tolist(),
            "gripper_approach_axis_base": [0, 0, 1], "ee_forward_axis": [1, 0, 0],
            "mount_calibration_verified": False, "tcp_calibration_verified": False,
            "tool_changer_present": False, "physical_grasp_verified": False}


def mount_frame_observation(stage, mount):
    """Observe model frame offset; a stable model frame is not a calibrated TCP."""
    from pxr import UsdGeom
    from fdw_sim.visualization.lula_model_validation import _pose_error
    meters = UsdGeom.GetStageMetersPerUnit(stage)
    cache = UsdGeom.XformCache()
    flange = _usd_rigid_matrix(cache, stage.GetPrimAtPath(mount["flange_path"]), meters)
    ee = _usd_rigid_matrix(cache, stage.GetPrimAtPath(mount["ee_path"]), meters)
    base = _usd_rigid_matrix(cache, stage.GetPrimAtPath(mount["gripper_base_path"]), meters)
    tcp = _usd_rigid_matrix(cache, stage.GetPrimAtPath(mount["tcp_candidate_path"]), meters)
    expected = flange @ np.array(mount["desired_flange_to_base_matrix_m"], dtype=float)
    return {"flange_to_base_error": _pose_error(expected, base),
            "flange_to_base_matrix_m": (np.linalg.inv(flange) @ base).tolist(),
            "gripper_forward_world": base[:3, 2].tolist(), "ee_forward_world": ee[:3, 0].tolist(),
            "forward_alignment_dot": float(base[:3, 2] @ ee[:3, 0]),
            "flange_to_tcp_candidate_matrix_m": (np.linalg.inv(flange) @ tcp).tolist(),
            "tcp_candidate_world_matrix_m": tcp.tolist(), "tcp_calibration_verified": False}


class StationaryArmGuard:
    """Hold measured arm positions without changing gains; reject drift/nonfinite feedback."""
    def __init__(self, view, *, max_drift_deg=0.5):
        self.view = view
        self.names = list(view.shared_metatype.dof_names)
        if len(set(self.names)) != len(self.names) or self.names.count("finger_joint") != 1:
            raise ValueError("merged arm/gripper feedback mapping invalid")
        self.indices = [self.names.index(f"J{i}") for i in range(1, 7)]
        self.start = self._read("get_dof_positions")[0, self.indices].copy()
        gains = self._read("get_dof_stiffnesses")[0, self.indices]
        if not (gains > 0).all():
            raise ValueError("FANUC position-hold drives require authored positive stiffness")
        self.max_drift_deg, self.observed_drift_deg = max_drift_deg, 0.0
        self.commands_issued = False

    def _read(self, name):
        values = np.array(getattr(self.view, name)(), dtype=np.float32, copy=True)
        if values.shape != (1, len(self.names)) or not np.isfinite(values).all():
            raise ValueError(f"finite merged arm feedback/settings missing: {name}")
        return values

    def hold(self):
        # Preserve all gripper/passive targets; command only the six arm indices.
        positions = self._read("get_dof_position_targets")
        velocities = self._read("get_dof_velocity_targets")
        positions[0, self.indices] = self.start
        velocities[0, self.indices] = 0
        rows = np.array([0], dtype=np.int32)
        self.commands_issued = True
        self.view.set_dof_position_targets(positions, rows)
        self.view.set_dof_velocity_targets(velocities, rows)

    def check(self):
        measured = self._read("get_dof_positions")[0, self.indices]
        drift = float(np.rad2deg(np.max(np.abs(measured-self.start))))
        self.observed_drift_deg = max(self.observed_drift_deg, drift)
        if drift > self.max_drift_deg:
            raise RuntimeError(f"stationary FANUC joint drift exceeded {self.max_drift_deg} deg: {drift}")

    def diagnostics(self):
        return {"arm_joint_names": [f"J{i}" for i in range(1, 7)],
                "hold_positions_rad": self.start.tolist(), "hold_commands_issued": self.commands_issued,
                "max_observed_drift_deg": self.observed_drift_deg, "allowed_drift_deg": self.max_drift_deg,
                "arm_production_motion_verified": False}
