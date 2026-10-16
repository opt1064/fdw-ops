"""Plan a short J1 arc between two separate static supports, without moving the object."""
import math
import itertools

import numpy as np


def plan_transfer(model, start, lifted, fixture):
    start, lifted = np.asarray(start, float), np.asarray(lifted, float)
    if (start.shape != (6,) or lifted.shape != (6,) or not np.isfinite(start).all()
            or not np.isfinite(lifted).all()):
        raise ValueError("finite six-joint start/lift poses required")
    initial = np.array(fixture["initial_object_world_matrix_m"])
    half = np.array(fixture["dimensions_base_m"])/2
    support_dim = np.array(fixture["support_dimensions_world_m"])
    source_half = np.abs(initial[:3, :3]) @ half
    target_dimensions = support_dim.copy()
    target_dimensions[1] = min(float(target_dimensions[1]), .02)
    candidates = []
    for sign in (-1, 1):
        down = start.copy(); down[0] += sign*math.radians(8)
        across = lifted.copy(); across[0] = down[0]
        path = [start, lifted, across, down]
        if any(np.any(q < model.limits[:, 0]) or np.any(q > model.limits[:, 1]) for q in path):
            continue
        delta = model.predict(down)[1] @ np.linalg.inv(model.predict(start)[1])
        if not np.allclose(delta[2, :3], [0, 0, 1], atol=1e-4) or abs(delta[2, 3]) > .001:
            continue  # Only a horizontal yaw arc can use the flat target pedestal.
        target = delta @ initial
        offset = target[:2, 3]-initial[:2, 3]
        distance = float(np.linalg.norm(offset))
        # Do not put the destination under any part of the initial block.
        target_half = np.abs(delta[:3, :3]) @ (target_dimensions/2)
        separation = np.abs(offset)-source_half[:2]-target_half[:2]
        if not .10 <= distance <= .15 or max(separation) < .01:
            continue
        target_top = float(target[2, 3]-(np.abs(target[2, :3]) @ half)-.0005)
        safe = True
        minimum_clearance = math.inf
        # Check the planned block bottom along the elevated sweep, not just its endpoints.
        for alpha in np.linspace(0, 1, 41):
            q = lifted+(across-lifted)*alpha
            pose = model.predict(q)[1] @ np.linalg.inv(model.predict(start)[1]) @ initial
            bottom = pose[2, 3]-(np.abs(pose[2, :3]) @ half)
            clearance = float(bottom-max(target_top, fixture["support_top_world_z_m"]))
            minimum_clearance = min(minimum_clearance, clearance)
            if clearance < .008:
                safe = False
                break
        if safe:
            candidates.append({"across_rad": across.tolist(), "down_rad": down.tolist(),
                               "target_object_world_matrix_m": target.tolist(),
                               "target_support_top_world_z_m": target_top,
                               "target_support_rotation": delta[:3, :3].tolist(),
                               "target_support_dimensions_m": target_dimensions.tolist(),
                               "horizontal_distance_m": distance, "j1_delta_deg": sign*8,
                               "sweep_samples": 41, "minimum_predicted_sweep_clearance_m": minimum_clearance})
    if not candidates:
        raise ValueError("no limit-safe 10-15cm J1 transfer with separate support and 8mm planned clearance")
    return candidates[0]


def create_target_support(stage, fixture, plan):
    from pxr import Gf, Sdf, UsdGeom
    if not stage.GetRootLayer().anonymous or stage.GetEditTarget().GetLayer() != stage.GetRootLayer():
        raise ValueError("target support requires anonymous experiment edit target")
    path = "/World/GraspTest/TargetSupport"
    if stage.GetPrimAtPath(path):
        raise ValueError("target support already exists")
    if not Sdf.CopySpec(stage.GetRootLayer(), fixture["support_path"], stage.GetRootLayer(), path):
        raise ValueError("could not copy static test support")
    target = UsdGeom.Cube(stage.GetPrimAtPath(path))
    target.ClearXformOpOrder()
    position = np.array(plan["target_object_world_matrix_m"])[:3, 3].copy()
    position[2] = plan["target_support_top_world_z_m"]-.005
    target.AddTranslateOp(opSuffix="transfer").Set(Gf.Vec3d(*(position/fixture["meters_per_unit"])))
    rotation = np.array(plan["target_support_rotation"])
    quat = Gf.Matrix3d(*rotation.T.flatten().tolist()).ExtractRotation().GetQuat()
    target.AddOrientOp(UsdGeom.XformOp.PrecisionDouble, opSuffix="transfer").Set(quat)
    target.AddScaleOp(opSuffix="transfer").Set(Gf.Vec3f(*(np.array(plan["target_support_dimensions_m"])/fixture["meters_per_unit"])))
    target.GetDisplayColorAttr().Set([Gf.Vec3f(.55, .15, .85)])
    fixture.update(target_support_path=path, transfer=plan)


def audit_target_clearance(stage, fixture, robot_roots):
    """Conservative measured AABBs at the initial pose, including invisible collision proxies.

    This checks the new static fixture before physics incorporation, not the full swept arm.
    """
    from pxr import Usd, UsdGeom, UsdPhysics
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render,
                                                    UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide], False, True)
    meters = fixture["meters_per_unit"]
    def bound(prim):
        box = cache.ComputeWorldBound(prim).ComputeAlignedBox()
        values = np.array([box.GetMin(), box.GetMax()])*meters
        if box.IsEmpty() or not np.isfinite(values).all():
            raise ValueError(f"cannot measure collision clearance: {prim.GetPath()}")
        return values
    target = bound(stage.GetPrimAtPath(fixture["target_support_path"]))
    checked, nearest, minimum = 0, None, math.inf
    for path in robot_roots:
        root = stage.GetPrimAtPath(path)
        if not root:
            raise ValueError(f"robot collision root missing: {path}")
        for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
            if not prim.HasAPI(UsdPhysics.CollisionAPI) or UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
                continue
            box = bound(prim)
            gap = np.maximum(np.maximum(target[0]-box[1], box[0]-target[1]), 0.)
            distance = float(np.linalg.norm(gap))
            checked += 1
            if distance < minimum:
                minimum, nearest = distance, str(prim.GetPath())
            if distance < .003:
                raise RuntimeError(f"target pedestal overlaps or is within 3mm of robot collider: {prim.GetPath()} ({distance*1000:.3f}mm)")
    if not checked:
        raise ValueError("no robot collision geometry measured for target pedestal")
    result = {"checked_colliders": checked, "nearest_collider": nearest, "minimum_aabb_distance_m": minimum,
              "scope": "initial pose conservative bounds only; not full trajectory collision validation"}
    fixture["target_initial_clearance_audit"] = result
    return result


def gripper_collision_world_points(stage, mount):
    """Eight conservative local-bound corners per enabled gripper collider, in world metres."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render,
                                                    UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide], False, True)
    transforms = UsdGeom.XformCache()
    root = stage.GetPrimAtPath(mount["gripper_root_path"])
    if not root:
        raise ValueError("gripper collision root missing")
    meters, points, paths = UsdGeom.GetStageMetersPerUnit(stage), [], []
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI) or UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
            continue
        box = cache.ComputeUntransformedBound(prim).ComputeAlignedBox()
        if box.IsEmpty():
            raise ValueError(f"empty gripper return collision bounds: {prim.GetPath()}")
        transform = transforms.GetLocalToWorldTransform(prim)
        for bits in itertools.product((0, 1), repeat=3):
            corner = Gf.Vec3d(*[box.GetMax()[i] if bits[i] else box.GetMin()[i] for i in range(3)])
            points.append(np.array(transform.Transform(corner))*meters)
        paths.append(str(prim.GetPath()))
    points = np.array(points)
    if not paths or points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("finite measured gripper collision bounds required for empty return")
    return points, paths


def object_top_world_z(object_pose, fixture):
    pose = np.array(object_pose, dtype=float)
    if pose.shape != (4, 4) or not np.isfinite(pose).all():
        raise ValueError("finite placed-object pose required")
    return float(pose[2, 3]+np.abs(pose[2, :3]) @ (np.array(fixture["dimensions_base_m"])/2))


def plan_empty_return(model, world_points, measured_q, arm_start, across, fixture, object_pose):
    """After release/retreat, raise J2 until the entire open gripper clears the placed block.

    Snapshot measured open geometry in TCP coordinates. Do not use the TCP marker height
    as a proxy for the lowest finger/body. No object transform is authored here.
    """
    points = np.asarray(world_points, float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 8 or not np.isfinite(points).all():
        raise ValueError("measured collision corners required for empty return")
    current_tcp = model.predict(measured_q)[1]
    local = (np.linalg.inv(current_tcp) @ np.c_[points, np.ones(len(points))].T).T
    obstacle_top = max(object_top_world_z(object_pose, fixture), fixture["support_top_world_z_m"],
                       fixture["transfer"]["target_support_top_world_z_m"])
    across = np.array(across, dtype=float)
    baseline = (model.predict(across)[1] @ local.T).T[:, 2]
    for degrees in range(1, 9):
        for sign in (-1, 1):
            high = across.copy()
            high[1] = arm_start[1]+sign*math.radians(degrees)
            home_high = high.copy(); home_high[0] = arm_start[0]
            if any(np.any(q < model.limits[:, 0]) or np.any(q > model.limits[:, 1]) for q in (high, home_high)):
                continue
            # Reject a proposed raising motion that lowers any measured collider corner.
            if any(np.min((model.predict(across+(high-across)*a)[1] @ local.T).T[:, 2]-baseline) < -.001
                   for a in np.linspace(0, 1, 21)):
                continue
            minimum = min(float(np.min((model.predict(high+(home_high-high)*a)[1] @ local.T).T[:, 2])-obstacle_top)
                          for a in np.linspace(0, 1, 41))
            if minimum >= .008:
                return high, home_high, {"joint": "J2", "delta_deg": sign*degrees,
                                        "high_target_rad": high.tolist(), "home_high_target_rad": home_high.tolist(),
                                        "obstacle_top_world_z_m": obstacle_top,
                                        "minimum_predicted_gripper_clearance_m": minimum,
                                        "collision_corner_count": len(points), "return_samples": 41,
                                        "scope": "measured open gripper bounds; post-release empty return only"}
    raise ValueError("no authored-limit-safe empty J2 raise within 8deg clears the placed block by 8mm")
