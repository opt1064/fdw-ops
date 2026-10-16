"""Plan a short J1 arc between two separate static supports, without moving the object."""
import math

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
