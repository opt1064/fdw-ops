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
        target_half = np.abs(delta[:3, :3]) @ (support_dim/2)
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
    target.AddScaleOp(opSuffix="transfer").Set(Gf.Vec3f(*(np.array(fixture["support_dimensions_world_m"])/fixture["meters_per_unit"])))
    target.GetDisplayColorAttr().Set([Gf.Vec3f(.55, .15, .85)])
    fixture.update(target_support_path=path, transfer=plan)
