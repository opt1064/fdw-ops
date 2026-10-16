"""Synthetic block contact/lift/hold/release experiment; no object attachment/teleport."""
import itertools
import math
import time
import traceback

import numpy as np

from fdw_sim.visualization.robot_model_export import _usd_rigid_matrix
from fdw_sim.visualization.lula_model_validation import _pose_error


def plan_lift(model, start):
    """Choose upward J5 motion using measured start and authored USD kinematics."""
    start = np.asarray(start, dtype=float)
    if start.shape != (6,) or not np.isfinite(start).all():
        raise ValueError("six finite arm positions required for lift planning")
    candidates = []
    initial = model.predict(start)[1]
    for sign in (-1, 1):
        target = start.copy()
        target[4] += sign*math.radians(2)
        if np.any(target < model.limits[:, 0]) or np.any(target > model.limits[:, 1]):
            continue
        gain = float(model.predict(target)[1][2, 3]-initial[2, 3])
        candidates.append((gain, target))
    if not candidates or max(item[0] for item in candidates) < .008:
        raise ValueError("no authored-limit-safe J5 +/-2 degree lift of at least 8mm at this pose")
    gain, target = max(candidates, key=lambda item: item[0])
    return target, {"joint": "J5", "amplitude_deg": 2, "predicted_tcp_rise_m": gain,
                    "start_rad": start.tolist(), "target_rad": target.tolist(), "model": model.diagnostics()}


def pad_bounds_in_base(stage, mount, pad_paths):
    from pxr import Gf, Usd, UsdGeom
    meters = UsdGeom.GetStageMetersPerUnit(stage)
    cache = UsdGeom.XformCache()
    inverse = np.linalg.inv(_usd_rigid_matrix(cache, stage.GetPrimAtPath(mount["gripper_base_path"]), meters))
    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
    result = []
    for path in pad_paths:
        prim = stage.GetPrimAtPath(path)
        box = bounds.ComputeUntransformedBound(prim).ComputeAlignedBox()
        if box.IsEmpty():
            raise ValueError("contact test requires measured fingertip geometry bounds")
        transform = cache.GetLocalToWorldTransform(prim)
        points = []
        for bits in itertools.product((0, 1), repeat=3):
            point = Gf.Vec3d(*[box.GetMax()[i] if bits[i] else box.GetMin()[i] for i in range(3)])
            world = np.array(transform.Transform(point))*meters
            points.append((inverse @ np.r_[world, 1])[:3])
        values = np.array(points)
        if not np.isfinite(values).all():
            raise ValueError("nonfinite contact geometry bounds")
        result.append([values.min(axis=0), values.max(axis=0)])
    return np.array(result)


def create_grasp_fixture(stage, mount, pad_paths):
    """Initial placement only, before this object's first physics step. Original assets untouched."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade
    if not stage.GetRootLayer().anonymous or stage.GetEditTarget().GetLayer() != stage.GetRootLayer():
        raise ValueError("grasp fixture requires anonymous experiment edit target")
    if UsdGeom.GetStageUpAxis(stage) != "Z":
        raise ValueError("grasp fixture currently requires world Z up")
    meters, kg = UsdGeom.GetStageMetersPerUnit(stage), UsdPhysics.GetStageKilogramsPerUnit(stage)
    if not all(math.isfinite(v) and v > 0 for v in (meters, kg)):
        raise ValueError("invalid test stage length/mass units")
    base = _usd_rigid_matrix(UsdGeom.XformCache(), stage.GetPrimAtPath(mount["gripper_base_path"]), meters)
    if abs(base[2, 0]) < .99 or abs(base[2, 1]) > .01:
        raise ValueError("fixture requires horizontal opposing fingers and nearly vertical base X")
    bounds = pad_bounds_in_base(stage, mount, pad_paths)
    bounds = bounds[np.argsort(bounds[:, :, 1].mean(axis=1))]
    gap = float(bounds[1, 0, 1]-bounds[0, 1, 1])
    low = np.maximum(bounds[0, 0], bounds[1, 0])
    high = np.minimum(bounds[0, 1], bounds[1, 1])
    if not .04 <= gap <= .18 or high[0]-low[0] < .008 or high[2]-low[2] < .008:
        raise ValueError("measured pad clearance/contact band cannot host synthetic test block")
    dimensions = np.array([min(.025, .7*(high[0]-low[0])), .85*gap, min(.025, .7*(high[2]-low[2]))])
    center = (low+high)/2
    center[1] = (bounds[0, 1, 1]+bounds[1, 0, 1])/2
    pose = base.copy()
    pose[:3, 3] = (base @ np.r_[center, 1])[:3]
    half_world = np.abs(pose[:3, :3]) @ (dimensions/2)
    support_top = float(pose[2, 3]-half_world[2]-.0005)
    # A narrow pedestal stays inside the block footprint, clear of both fingers.
    support_dim = np.array([1.3*half_world[0], 1.3*half_world[1], .01])
    root = "/World/GraspTest"
    if stage.GetPrimAtPath(root):
        raise ValueError("grasp fixture already exists; refusing duplicate experiment")
    UsdGeom.Xform.Define(stage, root)
    material = UsdShade.Material.Define(stage, root+"/TestPhysicsMaterial")
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr(.8)
    physics_material.CreateDynamicFrictionAttr(.8)
    physics_material.CreateRestitutionAttr(0)
    def cube(path, dims, position, color, rotation=None):
        geometry = UsdGeom.Cube.Define(stage, path)
        geometry.CreateSizeAttr(1)
        geometry.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        geometry.AddTranslateOp().Set(Gf.Vec3d(*(position/meters)))
        if rotation is not None:
            quat = Gf.Matrix3d(*rotation.T.flatten().tolist()).ExtractRotation().GetQuat()
            geometry.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat)
        geometry.AddScaleOp().Set(Gf.Vec3f(*(dims/meters)))
        prim = geometry.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")
        # Exact PhysX schema tokens allow CPU USD tests without importing the Kit PhysxSchema plugin.
        prim.AddAppliedSchema("PhysxCollisionAPI")
        prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(.001/meters)
        prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0)
        return prim
    support = cube(root+"/Support", support_dim, np.array([pose[0, 3], pose[1, 3], support_top-.005]), (.1, .25, .6))
    block = cube(root+"/Block", dimensions, pose[:3, 3], (1, .8, .05), pose[:3, :3])
    UsdPhysics.RigidBodyAPI.Apply(block).CreateKinematicEnabledAttr(False)
    UsdPhysics.MassAPI.Apply(block).CreateMassAttr(.05/kg)
    block.AddAppliedSchema("PhysxContactReportAPI")
    block.CreateAttribute("physxContactReport:threshold", Sdf.ValueTypeNames.Float).Set(0)
    return {"block_path": str(block.GetPath()), "support_path": str(support.GetPath()), "mass_kg": .05,
            "dimensions_base_m": dimensions.tolist(), "initial_object_world_matrix_m": pose.tolist(),
            "support_dimensions_world_m": support_dim.tolist(),
            "measured_open_clearance_m": gap, "support_top_world_z_m": support_top,
            "meters_per_unit": meters, "kilograms_per_unit": kg, "test_material_friction": .8,
            "object_downward_load_n": .05*9.81, "scene_gravity_m_s2": 0,
            "object_pose_writes_during_test": False, "object_attachment_joint_created": False,
            "scope": "synthetic 50g block; object-only gravity-equivalent load; original robot/material properties retained"}


class ContactEvidence:
    """Per-step PhysX reports; stale contact events cannot certify a hold/release."""
    def __init__(self, fixture, pad_paths, stage_id, dt, decode, lost_event):
        self.fixture, self.pads = fixture, list(pad_paths)
        self.stage_id, self.dt, self.decode, self.lost_event = int(stage_id), dt, decode, lost_event
        self.error = None
        self.callbacks = 0
        self.begin_step()

    def begin_step(self):
        self.forces = {"left": 0., "right": 0., "support": 0.}
        if "target_support_path" in self.fixture:
            self.forces["target_support"] = 0.
        self.unexpected_force_n = 0.

    def callback(self, headers, data):
        self.callbacks += 1
        try:
            for header in headers:
                if int(header.stage_id) != self.stage_id or header.type == self.lost_event:
                    continue
                pair = [str(self.decode(header.collider0)), str(self.decode(header.collider1))]
                block = self.fixture["block_path"]
                if not any(p == block or p.startswith(block+"/") for p in pair):
                    continue
                other = pair[1] if pair[0] == block or pair[0].startswith(block+"/") else pair[0]
                paths = dict(zip(("left", "right", "support"), [*self.pads, self.fixture["support_path"]]))
                if "target_support_path" in self.fixture:
                    paths["target_support"] = self.fixture["target_support_path"]
                key = next((name for name, path in paths.items()
                            if other == path or other.startswith(path+"/")), None)
                start, count = int(header.contact_data_offset), int(header.num_contact_data)
                if start < 0 or count < 0 or start+count > len(data):
                    raise ValueError("invalid contact data range")
                for point_index in range(start, start+count):
                    point = data[point_index]
                    separation = float(point.separation)*self.fixture["meters_per_unit"]
                    impulse = np.asarray(point.impulse, dtype=float)
                    normal = np.asarray(point.normal, dtype=float)
                    if (not math.isfinite(separation) or not np.isfinite(impulse).all()
                            or normal.shape != (3,) or not np.isfinite(normal).all()
                            or not math.isclose(float(normal @ normal), 1, abs_tol=.01)):
                        raise ValueError("invalid contact impulse/normal/separation")
                    if separation < -.002:
                        raise RuntimeError("test block penetrated contact geometry more than 2 mm")
                    if impulse.shape == (3,):
                        value = abs(float(impulse @ normal))
                    elif impulse.shape == ():
                        value = abs(float(impulse))
                    else:
                        raise ValueError("unsupported contact impulse shape")
                    force = value*self.fixture["kilograms_per_unit"]*self.fixture["meters_per_unit"]/self.dt
                    if separation <= .001:
                        if key:
                            self.forces[key] += force
                        elif other.startswith("/Robotiq_2F_140/") or other.startswith("/World/Fanuc/"):
                            self.unexpected_force_n += force
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"

    def read(self):
        if self.error:
            raise RuntimeError(self.error)
        if self.unexpected_force_n > .01 or max(self.forces.values()) > 50:
            raise RuntimeError("unexpected gripper contact or excessive synthetic block contact force")
        return {**self.forces, "callbacks": self.callbacks}


class LoadedTestBlock:
    def __init__(self, simulation_view, fixture):
        self.owner, self.fixture = simulation_view, fixture
        self.view = simulation_view.create_rigid_body_view(fixture["block_path"])
        if self.view is None or self.view.count != 1:
            raise ValueError("expected exactly one dynamic test block")
        mass = np.asarray(self.view.get_masses(), dtype=float)
        if mass.size != 1 or not np.isfinite(mass).all() or not math.isclose(float(mass.flat[0])*fixture["kilograms_per_unit"], .05, rel_tol=1e-4):
            raise ValueError("live test block mass differs from declared 50g")

    def load(self):
        f = self.fixture
        force = np.array([[0, 0, -f["object_downward_load_n"]/(f["kilograms_per_unit"]*f["meters_per_unit"])]], dtype=np.float32)
        self.view.apply_forces(force, np.array([0], dtype=np.int32), True)

    def measure(self):
        from pxr import Gf
        pose = np.asarray(self.view.get_transforms(), dtype=float)
        velocity = np.asarray(self.view.get_velocities(), dtype=float)
        if pose.shape != (1, 7) or velocity.shape != (1, 6) or not np.isfinite(pose).all() or not np.isfinite(velocity).all():
            raise ValueError("missing/nonfinite dynamic block feedback")
        q = pose[0, 3:]
        if not math.isclose(float(q@q), 1, abs_tol=1e-4):
            raise ValueError("invalid live test block quaternion")
        transform = Gf.Matrix4d(1)
        transform.SetRotate(Gf.Quatd(q[3], Gf.Vec3d(*q[:3])))  # Tensor quaternion is XYZW.
        matrix = np.asarray(transform, dtype=float).T.copy()
        matrix[:3, 3] = pose[0, :3]*self.fixture["meters_per_unit"]
        half_height = float(np.abs(matrix[2, :3]) @ (np.array(self.fixture["dimensions_base_m"])/2))
        data = {"object_world_matrix_m": matrix.tolist(),
                "object_speed_m_s": float(np.linalg.norm(velocity[0, :3])*self.fixture["meters_per_unit"]),
                "support_clearance_m": float(matrix[2, 3]-half_height-self.fixture["support_top_world_z_m"])}
        if "transfer" in self.fixture:
            data["target_support_clearance_m"] = float(matrix[2, 3]-half_height-self.fixture["transfer"]["target_support_top_world_z_m"])
        return data


def run_contact_grasp(command, observe, step, stop, phase_color, fixture, arm_start, lift_target,
                      open_rad, close_rad, *, dt, realtime=False, is_running=lambda: True,
                      progress=lambda event, **data: None):
    """Require bilateral contact, unsupported retention, gravity-driven fall, and supported release."""
    report = {"status": "FAILED", "errors": [], "phases": [], "samples": [], "motion_commands_issued": False,
              "physical_grasp_verified": False, "tcp_calibration_verified": False, "release_verified": False,
              "scope": fixture["scope"], "fixture": fixture, "object_attached": False,
              "transfer_verified": False, "target_placement_verified": False}
    try:
        if not math.isfinite(dt) or not 0 < dt <= 1/60 or not 0 <= open_rad < close_rad <= math.pi/4:
            raise ValueError("invalid contact test timestep/primary targets")
        arm_start, lift_target = np.asarray(arm_start, float), np.asarray(lift_target, float)
        if (arm_start.shape != (6,) or lift_target.shape != (6,) or not np.isfinite(arm_start).all()
                or not np.isfinite(lift_target).all() or np.max(np.abs(lift_target-arm_start)) > math.radians(2)+1e-6):
            raise ValueError("invalid bounded lift target")
        elapsed = 0.
        transfer = fixture.get("transfer")
        across = np.array(transfer["across_rad"]) if transfer else lift_target
        down = np.array(transfer["down_rad"]) if transfer else lift_target
        if transfer:
            for target in (across, down):
                if (target.shape != (6,) or not np.isfinite(target).all()
                        or abs(target[0]-arm_start[0]) > math.radians(8)+1e-6
                        or np.max(np.abs(target[1:]-arm_start[1:])) > math.radians(2)+1e-6):
                    raise ValueError("transfer target outside J1 8 / other-joint 2 degree bounds")
        def tick(grip, arm, phase):
            nonlocal elapsed
            if not is_running():
                raise RuntimeError("viewer closed during physical grasp test")
            started = time.monotonic()
            report["motion_commands_issued"] = True
            command(grip, arm)
            step()
            elapsed += dt
            data = observe()
            pose = np.asarray(data["object_world_matrix_m"], float)
            _pose_error(pose, pose)
            if (not math.isfinite(data["object_speed_m_s"]) or not math.isfinite(data["support_clearance_m"])
                    or data["support_clearance_m"] < -.002 or data["object_speed_m_s"] > 1):
                raise RuntimeError("block feedback/penetration/speed outside test envelope")
            forces = data["contacts"]
            force_keys = ("left", "right", "support", "target_support") if transfer else ("left", "right", "support")
            for key in ("gripper_rad", "arm_target_error_rad", "arm_speed_rad_s"):
                if not math.isfinite(data[key]):
                    raise RuntimeError("missing/nonfinite loaded joint feedback")
            if any(not math.isfinite(forces[k]) or forces[k] < 0 or forces[k] > 50 for k in force_keys):
                raise RuntimeError("invalid test contact force")
            if transfer and (not math.isfinite(data["target_support_clearance_m"]) or data["target_support_clearance_m"] < -.002):
                raise RuntimeError("invalid target support clearance/penetration")
            if round(elapsed/dt) % max(1, round(.1/dt)) == 0:
                report["samples"].append({"phase": phase, "time_sec": elapsed, **data})
            if round(elapsed/dt) % max(1, round(.5/dt)) == 0:
                progress("MEASUREMENT", phase=phase, time_sec=elapsed, status="IN_PROGRESS",
                         command_gripper_rad=float(grip), command_arm_rad=np.asarray(arm).tolist(), feedback=data)
            if realtime:
                time.sleep(max(0, dt-(time.monotonic()-started)))
            return data
        def bilateral(data):
            return data["contacts"]["left"] >= .01 and data["contacts"]["right"] >= .01
        def supported(data):
            return data["contacts"]["support"] >= .01 and abs(data["support_clearance_m"]) <= .002
        def target_supported(data):
            expected = np.array(transfer["target_object_world_matrix_m"])
            actual = np.array(data["object_world_matrix_m"])
            error = _pose_error(actual, expected)
            return (data["contacts"]["target_support"] >= .01 and data["contacts"]["support"] < .01
                    and abs(data["target_support_clearance_m"]) <= .002
                    and np.linalg.norm(actual[:2, 3]-expected[:2, 3]) <= .003
                    and error["orientation_error_rad"] <= math.radians(5))
        grasp_relative, release_z = None, None
        specs = [("SETTLE_ON_SUPPORT", 3.), ("CLOSE_CONTACT", 8.), ("LIFT", 8.),
                 ("HOLD_UNSUPPORTED", 3.), ("OPEN_RELEASE", 8.), ("RETURN_EMPTY", 8.)]
        if transfer:
            specs = specs[:4]+[("TRANSFER", 10.), ("HOLD_AT_TARGET", 1.), ("LOWER_TO_TARGET", 8.),
                              ("OPEN_RELEASE", 8.), ("RETREAT_EMPTY", 8.),
                              ("RETURN_ABOVE_SOURCE", 10.), ("RETURN_EMPTY", 8.)]
        for phase, timeout in specs:
            progress("PHASE_STARTED", phase=phase, status="IN_PROGRESS", time_sec=elapsed)
            phase_color(phase)
            print(f"[PHYSICAL GRASP] {phase}", flush=True)
            row = {"phase": phase, "status": "FAILED"}
            report["phases"].append(row)
            stable, lost, bilateral_time = 0., 0., 0.
            for index in range(math.ceil(timeout/dt)):
                t = (index+1)*dt
                ramp = 6 if phase in ("TRANSFER", "RETURN_ABOVE_SOURCE") else 4
                a = min(1., t/ramp)
                blend = a**3*(10-15*a+6*a*a)
                grip = open_rad if phase in ("SETTLE_ON_SUPPORT", "RETURN_EMPTY") else close_rad
                arm = arm_start if phase in ("SETTLE_ON_SUPPORT", "CLOSE_CONTACT") else lift_target
                if phase == "CLOSE_CONTACT": grip = open_rad+(close_rad-open_rad)*blend
                if phase == "LIFT": arm = arm_start+(lift_target-arm_start)*blend
                if phase == "OPEN_RELEASE": grip = close_rad+(open_rad-close_rad)*blend
                if phase == "RETURN_EMPTY": arm = lift_target+(arm_start-lift_target)*blend
                if transfer:
                    if phase == "TRANSFER": arm = lift_target+(across-lift_target)*blend
                    if phase == "HOLD_AT_TARGET": arm = across
                    if phase == "LOWER_TO_TARGET": arm = across+(down-across)*blend
                    if phase == "OPEN_RELEASE": arm = down
                    if phase == "RETREAT_EMPTY":
                        grip, arm = open_rad, down+(across-down)*blend
                    if phase == "RETURN_ABOVE_SOURCE":
                        grip, arm = open_rad, across+(lift_target-across)*blend
                data = tick(grip, arm, phase)
                row["last_feedback"] = data
                both = bilateral(data)
                if phase in ("LIFT", "HOLD_UNSUPPORTED", "TRANSFER", "HOLD_AT_TARGET", "LOWER_TO_TARGET"):
                    lost = 0 if both else lost+dt
                    relative_error = _pose_error(np.array(data["object_relative_base_matrix_m"]), grasp_relative)
                    row["relative_object_error"] = relative_error
                    if lost > .1 or relative_error["position_error_m"] > .003 or relative_error["orientation_error_rad"] > math.radians(5):
                        raise RuntimeError("bilateral contact lost or object slipped during lift/hold")
                if transfer and phase in ("TRANSFER", "HOLD_AT_TARGET"):
                    if (data["support_clearance_m"] < .005 or data["target_support_clearance_m"] < .005
                            or data["contacts"]["support"] >= .01 or data["contacts"]["target_support"] >= .01):
                        raise RuntimeError("block contacted a support or lost clearance during horizontal transfer")
                if transfer and phase in ("RETREAT_EMPTY", "RETURN_ABOVE_SOURCE", "RETURN_EMPTY"):
                    if (not target_supported(data) or data["contacts"]["left"] >= .01
                            or data["contacts"]["right"] >= .01 or data["object_speed_m_s"] > .02):
                        raise RuntimeError("released block moved off target or retained finger contact during retreat")
                if phase == "SETTLE_ON_SUPPORT":
                    good = (supported(data) and data["contacts"]["left"] < .01
                            and data["contacts"]["right"] < .01 and data["object_speed_m_s"] <= .01)
                    if transfer: good = good and data["contacts"]["target_support"] < .01
                    needed = .5
                elif phase == "CLOSE_CONTACT":
                    good = a == 1 and both and abs(data["gripper_rad"]-open_rad) >= math.radians(1)
                    needed = .3
                elif phase in ("LIFT", "HOLD_UNSUPPORTED"):
                    good = (both and data["support_clearance_m"] >= .005 and data["contacts"]["support"] < .01
                            and data["arm_target_error_rad"] <= math.radians(.1)
                            and data["arm_speed_rad_s"] <= math.radians(.5))
                    if phase == "HOLD_UNSUPPORTED":
                        if not good:
                            raise RuntimeError("object is not independently supported by both fingers")
                        bilateral_time += dt
                    needed = .5 if phase == "LIFT" else 3.
                    good = good and (a == 1 or phase == "HOLD_UNSUPPORTED")
                elif phase in ("TRANSFER", "HOLD_AT_TARGET", "LOWER_TO_TARGET"):
                    expected = np.array(transfer["target_object_world_matrix_m"])
                    measured = np.array(data["object_world_matrix_m"])
                    horizontal_error = float(np.linalg.norm(measured[:2, 3]-expected[:2, 3]))
                    distance = float(np.linalg.norm(measured[:2, 3]-np.array(fixture["initial_object_world_matrix_m"])[:2, 3]))
                    row.update(horizontal_target_error_m=horizontal_error, measured_horizontal_distance_m=distance)
                    good = (both and horizontal_error <= .003 and distance >= .10
                            and data["arm_target_error_rad"] <= math.radians(.1)
                            and data["arm_speed_rad_s"] <= math.radians(.5))
                    if phase == "LOWER_TO_TARGET": good = good and target_supported(data)
                    if phase == "HOLD_AT_TARGET":
                        if not good: raise RuntimeError("target hover lost measured placement or bilateral contact")
                    else: good = good and a == 1
                    needed = 1. if phase == "HOLD_AT_TARGET" else .5
                elif phase == "OPEN_RELEASE":
                    fall = release_z-data["object_world_matrix_m"][2][3]
                    good = (a == 1 and (target_supported(data) if transfer else supported(data)) and not both and data["contacts"]["left"] < .01
                            and data["contacts"]["right"] < .01 and fall >= .005
                            and abs(data["gripper_rad"]-open_rad) <= math.radians(.5) and data["object_speed_m_s"] <= .02)
                    row["measured_setdown_descent_m" if transfer else "measured_release_fall_m"] = fall
                    needed = .5
                else:
                    good = (a == 1 and (target_supported(data) if transfer else supported(data)) and data["contacts"]["left"] < .01
                            and data["contacts"]["right"] < .01 and data["arm_target_error_rad"] <= math.radians(.1)
                            and data["arm_speed_rad_s"] <= math.radians(.5))
                    needed = .5
                stable = stable+dt if good else 0.
                if stable+1e-9 >= needed:
                    row.update(status="PASSED", elapsed_sec=t)
                    if phase == "CLOSE_CONTACT": grasp_relative = np.array(data["object_relative_base_matrix_m"])
                    if phase == "HOLD_UNSUPPORTED":
                        release_z = data["object_world_matrix_m"][2][3]
                        row["bilateral_hold_sec"] = bilateral_time
                    if phase == "HOLD_AT_TARGET": release_z = data["object_world_matrix_m"][2][3]
                    progress("PHASE_COMPLETED", phase=phase, status="PASSED", time_sec=elapsed,
                             last_feedback=data)
                    print(f"[PHYSICAL GRASP] {phase}: PASSED", flush=True)
                    break
            else:
                raise RuntimeError(f"physical grasp phase timed out: {phase}")
        report.update(status="PASSED_SIMULATED_CONTACT_GRASP_RELEASE", physical_grasp_verified=True,
                      release_verified=True, returned_to_start=True,
                      verification_scope="this synthetic object, object-only load and temporary mount only")
        if transfer:
            report.update(status="PASSED_SIMULATED_GRASP_TRANSFER_PLACE", transfer_verified=True,
                          target_placement_verified=True, transfer_plan=transfer)
    except (Exception, KeyboardInterrupt) as exc:
        report["errors"].append(f"{type(exc).__name__}: {exc}")
        print(f"[PHYSICAL GRASP] FAILED: {type(exc).__name__}: {exc}", flush=True)
        try:
            progress("GRASP_FAILED", status="FAILED", errors=list(report["errors"]), phases=report["phases"],
                     traceback=traceback.format_exc())
        except Exception as diagnostic_error:
            report["errors"].append(f"failure checkpoint failed: {diagnostic_error}")
        try:
            phase_color("FAILED")
        except Exception as color_error:
            report["errors"].append(f"failure marker update failed: {color_error}")
        # No automatic opening or transport after a failed grasp; pause physics immediately.
    try:
        stop()
        report["simulation_paused"] = True
    except Exception as exc:
        report.update(status="FAILED", physical_grasp_verified=False, release_verified=False)
        report.update(transfer_verified=False, target_placement_verified=False)
        report["errors"].append(f"pause failed: {exc}")
    report["passed_phases"] = sum(row["status"] == "PASSED" for row in report["phases"])
    return report
