"""Bounded mounted-arm motion and USD-kinematic TCP-candidate consistency checks."""
import math
import time

import numpy as np

from fdw_sim.visualization.gripper_mount_test import StationaryArmGuard
from fdw_sim.visualization.robot_model_export import _usd_rigid_matrix, _gf_rigid_matrix
from fdw_sim.visualization.lula_model_validation import _pose_error


def _rotation(axis, angle):
    vector = np.eye(3)["XYZ".index(axis)]
    x, y, z = vector
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    result = np.eye(4)
    result[:3, :3] = np.eye(3) + math.sin(angle)*cross + (1-math.cos(angle))*(cross@cross)
    return result


def _joint_frame(joint, side, meters):
    from pxr import Gf
    pos = getattr(joint, f"GetLocalPos{side}Attr")().Get()
    quat = getattr(joint, f"GetLocalRot{side}Attr")().Get()
    if pos is None or quat is None:
        raise ValueError("joint frame unavailable")
    transform = Gf.Matrix4d(1)
    transform.SetRotate(Gf.Quatd(quat))
    transform.SetTranslateOnly(Gf.Vec3d(pos))
    return _gf_rigid_matrix(transform, meters)


class UsdMountedArmModel:
    """Snapshot the six-joint USD chain; predictions do not read moving link transforms."""
    def __init__(self, stage, mount):
        from pxr import Usd, UsdGeom, UsdPhysics
        self.names = [f"J{i}" for i in range(1, 7)]
        meters = UsdGeom.GetStageMetersPerUnit(stage)
        cache = UsdGeom.XformCache()
        arm = stage.GetPrimAtPath(mount["robot_path"])
        if not arm:
            raise ValueError("mounted arm missing")
        candidates = [p for p in Usd.PrimRange(arm, Usd.TraverseInstanceProxies())
                      if p.IsA(UsdPhysics.RevoluteJoint)]
        if sorted(p.GetName() for p in candidates) != self.names:
            raise ValueError("USD FK requires exact J1-J6 inventory")
        self.base_world = _usd_rigid_matrix(cache, stage.GetPrimAtPath(mount["arm_base_path"]), meters)
        self.joints, self.limits = [], []
        parent = mount["arm_base_path"]
        for name in self.names:
            joint = UsdPhysics.RevoluteJoint(next(p for p in candidates if p.GetName() == name))
            b0, b1 = list(map(str, joint.GetBody0Rel().GetTargets())), list(map(str, joint.GetBody1Rel().GetTargets()))
            axis = joint.GetAxisAttr().Get()
            if (joint.GetJointEnabledAttr().Get() is False or b0 != [parent] or len(b1) != 1
                    or axis not in ("X", "Y", "Z")):
                raise ValueError("USD FK requires enabled, ordered J1-J6 serial chain")
            bounds = [float(joint.GetLowerLimitAttr().Get()), float(joint.GetUpperLimitAttr().Get())]
            if not all(map(math.isfinite, bounds)) or bounds[0] >= bounds[1]:
                raise ValueError(f"finite authored joint limits required: {name}")
            self.limits.append(np.deg2rad(bounds))
            self.joints.append((axis, _joint_frame(joint, 0, meters), np.linalg.inv(_joint_frame(joint, 1, meters))))
            parent = b1[0]
        if parent != mount["terminal_body_path"]:
            raise ValueError("USD FK chain does not end at mounted terminal body")
        terminal = _usd_rigid_matrix(cache, stage.GetPrimAtPath(parent), meters)
        flange = _usd_rigid_matrix(cache, stage.GetPrimAtPath(mount["flange_path"]), meters)
        base = _usd_rigid_matrix(cache, stage.GetPrimAtPath(mount["gripper_base_path"]), meters)
        tcp = _usd_rigid_matrix(cache, stage.GetPrimAtPath(mount["tcp_candidate_path"]), meters)
        self.terminal_to_flange = np.linalg.inv(terminal) @ flange
        self.base_to_tcp = np.linalg.inv(base) @ tcp
        self.flange_to_tcp = np.array(mount["desired_flange_to_base_matrix_m"]) @ self.base_to_tcp
        self.limits = np.array(self.limits)

    def predict(self, values):
        q = np.asarray(values, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError("USD FK requires six finite measured arm positions")
        pose = self.base_world.copy()
        for angle, (axis, frame0, inverse1) in zip(q, self.joints):
            pose = pose @ frame0 @ _rotation(axis, angle) @ inverse1
        flange = pose @ self.terminal_to_flange
        return flange, flange @ self.flange_to_tcp

    def diagnostics(self):
        return {"prediction_source": "snapshot_USD_joint_local_frames_and_measured_radian_positions",
                "joint_names": self.names, "limits_rad": self.limits.tolist(),
                "base_world_matrix_m": self.base_world.tolist(),
                "flange_to_tcp_candidate_matrix_m": self.flange_to_tcp.tolist(),
                "tcp_calibration_verified": False, "independent_lula_validation": False}


class MovingArmController(StationaryArmGuard):
    def __init__(self, view, amplitude_deg):
        super().__init__(view)
        if not math.isfinite(amplitude_deg) or not 0 < amplitude_deg <= 2:
            raise ValueError("mounted arm amplitude must be in (0, 2] degrees")
        self.target = self.start.copy()
        self.envelope = np.deg2rad([amplitude_deg+.5, .5, .5, .5, amplitude_deg+.5, amplitude_deg+.5])
        self.max_tracking_error_deg = 0.0

    def hold(self):
        super().hold(self.target)

    def set_target(self, target):
        q = np.asarray(target, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all() or np.any(np.abs(q-self.start) > self.envelope):
            raise ValueError("mounted arm command outside bounded motion envelope")
        self.target = q.copy()

    def read(self):
        return self._read("get_dof_positions")[0, self.indices].astype(float)

    def velocities(self):
        return self._read("get_dof_velocities")[0, self.indices].astype(float)

    def check(self):
        actual = self.read()
        self.observed_drift_deg = max(self.observed_drift_deg, float(np.rad2deg(np.max(np.abs(actual-self.start)))))
        error = float(np.rad2deg(np.max(np.abs(actual-self.target))))
        self.max_tracking_error_deg = max(self.max_tracking_error_deg, error)
        if np.any(np.abs(actual-self.start) > self.envelope) or error > 2:
            raise RuntimeError("mounted arm excursion/tracking exceeded small-motion envelope")


def run_mounted_arm_motion(controller, model, command_gripper_hold, read_gripper, read_pads,
                           observe, step, stop, *, dt, amplitude_deg=1, realtime=False, is_running=lambda: True):
    """Run J1/J5/J6 +/- and return; verify real travel, mount and predicted TCP each tick."""
    report = {"status": "FAILED", "errors": [], "legs": [], "samples": [], "planned_legs": 12,
              "motion_commands_issued": False, "returned_to_start": False,
              "tcp_calibration_verified": False, "physical_grasp_verified": False, "collision_verified": False,
              "scope": "unloaded zero-gravity mounted arm; USD FK/TCP candidate consistency; not calibrated TCP"}
    try:
        if not math.isfinite(dt) or not 0 < dt <= 1/60 or not math.isfinite(amplitude_deg) or not 0 < amplitude_deg <= 2:
            raise ValueError("invalid mounted arm timestep/amplitude")
        baseline = controller.read()
        start_gripper = float(read_gripper())
        start_pads = np.array(read_pads(), dtype=float)
        if not math.isfinite(start_gripper) or start_pads.shape != (2, 3) or not np.isfinite(start_pads).all():
            raise ValueError("invalid mounted gripper hold baseline")
        limits = np.asarray(model.limits, dtype=float)
        if limits.shape != (6, 2) or not np.isfinite(limits).all() or np.any(limits[:, 0] >= limits[:, 1]):
            raise ValueError("finite ordered arm joint limits required")
        targets = []
        for index in (0, 4, 5):
            for sign in (1, -1):
                target = baseline.copy()
                target[index] += math.radians(sign*amplitude_deg)
                if np.any(target < limits[:, 0]) or np.any(target > limits[:, 1]):
                    raise ValueError("mounted arm test exceeds authored limits")
                targets.extend([(f"J{index+1}_{sign*amplitude_deg:+g}deg", target, index, False),
                                (f"J{index+1}_{sign*amplitude_deg:+g}_return", baseline.copy(), index, True)])
        report.update(initial_arm_rad=baseline.tolist(), gripper_hold_rad=start_gripper, amplitude_deg=amplitude_deg,
                      prediction=model.diagnostics(), max_mount_position_error_m=0., max_mount_orientation_error_rad=0.,
                      max_tcp_position_error_m=0., max_tcp_orientation_error_rad=0.)

        def check():
            controller.check()
            q, speeds = controller.read(), controller.velocities()
            if (not np.isfinite(q).all() or not np.isfinite(speeds).all()
                    or np.any(q < limits[:, 0]-1e-4) or np.any(q > limits[:, 1]+1e-4)):
                raise RuntimeError("mounted arm feedback invalid or exceeds limits")
            grip = float(read_gripper())
            pads = np.asarray(read_pads(), dtype=float)
            if (not math.isfinite(grip) or abs(grip-start_gripper) > math.radians(.5)
                    or pads.shape != (2, 3) or not np.isfinite(pads).all()
                    or np.max(np.linalg.norm(pads-start_pads, axis=1)) > .002):
                raise RuntimeError("gripper posture changed during mounted arm motion")
            observation = observe()
            _, tcp = model.predict(q)
            observed_tcp = np.asarray(observation["tcp_candidate_world_matrix_m"], dtype=float)
            tcp_error = _pose_error(observed_tcp, tcp)
            mount_error = observation["flange_to_base_error"]
            for key in ("position_error_m", "orientation_error_rad"):
                if not math.isfinite(mount_error[key]) or not math.isfinite(tcp_error[key]):
                    raise ValueError("nonfinite mounted pose error")
            report["max_mount_position_error_m"] = max(report["max_mount_position_error_m"], mount_error["position_error_m"])
            report["max_mount_orientation_error_rad"] = max(report["max_mount_orientation_error_rad"], mount_error["orientation_error_rad"])
            report["max_tcp_position_error_m"] = max(report["max_tcp_position_error_m"], tcp_error["position_error_m"])
            report["max_tcp_orientation_error_rad"] = max(report["max_tcp_orientation_error_rad"], tcp_error["orientation_error_rad"])
            if (mount_error["position_error_m"] > .002 or mount_error["orientation_error_rad"] > .01
                    or tcp_error["position_error_m"] > .002 or tcp_error["orientation_error_rad"] > .01):
                raise RuntimeError("mounted frame/TCP candidate differs from USD kinematics")
            return q, speeds, {"measured_tcp_world_matrix_m": observed_tcp.tolist(),
                               "predicted_tcp_world_matrix_m": tcp.tolist(), "tcp_error": tcp_error,
                               "mount_error": mount_error, "gripper_rad": grip}

        _, _, initial = check()  # No commands until initial model/feedback agreement is established.
        report["initial_tcp"] = initial
        previous = baseline.copy()
        for label, target, index, is_return in targets:
            row = {"leg": label, "status": "FAILED", "target_rad": target.tolist()}
            report["legs"].append(row)
            print(f"[MOUNTED ARM] {label}: 4-second smooth ramp", flush=True)
            stable = 0
            for tick in range(math.ceil(8/dt)):
                if not is_running():
                    raise RuntimeError("viewer closed during mounted arm test")
                started = time.monotonic()
                alpha = min(1., (tick+1)*dt/4)
                blend = alpha**3*(10-15*alpha+6*alpha**2)
                controller.set_target(previous+(target-previous)*blend)
                report["motion_commands_issued"] = True
                command_gripper_hold(start_gripper)  # Also writes current six arm drive targets.
                step()
                actual, speed, detail = check()
                row["last_feedback"] = {"arm_rad": actual.tolist(), "elapsed_sec": (tick+1)*dt, **detail}
                if tick % max(1, round(.1/dt)) == 0:
                    report["samples"].append({"leg": label, "time_sec": (tick+1)*dt,
                                              "command_arm_rad": controller.target.tolist(),
                                              "measured_arm_rad": actual.tolist(), **detail})
                at_target = (alpha == 1 and np.max(np.abs(actual-target)) <= math.radians(.1)
                             and np.max(np.abs(speed)) <= math.radians(.5))
                stable = stable+1 if at_target else 0
                if stable >= math.ceil(.5/dt):
                    if not is_return and abs(actual[index]-baseline[index]) < math.radians(amplitude_deg*.8):
                        raise RuntimeError("insufficient measured arm joint travel")
                    row.update(status="PASSED", elapsed_sec=(tick+1)*dt)
                    print(f"[MOUNTED ARM] {label}: PASSED", flush=True)
                    break
                if realtime:
                    time.sleep(max(0, dt-(time.monotonic()-started)))
            else:
                raise RuntimeError(f"mounted arm target/stable dwell timed out: {label}")
            previous = target.copy()
        actual, _, final = check()
        report["final_tcp"] = final
        report["tcp_return_error"] = _pose_error(np.array(initial["measured_tcp_world_matrix_m"]),
                                                   np.array(final["measured_tcp_world_matrix_m"]))
        report["returned_to_start"] = bool(np.max(np.abs(actual-baseline)) <= math.radians(.1))
        if (not report["returned_to_start"] or report["tcp_return_error"]["position_error_m"] > .002
                or report["tcp_return_error"]["orientation_error_rad"] > .01):
            report["returned_to_start"] = False
            raise RuntimeError("mounted arm did not return to initial measured joints")
        report["status"] = "PASSED_MOUNTED_ARM_TCP_TEST"
    except (Exception, KeyboardInterrupt) as exc:
        report["errors"].append(f"{type(exc).__name__}: {exc}")
        if report["motion_commands_issued"]:
            try:
                controller.set_target(controller.read())
                command_gripper_hold(float(read_gripper()))
                report["abort_hold_arm_rad"] = controller.target.tolist()
            except Exception as error:
                report["errors"].append(f"hold failed: {error}")
    try:
        stop()
        report["simulation_paused"] = True
    except Exception as error:
        report["status"] = "FAILED"
        report["errors"].append(f"pause failed: {error}")
    report["passed_legs"] = sum(row["status"] == "PASSED" for row in report["legs"])
    report["max_arm_tracking_error_deg"] = controller.max_tracking_error_deg
    return report
