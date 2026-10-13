"""Small simulated-robot drive tests with measured feedback and fail-closed holds."""
from __future__ import annotations

import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np

from fdw_sim.visualization.lula_model_validation import _pose_error, _unique_prim
from fdw_sim.visualization.robot_model_export import _usd_rigid_matrix


def run_joint_test(articulation, names, limits, step, measured_pose, predicted_pose, stop,
                   *, dt=1/60, is_running=lambda: True, realtime=False, action_factory=None,
                   amplitude_deg=1.0, selected_joints=None):
    """Drive targets only; callback interfaces keep the control loop testable."""
    report = {"status": "FAILED", "errors": [], "legs": [], "motion_commands_issued": False,
              "returned_to_start": False, "collision_verified": False,
              "physical_grasp_verified": False, "scope": "simulated joint drives and measured end-link feedback",
              "amplitude_deg": 1.0, "joint_tolerance_deg": 0.1,
              "position_tolerance_m": 0.002, "orientation_tolerance_rad": 0.01}
    baseline = None
    try:
        if not math.isfinite(amplitude_deg) or not 0 < amplitude_deg <= 5:
            raise ValueError("joint demonstration amplitude must be in (0, 5] degrees")
        selected = list(names) if selected_joints is None else list(selected_joints)
        if not selected or len(set(selected)) != len(selected) or any(n not in names for n in selected):
            raise ValueError("selected joints must be a nonempty unique subset of live joints")
        report.update(amplitude_deg=amplitude_deg, selected_joints=selected)
        if not math.isfinite(dt) or dt <= 0 or dt > 0.1:
            raise ValueError("invalid physics timestep")
        if list(articulation.dof_names) != list(names) or len(set(names)) != len(names):
            raise ValueError("live articulation joint order must match the validated model")
        if not names or any(n not in limits or not all(math.isfinite(v) for v in limits[n])
                            or limits[n][0] >= limits[n][1] for n in names):
            raise ValueError("finite ordered joint limits are required")
        setter = getattr(articulation, "set_joint_position_targets", None)
        if not callable(setter):
            apply_action = getattr(articulation, "apply_action", None)
            if not callable(apply_action):
                raise RuntimeError("articulation has no joint drive API; teleport is not allowed")
            if action_factory is None:
                from isaacsim.core.utils.types import ArticulationAction
                action_factory = ArticulationAction
            indices = np.arange(len(names), dtype=int)
            setter = lambda q: apply_action(action_factory(joint_positions=q, joint_indices=indices))
            report["target_api"] = "apply_action (explicit joint indices)"
        else:
            report["target_api"] = "set_joint_position_targets"

        def read():
            values = articulation.get_joint_positions()
            q = np.asarray(values, dtype=float) if values is not None else np.array([])
            if q.shape != (len(names),) or not np.isfinite(q).all():
                raise RuntimeError("joint feedback missing or invalid")
            if any(q[i] < limits[n][0]-1e-5 or q[i] > limits[n][1]+1e-5 for i, n in enumerate(names)):
                raise RuntimeError("measured joints exceed model limits")
            return q.copy()

        baseline = read()
        targets = []
        for index, name in enumerate(names):
            if name not in selected:
                continue
            for sign in (1, -1):
                target = baseline.copy()
                target[index] += sign * math.radians(amplitude_deg)
                if any(not limits[n][0] <= target[i] <= limits[n][1] for i, n in enumerate(names)):
                    raise ValueError(f"test target exceeds joint limits: {name}")
                label = f"{name}_{sign*amplitude_deg:+g}deg"
                targets.extend([(label, target), (f"{name}_{sign*amplitude_deg:+g}_return", baseline.copy())])
        initial_pose = measured_pose()
        initial_error = _pose_error(initial_pose, predicted_pose(baseline))
        if initial_error["position_error_m"] > 0.002 or initial_error["orientation_error_rad"] > 0.01:
            raise RuntimeError("initial measured end pose differs from validated Lula model")
        report["initial_joints_rad"] = baseline.tolist()
        report["initial_measured_pose_m"] = initial_pose.tolist()
        report["initial_pose_error"] = initial_error
        report["planned_legs"] = len(targets)
        tolerance = math.radians(0.1)
        stable_frames = max(2, math.ceil(0.1/dt))
        duration, settle_timeout = max(2.0, amplitude_deg / 0.5), 3.0
        report["ramp_duration_sec"] = duration
        envelope = np.array([amplitude_deg + 0.5 if n in selected else 1.5 for n in names])
        previous_target = baseline.copy()
        for label, target in targets:
            read()  # Validate feedback without introducing a target jump between legs.
            start = previous_target.copy()
            stable = 0
            row = {"leg": label, "target_rad": target.tolist(), "status": "FAILED",
                   "max_tracking_error_rad": 0.0, "max_position_error_m": 0.0,
                   "max_orientation_error_rad": 0.0}
            report["legs"].append(row)
            print(f"[JOINT TEST] moving: {label} ({duration:g}s ramp)", flush=True)
            for tick in range(math.ceil((duration+settle_timeout)/dt)):
                if not is_running():
                    raise RuntimeError("application closed or test interrupted")
                started = time.monotonic()
                alpha = min(1.0, (tick+1)*dt/duration)
                command = start + alpha*(target-start)
                report["motion_commands_issued"] = True
                setter(command.copy())
                step()
                actual = read()
                if np.any(np.abs(actual-baseline) > np.deg2rad(envelope)):
                    raise RuntimeError("joint excursion exceeded small-test envelope")
                tracking = float(np.max(np.abs(command-actual)))
                actual_pose = measured_pose()
                error = _pose_error(actual_pose, predicted_pose(actual))
                row["max_tracking_error_rad"] = max(row["max_tracking_error_rad"], tracking)
                row["max_position_error_m"] = max(row["max_position_error_m"], error["position_error_m"])
                row["max_orientation_error_rad"] = max(row["max_orientation_error_rad"], error["orientation_error_rad"])
                if tracking > math.radians(2):
                    raise RuntimeError("excessive joint tracking error")
                if error["position_error_m"] > 0.002 or error["orientation_error_rad"] > 0.01:
                    raise RuntimeError("measured end pose differs from Lula during movement")
                stable = stable+1 if alpha == 1 and np.max(np.abs(actual-target)) <= tolerance else 0
                if stable >= stable_frames:
                    row.update(status="PASSED", final_joints_rad=actual.tolist(),
                               final_measured_pose_m=actual_pose.tolist(), final_pose_error=error,
                               elapsed_sim_sec=(tick+1)*dt)
                    print(f"[JOINT TEST] {label}: PASSED", flush=True)
                    break
                if realtime:
                    time.sleep(max(0, dt-(time.monotonic()-started)))
            else:
                raise RuntimeError(f"joint target timed out: {label}")
            previous_target = target.copy()
        final = read()
        report["final_joints_rad"] = final.tolist()
        report["returned_to_start"] = bool(np.max(np.abs(final-baseline)) <= tolerance)
        if not report["returned_to_start"]:
            raise RuntimeError("final measured joints did not return to start")
        report["status"] = "PASSED_SIMULATED_JOINT_FEEDBACK"
    except (Exception, KeyboardInterrupt) as exc:
        report["errors"].append(f"{type(exc).__name__}: {exc}")
        if report["motion_commands_issued"]:
            try:
                hold = np.asarray(articulation.get_joint_positions(), dtype=float)
                if hold.shape != (len(names),) or not np.isfinite(hold).all():
                    raise ValueError("finite hold feedback unavailable")
                setter(hold)
                report["abort_hold_joints_rad"] = hold.tolist()
            except Exception as hold_error:
                report["errors"].append(f"hold failed: {hold_error}")
        try:
            stop()
            report["simulation_paused"] = True
        except Exception as stop_error:
            report["errors"].append(f"pause failed: {stop_error}")
    report["passed_legs"] = sum(row["status"] == "PASSED" for row in report["legs"])
    return report


def test_live_robot_joints(sim, stage, robot_path, articulation, validation, *, output_dir, show_motion=False):
    """Use the model just validated in this same scene; no env/config overrides."""
    from pxr import UsdGeom
    path = Path(output_dir) / "joint_feedback_test.json"
    if path.exists():
        raise FileExistsError(path)
    try:
        if validation["status"] != "PASSED_COMPUTED_MULTI_POSE":
            raise ValueError("computed multi-pose validation must pass before motor commands")
        from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver
        solver = LulaKinematicsSolver(robot_description_path=validation["robot_description_path"],
                                      urdf_path=validation["urdf_path"])
        names = validation["joint_names"]
        solver_names = list(solver.get_joint_names())
        if set(solver_names) != set(names) or len(solver_names) != len(names):
            raise ValueError("Lula joint mapping changed")
        meters = UsdGeom.GetStageMetersPerUnit(stage)
        base = _unique_prim(stage, robot_path, validation["root_link"])
        transform = UsdGeom.XformCache().GetLocalToWorldTransform(base)
        quat = transform.ExtractRotationQuat()
        solver.set_robot_base_pose(np.array(transform.ExtractTranslation()),
                                  np.array([quat.GetReal(), *quat.GetImaginary()]))
        frame = _unique_prim(stage, robot_path, validation["end_link"])
        def measured():
            return _usd_rigid_matrix(UsdGeom.XformCache(), frame, meters)
        def predicted(q):
            values = dict(zip(names, q))
            pos, rot = solver.compute_forward_kinematics(validation["end_link"],
                                                        np.array([values[n] for n in solver_names]))
            matrix = np.eye(4)
            matrix[:3, 3] = np.asarray(pos).reshape(3)*meters
            matrix[:3, :3] = np.asarray(rot).reshape(3, 3)
            return matrix
        robot = ET.parse(validation["urdf_path"]).getroot()
        limits = {j.get("name"): (float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
                  for j in robot.findall("joint") if j.get("type") != "fixed"}
        if show_motion:
            import os
            if not sim.config.skip_auto_camera:
                end_position = UsdGeom.XformCache().GetLocalToWorldTransform(frame).ExtractTranslation()
                center = (np.array(transform.ExtractTranslation()) + np.array(end_position)) / 2
                sim.visualizer.scene.frame_viewport_to_scene(
                    center=tuple(center), distance=2.5/meters, height=1.2/meters,
                    set_viewport_camera=os.environ.get("FDW_DISABLE_VIEWPORT_SWITCH") != "1")
            print("[ROBOT MOTION] J1: +5 degrees, return, -5 degrees, return. Other joints hold.", flush=True)
            for seconds in range(5, 0, -1):
                print(f"[ROBOT MOTION] starts in {seconds}...", flush=True)
                deadline = time.monotonic() + 1
                while time.monotonic() < deadline:
                    if not sim._isaac_app.is_running():
                        raise RuntimeError("application closed before demonstration")
                    sim._isaac_app.update()
                    time.sleep(0.02)
        sim._isaac_world.play()
        result = run_joint_test(articulation, names, limits,
            lambda: sim._isaac_world.step(render=not sim.config.headless), measured, predicted,
            sim._isaac_world.pause, dt=sim._dt, is_running=sim._isaac_app.is_running,
            realtime=sim.config.realtime, amplitude_deg=5.0 if show_motion else 1.0,
            selected_joints=["J1"] if show_motion else None)
        result["demonstration_mode"] = show_motion
    except (Exception, KeyboardInterrupt) as exc:
        result = {"status": "FAILED", "errors": [f"{type(exc).__name__}: {exc}"],
                  "motion_commands_issued": False, "physical_grasp_verified": False}
        try:
            sim._isaac_world.pause()
            result["simulation_paused"] = True
        except Exception as pause_error:
            result["errors"].append(f"pause failed: {pause_error}")
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    return result, path
