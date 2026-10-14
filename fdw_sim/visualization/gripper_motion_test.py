"""Unloaded single-input gripper cycle with independent left/right geometry checks."""
import math
import time

import numpy as np


def inspect_motion_prerequisites(stage):
    """Require authored primary drive, live mimic relationship and a world-fixed base."""
    from pxr import Usd, UsdPhysics
    prims = list(Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()))
    def unique(name):
        matches = [p for p in prims if p.GetName() == name]
        if len(matches) != 1:
            raise ValueError(f"missing or ambiguous gripper component: {name}")
        return matches[0]
    primary, follower, base = unique("finger_joint"), unique("right_outer_knuckle_joint"), unique("robotiq_base_link")
    if not primary.IsA(UsdPhysics.RevoluteJoint) or not follower.IsA(UsdPhysics.RevoluteJoint):
        raise ValueError("expected revolute primary and follower joints")
    references = [r for r in follower.GetRelationships() if r.GetName().startswith("physxMimicJoint:")
                  and r.GetName().endswith(":referenceJoint")]
    matching = [r for r in references if list(r.GetTargets()) == [primary.GetPath()]]
    if len(matching) != 1:
        raise ValueError("right follower must reference finger_joint through exactly one mimic relationship")
    prefix = matching[0].GetName().rsplit(":", 1)[0]
    gearing = follower.GetAttribute(prefix + ":gearing").Get()
    offset_attr = follower.GetAttribute(prefix + ":offset")
    offset = offset_attr.Get() if offset_attr else 0
    if gearing is None or not math.isclose(float(gearing), -1, abs_tol=1e-6) or not math.isclose(float(offset or 0), 0, abs_tol=1e-6):
        raise ValueError("unexpected mimic gearing/offset; no auto-correction")
    for key in ("stiffness", "damping"):
        attr = follower.GetAttribute("drive:angular:physics:" + key)
        if attr and attr.Get() is not None and float(attr.Get()) != 0:
            raise ValueError("active follower drive conflicts with mimic")
    stiffness = primary.GetAttribute("drive:angular:physics:stiffness").Get()
    if stiffness is None or not math.isfinite(float(stiffness)) or stiffness <= 0:
        raise ValueError("primary joint has no positive finite position-drive stiffness")
    bounds = [float(primary.GetAttribute("physics:" + k).Get()) for k in ("lowerLimit", "upperLimit")]
    if not all(map(math.isfinite, bounds)) or bounds[0] >= bounds[1]:
        raise ValueError("primary joint requires finite ordered limits")
    anchors = []
    for p in prims:
        if p.IsA(UsdPhysics.FixedJoint):
            joint = UsdPhysics.Joint(p)
            if (not joint.GetBody0Rel().GetTargets() and list(joint.GetBody1Rel().GetTargets()) == [base.GetPath()]
                    and joint.GetJointEnabledAttr().Get()):
                anchors.append(str(p.GetPath()))
    if len(anchors) != 1:
        raise ValueError("expected one enabled authored world-to-base fixed joint")
    pads = []
    for side in ("left", "right"):
        matches = [p for p in prims if p.GetName() == "Fingertip_01" and f"/{side}_inner_finger/" in str(p.GetPath())]
        if len(matches) != 1:
            raise ValueError(f"missing or ambiguous {side} fingertip geometry")
        pads.append(str(matches[0].GetPath()))
    return {"primary_joint_path": str(primary.GetPath()), "follower_joint_path": str(follower.GetPath()),
            "mimic_relationship": matching[0].GetName(), "mimic_reference": str(primary.GetPath()),
            "mimic_gearing": float(gearing), "limit_rad": list(np.deg2rad(bounds)),
            "base_path": str(base.GetPath()), "anchor_path": anchors[0], "pad_paths": pads}


def run_gripper_cycle(read_joint, command, read_pads, step, stop, limits, *, dt=1/60,
                       realtime=False, is_running=lambda: True):
    """Commands only one measured joint; pad positions are measured in the base frame."""
    report = {"status": "FAILED", "errors": [], "legs": [], "samples": [],
              "motion_commands_issued": False, "physical_grasp_verified": False,
              "tcp_calibration_verified": False, "mounted_on_fanuc": False,
              "scope": "unloaded zero-gravity partial closing and return; not full-stroke/grasp validation",
              "amplitude_deg": 10, "cycles": 2, "pad_span_is_center_distance_not_clearance": True}
    try:
        if not math.isfinite(dt) or not 0 < dt <= 0.1:
            raise ValueError("invalid physics timestep")
        if len(limits) != 2 or not all(map(math.isfinite, limits)) or limits[0] >= limits[1]:
            raise ValueError("invalid primary joint limits")
        def read():
            q = float(read_joint())
            pads = np.asarray(read_pads(), dtype=float)
            if not math.isfinite(q) or pads.shape != (2, 3) or not np.isfinite(pads).all():
                raise ValueError("missing or nonfinite measured joint/pad feedback")
            if not limits[0]-1e-4 <= q <= limits[1]+1e-4:
                raise ValueError("measured primary joint exceeds limits")
            return q, pads
        baseline, initial = read()
        # Permit <=1e-4 rad constraint roundoff in feedback, but all targets stay inside limits.
        open_target = float(np.clip(baseline, limits[0], limits[1]))
        closed = open_target + math.radians(10)
        if not limits[0] <= closed <= limits[1] or abs(baseline) > math.radians(5):
            raise ValueError("unexpected initial posture or partial-close target exceeds limits")
        axis = initial[1]-initial[0]
        span = float(np.linalg.norm(axis))
        if span < 0.005:
            raise ValueError("pad-center span too small to observe safely")
        axis /= span
        report.update(initial_joint_rad=baseline, initial_pad_positions_m=initial.tolist(), initial_pad_span_m=span,
                      limits_rad=list(limits), open_target_rad=open_target, planned_legs=4)
        previous = open_target
        for cycle in (1, 2):
            for label, target in (("partial_close", closed), ("return_open", open_target)):
                row = {"cycle": cycle, "leg": label, "target_rad": target, "status": "FAILED", "max_tracking_error_rad": 0.0}
                report["legs"].append(row)
                print(f"[GRIPPER MOTION] cycle {cycle}: {label}", flush=True)
                stable, dwell = 0, 0
                for tick in range(math.ceil(14/dt)):
                    if not is_running():
                        raise RuntimeError("viewer closed or interrupted")
                    started = time.monotonic()
                    alpha = min(1.0, (tick+1)*dt/4)
                    requested = previous + alpha*(target-previous)
                    report["motion_commands_issued"] = True
                    command(requested)
                    step()
                    q, pads = read()
                    tracking = abs(requested-q)
                    row["max_tracking_error_rad"] = max(row["max_tracking_error_rad"], tracking)
                    if abs(q-baseline) > math.radians(15) or tracking > math.radians(8):
                        raise RuntimeError("excessive primary joint excursion/tracking error")
                    if tick % max(1, round(0.1/dt)) == 0:
                        report["samples"].append({"cycle": cycle, "leg": label, "time_sec": (tick+1)*dt,
                                                  "command_rad": requested, "measured_rad": q,
                                                  "pad_positions_m": pads.tolist(),
                                                  "pad_span_m": float(np.linalg.norm(pads[1]-pads[0]))})
                    at_target = alpha == 1 and abs(q-target) <= math.radians(0.5)
                    stable = stable+1 if at_target else 0
                    dwell = dwell+1 if stable >= max(2, math.ceil(0.2/dt)) else 0
                    if dwell >= math.ceil(1.5/dt):
                        movement = pads-initial
                        projections = movement @ axis
                        current_span = float(np.linalg.norm(pads[1]-pads[0]))
                        if label == "partial_close":
                            if projections[0] < 0.001 or projections[1] > -0.001 or span-current_span < 0.002:
                                raise RuntimeError("both fingers must independently move inward and reduce pad-center span")
                        elif float(np.max(np.linalg.norm(movement, axis=1))) > 0.002:
                            raise RuntimeError("fingers did not return to their initial measured positions")
                        row.update(status="PASSED", measured_joint_rad=q, measured_pad_positions_m=pads.tolist(),
                                   inward_displacements_m=[float(projections[0]), float(-projections[1])],
                                   pad_span_m=current_span, elapsed_sim_sec=(tick+1)*dt)
                        print(f"[GRIPPER MOTION] {label}: PASSED; pad-center span={current_span*1000:.2f} mm", flush=True)
                        break
                    if realtime:
                        time.sleep(max(0, dt-(time.monotonic()-started)))
                else:
                    raise RuntimeError(f"target/stable dwell timed out: {label}")
                previous = target
        report["status"] = "PASSED_UNLOADED_GRIPPER_CYCLE"
        report["returned_to_start"] = True
    except (Exception, KeyboardInterrupt) as exc:
        report["errors"].append(f"{type(exc).__name__}: {exc}")
        if report["motion_commands_issued"]:
            try:
                hold = float(read_joint())
                if not math.isfinite(hold):
                    raise ValueError("finite hold feedback unavailable")
                command(hold)
                report["abort_hold_joint_rad"] = hold
            except Exception as error:
                report["errors"].append(f"hold failed: {error}")
    try:
        stop()
        report["simulation_paused"] = True
    except Exception as error:
        report["errors"].append(f"pause failed: {error}")
        report["status"] = "FAILED"
    report["passed_legs"] = sum(r["status"] == "PASSED" for r in report["legs"])
    return report
