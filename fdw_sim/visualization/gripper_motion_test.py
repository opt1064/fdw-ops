"""Unloaded single-input gripper cycle with independent left/right geometry checks."""
import math
import time

import numpy as np


def articulation_root_paths(stage):
    """Fresh traversal: prim/proxy handles cannot survive physics scene initialization."""
    from pxr import Usd, UsdPhysics
    return [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies())
            if p.HasAPI(UsdPhysics.ArticulationRootAPI)]


def compose_gripper_experiment(stage, source_identifier, *, meters, up_axis, default_prim_path):
    """Keep source intact; experiment settings belong to an anonymous root, not session."""
    from pxr import UsdGeom
    root = stage.GetRootLayer()
    if not root.anonymous or root.identifier == source_identifier:
        raise ValueError("experiment must have a separate anonymous root layer")
    if not math.isfinite(meters) or meters <= 0 or up_axis not in ("Y", "Z"):
        raise ValueError("unsupported gripper stage units/up axis")
    root.subLayerPaths = [source_identifier]
    stage.SetEditTarget(root)
    UsdGeom.SetStageMetersPerUnit(stage, meters)
    UsdGeom.SetStageUpAxis(stage, up_axis)
    if default_prim_path:
        default = stage.GetPrimAtPath(default_prim_path)
        if not default:
            raise ValueError("source default prim missing in experiment composition")
        stage.SetDefaultPrim(default)
    return {"experiment_root_is_anonymous": True, "source_layer_identifier": source_identifier,
            "experiment_edits_layer": "anonymous_root", "source_is_read_only_sublayer": True}


def require_physics_scene(stage, path, phase):
    from pxr import UsdPhysics
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsActive() or not prim.IsA(UsdPhysics.Scene):
        raise ValueError(f"physics scene missing {phase}: {path}")


def ensure_physx_attachment(interface, stage_id):
    """Verify the real PhysX stage, attaching this isolated stage if currently detached."""
    stage_id = int(stage_id)
    if stage_id <= 0:
        raise ValueError("invalid current USD stage id")
    before = int(interface.get_attached_stage())
    attached_explicitly = before != stage_id
    if attached_explicitly:
        if before > 0 and before != 2**64-1:
            raise ValueError(f"PhysX attached to another stage ({before}); current stage is {stage_id}")
        if not interface.attach_stage(stage_id):
            raise RuntimeError(f"PhysX failed to attach current USD stage {stage_id}")
    interface.flush_changes()
    after = int(interface.get_attached_stage())
    if after != stage_id:
        raise RuntimeError(f"PhysX stage attachment mismatch: expected {stage_id}, got {after}")
    return {"usd_stage_id": stage_id, "physx_stage_id_before": before,
            "physx_stage_id_after": after, "physx_stage_attached_explicitly": attached_explicitly}


def bind_live_gripper(get_stage, world, make_articulation, make_simulation_view, *, physics_scene_path=None,
                      prepare_attachment=None):
    """Bind fresh tensors after all scene edits, countdown and the final play transition."""
    if physics_scene_path:
        require_physics_scene(get_stage(), physics_scene_path, "before final play")
    world.play()
    world.step(render=True)  # Process play/physics events before constructing any tensor view.
    stage = get_stage()
    if physics_scene_path:
        require_physics_scene(stage, physics_scene_path, "after final play/step")
    attachment = prepare_attachment() if prepare_attachment is not None else {}
    roots = articulation_root_paths(stage)
    default = stage.GetDefaultPrim()
    candidates = list(dict.fromkeys([str(default.GetPath()) if default else "/Robotiq_2F_140", *roots]))
    physics_view = make_simulation_view("numpy")
    physics_view.set_subspace_roots("/")
    failures = []
    for candidate in candidates:
        try:
            art = make_articulation(prim_path=candidate, name="standalone_gripper", reset_xform_properties=False)
            art.initialize(physics_sim_view=physics_view)
            names = list(art.dof_names)
            positions = art.get_joint_positions()
            q = np.asarray(positions, dtype=float) if positions is not None else np.array([])
            if len(set(names)) != len(names) or names.count("finger_joint") != 1 or q.shape != (len(names),) or not np.isfinite(q).all():
                raise ValueError("primary joint mapping or finite runtime feedback unavailable")
            metadata = {"articulation_root_paths": roots, "live_articulation_path": candidate,
                        "live_joint_names": names, "commanded_joint_index": names.index("finger_joint"),
                        "initial_live_joint_positions_rad": q.tolist(), "wrapper_probe_failures": failures,
                        "fresh_physics_view_after_final_play": True}
            metadata.update(attachment)
            return art, names, metadata
        except Exception as exc:
            failures.append(f"{candidate}: {exc}")
    raise ValueError("no usable fresh live articulation: " + "; ".join(failures))


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
