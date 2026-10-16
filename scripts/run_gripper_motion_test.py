"""GUI Robotiq cycles, optional FANUC mounting and small mounted-arm/TCP checks."""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", help="Explicit Robotiq USD path/URI")
    parser.add_argument("--keep-alive", action="store_true")
    parser.add_argument("--skip-auto-camera", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "fdw_sim/logs")
    parser.add_argument("--drive-damping-scale", type=float, default=10.0,
                        help="Runtime-only primary damping multiplier (1=authored baseline; default=10)")
    parser.add_argument("--mounted-on-fanuc", action="store_true",
                        help="Temporary direct flange/base mount; hold FANUC stationary (no TCP calibration)")
    parser.add_argument("--fanuc-usd", help="Explicit FANUC asset path/URI; requires --mounted-on-fanuc")
    parser.add_argument("--mounted-arm-motion", action="store_true",
                        help="After mounted gripper cycle passes, move J1/J5/J6 +/- and verify mount/TCP candidate")
    parser.add_argument("--arm-amplitude-deg", type=float, default=1.0,
                        help="Mounted arm test amplitude in degrees (0 < value <= 2; default=1)")
    parser.add_argument("--grasp-test", action="store_true",
                        help="After unloaded cycles, contact/lift/hold/release a synthetic 50g block")
    parser.add_argument("--transfer-test", action="store_true",
                        help="With --grasp-test, carry the block 10-15cm onto a separate target support")
    args = parser.parse_args()
    if not 1 <= args.drive_damping_scale <= 100:
        parser.error("--drive-damping-scale must be finite and between 1 and 100")
    if args.fanuc_usd and not args.mounted_on_fanuc:
        parser.error("--fanuc-usd requires --mounted-on-fanuc")
    if args.mounted_arm_motion and not args.mounted_on_fanuc:
        parser.error("--mounted-arm-motion requires --mounted-on-fanuc")
    if args.grasp_test and (not args.mounted_on_fanuc or args.mounted_arm_motion):
        parser.error("--grasp-test requires --mounted-on-fanuc and cannot combine with --mounted-arm-motion")
    if args.transfer_test and not args.grasp_test:
        parser.error("--transfer-test requires --grasp-test --mounted-on-fanuc")
    if not 0 < args.arm_amplitude_deg <= 2:
        parser.error("--arm-amplitude-deg must be finite and in (0, 2]")
    from fdw_sim.visualization.experiment_log import ExperimentLog
    diagnostics = ExperimentLog(args.output_dir)
    print(f"[GRIPPER MOTION] durable events: {diagnostics.events}", flush=True)
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": False})
    exit_code = 2
    world, source = None, args.usd
    setup = {}
    mount, arm_guard = None, None
    arm_motion_active = False
    report = None
    try:
        import numpy as np
        import omni.usd
        import omni.physics.tensors
        import omni.physx
        import omni.timeline
        from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics
        from isaacsim.core.api import World
        from fdw_sim.visualization.robot_loader import get_isaac_assets_root, resolve_robot_usd_path, ROBOT_CATALOG
        from fdw_sim.visualization.gripper_mount_test import mount_fanuc_gripper, mount_frame_observation, StationaryArmGuard
        from fdw_sim.visualization.mounted_arm_motion_test import UsdMountedArmModel, MovingArmController, run_mounted_arm_motion
        from fdw_sim.visualization.tool_diagnostics import load_tool_configuration
        from fdw_sim.visualization.gripper_asset_inventory import inspect_gripper_stage
        from fdw_sim.visualization.gripper_motion_test import (
            inspect_motion_prerequisites, run_gripper_cycle, bind_live_gripper,
            compose_gripper_experiment, require_physics_scene,
            ensure_physx_attachment,
            NativeGripperArticulation,
            ManualPhysxStepper,
            PrimaryPositionServo,
        )
        from fdw_sim.visualization.robot_model_export import _usd_rigid_matrix
        from fdw_sim.visualization.lula_model_validation import _pose_error
        from fdw_sim.visualization.scene_builder import SceneBuilder, SceneConfig
        try:
            if not source:
                config = load_tool_configuration(ROOT / "fdw_sim/config/fanuc_tooling.yaml")
                source = get_isaac_assets_root().rstrip("/") + "/" + config["tools"]["gripper"]["candidate_usd_subpath"]
            if not omni.usd.get_context().open_stage(source):
                raise RuntimeError("could not open standalone gripper stage")
            for _ in range(3):
                app.update()
            stage = omni.usd.get_context().get_stage()
            inventory = inspect_gripper_stage(stage)
            if inventory["status"] == "FAILED":
                raise ValueError(str(inventory["errors"]))
            setup = inspect_motion_prerequisites(stage)
            source_identifier = stage.GetRootLayer().identifier
            meters = UsdGeom.GetStageMetersPerUnit(stage)
            up_axis = str(UsdGeom.GetStageUpAxis(stage))
            default_path = str(stage.GetDefaultPrim().GetPath()) if stage.GetDefaultPrim() else None
            if not omni.usd.get_context().new_stage():
                raise RuntimeError("could not create isolated experiment stage")
            stage = omni.usd.get_context().get_stage()
            setup.update(compose_gripper_experiment(stage, source_identifier, meters=meters,
                                                    up_axis=up_axis, default_prim_path=default_path))
            if args.mounted_on_fanuc:
                robot_source = args.fanuc_usd or resolve_robot_usd_path(ROBOT_CATALOG["fanuc_crx10ia"])
                mount = mount_fanuc_gripper(stage, robot_source, setup)
                setup["mount"] = mount
                setup["mount_initial_observation"] = mount_frame_observation(stage, mount)
                print("[GRIPPER MOUNT] temporary direct flange/base mount; changer and TCP calibration unverified.", flush=True)
            prims = list(Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()))
            if any(p.GetTypeName() == "OmniGraph" for p in prims):
                raise ValueError("asset contains an OmniGraph; inspect controller conflicts before motion")
            scenes = [str(p.GetPath()) for p in prims if p.IsA(UsdPhysics.Scene)]
            if len(scenes) > 1:
                raise ValueError("multiple physics scenes are unsupported")
            del prims  # Do not retain instance-proxy handles across World/reset mutations.
            # Never save the anonymous experiment or write the referenced source layer.
            stage.SetEditTarget(stage.GetRootLayer())
            dt = 1/120
            physics_scene_path = scenes[0] if scenes else "/World/GripperPhysicsScene"
            setup["physics_scene_path"] = physics_scene_path
            world = World(stage_units_in_meters=meters, physics_dt=dt, rendering_dt=dt,
                          physics_prim_path=physics_scene_path)
            world.get_physics_context().set_gravity(0)
            world.reset()
            stage = omni.usd.get_context().get_stage()
            stage.SetEditTarget(stage.GetRootLayer())
            require_physics_scene(stage, physics_scene_path, "after world.reset")
            setup.update(physics_dt_sec=dt, gravity_m_s2=0)
            world.pause()
            base_start = _usd_rigid_matrix(UsdGeom.XformCache(), stage.GetPrimAtPath(setup["base_path"]), meters)
            marker_ops = []
            for side, color in (("LeftBlue", (0.1, 0.4, 1)), ("RightOrange", (1, 0.4, 0.1))):
                marker = UsdGeom.Sphere.Define(stage, "/World/GripperTest/" + side)
                marker.CreateRadiusAttr(0.004/meters)
                marker.CreateDisplayColorAttr([Gf.Vec3f(*color)])
                marker_ops.append(UsdGeom.Xformable(marker.GetPrim()).AddTranslateOp())
            tcp_marker_op = None
            if mount:
                marker = UsdGeom.Sphere.Define(stage, "/World/GripperTest/TcpCandidateCyan")
                marker.CreateRadiusAttr(0.004/meters)
                marker.CreateDisplayColorAttr([Gf.Vec3f(0, 1, 1)])
                tcp_marker_op = UsdGeom.Xformable(marker.GetPrim()).AddTranslateOp()
            def read_pads():
                if arm_guard is not None:
                    arm_guard.check()
                    observed = mount_frame_observation(stage, mount)
                    error = observed["flange_to_base_error"]
                    if error["position_error_m"] > 0.002 or error["orientation_error_rad"] > 0.01:
                        raise RuntimeError("FANUC/gripper mount relative frame drift exceeded tolerance")
                cache = UsdGeom.XformCache()
                if tcp_marker_op is not None:
                    tcp_marker_op.Set(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(mount["tcp_candidate_path"])).ExtractTranslation())
                base_now = _usd_rigid_matrix(cache, stage.GetPrimAtPath(setup["base_path"]), meters)
                error = _pose_error(base_now, base_start)
                if not arm_motion_active and (error["position_error_m"] > 0.002 or error["orientation_error_rad"] > 0.01):
                    raise RuntimeError("fixed gripper base moved unexpectedly")
                inverse = np.linalg.inv(base_now)
                bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
                centers = []
                for path, op in zip(setup["pad_paths"], marker_ops):
                    box = bounds.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedBox()
                    if box.IsEmpty():
                        raise ValueError("fingertip geometry bounds empty")
                    center = np.array(box.GetMidpoint(), dtype=float)
                    op.Set(Gf.Vec3d(*center))
                    centers.append((inverse @ np.r_[center*meters, 1])[:3])
                return np.array(centers)
            read_pads()
            light = UsdLux.DomeLight.Define(stage, "/World/GripperTest/Light")
            light.CreateIntensityAttr(1000)
            if not args.skip_auto_camera:
                bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]).ComputeWorldBound(
                    stage.GetPrimAtPath("/Robotiq_2F_140")).ComputeAlignedBox()
                extent = max(bbox.GetSize())
                builder = SceneBuilder(SceneConfig(root_prim_path="/World/GripperTest"))
                builder.frame_viewport_to_scene(center=tuple(bbox.GetMidpoint()), distance=max(extent*2.2, 0.6/meters),
                                               height=max(extent*0.9, 0.25/meters))
            print("[GRIPPER MOTION] Left fingertip: BLUE; right fingertip: ORANGE. Two partial-close/return cycles.", flush=True)
            if mount:
                print("[GRIPPER MOUNT] CYAN: asset grasp_frame (TCP candidate, uncalibrated). FANUC joints held stationary.", flush=True)
            for seconds in range(5, 0, -1):
                print(f"[GRIPPER MOTION] starts in {seconds}...", flush=True)
                deadline = time.monotonic()+1
                while time.monotonic() < deadline:
                    if not app.is_running():
                        raise RuntimeError("viewer closed before test")
                    app.update()
                    time.sleep(0.02)
            def prepare_attachment():
                # Drain pause events before attaching and creating the final tensor view.
                # Keep timeline paused: only our explicit simulate/fetch advances physics.
                world.pause()
                app.update()
                interface = omni.physx.get_physx_simulation_interface()
                stage_id = omni.usd.get_context().get_stage_id()
                setup.update(usd_stage_id=int(stage_id), physx_stage_id_before=int(interface.get_attached_stage()))
                attachment = ensure_physx_attachment(interface, stage_id)
                omni.physx.get_physx_interface().set_simulation_layer(
                    omni.usd.get_context().get_stage().GetRootLayer().identifier)
                setup.update(attachment)
                print(f"[GRIPPER MOTION] PhysX attached stage={attachment['physx_stage_id_after']} "
                      f"(explicit={attachment['physx_stage_attached_explicitly']})", flush=True)
                return attachment
            setup["control_backend"] = "direct_omni_physics_tensors"
            art, names, binding = bind_live_gripper(lambda: omni.usd.get_context().get_stage(), world,
                                                    NativeGripperArticulation, omni.physics.tensors.create_simulation_view,
                                                    physics_scene_path=physics_scene_path,
                                                    prepare_attachment=prepare_attachment,
                                                    articulation_path=mount["robot_path"] if mount else None)
            setup.update(binding)
            index = binding["commanded_joint_index"]
            stage = omni.usd.get_context().get_stage()
            stage.SetEditTarget(stage.GetRootLayer())
            base_start = _usd_rigid_matrix(UsdGeom.XformCache(), stage.GetPrimAtPath(setup["base_path"]), meters)
            print("[GRIPPER MOTION] fresh physics view bound; timeline paused, explicit PhysX stepping enabled.", flush=True)
            def read_joint():
                q = art.get_joint_positions()
                if q is None or len(q) != len(names) or not np.isfinite(np.asarray(q)).all():
                    raise ValueError("runtime joint feedback lost or nonfinite")
                return q[index]
            def write_target(q):
                if arm_guard is not None:
                    arm_guard.hold()
                art.set_primary_target(q, index)
            if mount:
                arm_guard = StationaryArmGuard(art._view)
            setup["drive_experiment"] = art.configure_primary_damping(index, args.drive_damping_scale)
            print(f"[GRIPPER MOTION] runtime primary damping scale={args.drive_damping_scale}; "
                  "stiffness/maxForce/passive gains unchanged.", flush=True)
            def read_velocity():
                return art.get_joint_velocities()[index]
            servo = PrimaryPositionServo(read_joint, write_target, setup["limit_rad"], dt)
            print("[GRIPPER MOTION] preparing measured 4-degree unloaded start; bounded feedback trim enabled.", flush=True)
            stepper = ManualPhysxStepper(omni.physx.get_physx_simulation_interface(),
                                        lambda: omni.physx.get_physx_interface().update_transformations(False, True),
                                        app.update, omni.timeline.get_timeline_interface().is_playing,
                                        stage_id=setup["usd_stage_id"], dt=dt)
            report = run_gripper_cycle(read_joint, servo.command, read_pads, stepper.step, world.pause,
                                       setup["limit_rad"], dt=dt, realtime=True, is_running=app.is_running,
                                       read_velocity=read_velocity, settle_initial=True,
                                       initial_target=np.deg2rad(4), read_command=lambda: dict(servo.last))
            setup.update(stepper.diagnostics())
            setup["position_servo"] = servo.diagnostics()
            if mount:
                setup["arm_hold"] = arm_guard.diagnostics()
                setup["mount_final_observation"] = mount_frame_observation(stage, mount)
                report.update(mounted_on_fanuc=True,
                              scope="temporary direct mount; stationary FANUC; unloaded zero-gravity partial cycle; no TCP/contact calibration")
                if report["status"] == "PASSED_UNLOADED_GRIPPER_CYCLE":
                    report["status"] = "PASSED_MOUNTED_UNLOADED_GRIPPER_CYCLE"
            if args.mounted_arm_motion:
                if report["status"] != "PASSED_MOUNTED_UNLOADED_GRIPPER_CYCLE":
                    report["arm_motion"] = {"status": "SKIPPED_PREREQUISITE_FAILED", "motion_commands_issued": False}
                else:
                    print("[MOUNTED ARM] gripper 4/4 prerequisite passed; starting J1/J5/J6 small motion.", flush=True)
                    # Same attached PhysX stage and retained tensor view; no play/reset/rebind between phases.
                    model = UsdMountedArmModel(stage, mount)
                    arm_guard = MovingArmController(art._view, args.arm_amplitude_deg)
                    arm_motion_active = True
                    arm_result = run_mounted_arm_motion(
                        arm_guard, model, servo.command, read_joint, read_pads,
                        lambda: mount_frame_observation(stage, mount), stepper.step, world.pause,
                        dt=dt, amplitude_deg=args.arm_amplitude_deg, realtime=True, is_running=app.is_running)
                    report["gripper_cycle_status"] = report["status"]
                    report["gripper_returned_to_start"] = report["returned_to_start"]
                    report["gripper_passed_legs"] = report["passed_legs"]
                    report["arm_motion"] = arm_result
                    report["arm_motion_passed_legs"] = arm_result["passed_legs"]
                    report["status"] = arm_result["status"]
                    report["returned_to_start"] = arm_result["returned_to_start"]
                    report["errors"].extend(arm_result["errors"])
                    report["scope"] = "mounted unloaded arm motion and USD FK/TCP candidate consistency; no hardware/contact/TCP calibration"
                    setup.update(stepper.diagnostics())
                    setup["mount_final_observation"] = mount_frame_observation(stage, mount)
            if args.grasp_test:
                # Set failure status before any preparation; an unloaded pass cannot stand in for a grasp.
                prerequisite = report["status"]
                report["gripper_cycle_status"] = prerequisite
                report["gripper_returned_to_start"] = report.get("returned_to_start", False)
                report["returned_to_start"] = False
                report["status"] = "FAILED"
                if prerequisite != "PASSED_MOUNTED_UNLOADED_GRIPPER_CYCLE":
                    report["grasp_test"] = {"status": "SKIPPED_PREREQUISITE_FAILED"}
                else:
                    from pxr import PhysicsSchemaTools
                    from omni.physx.bindings._physx import ContactEventType
                    from fdw_sim.visualization.physical_grasp_test import (
                        create_grasp_fixture, ContactEvidence, LoadedTestBlock, run_contact_grasp, plan_lift)
                    print("[PHYSICAL GRASP] YELLOW block / BLUE pedestal; close, lift, hold 3s, open, return.", flush=True)
                    initial_q = art.get_joint_positions()
                    prerequisite_art = art  # Retain the previous simulation-view owner through the rebind.
                    gain_methods = ("get_dof_stiffnesses", "get_dof_dampings", "get_dof_max_forces")
                    gains = {name: np.array(getattr(art._view, name)(), copy=True) for name in gain_methods}
                    fixture = create_grasp_fixture(stage, mount, setup["pad_paths"])
                    setup["grasp_fixture"] = fixture
                    model = UsdMountedArmModel(stage, mount)
                    current_arm = initial_q[arm_guard.indices]
                    lift_target, lift_plan = plan_lift(model, current_arm)
                    fixture["lift_plan"] = lift_plan
                    if args.transfer_test:
                        from fdw_sim.visualization.grasp_transfer_test import plan_transfer, create_target_support
                        transfer_plan = plan_transfer(model, current_arm, lift_target, fixture)
                        create_target_support(stage, fixture, transfer_plan)
                        print(f"[GRASP TRANSFER] BLUE source -> PURPLE target; "
                              f"{transfer_plan['horizontal_distance_m']*1000:.1f}mm horizontal arc; "
                              "lower, open, retreat and return.", flush=True)
                        if not args.skip_auto_camera:
                            bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
                            box = bounds.ComputeWorldBound(stage.GetPrimAtPath("/Robotiq_2F_140")).ComputeAlignedBox()
                            box.UnionWith(bounds.ComputeWorldBound(stage.GetPrimAtPath("/World/GraspTest")).ComputeAlignedBox())
                            extent = max(box.GetSize())
                            SceneBuilder(SceneConfig(root_prim_path="/World/GripperTest")).frame_viewport_to_scene(
                                center=tuple(box.GetMidpoint()), distance=max(extent*2.4, .7/meters),
                                height=max(extent, .25/meters))
                    # Adding an independent rigid body changes the scene inventory. Flush and bind
                    # fresh tensor views without play/reset; retain and verify physical state/gains.
                    omni.physx.get_physx_simulation_interface().flush_changes()
                    fresh_view = omni.physics.tensors.create_simulation_view("numpy")
                    fresh_view.set_subspace_roots("/")
                    fresh_art = NativeGripperArticulation(prim_path=binding["live_articulation_path"])
                    fresh_art.initialize(physics_sim_view=fresh_view)
                    if fresh_art.dof_names != names or not np.allclose(fresh_art.get_joint_positions(), initial_q, atol=1e-6, rtol=0):
                        raise RuntimeError("fixture rebind changed articulation inventory or measured positions")
                    for name, expected in gains.items():
                        if not np.allclose(getattr(fresh_art._view, name)(), expected, rtol=1e-6, atol=1e-7):
                            raise RuntimeError(f"fixture rebind changed runtime drive properties: {name}")
                    art = fresh_art
                    arm_guard = MovingArmController(art._view, 2, transfer_j1_deg=8 if args.transfer_test else None)
                    arm_motion_active = True
                    block = LoadedTestBlock(fresh_view, fixture)
                    contacts = ContactEvidence(fixture, setup["pad_paths"], setup["usd_stage_id"], dt,
                                               PhysicsSchemaTools.intToSdfPath, ContactEventType.CONTACT_LOST)
                    contact_subscription = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(contacts.callback)
                    if contact_subscription is None:
                        raise RuntimeError("PhysX contact report subscription unavailable")
                    def grasp_command(grip, target):
                        arm_guard.set_target(target)
                        servo.command(grip)
                    def grasp_step():
                        contacts.begin_step()
                        block.load()
                        stepper.step()
                    def grasp_observe():
                        read_pads()  # Independent mount drift checks and visible fingertip/TCP markers.
                        q = arm_guard.read()
                        if np.any(q < model.limits[:, 0]-.001) or np.any(q > model.limits[:, 1]+.001):
                            raise RuntimeError("loaded arm feedback outside authored joint limits")
                        observed = mount_frame_observation(stage, mount)
                        tcp_error = _pose_error(np.array(observed["tcp_candidate_world_matrix_m"]), model.predict(q)[1])
                        if tcp_error["position_error_m"] > .002 or tcp_error["orientation_error_rad"] > .01:
                            raise RuntimeError("loaded TCP candidate disagrees with measured-joint USD FK")
                        primary = float(read_joint())
                        if not setup["limit_rad"][0]-.001 <= primary <= setup["limit_rad"][1]+.001:
                            raise RuntimeError("loaded primary joint outside authored limits")
                        data = block.measure()
                        base = _usd_rigid_matrix(UsdGeom.XformCache(), stage.GetPrimAtPath(setup["base_path"]), meters)
                        data.update(object_relative_base_matrix_m=(np.linalg.inv(base) @ np.array(data["object_world_matrix_m"])).tolist(),
                                    contacts=contacts.read(), gripper_rad=primary,
                                    arm_target_error_rad=float(np.max(np.abs(q-arm_guard.target))),
                                    arm_speed_rad_s=float(np.max(np.abs(arm_guard.velocities()))),
                                    tcp_candidate_error=tcp_error)
                        return data
                    def phase_color(phase):
                        color = ((1, .1, .1) if phase == "FAILED" else
                                 (.1, 1, .2) if phase in ("LIFT", "HOLD_UNSUPPORTED", "TRANSFER", "HOLD_AT_TARGET") else
                                 (1, .4, .05) if phase == "OPEN_RELEASE" else
                                 (.3, .7, 1) if phase in ("RETREAT_EMPTY", "RETURN_ABOVE_SOURCE", "RETURN_EMPTY") else (1, .8, .05))
                        UsdGeom.Gprim(stage.GetPrimAtPath(fixture["block_path"])).GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
                    open_rad = float(read_joint())
                    grasp_result = run_contact_grasp(
                        grasp_command, grasp_observe, grasp_step, world.pause, phase_color, fixture,
                        arm_guard.start, lift_target, open_rad, open_rad+np.deg2rad(10),
                        dt=dt, realtime=True, is_running=app.is_running, progress=diagnostics.event)
                    # Keep subscription and tensor owners alive through the last measurement.
                    report["grasp_test"] = grasp_result
                    report["status"] = grasp_result["status"]
                    report["physical_grasp_verified"] = grasp_result["physical_grasp_verified"]
                    report["transfer_verified"] = grasp_result.get("transfer_verified", False)
                    report["target_placement_verified"] = grasp_result.get("target_placement_verified", False)
                    report["returned_to_start"] = grasp_result.get("returned_to_start", False)
                    report["errors"].extend(grasp_result["errors"])
                    report["scope"] = grasp_result["scope"]
                    report["verification_scope"] = "synthetic block only; production adapter and TCP calibration remain unverified"
                    setup.update(stepper.diagnostics())
                    setup["position_servo"] = servo.diagnostics()
                    setup["mount_final_observation"] = mount_frame_observation(stage, mount)
        except (Exception, KeyboardInterrupt) as exc:
            diagnostics.exception("experiment execution", exc)
            if report is None:
                report = {"status": "FAILED", "errors": [],
                          "motion_commands_issued": False, "physical_grasp_verified": False}
            report["status"] = "FAILED"
            if args.grasp_test:
                report["physical_grasp_verified"] = False
                report["transfer_verified"] = False
                report["target_placement_verified"] = False
                report["returned_to_start"] = False
            report["errors"].append(f"{type(exc).__name__}: {exc}")
            if arm_guard is not None:
                try:
                    setup["arm_hold"] = arm_guard.diagnostics()
                except Exception as diagnostic_error:
                    report["errors"].append(f"arm diagnostic failed: {diagnostic_error}")
            if world is not None:
                try:
                    world.pause()
                    report["simulation_paused"] = True
                except Exception as pause_error:
                    report["errors"].append(f"pause failed: {pause_error}")
        report.update(source_usd=source, setup=setup)
        output = diagnostics.save(report)
        print(f"[GRIPPER MOTION] saved: {output}", flush=True)
        summary = {k: v for k, v in report.items() if k != "samples"}
        if "arm_motion" in summary:
            summary["arm_motion"] = {k: v for k, v in summary["arm_motion"].items() if k != "samples"}
        if "grasp_test" in summary:
            summary["grasp_test"] = {k: v for k, v in summary["grasp_test"].items() if k != "samples"}
        print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
        exit_code = 0 if report["status"] in (
            "PASSED_UNLOADED_GRIPPER_CYCLE", "PASSED_MOUNTED_UNLOADED_GRIPPER_CYCLE",
            "PASSED_MOUNTED_ARM_TCP_TEST", "PASSED_SIMULATED_CONTACT_GRASP_RELEASE",
            "PASSED_SIMULATED_GRASP_TRANSFER_PLACE") else 2
        if args.keep_alive:
            print("[GRIPPER MOTION] paused viewer; close window or Ctrl+C to exit.", flush=True)
            try:
                while app.is_running():
                    app.update()
            except KeyboardInterrupt:
                pass
        return exit_code
    except BaseException as exc:
        exit_code = 2
        # Kit fast shutdown can terminate the process before Python prints an escaping exception.
        diagnostics.exception("unhandled exception before app.close", exc)
        if not diagnostics.output.exists():
            try:
                diagnostics.save({"status": "FAILED", "physical_grasp_verified": False,
                                  "transfer_verified": False, "target_placement_verified": False,
                                  "report_incomplete": True, "errors": [f"{type(exc).__name__}: {exc}"],
                                  "events_file": str(diagnostics.events), "traceback_file": str(diagnostics.trace)})
            except Exception as save_error:
                print(f"[GRIPPER ERROR] Emergency result save failed: {save_error}", file=sys.stderr, flush=True)
        raise
    finally:
        try:
            diagnostics.event("APP_CLOSE_REQUESTED", exit_code=exit_code)
        finally:
            app.close(exit_code=exit_code)


if __name__ == "__main__":
    raise SystemExit(main())
