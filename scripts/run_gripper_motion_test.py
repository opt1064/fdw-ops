"""GUI-only standalone Robotiq partial close/return with bilateral measured motion."""
import argparse
import json
from pathlib import Path
import sys
import time
from uuid import uuid4

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
    args = parser.parse_args()
    if not 1 <= args.drive_damping_scale <= 100:
        parser.error("--drive-damping-scale must be finite and between 1 and 100")
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": False})
    world, source = None, args.usd
    setup = {}
    try:
        import numpy as np
        import omni.usd
        import omni.physics.tensors
        import omni.physx
        import omni.timeline
        from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics
        from isaacsim.core.api import World
        from fdw_sim.visualization.robot_loader import get_isaac_assets_root
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
            def read_pads():
                cache = UsdGeom.XformCache()
                base_now = _usd_rigid_matrix(cache, stage.GetPrimAtPath(setup["base_path"]), meters)
                error = _pose_error(base_now, base_start)
                if error["position_error_m"] > 0.002 or error["orientation_error_rad"] > 0.01:
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
                                                    prepare_attachment=prepare_attachment)
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
                art.set_primary_target(q, index)
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
        except (Exception, KeyboardInterrupt) as exc:
            report = {"status": "FAILED", "errors": [f"{type(exc).__name__}: {exc}"],
                      "motion_commands_issued": False, "physical_grasp_verified": False}
            if world is not None:
                try:
                    world.pause()
                    report["simulation_paused"] = True
                except Exception as pause_error:
                    report["errors"].append(f"pause failed: {pause_error}")
        report.update(source_usd=source, setup=setup)
        args.output_dir.mkdir(parents=True, exist_ok=True)
        output = args.output_dir / (time.strftime("gripper_motion_%Y%m%d_%H%M%S_") + uuid4().hex[:8] + ".json")
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        print(f"[GRIPPER MOTION] saved: {output}")
        print(json.dumps({k: v for k, v in report.items() if k != "samples"}, ensure_ascii=False, indent=2))
        if args.keep_alive:
            print("[GRIPPER MOTION] paused viewer; close window or Ctrl+C to exit.", flush=True)
            try:
                while app.is_running():
                    app.update()
            except KeyboardInterrupt:
                pass
        return 0 if report["status"] == "PASSED_UNLOADED_GRIPPER_CYCLE" else 2
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
