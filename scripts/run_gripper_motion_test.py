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
    args = parser.parse_args()
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": False})
    world, source = None, args.usd
    setup = {}
    try:
        import numpy as np
        import omni.usd
        from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.types import ArticulationAction
        from fdw_sim.visualization.robot_loader import get_isaac_assets_root
        from fdw_sim.visualization.tool_diagnostics import load_tool_configuration
        from fdw_sim.visualization.gripper_asset_inventory import inspect_gripper_stage
        from fdw_sim.visualization.gripper_motion_test import inspect_motion_prerequisites, run_gripper_cycle
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
            prims = list(Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()))
            if any(p.GetTypeName() == "OmniGraph" for p in prims):
                raise ValueError("asset contains an OmniGraph; inspect controller conflicts before motion")
            scenes = [p for p in prims if p.IsA(UsdPhysics.Scene)]
            if len(scenes) > 1:
                raise ValueError("multiple physics scenes are unsupported")
            # All experiment edits live in the transient session layer. Never save source.
            stage.SetEditTarget(stage.GetSessionLayer())
            meters = UsdGeom.GetStageMetersPerUnit(stage)
            dt = 1/120
            world = World(stage_units_in_meters=meters, physics_dt=dt, rendering_dt=dt,
                          physics_prim_path=str(scenes[0].GetPath()) if scenes else "/World/GripperTest/PhysicsScene")
            world.get_physics_context().set_gravity(0)
            world.reset()
            roots = [str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
            setup["articulation_root_paths"] = roots
            candidates = list(dict.fromkeys([str(stage.GetDefaultPrim().GetPath()) if stage.GetDefaultPrim() else "/Robotiq_2F_140", *roots]))
            failures, art = [], None
            for candidate in candidates:
                try:
                    probe = SingleArticulation(prim_path=candidate, name="standalone_gripper", reset_xform_properties=False)
                    probe.initialize()
                    names = list(probe.dof_names)
                    positions = probe.get_joint_positions()
                    if names.count("finger_joint") != 1 or positions is None or len(positions) != len(names):
                        raise ValueError("primary joint not uniquely available in runtime feedback")
                    if not np.isfinite(np.asarray(positions)).all():
                        raise ValueError("nonfinite runtime joint feedback")
                    art = probe
                    break
                except Exception as exc:
                    failures.append(f"{candidate}: {exc}")
            if art is None:
                raise ValueError("no usable live articulation: " + "; ".join(failures))
            index = names.index("finger_joint")
            setup.update(live_articulation_path=candidate, live_joint_names=names,
                         commanded_joint_index=index, wrapper_probe_failures=failures,
                         initial_live_joint_positions_rad=np.asarray(positions, dtype=float).tolist(),
                         physics_dt_sec=dt, gravity_m_s2=0)
            world.pause()
            base = stage.GetPrimAtPath(setup["base_path"])
            base_start = _usd_rigid_matrix(UsdGeom.XformCache(), base, meters)
            marker_ops = []
            for side, color in (("LeftBlue", (0.1, 0.4, 1)), ("RightOrange", (1, 0.4, 0.1))):
                marker = UsdGeom.Sphere.Define(stage, "/World/GripperTest/" + side)
                marker.CreateRadiusAttr(0.004/meters)
                marker.CreateDisplayColorAttr([Gf.Vec3f(*color)])
                marker_ops.append(UsdGeom.Xformable(marker.GetPrim()).AddTranslateOp())
            def read_pads():
                cache = UsdGeom.XformCache()
                base_now = _usd_rigid_matrix(cache, base, meters)
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
            def read_joint():
                q = art.get_joint_positions()
                if q is None or len(q) != len(names) or not np.isfinite(np.asarray(q)).all():
                    raise ValueError("runtime joint feedback lost or nonfinite")
                return q[index]
            def command(q):
                art.apply_action(ArticulationAction(joint_positions=np.array([q]), joint_indices=np.array([index])))
            world.play()
            report = run_gripper_cycle(read_joint, command, read_pads, lambda: world.step(render=True), world.pause,
                                       setup["limit_rad"], dt=dt, realtime=True, is_running=app.is_running)
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
