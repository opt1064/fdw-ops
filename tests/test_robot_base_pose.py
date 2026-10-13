"""Base-frame regressions with strict Isaac/USD stubs, not physics validation.

Run with ``python tests/test_robot_base_pose.py`` (or pytest).
"""
from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fdw_sim.messaging.bus import InMemoryBus
from fdw_sim.visualization.robot_base_pose import RobotBasePose, read_robot_base_pose
from fdw_sim.visualization.robot_loader import ROBOT_CATALOG, RobotLoader
from fdw_sim.visualization import ik_controller as ik
from fdw_sim.visualization import rmpflow_controller as rmp
from fdw_sim.visualization.scene_builder import SceneBuilder
from fdw_sim.visualization.workshop_visualizer import WorkshopVisualizer, WorkshopVizConfig


def _module(name, **attrs):
    module = ModuleType(name)
    module.__dict__.update(attrs)
    return module


class _Articulation:
    def __init__(self, position=(25.1, 10.45, 0.85), orientation=(1, 0, 0, 0)):
        self.position = position
        self.orientation = orientation

    def get_world_pose(self):
        return self.position, self.orientation

    def get_local_pose(self):
        raise AssertionError("Controller must not read a parent-local base pose")


class _Solver:
    def __init__(self, **kwargs):
        self.events = []

    def set_robot_base_pose(self, position, orientation):
        self.events.append(("base", tuple(position), tuple(orientation)))

    def compute_inverse_kinematics(self, frame_name, target_position, target_orientation,
                                   warm_start=None):
        self.warm_start = warm_start
        self.events.append(("solve", tuple(target_position), tuple(target_orientation)))
        return np.zeros(7), True


class _Policy(_Solver):
    # Match NVIDIA's target_translation parameter, rather than accepting any
    # kwargs and accidentally hiding a broken real-runtime call signature.
    def set_end_effector_target(self, target_translation, target_orientation):
        self.events.append(("target", tuple(target_translation), tuple(target_orientation)))

    def update_world(self):
        self.events.append(("world",))


class _ArticulationPolicy:
    def __init__(self, robot_articulation, motion_policy):
        self.policy = motion_policy

    def get_next_articulation_action(self, dt):
        self.policy.events.append(("action",))
        return SimpleNamespace(joint_positions=np.zeros(7))


def _motion_modules():
    lula = _module("isaacsim.robot_motion.motion_generation.lula", LulaKinematicsSolver=_Solver)
    generation = _module("isaacsim.robot_motion.motion_generation", lula=lula,
                         RmpFlow=_Policy, ArticulationMotionPolicy=_ArticulationPolicy)
    motion = _module("isaacsim.robot_motion", motion_generation=generation)
    return {"isaacsim": _module("isaacsim", robot_motion=motion),
            motion.__name__: motion, generation.__name__: generation, lula.__name__: lula}


class _Prim:
    def __init__(self, path, stage):
        self.path, self.stage = path, stage
        self.position, self.yaw = (0.0, 0.0, 0.0), 0.0

    def IsValid(self):
        return True

    def GetReferences(self):
        return SimpleNamespace(AddReference=lambda path: None)

    def GetChildren(self):
        return [object()]  # Simulate a successfully resolved robot reference.


class _WorldTransform:
    def __init__(self, position, yaw):
        self.position, self.yaw = position, yaw

    def ExtractTranslation(self):
        return self.position

    def ExtractRotationQuat(self):
        return SimpleNamespace(GetReal=lambda: math.cos(self.yaw / 2),
                               GetImaginary=lambda: (0, 0, math.sin(self.yaw / 2)))


class _Stage:
    def __init__(self):
        self.prims = {}

    def define(self, path, position=(0, 0, 0), yaw=0.0):
        prim = self.prims.setdefault(path, _Prim(path, self))
        prim.position, prim.yaw = position, yaw
        return prim

    def GetPrimAtPath(self, path):
        return self.prims.get(path)

    def world(self, prim):
        parent = self.prims.get(prim.path.rsplit("/", 1)[0])
        if parent is None:
            return _WorldTransform(prim.position, prim.yaw)
        world = self.world(parent)
        px, py, pz = world.position
        x, y, z = prim.position
        c, s = math.cos(world.yaw), math.sin(world.yaw)
        return _WorldTransform((px + c*x - s*y, py + s*x + c*y, pz + z),
                               world.yaw + prim.yaw)


class _Xformable:
    def __init__(self, prim):
        self.prim = prim

    def ClearXformOpOrder(self):
        pass

    def AddTranslateOp(self):
        return SimpleNamespace(Set=lambda position: setattr(self.prim, "position", position))

    def AddRotateZOp(self):
        return SimpleNamespace(Set=lambda degrees: setattr(self.prim, "yaw", math.radians(degrees)))


def _usd_modules(stage):
    geom = SimpleNamespace(
        Xform=SimpleNamespace(Define=lambda stage, path: stage.define(path)),
        Xformable=_Xformable,
        XformCache=lambda: SimpleNamespace(GetLocalToWorldTransform=stage.world),
    )
    pxr = _module("pxr", Usd=object(), UsdGeom=geom, Sdf=object(),
                  Gf=SimpleNamespace(Vec3d=lambda *xyz: tuple(xyz)))
    usd = _module("omni.usd", get_context=lambda: SimpleNamespace(get_stage=lambda: stage))
    return {"pxr": pxr, "omni": _module("omni", usd=usd), "omni.usd": usd}


class TestWorldBasePose(unittest.TestCase):
    def test_live_world_pose_takes_precedence_over_fallback(self):
        art = _Articulation(orientation=(2, 0, 0, 0))
        actual = read_robot_base_pose(art, RobotBasePose((5, 0, 0)))
        self.assertEqual(actual.position, (25.1, 10.45, 0.85))
        self.assertEqual(actual.orientation, (1, 0, 0, 0))

    def test_invalid_or_unavailable_live_pose_keeps_last_known_pose(self):
        fallback = RobotBasePose((25.1, 10.45, 0.85))
        for art in (None, object(), _Articulation(position=(float("nan"), 0, 0)),
                    _Articulation(orientation=(0, 0, 0, 0))):
            with self.subTest(art=art):
                self.assertIs(read_robot_base_pose(art, fallback), fallback)

    def test_usd_world_transform_includes_rotated_parent_and_height(self):
        stage = _Stage()
        stage.define("/Cell", (25.1, 10.75, 0.2), math.pi / 2)
        stage.define("/Cell/RobotArm", (0, -0.3, 0.95))
        art = SimpleNamespace(stage=stage, prim_path="/Cell/RobotArm")
        with patch.dict(sys.modules, _usd_modules(stage)):
            actual = read_robot_base_pose(art, RobotBasePose((5, 0, 0)))
        np.testing.assert_allclose(actual.position, (25.4, 10.75, 1.15))
        np.testing.assert_allclose(actual.orientation, (math.sqrt(0.5), 0, 0, math.sqrt(0.5)))

    def test_local_conversion_accounts_for_all_orientation_axes(self):
        pose = RobotBasePose((10, 20, 30), (math.sqrt(0.5), math.sqrt(0.5), 0, 0))
        np.testing.assert_allclose(pose.to_local((10, 18, 31)), (0, 1, 2), atol=1e-12)

    def test_legacy_base_xy_remains_supported(self):
        controller = rmp.RMPflowController(None, ROBOT_CATALOG["franka_panda"], (4, 3))
        self.assertEqual(controller.base_pose.position, (4, 3, 0))


class TestControllerBaseFrames(unittest.TestCase):
    spec = ROBOT_CATALOG["franka_panda"]

    def _heuristic_joints(self, position, target, orientation=(1, 0, 0, 0), obstacles=()):
        controller = rmp.RMPflowController(
            None, self.spec, base_position=position, base_orientation=orientation,
            config=rmp.RMPflowConfig(preferred_backend="heuristic"))
        for obstacle in obstacles:
            controller.add_obstacle(obstacle)
        controller._ensure_backend()
        controller._track_target(target)
        return controller._target_q

    def test_heuristic_uses_welding_base_for_yaw_and_reach(self):
        expected = self._heuristic_joints((0, 0, 0), (0.4, 0.2, 0.1))
        actual = self._heuristic_joints((25.1, 10.45, 0.85), (25.5, 10.65, 0.95))
        np.testing.assert_allclose(actual, expected)
        self.assertLess(actual[1] - self.spec.home_joint_positions[1], 0.4)

    def test_heuristic_applies_world_obstacles_before_base_rotation(self):
        expected = self._heuristic_joints(
            (0, 0, 0), (0.4, 0.2, 0.1),
            obstacles=[rmp.CollisionSphere("part", (0.4, 0.1, 0.1), 0.2)])
        # Rotate both target and obstacle +90 degrees and translate the scene.
        actual = self._heuristic_joints(
            (25.1, 10.45, 0.85), (24.9, 10.85, 0.95),
            orientation=(math.sqrt(0.5), 0, 0, math.sqrt(0.5)),
            obstacles=[rmp.CollisionSphere("part", (25.0, 10.85, 0.95), 0.2)])
        np.testing.assert_allclose(actual, expected)

    def test_heuristic_refreshes_moving_base_and_keeps_last_good_pose(self):
        art = _Articulation()
        controller = rmp.RMPflowController(
            art, self.spec, config=rmp.RMPflowConfig(preferred_backend="heuristic"))
        controller._ensure_backend()
        controller._track_target((25.5, 10.65, 0.95))
        expected = controller._target_q
        art.position = (30.1, 12.45, 0.85)
        controller._track_target((30.5, 12.65, 0.95))
        np.testing.assert_allclose(controller._target_q, expected)
        art.position = None
        controller._track_target((30.5, 12.65, 0.95))
        np.testing.assert_allclose(controller._target_q, expected)

    def test_rmpflow_and_lula_receive_world_pose_before_world_target(self):
        for backend_name in ("rmpflow", "ik"):
            with self.subTest(backend=backend_name), patch.dict(sys.modules, _motion_modules()), \
                    patch.object(rmp, "_ensure_motion_generation_extension"):
                art = _Articulation(orientation=(math.sqrt(0.5), 0, 0, math.sqrt(0.5)))
                controller = rmp.RMPflowController(
                    art, self.spec, config=rmp.RMPflowConfig(preferred_backend=backend_name))
                controller._locate_rmp_config = lambda: "/fake/rmp.yaml"
                controller._locate_urdf = lambda: "/fake/robot.urdf"
                controller._locate_robot_description = lambda **kwargs: "/fake/robot.yaml"
                controller._ensure_backend()
                self.assertEqual(controller.backend_name, backend_name)
                controller._track_target((25.5, 10.65, 0.95))
                backend = controller._backend
                delegate = backend._policy if backend_name == "rmpflow" else backend._solver
                self.assertEqual(delegate.events[0][0], "base")
                np.testing.assert_allclose(delegate.events[0][1], art.position)
                np.testing.assert_allclose(delegate.events[0][2], art.orientation)
                target_event = next(e for e in delegate.events if e[0] in ("target", "solve"))
                self.assertEqual(target_event[1], (25.5, 10.65, 0.95))
                self.assertEqual(target_event[2], (0, 1, 0, 0))
                if backend_name == "rmpflow":
                    self.assertEqual([e[0] for e in delegate.events],
                                     ["base", "world", "target", "action"])
                delegate.events.clear()
                art.position = (26.1, 10.45, 0.85)
                controller._track_target((26.5, 10.65, 0.95))
                self.assertEqual(delegate.events[0][1], art.position)
                if backend_name == "ik":
                    np.testing.assert_array_equal(delegate.warm_start, np.zeros(7))

    def test_explicit_ik_heuristic_uses_local_target(self):
        q = (math.sqrt(0.5), 0, 0, math.sqrt(0.5))
        original = ik.IKController(None, self.spec, base_position=(0, 0, 0))
        relocated = ik.IKController(None, self.spec, base_position=(25.1, 10.45, 0.85),
                                    base_orientation=q)
        original._track_target((0.4, 0.2, 0.1))
        relocated._track_target((24.9, 10.85, 0.95))
        np.testing.assert_allclose(relocated._target_q, original._target_q)

    def test_explicit_ik_lula_receives_current_world_pose(self):
        with patch.dict(sys.modules, _motion_modules()):
            art = _Articulation()
            controller = ik.IKController(art, self.spec)
            controller._ik_backend = ik._LulaIKBackend("/fake/robot.yaml", "/fake/robot.urdf", "hand")
            controller._current_q = [0.1] * 7
            art.position = (30, 20, 1)
            controller._track_target((30.5, 20.2, 1.1))
            events = controller._ik_backend._solver.events
            self.assertEqual(events[0], ("base", (30, 20, 1), (1, 0, 0, 0)))
            self.assertEqual(events[1][0:2], ("solve", (30.5, 20.2, 1.1)))
            np.testing.assert_array_equal(controller._ik_backend._solver.warm_start, [0.1] * 7)


class TestVisualizerBaseWiring(unittest.TestCase):
    def _visualizer(self, mode, art):
        viz = WorkshopVisualizer(InMemoryBus(), WorkshopVizConfig(
            use_real_robot=True, robot_name="franka_panda", motion_mode=mode,
            enable_sparks=False))
        viz.register_cell("WELD", "welding", (25.1, 10.75, 0.2))
        offsets = []
        viz.scene = SimpleNamespace(
            add_real_robot_arm=lambda cell_id, **kw: offsets.append(kw["offset"]) or art,
            get_cell_path=lambda cell_id: "/Cell",
        )
        return viz, offsets

    def test_all_modes_receive_registered_world_base_and_catalog_height(self):
        spec = replace(ROBOT_CATALOG["franka_panda"], base_offset_z=0.1)
        for mode in ("auto", "rmpflow", "heuristic", "ik"):
            with self.subTest(mode=mode), patch.dict(ROBOT_CATALOG, franka_panda=spec):
                viz, offsets = self._visualizer(mode, object())
                viz._spawn_robot_arm("WELD")
                controller = viz._robots["WELD"]["ik" if mode == "ik" else "rmp"]
                self.assertIsNotNone(controller)
                np.testing.assert_allclose(controller.base_pose.position, (25.1, 10.45, 1.15))
                np.testing.assert_allclose(offsets, [(0, -0.3, 0.85)])

    def test_live_articulation_overrides_registered_coordinates(self):
        art = _Articulation((42, 16, 1.2), (math.sqrt(0.5), 0, 0, math.sqrt(0.5)))
        for mode in ("heuristic", "ik"):
            with self.subTest(mode=mode):
                viz, _ = self._visualizer(mode, art)
                viz._spawn_robot_arm("WELD")
                controller = viz._robots["WELD"]["ik" if mode == "ik" else "rmp"]
                self.assertEqual(controller.base_pose.position, art.position)
                np.testing.assert_allclose(controller.base_pose.orientation, art.orientation)

    def test_rmp_setup_failure_preserves_base_in_ik_fallback(self):
        viz, _ = self._visualizer("auto", object())
        with patch.object(rmp, "RMPflowController", side_effect=RuntimeError("stub setup failure")):
            viz._spawn_robot_arm("WELD")
        np.testing.assert_allclose(viz._robots["WELD"]["ik"].base_pose.position, (25.1, 10.45, 1.05))

    def test_real_visualizer_builder_and_loader_compose_parent_once(self):
        # Exercise actual production wiring and local USD authoring. Stub only
        # Isaac/USD services and the asset lookup; do not mock the call chain.
        for yaw in (0, math.pi / 2):
            with self.subTest(parent_yaw=yaw):
                stage = _Stage()
                stage.define("/Cell", (25.1, 10.75, 0.2), yaw)
                modules = _usd_modules(stage)
                art = SimpleNamespace(stage=stage, prim_path="/Cell/RobotArm")
                viz, _ = self._visualizer("heuristic", art)
                scene = object.__new__(SceneBuilder)
                scene._stage = stage
                scene._UsdGeom = modules["pxr"].UsdGeom
                scene._cell_prims = {"WELD": "/Cell"}
                viz.scene = scene
                spec = replace(ROBOT_CATALOG["franka_panda"], base_offset_z=0.1)
                with patch.dict(sys.modules, modules), \
                        patch.dict(ROBOT_CATALOG, franka_panda=spec), \
                        patch.object(RobotLoader, "_wrap_articulation", return_value=art), \
                        patch("fdw_sim.visualization.robot_loader.resolve_robot_usd_path",
                              return_value="/fake/robot.usd"):
                    viz._spawn_robot_arm("WELD")
                    controller = viz._robots["WELD"]["rmp"]
                    self.assertIsNotNone(controller)
                    np.testing.assert_allclose(stage.prims["/Cell/RobotArm"].position,
                                               (0, -0.3, 0.95))
                    np.testing.assert_allclose(controller.base_pose.position,
                                               (25.1 + 0.3 * math.sin(yaw),
                                                10.75 - 0.3 * math.cos(yaw), 1.15))
                    np.testing.assert_allclose(controller.base_pose.orientation,
                                               (math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)))
                    # USD remains authoritative after a parent moves.
                    stage.prims["/Cell"].position = (30, 20, 0.2)
                    controller._ensure_backend()
                    controller._track_target((30.5, 20, 1.2))
                    np.testing.assert_allclose(controller.base_pose.position,
                                               (30 + 0.3 * math.sin(yaw),
                                                20 - 0.3 * math.cos(yaw), 1.15))


if __name__ == "__main__":
    unittest.main(verbosity=2)
