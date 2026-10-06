"""Controller evidence tests with stubs; these do not validate Isaac physics."""
from __future__ import annotations

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fdw_sim.visualization.ik_controller import IKConfig, IKController, WeldingPath
from fdw_sim.visualization.rmpflow_controller import (
    RMPflowConfig, RMPflowController, _HeuristicBackend,
)


class _Articulation:
    def __init__(self, joints=None, tcp=None):
        self.joints = [0.25] * 7 if joints is None else joints
        self.tcp = tcp
        self.commands = []
        self.teleports = []
        self.fail_drive = False

    def get_joint_positions(self):
        return self.joints

    def get_link_world_pose(self, frame):
        if self.tcp is None:
            return None
        return self.tcp, (1, 0, 0, 0)

    def set_joint_position_targets(self, values):
        if self.fail_drive:
            raise RuntimeError("drive disconnected")
        self.commands.append(list(values))

    def set_joint_positions(self, values):
        self.teleports.append(list(values))
        raise AssertionError("controller must not teleport the articulation")


class _Backend:
    name = "ik"

    def __init__(self, result=None, fail=False):
        self.result = [1.0] * 7 if result is None else result
        self.fail = fail
        self.warm_start = None

    def set_robot_base_pose(self, pose):
        pass

    def _solve(self, warm_start_q):
        self.warm_start = warm_start_q
        if self.fail:
            raise RuntimeError("solver disconnected")
        return self.result

    def set_target(self, target, warm_start_q=None, obstacles=None):
        return self._solve(warm_start_q)

    def compute(self, target, warm_start_q=None):
        return self._solve(warm_start_q)


class TestMotionTruth(unittest.TestCase):
    goal = (1.0, 2.0, 3.0)

    def controller(self, kind, art=None, backend=None, home=None):
        spec = SimpleNamespace(name="test_robot", end_effector_frame="tool",
                               home_joint_positions=[0.0] * 7 if home is None else home)
        if kind == "rmp":
            ctrl = RMPflowController(art, spec, config=RMPflowConfig(preferred_backend="heuristic"))
            if backend is not None:
                ctrl._backend, ctrl._backend_ready = backend, True
        else:
            ctrl = IKController(art, spec, config=IKConfig(use_lula=False))
            ctrl._ik_backend = backend
        return ctrl

    def start(self, ctrl):
        if isinstance(ctrl, RMPflowController):
            ctrl.start_path(self.goal, self.goal, travel_time_sec=0.1, approach_height=0)
        else:
            ctrl.start_path(WeldingPath(self.goal, self.goal, travel_time_sec=0.1, approach_height=0))

    def test_first_joint_command_blends_from_measurement_without_teleport(self):
        for kind in ("rmp", "ik"):
            with self.subTest(kind=kind):
                art = _Articulation()
                ctrl = self.controller(kind, art, _Backend())
                self.assertEqual(ctrl._current_q, [0.25] * 7)
                self.assertTrue(ctrl._set_target_joints([1.0] * 7, 1.0))
                ctrl.update(0.1)
                self.assertTrue(all(0.25 < q < 1.0 for q in art.commands[0]))
                self.assertEqual(art.teleports, [])
                self.assertEqual(art.joints, [0.25] * 7)

    def test_path_start_reseeds_from_current_measurement(self):
        for kind in ("rmp", "ik"):
            art = _Articulation()
            ctrl = self.controller(kind, art, _Backend())
            art.joints = [0.6] * 7
            self.start(ctrl)
            self.assertEqual(ctrl._current_q, [0.6] * 7)
            ctrl.update(0.05)
            self.assertTrue(all(0.6 < q < 1.0 for q in art.commands[-1]))

    def test_planned_tcp_never_becomes_measured_reach(self):
        for kind in ("rmp", "ik"):
            ctrl = self.controller(kind, None, _Backend())
            self.start(ctrl)
            ctrl.update(0.1)
            self.assertEqual(ctrl.get_commanded_tcp_position(), self.goal)
            self.assertEqual(ctrl.get_tcp_position(), self.goal)  # legacy display only
            self.assertIsNone(ctrl.get_measured_tcp_position())
            self.assertFalse(ctrl.has_reached_target(self.goal))
            self.assertFalse(ctrl.motion_succeeded())
            self.assertEqual(ctrl.get_motion_failure(), "measured_initial_joints_unavailable")

    def test_measured_tcp_is_fresh_and_rejects_nonfinite_feedback(self):
        for kind in ("rmp", "ik"):
            art = _Articulation(tcp=self.goal)
            ctrl = self.controller(kind, art, _Backend())
            self.assertTrue(ctrl.has_reached_target(self.goal))
            art.tcp = None
            self.assertIsNone(ctrl.get_measured_tcp_position())
            self.assertFalse(ctrl.has_reached_target(self.goal))
            art.tcp = (float("nan"), 0, 0)
            self.assertIsNone(ctrl.get_measured_tcp_position())
            self.assertFalse(ctrl.has_reached_target(self.goal))

    def test_solver_failure_is_terminal_and_never_replaced_by_heuristic(self):
        for kind in ("rmp", "ik"):
            for fail in (False, True):
                with self.subTest(kind=kind, exception=fail):
                    art = _Articulation(tcp=self.goal)
                    backend = _Backend(fail=fail)
                    backend.result = None
                    ctrl = self.controller(kind, art, backend)
                    self.start(ctrl)
                    ctrl.update(0.1)
                    self.assertEqual(ctrl.get_motion_status(), "failed")
                    self.assertFalse(ctrl.motion_succeeded())
                    self.assertTrue(ctrl.is_idle())
                    self.assertEqual(ctrl.get_phase(), "failed")
                    self.assertEqual(art.commands, [[0.25] * 7])  # measured hold only

    def test_missing_tcp_cannot_succeed_after_the_path_timer_expires(self):
        for kind in ("rmp", "ik"):
            ctrl = self.controller(kind, _Articulation(), _Backend())
            self.start(ctrl)
            ctrl.update(1.2)
            self.assertTrue(ctrl.is_idle())
            self.assertEqual(ctrl.get_motion_status(), "unverified")
            self.assertEqual(ctrl.get_motion_failure(), "measured_tcp_unavailable")
            self.assertFalse(ctrl.motion_succeeded())

    def test_far_measured_tcp_times_out_instead_of_succeeding(self):
        for kind in ("rmp", "ik"):
            ctrl = self.controller(kind, _Articulation(tcp=(0, 0, 0)), _Backend())
            self.start(ctrl)
            ctrl.update(1.2)
            self.assertEqual(ctrl.get_motion_status(), "running")
            self.assertFalse(ctrl.is_idle())
            ctrl.update(1.0)
            self.assertEqual(ctrl.get_motion_status(), "running")
            ctrl.update(1.0)
            self.assertEqual(ctrl.get_motion_status(), "failed")
            self.assertEqual(ctrl.get_motion_failure(), "measured_tcp_did_not_reach_endpoint")

    def test_measured_endpoint_success_still_does_not_claim_grasp_or_collision_safety(self):
        for kind in ("rmp", "ik"):
            ctrl = self.controller(kind, _Articulation(tcp=self.goal), _Backend())
            self.start(ctrl)
            ctrl.update(1.2)
            self.assertTrue(ctrl.motion_succeeded())
            self.assertTrue(ctrl.is_idle())
            capabilities = ctrl.get_capabilities()
            self.assertTrue(capabilities["cartesian_target_solver"])
            self.assertFalse(capabilities["supports_physical_manipulation"])
            self.assertFalse(capabilities["verified_collision_safety"])
            self.assertFalse(ctrl.supports_physical_manipulation)

    def test_heuristic_is_unverified_even_if_measured_tcp_coincides(self):
        for kind in ("rmp", "ik"):
            ctrl = self.controller(kind, _Articulation(tcp=self.goal))
            self.start(ctrl)
            ctrl.update(1.2)
            self.assertEqual(ctrl.get_motion_status(), "unverified")
            self.assertFalse(ctrl.motion_succeeded())
            self.assertTrue(ctrl.get_capabilities()["unverified_demo"])

    def test_no_generic_fanuc_home_is_invented(self):
        spec = SimpleNamespace(name="fanuc_crx10ia", home_joint_positions=[])
        backend = _HeuristicBackend(spec, RMPflowConfig())
        self.assertEqual(backend._home_q, [])
        self.assertIsNone(backend.set_target(self.goal))
        backend.reset([0.2] * 6)
        self.assertEqual(backend._home_q, [0.2] * 6)
        for kind in ("rmp", "ik"):
            ctrl = self.controller(kind, _Articulation(), home=[])
            ctrl.go_home()
            self.assertEqual(ctrl.get_motion_status(), "failed")
            self.assertEqual(ctrl.get_motion_failure(), "validated_home_configuration_unavailable")
            self.assertFalse(ctrl.motion_succeeded())

    def test_trajectory_callback_exception_stops_motion(self):
        class BrokenPath:
            phase = "weld"

            def update(self, dt):
                raise RuntimeError("bad waypoint")

        for kind in ("rmp", "ik"):
            ctrl = self.controller(kind, _Articulation(), _Backend())
            ctrl._begin_motion()
            ctrl._active_path = BrokenPath()
            ctrl.update(0.1)
            self.assertEqual(ctrl.get_motion_status(), "failed")
            self.assertIn("bad waypoint", ctrl.get_motion_failure())
            self.assertFalse(ctrl.motion_succeeded())

    def test_drive_failure_and_unknown_joint_mapping_do_not_succeed(self):
        for kind in ("rmp", "ik"):
            for mismatch in (False, True):
                with self.subTest(kind=kind, mismatch=mismatch):
                    art = _Articulation(tcp=self.goal)
                    art.fail_drive = not mismatch
                    ctrl = self.controller(kind, art, _Backend(result=[1.0] * (6 if mismatch else 7)))
                    self.start(ctrl)
                    ctrl.update(1.2)
                    self.assertEqual(ctrl.get_motion_status(), "failed")
                    self.assertFalse(ctrl.motion_succeeded())
                    self.assertEqual(art.teleports, [])

    def test_new_path_can_recover_after_an_explicit_failed_attempt(self):
        for kind in ("rmp", "ik"):
            art, backend = _Articulation(tcp=self.goal), _Backend(fail=True)
            ctrl = self.controller(kind, art, backend)
            self.start(ctrl)
            ctrl.update(0.1)
            self.assertEqual(ctrl.get_motion_status(), "failed")
            backend.fail = False
            self.start(ctrl)
            ctrl.update(1.2)
            self.assertTrue(ctrl.motion_succeeded())
            self.assertIsNone(ctrl.get_motion_failure())


if __name__ == "__main__":
    unittest.main(verbosity=2)
