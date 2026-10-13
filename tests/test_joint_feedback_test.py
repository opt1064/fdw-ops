"""Control-loop regression tests; doubles do not validate Isaac motor dynamics."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fdw_sim.visualization.joint_feedback_test import run_joint_test, test_live_robot_joints as live_test


def pose(q):
    # Independent measured geometry with a 1 m lever arm.
    matrix = np.eye(4)
    c, s = math.cos(q[0]), math.sin(q[0])
    matrix[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    matrix[:3, 3] = [c, s, q[1]]
    return matrix


class Plant:
    dof_names = ["J1", "J2"]

    def __init__(self):
        self.q = np.array([0.2, -0.1])
        self.target = self.q.copy()
        self.commands = []
        self.steps = 0
        self.paused = False
        self.stuck = False

    def get_joint_positions(self):
        return self.q.copy()

    def set_joint_positions(self, q):
        raise AssertionError("teleport must never be used")

    def set_joint_position_targets(self, q):
        self.target = q.copy()
        self.commands.append(q.copy())

    def step(self):
        self.steps += 1
        if not self.stuck:
            self.q += 0.6 * (self.target - self.q)

    def stop(self):
        self.paused = True


def run(plant, **overrides):
    kwargs = dict(articulation=plant, names=plant.dof_names,
                  limits={"J1": (-2, 2), "J2": (-2, 2)}, step=plant.step,
                  measured_pose=lambda: pose(plant.q), predicted_pose=pose,
                  stop=plant.stop, dt=0.05)
    kwargs.update(overrides)
    return run_joint_test(**kwargs)


def test_drives_track_and_return_with_measured_leg_evidence():
    plant = Plant()
    baseline = plant.q.copy()
    result = run(plant)
    assert result["status"] == "PASSED_SIMULATED_JOINT_FEEDBACK"
    assert result["passed_legs"] == result["planned_legs"] == 8
    assert result["returned_to_start"] and not plant.paused
    assert np.max(np.abs(plant.q-baseline)) < math.radians(0.1)
    assert all(row["final_pose_error"]["position_error_m"] == 0 for row in result["legs"])
    assert not result["collision_verified"] and not result["physical_grasp_verified"]
    assert np.max(np.abs(np.array(plant.commands)-baseline)) <= math.radians(1) + 1e-10


def test_stuck_feedback_cannot_pass_and_holds_actual_position():
    plant = Plant()
    plant.stuck = True
    result = run(plant)
    assert result["status"] == "FAILED"
    assert "timed out" in result["errors"][0]
    assert result["passed_legs"] == 0 and plant.paused
    np.testing.assert_array_equal(plant.commands[-1], plant.q)


def test_stale_end_pose_detected_despite_correct_joint_feedback():
    plant = Plant()
    initial = pose(plant.q)
    result = run(plant, measured_pose=lambda: initial)
    assert "during movement" in result["errors"][0]
    assert plant.paused and result["passed_legs"] == 0


@pytest.mark.parametrize("kind", ["limits", "mapping", "initial_pose", "nonfinite_pose", "missing_feedback", "no_target_api", "timestep"])
def test_preflight_blocks_all_motor_commands(kind):
    plant = Plant()
    kwargs = {}
    if kind == "limits":
        kwargs["limits"] = {"J1": (-2, 2), "J2": (-2, -0.095)}
    elif kind == "mapping":
        kwargs["names"] = ["J2", "J1"]
    elif kind == "initial_pose":
        kwargs["measured_pose"] = lambda: pose(plant.q + 0.1)
    elif kind == "nonfinite_pose":
        kwargs["measured_pose"] = lambda: np.full((4, 4), np.nan)
    elif kind == "missing_feedback":
        plant.get_joint_positions = lambda: None
    elif kind == "no_target_api":
        plant.set_joint_position_targets = None
    elif kind == "timestep":
        kwargs["dt"] = float("nan")
    result = run(plant, **kwargs)
    assert result["status"] == "FAILED"
    assert not result["motion_commands_issued"] and not plant.commands and plant.steps == 0
    assert plant.paused


def test_fault_holds_measured_position_without_automatic_return():
    plant = Plant()
    baseline = plant.q.copy()
    result = run(plant, is_running=lambda: plant.steps < 10)
    assert result["status"] == "FAILED" and plant.paused
    assert not np.array_equal(plant.q, baseline)
    np.testing.assert_array_equal(plant.commands[-1], plant.q)


def test_command_exception_still_attempts_hold_and_pause():
    plant = Plant()
    def broken(q):
        raise RuntimeError("target API failed")
    plant.set_joint_position_targets = broken
    result = run(plant)
    assert result["motion_commands_issued"] and plant.paused
    assert any("hold failed" in err for err in result["errors"])


def test_excursion_aborts_before_next_command():
    plant = Plant()
    def runaway():
        plant.q[0] += math.radians(3)
    result = run(plant, step=runaway)
    assert "envelope" in result["errors"][0] and plant.paused


def test_pause_failure_is_preserved_in_report():
    plant = Plant()
    def broken_pause():
        raise RuntimeError("pause unavailable")
    result = run(plant, is_running=lambda: False, stop=broken_pause)
    assert any("pause failed" in err for err in result["errors"])
    assert result["status"] == "FAILED"


def test_failed_model_validation_writes_report_without_requiring_urdf(tmp_path):
    plant = Plant()
    sim = SimpleNamespace(_isaac_world=SimpleNamespace(pause=plant.stop))
    result, path = live_test(sim, None, "/Robot", plant, {"status": "FAILED"}, output_dir=tmp_path)
    assert result["status"] == "FAILED" and plant.paused
    assert path.exists() and not plant.commands
    assert "must pass" in result["errors"][0]


def test_single_articulation_action_api_keeps_zero_targets_and_explicit_indices():
    plant = Plant()
    plant.q[:] = 0
    plant.set_joint_position_targets = None
    actions = []
    def apply(action):
        actions.append(action)
        plant.target = action.joint_positions.copy()
    plant.apply_action = apply
    result = run(plant, action_factory=SimpleNamespace)
    assert result["status"] == "PASSED_SIMULATED_JOINT_FEEDBACK"
    assert result["target_api"].startswith("apply_action")
    assert all(np.array_equal(a.joint_indices, [0, 1]) for a in actions)
    np.testing.assert_array_equal(actions[-1].joint_positions, [0, 0])


def test_six_joints_complete_twenty_four_legs():
    plant = Plant()
    plant.dof_names = [f"J{i}" for i in range(1, 7)]
    plant.q = np.zeros(6)
    plant.target = plant.q.copy()
    result = run(plant, limits={n: (-2, 2) for n in plant.dof_names})
    assert result["status"] == "PASSED_SIMULATED_JOINT_FEEDBACK"
    assert result["passed_legs"] == result["planned_legs"] == 24


def test_feedback_loss_after_motion_pauses_even_when_hold_is_unavailable():
    plant = Plant()
    plant.get_joint_positions = lambda: plant.q.copy() if plant.steps == 0 else None
    result = run(plant)
    assert result["status"] == "FAILED" and plant.paused
    assert "feedback missing" in result["errors"][0]
    assert any("hold failed" in err for err in result["errors"])


@pytest.mark.parametrize("args", [
    ["--joint-feedback-test"],
    ["--joint-feedback-test", "--validate-lula-model"],
    ["--joint-feedback-test", "--validate-lula-model", "--export-robot-model"],
    ["--joint-feedback-test", "--validate-lula-model", "--export-robot-model",
     "--robot-diagnostics", "--real-robot", "--robot", "ur10"],
])
def test_cli_rejects_incomplete_or_wrong_robot_flags_before_boot(monkeypatch, args):
    from scripts import run_poc1_level2_2 as cli
    def forbidden(*a, **kw):
        raise AssertionError("invalid CLI must not boot or load scene")
    monkeypatch.setattr(cli, "load_config", forbidden)
    monkeypatch.setattr(sys, "argv", ["run_poc1_level2_2.py", *args])
    with pytest.raises(SystemExit) as exc:
        cli.main()
    assert exc.value.code == 2
