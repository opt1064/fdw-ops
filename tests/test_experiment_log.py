"""Failure evidence survives diagnostic serialization problems and abrupt process exit."""
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from fdw_sim.visualization.experiment_log import ExperimentLog
from fdw_sim.visualization.gripper_motion_test import PrimaryPositionServo
from test_physical_grasp_test import fake_run


def test_numpy_grip_target_does_not_destroy_failed_trial_report(tmp_path):
    servo = PrimaryPositionServo(lambda: .2, lambda q: None, [0, .785], 1/120)
    servo.command(np.float64(.24))
    assert type(servo.last["drive_target_clipped"]) is bool
    assert type(servo.last["desired_rad"]) is float
    log = ExperimentLog(tmp_path)
    report = {"status": "FAILED", "errors": ["LIFT test failure"], "setup": servo.diagnostics()}
    log.save(report)
    saved = json.loads(log.output.read_text(encoding="utf-8"))
    assert saved["errors"] == ["LIFT test failure"]
    assert saved["setup"]["last_command"]["drive_target_clipped"] is False
    assert not log.output.with_suffix(".json.tmp").exists()


def test_bad_diagnostic_field_keeps_original_failure_in_fallback(tmp_path, capsys):
    log = ExperimentLog(tmp_path)
    with pytest.raises(TypeError):
        log.save({"status": "FAILED", "errors": ["object slipped in LIFT"], "bad": object()})
    saved = json.loads(log.output.read_text(encoding="utf-8"))
    assert saved["status"] == "FAILED" and saved["report_incomplete"]
    assert saved["errors"][0] == "object slipped in LIFT"
    assert not saved["physical_grasp_verified"] and not saved["transfer_verified"]
    assert "TypeError" in log.trace.read_text(encoding="utf-8")
    assert "result serialization" in capsys.readouterr().err


def test_nonfinite_report_cannot_be_published_as_passed(tmp_path):
    log = ExperimentLog(tmp_path)
    with pytest.raises(ValueError):
        log.save({"status": "PASSED", "errors": [], "value": float("nan")})
    saved = json.loads(log.output.read_text(encoding="utf-8"))
    assert saved["status"] == "FAILED" and not saved["target_placement_verified"]


def test_last_measurement_survives_os_exit_without_final_result(tmp_path):
    script = """
import os, sys
from fdw_sim.visualization.experiment_log import ExperimentLog
log=ExperimentLog(sys.argv[1])
log.event('PHASE_STARTED',phase='LIFT',status='IN_PROGRESS')
log.event('MEASUREMENT',phase='LIFT',status='IN_PROGRESS',feedback={'contacts':{'left':1.,'right':1.}})
os._exit(7)
"""
    result = subprocess.run([sys.executable, "-c", script, str(tmp_path)],
                            cwd=Path(__file__).resolve().parents[1], capture_output=True, timeout=20)
    assert result.returncode == 7, result.stderr
    events = list(tmp_path.glob("*.events.jsonl"))
    assert len(events) == 1
    rows = [json.loads(line) for line in events[0].read_text(encoding="utf-8").splitlines()]
    assert rows[-1]["phase"] == "LIFT" and rows[-1]["status"] == "IN_PROGRESS"
    assert not list(tmp_path.glob("*.json"))


def test_grasp_failure_is_printed_and_checkpointed_with_traceback(tmp_path, capsys):
    log = ExperimentLog(tmp_path)
    report, state = fake_run("slip", progress=log.event)
    log.save(report)
    rows = [json.loads(line) for line in log.events.read_text(encoding="utf-8").splitlines()]
    failure = next(row for row in rows if row["event"] == "GRASP_FAILED")
    assert "slipped" in failure["traceback"] and failure["status"] == "FAILED"
    assert any(row["event"] == "MEASUREMENT" and row["phase"] == "LIFT" for row in rows)
    assert "[PHYSICAL GRASP] FAILED:" in capsys.readouterr().out
    assert state["paused"] and report["passed_phases"] < 6
