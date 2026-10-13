"""Natural surrogate outcomes through the complete CPU transport pipeline."""
from copy import deepcopy
from pathlib import Path
import random
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.run_poc1 import build_simulation, load_config, submit_jobs, validate_dispositions
from fdw_sim.cells.base.cell_base import CellConfig
from fdw_sim.cells.inspection.inspection_cell import InspectionCell
from fdw_sim.messaging.bus import InMemoryBus
from fdw_sim.messaging.schemas import CellType


@pytest.fixture
def scenario(tmp_path):
    cfg = load_config(ROOT / "fdw_sim/config/poc1_quality_disposition.yaml")
    cfg["simulation"]["log_dir"] = str(tmp_path)
    sim = build_simulation(cfg, mode_override="discrete")
    sim.start()
    try:
        submit_jobs(sim, cfg)
        sim.run_until_done(extra_idle_sec=0.2)
        yield sim, cfg
    finally:
        sim.stop()


def test_real_recipes_ship_only_pass_and_keep_exact_dispositions(scenario, capsys):
    sim, cfg = scenario
    validate_dispositions(sim, cfg)
    assert "[VALIDATION] PASSED" in capsys.readouterr().out
    assert [tr.job.part_id for tr in sim.orchestrator.completed_jobs] == ["PART_DEMO_PASS"]
    assert {tr.job.part_id: tr.disposition_reason for tr in sim.orchestrator.quarantined_jobs} == {
        "PART_DEMO_FAIL": "FAIL", "PART_DEMO_REWORK": "REWORK"}
    insp = sim.cells["INSPECTION_CELL_01"]
    assert (insp.total_pass, insp.total_fail, insp.total_rework) == (1, 1, 1)
    summary = sim.kpi.summary()
    assert summary["amr_delivery_count"]["count"] == 6
    assert summary["job_makespan_sec"]["count"] == 1
    assert summary["quality_pass_rate"]["last"] == pytest.approx(1 / 3)


@pytest.mark.parametrize("seed", [1, 999])
def test_scenario_outcomes_do_not_depend_on_random_state(tmp_path, seed):
    random_state = random.getstate()
    random.seed(seed)
    cfg = load_config(ROOT / "fdw_sim/config/poc1_quality_disposition.yaml")
    cfg["simulation"]["log_dir"] = str(tmp_path)
    sim = build_simulation(cfg, mode_override="discrete")
    try:
        sim.start()
        submit_jobs(sim, cfg)
        sim.run_until_done(extra_idle_sec=0.2)
        validate_dispositions(sim, cfg)
    finally:
        sim.stop()
        random.setstate(random_state)


@pytest.mark.parametrize("change", ["verdict", "identity", "history", "active"])
def test_validation_rejects_wrong_or_incomplete_results(scenario, change):
    sim, cfg = scenario
    cfg = deepcopy(cfg)
    if change == "verdict":
        cfg["validation"]["expected_dispositions"]["JOB_DEMO_PASS"] = "FAIL"
    elif change == "identity":
        cfg["jobs"][0]["part_id"] = "WRONG_PART"
    elif change == "history":
        sim.orchestrator.completed_jobs[0].history = ["welding"]
    else:
        sim.orchestrator.active_jobs["UNFINISHED"] = object()
    with pytest.raises(RuntimeError, match="VALIDATION.*FAILED"):
        validate_dispositions(sim, cfg)


@pytest.mark.parametrize("noise", [-0.01, float("nan"), float("inf")])
def test_invalid_measurement_noise_is_rejected(noise):
    with pytest.raises(ValueError, match="measurement_noise_std"):
        InspectionCell(CellConfig("INSP", CellType.INSPECTION), InMemoryBus(), measurement_noise_std=noise)


def test_default_inspection_noise_is_preserved():
    inspection = InspectionCell(CellConfig("INSP", CellType.INSPECTION), InMemoryBus())
    assert inspection.measurement_noise_std == 0.03
