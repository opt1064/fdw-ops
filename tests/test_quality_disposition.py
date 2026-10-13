"""CPU-only quality custody and end-to-end shipment safety regressions."""
import sys
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest
from fdw_sim.cells.base.cell_base import CellConfig
from fdw_sim.cells.inspection.inspection_cell import InspectionCell, InspectionAgent
from fdw_sim.cells.material.material_cell import MaterialCell, CellLocation
from fdw_sim.cells.welding.welding_cell import WeldingCell
from fdw_sim.messaging.bus import InMemoryBus, Topics
from fdw_sim.messaging.schemas import CellType, CellState, JobSpec, QualityPrediction, MaterialTransferCommand
from fdw_sim.orchestrator.orchestrator import FDWOrchestrator, OrchestratorConfig
from fdw_sim.simulation.manager import SimulationConfig, SimulationManager


def make_cells():
    bus = InMemoryBus()
    mat = MaterialCell(CellConfig('MAT', CellType.MATERIAL), bus, num_amrs=1)
    weld = WeldingCell(CellConfig('WELD', CellType.WELDING), bus)
    insp = InspectionCell(CellConfig('INSP', CellType.INSPECTION), bus, default_cycle_time=.1)
    for cell in (mat, weld, insp):
        cell.boot()
    mat.register_cell(weld, CellLocation('WELD', (0, 0)))
    mat.register_cell(insp, CellLocation('INSP', (1, 0)))
    return bus, mat, weld, insp


def transfer(part='P'):
    return MaterialTransferCommand('WELD', 'INSP', part, 'AUTO')


def output(cell, part, quality):
    cell.output_buffer.occupied = True
    cell.output_buffer.part_id = part
    cell.quality = quality


def test_amr_quality_snapshot_survives_source_change_and_delivery_retry():
    _, mat, weld, insp = make_cells()
    q = QualityPrediction(.2, .8, .9)
    output(weld, 'P', q)
    mat._on_transfer_command(transfer())
    mat._dispatch_amrs()
    amr = mat.amrs[0]
    for _ in range(1000):
        mat._step_amr(amr, .05)
        if amr.phase == "to_destination":
            break
    assert amr.phase == "to_destination"
    assert amr.payload_quality == q and amr.payload_quality is not q
    q.score = 1.0
    insp.receive_part_with_quality('OTHER', QualityPrediction(.9, .1, .8))
    mat._step_amr(amr, 30.0)
    assert amr.busy and amr.payload_quality.score == .2
    assert insp._incoming_quality.score == .9
    insp.input_buffer.occupied = False
    insp.input_buffer.part_id = None
    carried = amr.payload_quality
    mat._step_amr(amr, 30.0)
    assert insp.input_buffer.part_id == 'P'
    assert insp._incoming_quality.score == .2
    assert insp._incoming_quality is not carried
    assert amr.payload_quality is None
    assert InspectionAgent().judge(insp._incoming_quality, 0)['verdict'] == 'FAIL'


def test_wrong_source_part_not_removed_or_mislabeled():
    _, mat, weld, _ = make_cells()
    output(weld, 'OTHER', QualityPrediction())
    mat._assign_amr(mat.amrs[0], transfer())
    assert weld.output_buffer.part_id == 'OTHER'
    assert weld.output_buffer.occupied
    assert mat.transfer_queue and not mat.amrs[0].busy


@pytest.mark.parametrize('quality', [None, QualityPrediction(float('nan')), QualityPrediction(1, 0, 0), QualityPrediction(None), QualityPrediction("bad")])
def test_missing_or_invalid_quality_never_passes(quality):
    assert InspectionAgent().judge(quality, 0)['verdict'] == 'FAIL'


def test_direct_receive_clears_previous_quality_and_rejected_receive_preserves_it():
    _, _, _, insp = make_cells()
    insp.receive_part_with_quality('FIRST', QualityPrediction(.1, .9, 1))
    assert not insp.receive_part_with_quality('REJECTED', QualityPrediction())
    assert insp._incoming_quality.score == .1
    insp.input_buffer.occupied = False
    insp.receive_part('NEXT')
    assert insp._incoming_quality is None
    insp.input_buffer.occupied = False
    insp.receive_part_with_quality('NEXT2', None)
    assert insp._incoming_quality is None


def run_flow(tmp_path, scores, route=None, inspection_hz=5.0):
    bus = InMemoryBus()
    events = {topic: [] for topic in (Topics.JOB_COMPLETED, Topics.JOB_QUARANTINED, Topics.KPI_LOG)}
    for topic, target in events.items():
        bus.subscribe(topic, target.append)
    orch = OrchestratorConfig(material_cell_id='MAT', process_to_cell={'welding': 'WELD', 'inspection': 'INSP'})
    sim = SimulationManager(SimulationConfig(mode='discrete', physics_hz=20, max_sim_time_sec=200, log_dir=tmp_path), bus=bus, orchestrator_config=orch)
    mat = MaterialCell(CellConfig('MAT', CellType.MATERIAL), bus, num_amrs=2)
    weld = WeldingCell(CellConfig('WELD', CellType.WELDING), bus)
    insp = InspectionCell(CellConfig('INSP', CellType.INSPECTION, status_publish_hz=inspection_hz), bus, default_cycle_time=.1)
    sim.register_material_cell(mat, (0, 0))
    sim.register_cell(weld, (1, 0))
    sim.register_cell(insp, (2, 0))
    sim.start()
    try:
        weld.quality_agent.predict = lambda **kw: QualityPrediction(scores[weld.input_buffer.part_id], 1-scores[weld.input_buffer.part_id], .9)
        with patch('fdw_sim.cells.inspection.inspection_cell.random.gauss', return_value=0):
            for part in scores:
                sim.submit_job(JobSpec('J'+part, part, 'test', route or ['welding', 'inspection']))
            sim.run_until_done(extra_idle_sec=.2)
        assert not sim.orchestrator.active_jobs
        assert sim._sim_time < sim.config.max_sim_time_sec
        assert sim._all_cells_quiescent()
    finally:
        sim.stop()
    return sim, events


@pytest.mark.parametrize('score,expected', [(1.0, 'PASS'), (.75, 'REWORK'), (.2, 'FAIL')])
def test_real_amr_pipeline_gates_each_verdict(tmp_path, score, expected):
    sim, events = run_flow(tmp_path, {'P': score})
    insp = sim.cells['INSP']
    assert insp.last_verdict == expected
    assert insp._incoming_quality.score == score
    assert len(sim.orchestrator.completed_jobs) == (expected == 'PASS')
    assert len(sim.orchestrator.quarantined_jobs) == (expected != 'PASS')
    assert len(events[Topics.JOB_COMPLETED]) == (expected == 'PASS')
    assert len(events[Topics.JOB_QUARANTINED]) == (expected != 'PASS')
    makespans = [k for k in events[Topics.KPI_LOG] if k.metric == 'job_makespan_sec']
    assert len(makespans) == (expected == 'PASS')
    if expected != 'PASS':
        tr = sim.orchestrator.quarantined_jobs[0]
        assert tr.disposition_reason == expected
        assert tr.history == ['welding', 'inspection']
        assert 'P' in sim.orchestrator.quarantined_parts


def test_mixed_concurrent_jobs_keep_identity_and_no_stale_quality(tmp_path):
    sim, events = run_flow(tmp_path, {'GOOD': 1, 'BAD': .1, 'REWORK': .75, 'GOOD2': 1})
    assert {t.job.part_id for t in sim.orchestrator.completed_jobs} == {'GOOD', 'GOOD2'}
    assert {t.job.part_id for t in sim.orchestrator.quarantined_jobs} == {'BAD', 'REWORK'}
    assert len(events[Topics.JOB_COMPLETED]) == 2
    assert len(events[Topics.JOB_QUARANTINED]) == 2


def test_rack_to_inspection_missing_quality_quarantines(tmp_path):
    sim, events = run_flow(tmp_path, {'P': 1}, route=['inspection'])
    assert not sim.orchestrator.completed_jobs
    assert events[Topics.JOB_QUARANTINED][0]['reason'] == 'missing_or_invalid_quality'


def test_nonterminal_inspection_failure_does_not_advance_or_reweld(tmp_path):
    sim, _ = run_flow(tmp_path, {'P': .1}, route=['welding', 'inspection', 'welding'])
    tr = sim.orchestrator.quarantined_jobs[0]
    assert tr.history == ['welding', 'inspection']
    assert tr.route_index == 1


def test_unsupported_rework_policy_is_rejected():
    with pytest.raises(ValueError, match='automatic rework is unsupported'):
        OrchestratorConfig(inspection_nonpass_disposition='rework')


def test_missing_or_mismatched_status_verdict_is_quarantined():
    bus, _, _, insp = make_cells()
    orch = FDWOrchestrator(bus, OrchestratorConfig(process_to_cell={'inspection': 'INSP'}))
    orch.register_cell(insp)
    from fdw_sim.orchestrator.orchestrator import JobTracker
    tracker = JobTracker(JobSpec('J', 'P', 'x', ['inspection']))
    orch.active_jobs['J'] = tracker
    output(insp, 'P', QualityPrediction())
    insp.last_verdict = 'PASS'
    insp.verdict_part_id = 'OTHER'
    insp.publish_status()
    orch._tick_decisions()
    assert not orch.completed_jobs
    assert orch.quarantined_jobs == [tracker]
    assert tracker.disposition_reason == 'missing_or_mismatched_verdict'


def test_final_takeout_requires_exact_part():
    bus, _, weld, _ = make_cells()
    orch = FDWOrchestrator(bus)
    orch.register_cell(weld)
    output(weld, 'OTHER', QualityPrediction())
    assert not orch._final_takeout('WELD', 'P')
    assert weld.output_buffer.occupied
    assert not orch._final_takeout('UNKNOWN', 'P')


def test_status_keeps_output_identity_and_verdict_atomic():
    bus, _, _, insp = make_cells()
    statuses = []
    bus.subscribe(Topics.CELL_STATUS, statuses.append)
    output(insp, 'FIRST', QualityPrediction(.2, .8, .9))
    insp.last_verdict = 'FAIL'
    insp.verdict_part_id = 'FIRST'
    insp.publish_status()
    output(insp, 'NEXT', QualityPrediction())
    insp.last_verdict = 'PASS'
    insp.verdict_part_id = 'NEXT'
    old = statuses[-1]
    assert old.output_buffer.part_id == old.verdict_part_id == 'FIRST'
    assert old.inspection_verdict == 'FAIL'
    assert old.quality_prediction.score == .2


def test_report_discloses_quarantine_separately(tmp_path, capsys):
    from scripts.run_poc1 import print_report
    sim, _ = run_flow(tmp_path, {'P': .1})
    print_report(sim)
    report = capsys.readouterr().out
    assert 'Quarantined jobs (not shipped): 1' in report
    assert 'reason=FAIL' in report
    assert 'Completed jobs       : 0' in report


def test_report_final_pass_rate_is_not_cumulative_snapshot_average(tmp_path, capsys):
    from scripts.run_poc1 import print_report
    sim, _ = run_flow(tmp_path, {'BAD': .1, 'GOOD': 1, 'BAD2': .1})
    print_report(sim)
    report = capsys.readouterr().out
    assert 'final_quality_pass_rate=0.333' in report
    assert 'not final pass rate' in report


def test_slow_inspection_status_does_not_mix_new_output_with_old_verdict(tmp_path):
    sim, _ = run_flow(tmp_path, {'GOOD': 1, 'BAD': .1, 'GOOD2': 1}, inspection_hz=.7)
    assert {t.job.part_id for t in sim.orchestrator.completed_jobs} == {'GOOD', 'GOOD2'}
    assert {t.job.part_id for t in sim.orchestrator.quarantined_jobs} == {'BAD'}


def test_missing_source_quality_arrives_as_unknown():
    _, mat, weld, insp = make_cells()
    output(weld, 'P', None)
    weld.publish_status()  # Missing quality must not break status publication either.
    mat._assign_amr(mat.amrs[0], transfer())
    mat._step_amr(mat.amrs[0], 30)
    assert insp.input_buffer.part_id == 'P'
    assert insp._incoming_quality is None
    assert insp.agent.judge(insp._incoming_quality)['verdict'] == 'FAIL'
