"""CPU integration of rendered poses, transport custody and process gates.

Fake scene/controller objects are not an Isaac or physical grasp test.
"""
from pathlib import Path
import math
import sys
from unittest.mock import patch

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fdw_sim.cells.base.cell_base import CellConfig
from fdw_sim.cells.material.material_cell import MaterialCell
from fdw_sim.cells.welding.welding_cell import WeldingCell
from fdw_sim.cells.inspection.inspection_cell import InspectionCell
from fdw_sim.messaging.bus import InMemoryBus, Topics
from fdw_sim.messaging.schemas import CellState, CellType, JobSpec, QualityPrediction
from fdw_sim.orchestrator.orchestrator import OrchestratorConfig
from fdw_sim.simulation.manager import SimulationManager, SimulationConfig
from fdw_sim.visualization.workshop_visualizer import WorkshopVisualizer, WorkshopVizConfig


class Scene:
    def __init__(self):
        self.amrs, self.parts = {}, {}
    def move_amr(self, amr_id, pos, heading=0):
        self.amrs[amr_id] = (pos, heading)
    def move_part(self, part_id, pos):
        self.parts[part_id] = pos
    def add_part(self, part_id, **kwargs):
        self.parts[part_id] = kwargs.get('position')


class UnsupportedController:
    def update(self, dt):
        pass
    def get_capabilities(self):
        return {'backend': 'heuristic', 'supports_physical_manipulation': False}
    def get_phase(self):
        return 'weld'  # Must never light sparks without a welding-purpose context.
    def get_measured_tcp_position(self):
        return None


class Sparks:
    def __init__(self):
        self.active = []
    def set_active(self, active):
        self.active.append(active)
    def set_tcp_position(self, tcp):
        pass
    def update(self, dt):
        pass


def setup(tmp_path, mode='schematic', robot=True):
    bus = InMemoryBus()
    sim = SimulationManager(SimulationConfig(mode='discrete', physics_hz=20,
        max_sim_time_sec=240, log_dir=tmp_path), bus=bus,
        orchestrator_config=OrchestratorConfig(material_cell_id='MAT',
            process_to_cell={'welding': 'WELD', 'inspection': 'INSP'}))
    mat = MaterialCell(CellConfig('MAT', CellType.MATERIAL), bus, num_amrs=2)
    weld = WeldingCell(CellConfig('WELD', CellType.WELDING), bus)
    insp = InspectionCell(CellConfig('INSP', CellType.INSPECTION), bus, default_cycle_time=.1)
    sim.register_material_cell(mat, (0, 0))
    sim.register_cell(weld, (4, 0))
    sim.register_cell(insp, (8, 0))
    viz = WorkshopVisualizer(bus, WorkshopVizConfig(use_real_robot=robot, motion_execution=mode))
    for cid, cell in sim.cells.items():
        viz.register_cell(cid, cell.cell_type.value, sim._cell_locations[cid])
    viz.attach_material_cell(mat)
    viz.scene = Scene()
    sparks = Sparks()
    viz._robots['WELD'] = {'rmp': UnsupportedController(), 'sparks': sparks}
    sim.visualizer = viz
    sim.start()
    return sim, mat, weld, viz, sparks


def test_schematic_two_amr_quality_pipeline_and_milestone_order(tmp_path):
    sim, mat, weld, viz, sparks = setup(tmp_path)
    timeline = {}
    original_place = mat.confirm_pickup
    def placed(amr_id, part, transfer):
        accepted = original_place(amr_id, part, transfer)
        if accepted and part not in timeline:
            timeline[part] = {'placement': sim.sim_time}
        return accepted
    mat.confirm_pickup = placed
    original_motion = weld.confirm_process_motion
    def motion(command, part, success=True):
        result = original_motion(command, part, success)
        if result and success:
            timeline[part]['motion_done'] = sim.sim_time
        return result
    weld.confirm_process_motion = motion
    sim.bus.subscribe(Topics.DISPATCH_COMMAND, lambda cmd:
        timeline[cmd.part_id].update(start=sim.sim_time) if cmd.target_cell == 'WELD' else None)
    weld.quality_agent.predict = lambda **kw: QualityPrediction(
        1.0 if weld.input_buffer.part_id == 'GOOD' else .1, 0, .9)
    previous = {amr.amr_id: amr.position for amr in mat.amrs}
    output_times = {}
    violations = []
    def observe(dt, now):
        for amr in mat.amrs:
            if math.dist(previous[amr.amr_id], amr.position) > amr.speed_mps * dt + 1e-8:
                violations.append("speed/teleport")
            if viz.scene.amrs[amr.amr_id][0][:2] != amr.position:
                violations.append("render pose")
            if viz.scene.amrs[amr.amr_id][1] != amr.heading:
                violations.append("render heading")
            previous[amr.amr_id] = amr.position
        if math.dist(*[a.position for a in mat.amrs]) < sum(a.footprint_radius for a in mat.amrs) + mat.clearance_m - 1e-8:
            violations.append("AMR clearance")
        if weld.output_buffer.occupied:
            output_times.setdefault(weld.output_buffer.part_id, now)
    sim.add_step_hook(observe)
    try:
        with patch('fdw_sim.cells.inspection.inspection_cell.random.gauss', return_value=0):
            for part in ('GOOD', 'BAD'):
                sim.submit_job(JobSpec('J'+part, part, 'test', ['welding', 'inspection']))
            sim.run_until_done(extra_idle_sec=.1)
        assert not sim.orchestrator.active_jobs
        assert sim._all_cells_quiescent()
        assert not violations
        assert {j.job.part_id for j in sim.orchestrator.completed_jobs} == {'GOOD'}
        assert {j.job.part_id for j in sim.orchestrator.quarantined_jobs} == {'BAD'}
        for part, t in timeline.items():
            assert t['placement'] <= t['start'] <= t['motion_done'] <= output_times[part]
        assert sparks.active and not any(sparks.active)
    finally:
        sim.stop()


def test_verified_missing_grasp_keeps_cargo_no_independent_part_flight(tmp_path):
    sim, mat, weld, viz, sparks = setup(tmp_path, 'verified')
    try:
        sim.submit_job(JobSpec('J', 'P', 'test', ['welding', 'inspection']))
        for _ in range(1600):
            sim.step()
        amr = next(a for a in mat.amrs if a.payload_part_id == 'P')
        assert amr.awaiting_pickup and amr.pickup_blocked
        assert amr.position == mat.dock_positions['WELD']
        assert viz.scene.parts['P'] == (*amr.position, viz.config.amr_deck_height)
        assert not weld.output_buffer.occupied and weld.fsm.state == CellState.FAULT
        assert len(sim.orchestrator.active_jobs) == 1
        assert not sim.orchestrator.completed_jobs
        assert not any(sparks.active)
        assert not mat.confirm_pickup(amr.amr_id, 'P', amr.transfer_command_id)
    finally:
        sim.stop()


def test_no_robot_explicit_logical_mode_completes_without_motion_dependency(tmp_path):
    sim, mat, weld, viz, sparks = setup(tmp_path, robot=False)
    try:
        assert not weld.motion_required
        sim.submit_job(JobSpec('J', 'P', 'test', ['welding', 'inspection']))
        sim.run_until_done(extra_idle_sec=.1)
        assert not sim.orchestrator.active_jobs
        assert sim._all_cells_quiescent()
        assert not viz._pick_context and not viz._weld_context
        assert not any(sparks.active)
    finally:
        sim.stop()


@pytest.mark.parametrize('extent', [(3.0, 2.0), None])
def test_scene_measures_large_loaded_amr_before_transport(tmp_path, monkeypatch, extent):
    sim, mat, weld, viz, sparks = setup(tmp_path)
    class BuildScene(Scene):
        def __init__(self, config):
            super().__init__()
        def __getattr__(self, name):
            return lambda *args, **kwargs: None
        def get_amr_footprint(self, amr_id, position):
            return extent
        def local_point_to_world_meters(self, point): return tuple(point)
        def get_cell_world_position(self, cid): return viz.cell_positions[cid]
        def get_amr_height(self, amr_id): return 1.0
        def get_static_navigation_map(self, **kwargs):
            from types import SimpleNamespace
            return SimpleNamespace(obstacles=(), bounds=(-20,20,-20,20),source='test-double',
                boundary_polygon=((-20,-20),(20,-20),(20,20),(-20,20)),floor_height_m=0.0)
    monkeypatch.setattr('fdw_sim.visualization.workshop_visualizer.SceneBuilder', BuildScene)
    monkeypatch.setattr(viz, '_spawn_robot_arm', lambda *a, **kw: None)
    viz.config.skip_auto_camera = True
    for amr in mat.amrs:
        viz.register_amr(amr.amr_id, (*amr.position, 0), heading=amr.heading)
    try:
        if extent is None:
            with pytest.raises(RuntimeError, match='footprint'):
                viz.build_scene()
            assert not mat._transport_started
        else:
            viz.build_scene()
            assert all(amr.footprint_size == extent for amr in mat.amrs)
            assert math.dist(mat.amrs[0].position, mat.amrs[1].position) >= math.hypot(*extent) + mat.clearance_m
            for amr in mat.amrs:
                assert viz.scene.amrs[amr.amr_id][0] == (*amr.position, 0)
    finally:
        sim.stop()


def test_mesh_footprint_accounts_for_offset_from_vehicle_origin():
    from pxr import Usd, UsdGeom, Gf
    from fdw_sim.visualization.scene_builder import SceneBuilder
    scene = SceneBuilder.__new__(SceneBuilder)
    scene._Usd, scene._UsdGeom, scene._Gf = Usd, UsdGeom, Gf
    scene._stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(scene._stage, 1.0)
    root = UsdGeom.Xform.Define(scene._stage, '/A')
    root.AddTranslateOp().Set((10,20,0))
    cube = UsdGeom.Cube.Define(scene._stage, '/A/body')
    cube.CreateSizeAttr(2)
    cube.AddTranslateOp().Set((-.5,0,.5))
    cube.AddScaleOp().Set((1.5,1,.5))
    scene._amr_prims = {'A':'/A'}
    assert scene.get_amr_footprint('A', (10,20,0)) == pytest.approx((4,2))


def test_measured_height_floor_and_oversized_payload_drive_runtime_map(tmp_path, monkeypatch):
    from types import SimpleNamespace
    sim, mat, weld, viz, sparks = setup(tmp_path)
    requested_heights = []
    class BuildScene(Scene):
        def __init__(self, config): super().__init__()
        def __getattr__(self, name): return lambda *args, **kwargs: None
        def get_amr_footprint(self, *args): return (1.2,.8)
        def get_amr_height(self, *args): return 4.2
        def local_point_to_world_meters(self, p): return (p[0],p[1],p[2]+5)
        def get_cell_world_position(self, cid):
            return self.local_point_to_world_meters(viz.cell_positions[cid])
        def get_static_navigation_map(self, **kwargs):
            requested_heights.append(kwargs['robot_height_m'])
            return SimpleNamespace(obstacles=(),bounds=(-20,20,-20,20),source='test-double',
                boundary_polygon=((-20,-20),(20,-20),(20,20),(-20,20)),floor_height_m=5.0)
        def get_part_footprint(self, part): return (1.2,.8)
        def get_part_height(self, part): return 1.1  # exceeds configured one metre
        def move_part_world(self, part, position): self.parts[part]=position
    monkeypatch.setattr('fdw_sim.visualization.workshop_visualizer.SceneBuilder',BuildScene)
    monkeypatch.setattr(viz,'_spawn_robot_arm',lambda *a,**kw:None)
    viz.config.skip_auto_camera=True
    for amr in mat.amrs: viz.register_amr(amr.amr_id,(*amr.position,0),heading=amr.heading)
    try:
        viz.build_scene()
        assert requested_heights==[4.2]
        viz._update_amr_positions(.1)
        assert all(pose[0][2]==5 for pose in viz.scene.amrs.values())
        viz.spawn_part('TALL','test','MAT')
        assert 'TALL' in mat.payload_geometry_issues
        mat.amrs[0].payload_part_id='TALL'
        viz._update_amr_positions(.1)
        assert viz.scene.parts['TALL'][2]==pytest.approx(5.28)
    finally: sim.stop()
