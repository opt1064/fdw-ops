"""Static layout, full carried envelope and fail-closed transport regressions."""
import math
import pytest
from fdw_sim.cells.material.material_cell import MaterialCell, CellLocation
from fdw_sim.cells.material.navigation_setup import select_transport_positions
from fdw_sim.cells.material.traffic import StaticObstacle, segment_is_clear
from fdw_sim.cells.base.cell_base import CellConfig
from fdw_sim.messaging.bus import InMemoryBus
from fdw_sim.messaging.schemas import CellType, MaterialTransferCommand


class Sink:
    def __init__(self, name): self.cell_id, self.received = name, []
    def receive_part(self, part): self.received.append(part); return True


def layout():
    return [StaticObstacle(f'MaterialRack_{i}',3.98,12.02,2.98+2*i,4.02+2*i) for i in range(3)] + [
        StaticObstacle('CantileverRack',14.425,17.875,1,3),
        StaticObstacle('PipeBundle_sibling',14.325,17.975,.9,3.1),
        StaticObstacle('Workbench_MAT',9.4,11.2,5.3,7.1),
        StaticObstacle('Workbench_WELD',24.2,26,9.85,11.65),
        StaticObstacle('Workbench_INSP',33.7,35.5,5.55,7.35),
        StaticObstacle('Wall_West',-.2,.2,-.2,12.6),
        StaticObstacle('Wall_East',35.9,36.3,-.2,12.6),
        StaticObstacle('Wall_North',-.2,36.3,12.2,12.6)]


def configured(load=(1.2,.8), offset=(0,0)):
    material=MaterialCell(CellConfig('MAT',CellType.MATERIAL),InMemoryBus(),num_amrs=2)
    centers={'MAT':(10.3,6.2),'WELD':(25.1,10.75),'INSP':(34.6,6.45)}
    for name in ('WELD','INSP'):
        material.register_cell(Sink(name),CellLocation(name,centers[name]))
    wanted={name:(p[0],p[1]-1.3) for name,p in centers.items()}
    radius=max(math.hypot(1.2,.8)/2,math.hypot(*(abs(offset[i])+load[i]/2 for i in range(2))))
    docks,bays=select_transport_positions(wanted,centers,layout(),(0,36.1,0,12.4),radius,.15,2)
    material.configure_transport_geometry(docks,parking_positions=bays,static_obstacles=layout(),
        navigation_bounds=(0,36.1,0,12.4),navigation_source='exact-fixture',
        payload_footprint_size=load,payload_offset=offset)
    return material


@pytest.mark.parametrize('load,offset',[((1.2,.8),(0,0)),((2.4,1.0),(.3,0))])
def test_exact_racks_loaded_envelope_two_amrs_repeated_jobs_clear(load,offset):
    material=configured(load,offset)
    assert material.dock_positions['MAT'] != (10.3,4.9)
    for i in range(4):
        part=f'P{i}'; material.stock_part(part,'test')
        material._on_transfer_command(MaterialTransferCommand('MAT','WELD' if i%2 else 'INSP',part,'AUTO'))
    seen=set()
    for tick in range(1500):
        for amr in material.amrs:
            points=[amr.position]+amr.waypoints
            for start,end in zip(points,points[1:]):
                assert segment_is_clear(start,end,material._obstacles(amr),bounds=material._route_bounds(amr))
            assert segment_is_clear(amr.position,amr.position,material._obstacles(amr),bounds=material._route_bounds(amr))
            if amr.busy: seen.add(amr.amr_id)
        material.step(.73,tick*.73)
        assert not any(a.phase=='blocked' for a in material.amrs), [a.blocked_reason for a in material.amrs]
        if not material.transfer_queue and not any(a.busy for a in material.amrs): break
    else: pytest.fail('static routing deadlocked, never treat timeout as success')
    assert seen=={'AMR_01','AMR_02'}
    assert sorted(sum([s.received for s in material.cell_registry.values()],[]))==['P0','P1','P2','P3']
    assert all(a.position==a.parking_position for a in material.amrs)


def test_conflicting_dock_and_spawn_are_rejected_without_pose_mutation():
    m=configured(); before=[a.position for a in m.amrs]
    with pytest.raises(ValueError,match='dock MAT.*MaterialRack_1'):
        m.configure_transport_geometry({'MAT':(10.3,4.9)},static_obstacles=layout())
    assert [a.position for a in m.amrs]==before
    with pytest.raises(ValueError,match='parking AMR_01.*MaterialRack_0'):
        m.configure_transport_geometry({'MAT':(13,6)},parking_positions=[(8,3.5),(2,2)],static_obstacles=layout())
    assert [a.position for a in m.amrs]==before


def test_full_barrier_no_route_holds_stock_pose_and_completion():
    m=MaterialCell(CellConfig('MAT',CellType.MATERIAL),InMemoryBus(),num_amrs=1)
    sink=Sink('DEST');m.register_cell(sink,CellLocation('DEST',(8,5)))
    m.configure_transport_geometry({'MAT':(2,5),'DEST':(8,5)},parking_positions=[(2,2)],
        static_obstacles=[StaticObstacle('solid_wall',4,6,0,10)],navigation_bounds=(0,10,0,10))
    m.stock_part('P','test');cmd=MaterialTransferCommand('MAT','DEST','P','AUTO');m._on_transfer_command(cmd)
    for i in range(20):m.step(1000,i*1000)
    a=m.amrs[0]
    assert a.phase=='blocked' and 'solid_wall' in a.blocked_reason
    assert a.position==(2,5) and a.payload_part_id=='P'
    assert sink.received==[] and cmd.command_id not in m._completed_commands
    assert m.aisle_owner==a.amr_id


def test_live_obstacle_before_rotation_stops_without_any_heading_or_pose_change():
    m=configured();m.stock_part('P','test');m._on_transfer_command(MaterialTransferCommand('MAT','WELD','P','AUTO'))
    m._dispatch_amrs();a=m.amrs[0];pose,heading=a.position,a.heading
    m.static_obstacles += (StaticObstacle('new_obstacle',pose[0]-.1,pose[0]+.1,pose[1]-.1,pose[1]+.1),)
    m._step_amr(a,1000)
    assert a.position==pose and a.heading==heading
    assert a.phase=='blocked' and 'new_obstacle' in a.blocked_reason
    assert 'P' in m.smart_rack


def test_invalid_map_and_oversized_part_failclosed():
    m=configured();m.stock_part('P','test');m.payload_geometry_issues['P']='oversized payload P'
    m._on_transfer_command(MaterialTransferCommand('MAT','WELD','P','AUTO'));before=m.amrs[0].position
    m.step(1000,0)
    assert m.amrs[0].position==before and m.amrs[0].phase=='blocked'
    assert 'oversized' in m.amrs[0].blocked_reason and 'P' in m.smart_rack


def test_runtime_rotated_floor_polygon_blocks_aabb_false_safe_waypoint():
    m=MaterialCell(CellConfig('MAT',CellType.MATERIAL),InMemoryBus(),num_amrs=1)
    m.register_cell(Sink('DEST'),CellLocation('DEST',(3,1)))
    m.configure_transport_geometry({'MAT':(0,0),'DEST':(3,1)},parking_positions=[(0,-4)],
        navigation_bounds=(-8,8,-8,8),navigation_boundary=((0,-8),(8,0),(0,8),(-8,0)))
    m.stock_part('P','test');m._on_transfer_command(MaterialTransferCommand('MAT','DEST','P','AUTO'))
    m._dispatch_amrs();a=m.amrs[0];pose,heading=a.position,a.heading
    a.waypoints=[(6,6)]  # Inside world AABB, outside transformed actual floor.
    m._step_amr(a,1000)
    assert a.phase=='blocked' and 'map_boundary' in a.blocked_reason
    assert (a.position,a.heading)==(pose,heading) and 'P' in m.smart_rack


def test_live_invalid_map_fails_with_diagnostic_without_arrival():
    m=configured();m.stock_part('P','test');m._on_transfer_command(MaterialTransferCommand('MAT','WELD','P','AUTO'))
    m._dispatch_amrs();a=m.amrs[0];pose=a.position
    m.static_obstacles=(None,)
    m._step_amr(a,1000)
    assert a.phase=='blocked' and 'invalid navigation map' in a.blocked_reason
    assert a.position==pose and a.last_delivered_command_id is None and 'P' in m.smart_rack


@pytest.mark.parametrize('phase',['to_source','to_destination','awaiting_pickup'])
def test_payload_geometry_failure_found_after_assignment_never_delivers_or_releases(phase):
    m=configured();m.require_pickup_confirmation=True
    m.stock_part('P','test');cmd=MaterialTransferCommand('MAT','WELD','P','AUTO');m._on_transfer_command(cmd)
    m._dispatch_amrs();a=m.amrs[0]
    for tick in range(2000):
        if a.phase==phase: break
        m.step(.1,tick*.1)
    else: pytest.fail('did not reach test phase')
    pose,heading=a.position,a.heading
    old_received=list(m.cell_registry['WELD'].received)
    m.payload_geometry_issues['P']='oversized payload P found after assignment'
    assert not m.confirm_pickup(a.amr_id,'P',cmd.command_id)
    m.step(1000,1000)
    assert a.phase=='blocked' and 'oversized' in a.blocked_reason
    assert (a.position,a.heading)==(pose,heading)
    assert m.cell_registry['WELD'].received==old_received
    assert cmd.command_id not in m._completed_commands
    assert m.aisle_owner==a.amr_id


def test_live_usd_mode_requires_positive_payload_validation_before_assignment():
    m=configured();m.require_payload_geometry_validation=True
    m.stock_part('P','test');cmd=MaterialTransferCommand('MAT','WELD','P','AUTO');m._on_transfer_command(cmd)
    pose=m.amrs[0].position;m.step(1000,0)
    assert m.amrs[0].position==pose and m.amrs[0].phase=='blocked'
    assert 'unmeasured payload P' in m.amrs[0].blocked_reason
    assert 'P' in m.smart_rack and cmd.command_id not in m._completed_commands
