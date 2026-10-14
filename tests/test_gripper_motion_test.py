"""Control-loop and composed USD regression checks, not actual Isaac validation."""
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fdw_sim.visualization.gripper_motion_test import (
    run_gripper_cycle, inspect_motion_prerequisites, articulation_root_paths,
    bind_live_gripper, compose_gripper_experiment, require_physics_scene,
    ensure_physx_attachment,
    NativeGripperArticulation,
    ManualPhysxStepper,
    PrimaryPositionServo,
)


class Plant:
    def __init__(self):
        self.q = 0.0
        self.target = 0.0
        self.commands = []
        self.paused = False
        self.steps = 0
        self.stuck = False
        self.right_stuck = False

    def read(self):
        return self.q

    def command(self, target):
        self.target = target
        self.commands.append(target)

    def pads(self):
        inward = 0.08*self.q
        return [[-0.07+inward, 0, 0], [0.07 if self.right_stuck else 0.07-inward, 0, 0]]

    def step(self):
        self.steps += 1
        if not self.stuck:
            self.q += 0.6*(self.target-self.q)

    def stop(self):
        self.paused = True


def run(plant, **overrides):
    args = dict(read_joint=plant.read, command=plant.command, read_pads=plant.pads,
                step=plant.step, stop=plant.stop, limits=(0, math.pi/4), dt=0.05)
    args.update(overrides)
    return run_gripper_cycle(**args)


def test_bilateral_close_return_twice_has_independent_measured_evidence():
    plant = Plant()
    result = run(plant)
    assert result["status"] == "PASSED_UNLOADED_GRIPPER_CYCLE"
    assert result["passed_legs"] == 4 and result["returned_to_start"] and plant.paused
    assert all(v > 0.001 for row in result["legs"] if row["leg"] == "partial_close" for v in row["inward_displacements_m"])
    assert result["samples"] and not result["physical_grasp_verified"]
    assert max(plant.commands) == pytest.approx(math.radians(10))
    assert max(abs(np.diff(plant.commands))) <= math.radians(0.126)
    json.dumps(result, allow_nan=False)


def test_primary_joint_motion_alone_cannot_pass_when_right_finger_stays_still():
    plant = Plant()
    plant.right_stuck = True
    result = run(plant)
    assert result["status"] == "FAILED" and result["passed_legs"] == 0
    assert "both fingers" in result["errors"][0] and plant.paused
    assert plant.commands[-1] == plant.q


def test_frozen_joint_feedback_times_out_and_pauses():
    plant = Plant()
    plant.stuck = True
    result = run(plant)
    assert result["status"] == "FAILED" and plant.paused
    assert result["passed_legs"] == 0
    feedback = result["legs"][0]["last_feedback"]
    assert feedback["measured_rad"] == 0
    assert feedback["command_rad"] > math.radians(8)


def test_static_usd_geometry_cannot_pass_even_if_joint_feedback_tracks():
    plant = Plant()
    result = run(plant, read_pads=lambda: [[-.07, 0, 0], [.07, 0, 0]])
    assert "both fingers" in result["errors"][0] and result["status"] == "FAILED"


def test_interrupt_holds_measured_joint_without_automatic_return():
    plant = Plant()
    result = run(plant, is_running=lambda: plant.steps < 20)
    assert result["status"] == "FAILED" and plant.paused and plant.q > 0
    assert plant.commands[-1] == plant.q


@pytest.mark.parametrize("options", [
    {"limits": (0, math.radians(5))}, {"dt": float("nan")},
    {"read_joint": lambda: float("nan")}, {"read_pads": lambda: np.full((2, 3), np.nan)},
])
def test_invalid_preflight_sends_no_commands(options):
    plant = Plant()
    result = run(plant, **options)
    assert result["status"] == "FAILED" and not plant.commands and plant.paused


def test_constraint_roundoff_does_not_produce_out_of_limit_commands():
    plant = Plant()
    plant.q = -0.00001
    result = run(plant)
    assert result["status"] == "PASSED_UNLOADED_GRIPPER_CYCLE"
    assert result["open_target_rad"] == 0
    assert min(plant.commands) >= 0


def test_finger_geometry_stuck_closed_cannot_pass_return_leg():
    plant = Plant()
    def pads():
        q = plant.q if plant.steps < 115 else math.radians(10)
        return [[-0.07+0.08*q, 0, 0], [0.07-0.08*q, 0, 0]]
    result = run(plant, read_pads=pads)
    assert result["status"] == "FAILED" and result["passed_legs"] == 1
    assert "return" in result["errors"][0]


@pytest.fixture
def usd_model():
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/Robotiq_2F_140/robotiq_base_link")
    for side in ("left", "right"):
        UsdGeom.Cube.Define(stage, f"/Robotiq_2F_140/{side}_inner_finger/Fingertip_01")
    primary = UsdPhysics.RevoluteJoint.Define(stage, "/Robotiq_2F_140/finger_joint")
    primary.CreateLowerLimitAttr(0)
    primary.CreateUpperLimitAttr(45)
    UsdPhysics.DriveAPI.Apply(primary.GetPrim(), "angular").CreateStiffnessAttr(0.1125)
    follower = UsdPhysics.RevoluteJoint.Define(stage, "/Robotiq_2F_140/right_outer_knuckle_joint").GetPrim()
    follower.CreateRelationship("physxMimicJoint:rotX:referenceJoint").SetTargets([primary.GetPath()])
    follower.CreateAttribute("physxMimicJoint:rotX:gearing", Sdf.ValueTypeNames.Float).Set(-1)
    joint = UsdPhysics.FixedJoint.Define(stage, "/FixedJoint")
    joint.CreateBody1Rel().SetTargets(["/Robotiq_2F_140/robotiq_base_link"])
    return stage


def test_prerequisites_read_live_relationships_without_mutating_asset(usd_model):
    before = usd_model.GetRootLayer().ExportToString()
    result = inspect_motion_prerequisites(usd_model)
    assert result["mimic_gearing"] == -1
    assert result["limit_rad"] == pytest.approx([0, math.pi/4])
    assert len(result["pad_paths"]) == 2
    assert usd_model.GetRootLayer().ExportToString() == before


@pytest.mark.parametrize("fault", ["mimic", "gearing", "follower_drive", "anchor"])
def test_bad_mimic_or_mount_configuration_blocks_test(usd_model, fault):
    from pxr import UsdPhysics
    follower = usd_model.GetPrimAtPath("/Robotiq_2F_140/right_outer_knuckle_joint")
    if fault == "mimic":
        follower.GetRelationship("physxMimicJoint:rotX:referenceJoint").SetTargets([])
    elif fault == "gearing":
        follower.GetAttribute("physxMimicJoint:rotX:gearing").Set(1)
    elif fault == "follower_drive":
        UsdPhysics.DriveAPI.Apply(follower, "angular").CreateStiffnessAttr(1)
    else:
        usd_model.RemovePrim("/FixedJoint")
    with pytest.raises(ValueError):
        inspect_motion_prerequisites(usd_model)


def test_articulation_paths_are_retraversed_after_instance_proxy_expires():
    from pxr import Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/Template")
    UsdGeom.Scope.Define(stage, "/Template/Looks")
    instance = UsdGeom.Xform.Define(stage, "/Gripper").GetPrim()
    instance.GetReferences().AddInternalReference("/Template")
    instance.SetInstanceable(True)
    old_proxy = stage.GetPrimAtPath("/Gripper/Looks")
    assert old_proxy.IsInstanceProxy()
    UsdPhysics.ArticulationRootAPI.Apply(UsdPhysics.FixedJoint.Define(stage, "/FixedJoint").GetPrim())
    instance.SetInstanceable(False)
    with pytest.raises(RuntimeError, match="expired"):
        old_proxy.GetTypeName()
    assert articulation_root_paths(stage) == ["/FixedJoint"]
    assert stage.GetPrimAtPath("/Gripper/Looks").GetTypeName() == "Scope"


def test_binding_creates_fresh_tensor_view_only_after_final_play_and_physics_step(usd_model):
    events = []
    class World:
        def play(self):
            events.append("play")
        def step(self, *, render):
            assert render
            events.append("step")
    class View:
        def set_subspace_roots(self, root):
            assert root == "/"
            events.append("subspace")
    fresh = View()
    def make_view(backend):
        assert backend == "numpy" and events == ["play", "step", "attach"]
        events.append("new_view")
        return fresh
    def prepare_attachment():
        assert events == ["play", "step"]
        events.append("attach")
        return {"usd_stage_id": 42, "physx_stage_id_after": 42}
    class Art:
        dof_names = ["right_outer_knuckle_joint", "finger_joint"]
        def __init__(self, **kwargs):
            assert not kwargs["reset_xform_properties"]
            events.append("new_art")
        def initialize(self, *, physics_sim_view):
            assert physics_sim_view is fresh
            events.append("initialize")
        def get_joint_positions(self):
            assert events[-1] == "initialize"
            events.append("read")
            return np.array([0.011, 0.010])
        def apply_action(self, *args):
            pytest.fail("binding must not issue drive commands")
    art, names, metadata = bind_live_gripper(lambda: usd_model, World(), Art, make_view,
                                            prepare_attachment=prepare_attachment)
    assert metadata["commanded_joint_index"] == 1
    assert metadata["fresh_physics_view_after_final_play"]
    assert metadata["usd_stage_id"] == metadata["physx_stage_id_after"] == 42
    assert events == ["play", "step", "attach", "new_view", "subspace", "new_art", "initialize", "read"]


def test_fresh_binding_rejects_missing_runtime_feedback(usd_model):
    from types import SimpleNamespace
    world = SimpleNamespace(play=lambda: None, step=lambda **kw: None)
    view = SimpleNamespace(set_subspace_roots=lambda root: None)
    art = SimpleNamespace(initialize=lambda **kw: None, dof_names=["finger_joint"], get_joint_positions=lambda: None)
    with pytest.raises(ValueError, match="finite runtime feedback"):
        bind_live_gripper(lambda: usd_model, world, lambda **kw: art, lambda backend: view)


def test_anonymous_root_preserves_scene_and_source_asset_when_session_is_cleared(usd_model):
    from pxr import Usd, UsdGeom, UsdPhysics
    source = usd_model
    source.SetDefaultPrim(source.GetPrimAtPath("/Robotiq_2F_140"))
    before = source.GetRootLayer().ExportToString()
    stage = Usd.Stage.CreateInMemory()
    metadata = compose_gripper_experiment(stage, source.GetRootLayer().identifier,
                                         meters=0.01, up_axis="Z", default_prim_path="/Robotiq_2F_140")
    scene = UsdPhysics.Scene.Define(stage, "/World/GripperPhysicsScene")
    scene.CreateGravityMagnitudeAttr(0)
    UsdGeom.Camera.Define(stage, "/World/GripperTest/Camera")
    stage.SetEditTarget(stage.GetSessionLayer())
    UsdGeom.Scope.Define(stage, "/Temporary")
    stage.GetSessionLayer().Clear()
    require_physics_scene(stage, "/World/GripperPhysicsScene", "after session clear")
    assert stage.GetPrimAtPath("/World/GripperTest/Camera")
    assert stage.GetPrimAtPath("/FixedJoint")  # Include the source's top-level world anchor too.
    assert UsdGeom.GetStageMetersPerUnit(stage) == 0.01
    assert stage.GetDefaultPrim().GetPath() == source.GetDefaultPrim().GetPath()
    assert source.GetRootLayer().ExportToString() == before
    assert metadata["experiment_root_is_anonymous"]


def test_session_only_scene_is_lost_and_guard_rejects_it(usd_model):
    from pxr import UsdPhysics
    usd_model.SetEditTarget(usd_model.GetSessionLayer())
    UsdPhysics.Scene.Define(usd_model, "/SessionPhysicsScene")
    usd_model.GetSessionLayer().Clear()
    with pytest.raises(ValueError, match="physics scene missing"):
        require_physics_scene(usd_model, "/SessionPhysicsScene", "after reset")


def test_missing_scene_blocks_tensor_creation_before_final_play(usd_model):
    from types import SimpleNamespace
    def forbidden(*a, **kw):
        pytest.fail("must reject missing scene before play or tensor binding")
    world = SimpleNamespace(play=forbidden, step=forbidden)
    with pytest.raises(ValueError, match="before final play"):
        bind_live_gripper(lambda: usd_model, world, forbidden, forbidden, physics_scene_path="/DeletedScene")


def test_experiment_refuses_to_edit_file_backed_source(tmp_path):
    from pxr import Usd
    stage = Usd.Stage.CreateNew(str(tmp_path / "source.usda"))
    with pytest.raises(ValueError, match="anonymous root"):
        compose_gripper_experiment(stage, "other.usda", meters=1, up_axis="Z", default_prim_path=None)


@pytest.mark.parametrize("initial", [0, -1, 2**64-1])
def test_detached_physx_stage_is_explicitly_attached_and_verified(initial):
    class Interface:
        attached = initial
        def get_attached_stage(self):
            return self.attached
        def attach_stage(self, stage_id):
            self.attached = stage_id
            return True
        def flush_changes(self):
            assert self.attached == 42
    result = ensure_physx_attachment(Interface(), 42)
    assert result["physx_stage_id_after"] == result["usd_stage_id"] == 42
    assert result["physx_stage_attached_explicitly"]


def test_valid_physx_attachment_is_not_detached_or_replaced():
    from types import SimpleNamespace
    def forbidden(*args):
        pytest.fail("must not replace an already valid physics attachment")
    interface = SimpleNamespace(get_attached_stage=lambda: 42, attach_stage=forbidden, flush_changes=lambda: None)
    assert not ensure_physx_attachment(interface, 42)["physx_stage_attached_explicitly"]


def test_other_attached_stage_is_rejected_before_replacing_it():
    from types import SimpleNamespace
    def forbidden(*args):
        pytest.fail("must not detach another stage")
    interface = SimpleNamespace(get_attached_stage=lambda: 15, attach_stage=forbidden)
    with pytest.raises(ValueError, match="another stage"):
        ensure_physx_attachment(interface, 42)


@pytest.mark.parametrize("attach_success", [False, True])
def test_failed_or_mismatched_attachment_cannot_proceed(attach_success):
    from types import SimpleNamespace
    interface = SimpleNamespace(get_attached_stage=lambda: 0, attach_stage=lambda stage_id: attach_success,
                                flush_changes=lambda: None)
    with pytest.raises(RuntimeError, match="attach|mismatch"):
        ensure_physx_attachment(interface, 42)


@pytest.fixture
def native_adapter():
    from types import SimpleNamespace
    class View:
        count = 1
        max_dofs = 3
        is_homogeneous = True
        shared_metatype = SimpleNamespace(dof_names=["passive_left", "finger_joint", "right_outer_knuckle_joint"])
        q = np.array([[0.2, 0.01, 0.011]], dtype=np.float32)
        targets = np.array([[0.2, 0, 0.7]], dtype=np.float32)
        writes = []
        def get_dof_positions(self):
            return self.q
        def get_dof_position_targets(self):
            return self.targets
        def set_dof_position_targets(self, targets, indices):
            self.writes.append((targets.copy(), indices.copy()))
    view = View()
    factory = SimpleNamespace(create_articulation_view=lambda path: view)
    adapter = NativeGripperArticulation(prim_path="/Robotiq_2F_140")
    adapter.initialize(physics_sim_view=factory)
    return adapter, view


def test_native_adapter_reads_supplied_tensor_view_and_preserves_passive_targets(native_adapter):
    adapter, view = native_adapter
    original = view.targets.copy()
    np.testing.assert_array_equal(adapter.get_joint_positions(), view.q[0])
    adapter.set_primary_target(0.1, 1)
    targets, indices = view.writes[0]
    np.testing.assert_array_equal(indices, [0])
    assert targets.dtype == np.float32 and indices.dtype == np.int32
    assert targets[0, 1] == pytest.approx(0.1)
    np.testing.assert_array_equal(targets[0, [0, 2]], original[0, [0, 2]])
    np.testing.assert_array_equal(view.targets, original)


@pytest.mark.parametrize("q,index", [(0.1, 0), (0.1, 2), (float("nan"), 1), (0.1, -2)])
def test_native_adapter_rejects_follower_or_invalid_commands(native_adapter, q, index):
    adapter, view = native_adapter
    with pytest.raises(ValueError):
        adapter.set_primary_target(q, index)
    assert not view.writes


def test_native_adapter_refuses_to_guess_missing_passive_targets(native_adapter):
    adapter, view = native_adapter
    view.targets = np.array([[np.nan, 0, 0]], dtype=np.float32)
    with pytest.raises(ValueError, match="targets unavailable"):
        adapter.set_primary_target(0.1, 1)
    assert not view.writes


def test_native_adapter_rejects_nonfinite_feedback(native_adapter):
    adapter, view = native_adapter
    view.q = np.array([[0, np.nan, 0]], dtype=np.float32)
    with pytest.raises(ValueError, match="feedback"):
        adapter.get_joint_positions()


def test_parent_simulation_view_survives_binding_return_and_garbage_collection(usd_model):
    import gc
    import weakref
    from types import SimpleNamespace
    owners = []
    class ChildView:
        count = 1
        max_dofs = 1
        is_homogeneous = True
        shared_metatype = SimpleNamespace(dof_names=["finger_joint"])
        def __init__(self, parent):
            self.parent = weakref.ref(parent)
        def get_dof_positions(self):
            if self.parent() is None:
                raise RuntimeError("parent simulation view destroyed")
            return np.array([[0.01]], dtype=np.float32)
        def get_dof_position_targets(self):
            assert self.parent() is not None
            return np.array([[0.0]], dtype=np.float32)
        def set_dof_position_targets(self, targets, indices):
            assert self.parent() is not None
    class ParentView:
        def __init__(self):
            owners.append(weakref.ref(self))
        def set_subspace_roots(self, root):
            pass
        def create_articulation_view(self, path):
            return ChildView(self)
    world = SimpleNamespace(play=lambda: None, step=lambda **kw: None)
    art, names, metadata = bind_live_gripper(lambda: usd_model, world, NativeGripperArticulation,
                                            lambda backend: ParentView())
    gc.collect()
    assert owners[0]() is not None
    assert metadata["simulation_view_owner_retained"]
    assert art.get_joint_positions()[0] == pytest.approx(0.01)
    art.set_primary_target(0.1, 0)
    # Demonstrate the failure being guarded: losing the sole parent reference invalidates the child.
    art._simulation_view = None
    gc.collect()
    assert owners[0]() is None
    with pytest.raises(RuntimeError, match="parent simulation view destroyed"):
        art.get_joint_positions()


class ManualInterface:
    def __init__(self, plant=None):
        self.stage_id = 42
        self.events = []
        self.plant = plant
        self.fetch_failed = False

    def get_attached_stage(self):
        return self.stage_id

    def flush_changes(self):
        self.events.append("flush")

    def simulate(self, dt, time_sec):
        self.events.append(("simulate", dt, time_sec))

    def fetch_results(self):
        self.events.append("fetch")
        if self.fetch_failed:
            raise RuntimeError("fetch failed")
        if self.plant:
            self.plant.step()


def manual_stepper(interface, is_playing=lambda: False):
    return ManualPhysxStepper(interface, lambda: interface.events.append("writeback"),
                             lambda: interface.events.append("render"), is_playing,
                             stage_id=42, dt=1/120)


def test_manual_step_fetches_before_geometry_and_render_and_advances_clock():
    interface = ManualInterface()
    stepper = manual_stepper(interface)
    stepper.step()
    stepper.step()
    assert interface.events == ["flush", ("simulate", 1/120, 0), "fetch", "writeback", "render",
                                "flush", ("simulate", 1/120, 1/120), "fetch", "writeback", "render"]
    assert stepper.diagnostics()["completed_physics_steps"] == 2
    assert stepper.diagnostics()["explicit_physics_time_sec"] == pytest.approx(2/120)


@pytest.mark.parametrize("playing,stage_id", [(True, 42), (False, 43), (False, 0)])
def test_manual_step_refuses_double_stepping_or_changed_stage(playing, stage_id):
    interface = ManualInterface()
    interface.stage_id = stage_id
    stepper = manual_stepper(interface, lambda: playing)
    with pytest.raises(RuntimeError):
        stepper.step()
    assert not interface.events and stepper.steps == 0 and stepper.failed


def test_manual_fetch_failure_does_not_render_or_advance_and_blocks_retries():
    interface = ManualInterface()
    interface.fetch_failed = True
    stepper = manual_stepper(interface)
    with pytest.raises(RuntimeError, match="fetch failed"):
        stepper.step()
    assert interface.events == ["flush", ("simulate", 1/120, 0), "fetch"]
    assert stepper.steps == 0 and stepper.failed
    with pytest.raises(RuntimeError, match="stopped after an error"):
        stepper.step()
    assert len(interface.events) == 3


@pytest.mark.parametrize("dt,stage_id", [(0, 42), (float("nan"), 42), (0.1, 42), (1/120, 0)])
def test_manual_step_rejects_invalid_setup(dt, stage_id):
    with pytest.raises(ValueError):
        ManualPhysxStepper(None, None, None, None, stage_id=stage_id, dt=dt)


def test_cycle_uses_explicit_simulate_fetch_for_joint_feedback():
    plant = Plant()
    interface = ManualInterface(plant)
    stepper = manual_stepper(interface)
    result = run(plant, step=stepper.step, dt=stepper.dt)
    assert result["status"] == "PASSED_UNLOADED_GRIPPER_CYCLE"
    assert stepper.steps == plant.steps and stepper.steps > 0
    assert len(interface.events) == stepper.steps*5


@pytest.fixture
def damping_adapter(native_adapter):
    adapter, view = native_adapter
    view.stiffness = np.array([[0, 6.4, 0]], dtype=np.float32)
    view.damping = np.array([[0, 0.057, 0]], dtype=np.float32)
    view.force = np.array([[np.inf, 10, np.inf]], dtype=np.float32)
    view.velocity_targets = np.array([[0.2, 0.1, -0.3]], dtype=np.float32)
    view.velocity = np.zeros((1, 3), dtype=np.float32)
    view.gain_writes = []
    view.get_dof_stiffnesses = lambda: view.stiffness
    view.get_dof_dampings = lambda: view.damping
    view.get_dof_max_forces = lambda: view.force
    view.get_dof_velocity_targets = lambda: view.velocity_targets
    view.get_dof_velocities = lambda: view.velocity
    def setter(field):
        def write(data, indices):
            view.gain_writes.append((field, data.copy(), indices.copy()))
            setattr(view, field, data.copy())
        return write
    view.set_dof_dampings = setter("damping")
    view.set_dof_velocity_targets = setter("velocity_targets")
    return adapter, view


def test_damping_experiment_changes_only_primary_damping_and_velocity_target(damping_adapter):
    adapter, view = damping_adapter
    before = view.damping.copy()
    velocities = view.velocity_targets.copy()
    report = adapter.configure_primary_damping(1, 10)
    assert view.damping[0, 1] == pytest.approx(before[0, 1]*10)
    np.testing.assert_array_equal(view.damping[0, [0, 2]], before[0, [0, 2]])
    assert view.velocity_targets[0, 1] == 0
    np.testing.assert_array_equal(view.velocity_targets[0, [0, 2]], velocities[0, [0, 2]])
    assert report["max_forces_unchanged"] == ["inf", 10, "inf"]
    json.dumps(report, allow_nan=False)


@pytest.mark.parametrize("index,scale", [(0, 10), (1, 0), (1, 101), (1, float("nan"))])
def test_invalid_damping_experiment_does_not_modify_settings(damping_adapter, index, scale):
    adapter, view = damping_adapter
    with pytest.raises(ValueError):
        adapter.configure_primary_damping(index, scale)
    assert not view.gain_writes


@pytest.mark.parametrize("field,value", [("damping", 0), ("stiffness", 0), ("force", np.inf), ("damping", np.nan)])
def test_experiment_refuses_missing_authored_primary_gains(damping_adapter, field, value):
    adapter, view = damping_adapter
    getattr(view, field)[0, 1] = value
    with pytest.raises(ValueError):
        adapter.configure_primary_damping(1, 10)
    assert not view.gain_writes


def test_damping_experiment_checks_readback(damping_adapter):
    adapter, view = damping_adapter
    view.set_dof_dampings = lambda data, indices: None
    with pytest.raises(RuntimeError, match="readback mismatch"):
        adapter.configure_primary_damping(1, 10)


def test_native_velocity_feedback_is_copied_and_must_be_finite(damping_adapter):
    adapter, view = damping_adapter
    feedback = adapter.get_joint_velocities()
    feedback[0] = 2
    assert view.velocity[0, 0] == 0
    view.velocity[0, 1] = np.nan
    with pytest.raises(ValueError, match="velocity feedback"):
        adapter.get_joint_velocities()


def test_high_velocity_cannot_pass_even_with_position_at_target():
    plant = Plant()
    report = run(plant, read_velocity=lambda: math.radians(2))
    assert report["status"] == "FAILED" and report["passed_legs"] == 0
    assert "timed out" in report["errors"][0]
    assert report["legs"][0]["last_feedback"]["velocity_rad_sec"] == pytest.approx(math.radians(2))


def test_initial_stabilization_precedes_four_measured_legs():
    plant = Plant()
    report = run(plant, read_velocity=lambda: 0, settle_initial=True)
    assert report["initial_stabilization"]["status"] == "PASSED"
    assert report["initial_stabilization"]["elapsed_sec"] >= 0.5
    assert report["passed_legs"] == 4


def test_initial_velocity_failure_blocks_cycle_and_holds_feedback():
    plant = Plant()
    report = run(plant, read_velocity=lambda: math.radians(2), settle_initial=True)
    assert report["status"] == "FAILED" and not report["legs"] and plant.paused
    assert "initial position/velocity" in report["errors"][0]
    assert report["abort_hold_joint_rad"] == plant.q


def test_nonfinite_velocity_aborts_cycle():
    plant = Plant()
    report = run(plant, read_velocity=lambda: float("nan"))
    assert report["status"] == "FAILED" and plant.paused
    assert "nonfinite primary joint velocity" in report["errors"][0]


def test_damped_spring_control_fixture_closes_and_returns_twice():
    # Numerical controller fixture only; this does not model or validate Robotiq PhysX.
    class SpringPlant(Plant):
        def __init__(self):
            super().__init__()
            self.q = math.radians(1)
            self.v = 0
        def step(self):
            self.steps += 1
            self.v += (25*(self.target-self.q)-5*self.v)/120
            self.q += self.v/120
    plant = SpringPlant()
    report = run(plant, dt=1/120, read_velocity=lambda: plant.v, settle_initial=True)
    assert report["status"] == "PASSED_UNLOADED_GRIPPER_CYCLE"
    assert report["passed_legs"] == 4 and report["returned_to_start"]


def test_feedback_trim_compensates_constant_bias_and_verifies_two_cycles():
    # Controller fixture with an imposed bias, not a physical model of the asset.
    class BiasedPlant(Plant):
        def __init__(self):
            super().__init__()
            self.q = math.radians(4)
            self.v = 0
        def step(self):
            old = self.q
            self.q = self.target+math.radians(3.1)
            self.v = (self.q-old)*120
    plant = BiasedPlant()
    servo = PrimaryPositionServo(plant.read, plant.command, (0, math.pi/4), 1/120)
    report = run(plant, command=servo.command, dt=1/120, read_velocity=lambda: plant.v,
                 settle_initial=True, initial_target=math.radians(4), read_command=lambda: dict(servo.last))
    assert report["status"] == "PASSED_UNLOADED_GRIPPER_CYCLE"
    assert report["passed_legs"] == 4 and report["returned_to_start"]
    assert all(0 <= q <= math.pi/4 for q in plant.commands)
    assert abs(servo.trim) <= math.radians(5)
    assert report["samples"][0]["drive_command"]["drive_target_rad"] != report["samples"][0]["command_rad"]


def test_servo_clamps_authored_limits_and_does_not_wind_up():
    writes = []
    servo = PrimaryPositionServo(lambda: math.radians(4), writes.append, (0, math.pi/4), 1/120)
    for _ in range(1500):
        servo.command(0)
    assert min(writes) == 0
    assert servo.trim == 0  # Error cannot accumulate when the desired target is already at its lower limit.
    servo = PrimaryPositionServo(lambda: 0, writes.append, (0, math.pi/4), 1/120)
    for _ in range(1500):
        servo.command(math.radians(10))
    assert servo.trim == pytest.approx(math.radians(5))
    assert max(writes) <= math.pi/4


@pytest.mark.parametrize("desired,measured", [(float("nan"), 0), (0, float("nan")), (-0.1, 0), (math.pi, 0)])
def test_servo_rejects_invalid_feedback_or_targets(desired, measured):
    writes = []
    servo = PrimaryPositionServo(lambda: measured, writes.append, (0, math.pi/4), 1/120)
    with pytest.raises(ValueError):
        servo.command(desired)
    assert not writes
