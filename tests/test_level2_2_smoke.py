"""Level 2.2 smoke tests — RMPflow controller + welding spark emitter.

Isaac Sim이 없는 환경(CI/로컬)에서도 다음을 확인한다:
 1) rmpflow_controller / spark_emitter 모듈이 USD/Isaac 없이 import 가능
 2) CollisionSphere / RMPflowConfig / SparkEmitterConfig 인스턴스화
 3) WorkshopVizConfig + SimulationConfig의 Level 2.2 필드 존재
 4) RMPflowController 헬퍼 메서드 (locate paths) — 환경에 따라 None 허용
 5) PointInstancer USD 작업이 없는 numpy 경로(spark _spawn 내부 RNG/회전 헬퍼) 정합

USD 의존 부분 (실제 PointInstancer 생성, articulation set_joint_positions 등) 은
별도의 Isaac Sim 환경에서만 검증한다.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


# -----------------------------------------------------------------------------
# 1. 모듈 import 단계
# -----------------------------------------------------------------------------
def test_rmpflow_controller_imports() -> None:
    """RMPflow controller 모듈은 Isaac Sim 없이 import 되어야 한다."""
    from fdw_sim.visualization import rmpflow_controller  # noqa: F401
    from fdw_sim.visualization.rmpflow_controller import (
        CollisionSphere,
        RMPflowConfig,
        RMPflowController,
    )
    assert CollisionSphere is not None
    assert RMPflowConfig is not None
    assert isinstance(RMPflowController, type)
    print("[OK] rmpflow_controller module imports without Isaac Sim")


def test_spark_emitter_imports() -> None:
    """Spark emitter 모듈은 USD 없이 import 되어야 한다."""
    from fdw_sim.visualization import spark_emitter  # noqa: F401
    from fdw_sim.visualization.spark_emitter import (
        SparkEmitterConfig,
        WeldingSparkEmitter,
        add_sparks_pointinstancer,
    )
    assert SparkEmitterConfig is not None
    assert isinstance(WeldingSparkEmitter, type)
    assert callable(add_sparks_pointinstancer)
    print("[OK] spark_emitter module imports without USD")


# -----------------------------------------------------------------------------
# 2. 설정 객체 (dataclass) 기본값
# -----------------------------------------------------------------------------
def test_rmpflow_config_defaults() -> None:
    """RMPflowConfig 기본값 검증."""
    from fdw_sim.visualization.rmpflow_controller import RMPflowConfig

    cfg = RMPflowConfig()
    assert cfg.preferred_backend == "auto"
    assert cfg.max_step_distance_m > 0
    assert cfg.obstacle_clearance_m > 0
    assert cfg.obstacle_repulsion_gain >= 0
    assert isinstance(cfg.rmp_config_candidates, list)
    print("[OK] RMPflowConfig defaults (backend=%s)" % cfg.preferred_backend)


def test_collision_sphere_dataclass() -> None:
    """CollisionSphere 인스턴스화 — 위치/반지름 dataclass."""
    from fdw_sim.visualization.rmpflow_controller import CollisionSphere

    s = CollisionSphere(name="bench", center=(1.0, 2.0, 3.0),
                        radius=0.5, static=True)
    assert s.name == "bench"
    assert s.center == (1.0, 2.0, 3.0)
    assert s.radius == 0.5
    assert s.static is True
    print("[OK] CollisionSphere dataclass works")


def test_spark_emitter_config_defaults() -> None:
    """SparkEmitterConfig 기본값 검증."""
    from fdw_sim.visualization.spark_emitter import SparkEmitterConfig

    cfg = SparkEmitterConfig()
    assert cfg.pool_size > 0
    assert cfg.spark_rate > 0
    assert cfg.lifetime_sec > 0
    assert cfg.initial_speed_min <= cfg.initial_speed_max
    assert 0 <= cfg.spread_angle_deg <= 360
    assert len(cfg.emit_direction) == 3
    assert len(cfg.gravity) == 3
    print("[OK] SparkEmitterConfig defaults (pool=%d, rate=%.1f)"
          % (cfg.pool_size, cfg.spark_rate))


def test_workshop_viz_config_has_level22_fields() -> None:
    """WorkshopVizConfig가 Level 2.2 필드를 모두 갖는지 확인."""
    from fdw_sim.visualization.workshop_visualizer import WorkshopVizConfig

    c = WorkshopVizConfig()
    # Level 2.1 호환성
    assert hasattr(c, "use_real_robot")
    assert hasattr(c, "enable_ik")
    # Level 2.2 신규
    assert hasattr(c, "motion_mode")
    assert hasattr(c, "enable_sparks")
    assert hasattr(c, "spark_rate")
    assert hasattr(c, "spark_lifetime_sec")
    assert hasattr(c, "rmpflow_register_obstacles")
    assert c.motion_mode in ("auto", "rmpflow", "ik", "heuristic")
    print("[OK] WorkshopVizConfig has all Level 2.2 fields (mode=%s)"
          % c.motion_mode)


def test_simulation_config_has_level22_fields() -> None:
    """SimulationConfig가 Level 2.2 필드를 모두 갖고 기본값이 유효한지."""
    from fdw_sim.simulation.manager import SimulationConfig

    c = SimulationConfig()
    assert hasattr(c, "motion_mode")
    assert hasattr(c, "enable_sparks")
    assert hasattr(c, "spark_rate")
    assert hasattr(c, "spark_lifetime_sec")
    assert hasattr(c, "rmpflow_register_obstacles")
    assert c.motion_mode in ("auto", "rmpflow", "ik", "heuristic")
    assert c.spark_rate > 0
    assert c.spark_lifetime_sec > 0
    print("[OK] SimulationConfig has all Level 2.2 fields")


# -----------------------------------------------------------------------------
# 3. RMPflow Controller — 내부 헬퍼는 Isaac 없이 호출 가능
# -----------------------------------------------------------------------------
def test_rmpflow_controller_locate_methods_safe() -> None:
    """_locate_* 헬퍼는 자산이 없어도 None 반환 (예외 X)."""
    from fdw_sim.visualization.rmpflow_controller import (
        RMPflowController, RMPflowConfig,
    )

    # articulation/spec 주입 없이 직접 메서드만 호출
    # __init__는 articulation을 필요로 하므로 직접 인스턴스화 대신
    # __new__로 빈 객체를 생성해 헬퍼만 호출
    obj = RMPflowController.__new__(RMPflowController)
    obj.config = RMPflowConfig()

    # _locate_* 가 정의되어 있다면 호출 가능해야 한다
    for name in ("_locate_rmp_config", "_locate_urdf", "_locate_robot_description"):
        if hasattr(obj, name):
            try:
                result = getattr(obj, name)()
                # 자산이 없으면 None 또는 빈 문자열일 수 있음
                assert result is None or isinstance(result, (str, Path)), \
                    f"{name} returned unexpected type: {type(result)}"
            except Exception as e:
                # 헬퍼는 missing-asset에 대해 raise 하지 말아야 함
                # (일부 구현은 spec을 필요로 할 수 있어 AttributeError 허용)
                if not isinstance(e, AttributeError):
                    raise
    print("[OK] RMPflowController locate helpers do not raise")


def test_rmpflow_controller_diagnose_exists() -> None:
    """diagnose() 메서드 시그니처 존재 확인 (호출은 Isaac 환경에서만)."""
    from fdw_sim.visualization.rmpflow_controller import RMPflowController

    assert hasattr(RMPflowController, "diagnose")
    assert callable(getattr(RMPflowController, "diagnose"))
    # IK-compatible API
    for api in ("start_path", "update", "go_home", "is_idle", "get_phase",
                "get_tcp_position", "add_obstacle", "clear_obstacles"):
        assert hasattr(RMPflowController, api), \
            f"RMPflowController missing API: {api}"
    print("[OK] RMPflowController exposes IK-compatible API + diagnose")


# -----------------------------------------------------------------------------
# 4. Spark Emitter 내부 수치 헬퍼 (numpy만 사용 — USD 불필요)
# -----------------------------------------------------------------------------
def test_spark_emitter_rotate_z_to_helper() -> None:
    """_rotate_z_to: +z 콘 샘플 → 임의 방향 회전 검증."""
    try:
        import numpy as np
    except Exception:
        print("[SKIP] numpy unavailable")
        return

    from fdw_sim.visualization.spark_emitter import _rotate_z_to

    # 입력: z축 단일 벡터
    local = np.array([[0.0, 0.0, 1.0]], dtype=np.float32)

    # case 1: target=+z → 그대로
    out = _rotate_z_to(local, np.array([0.0, 0.0, 1.0], dtype=np.float32), np)
    assert np.allclose(out[0], [0.0, 0.0, 1.0], atol=1e-5), out[0]

    # case 2: target=+x → (1,0,0)에 가까워야 함
    out = _rotate_z_to(local, np.array([1.0, 0.0, 0.0], dtype=np.float32), np)
    assert math.isclose(float(out[0, 0]), 1.0, abs_tol=1e-4), out
    assert abs(float(out[0, 1])) < 1e-4
    assert abs(float(out[0, 2])) < 1e-4

    # case 3: target=-z → (0,0,-1)
    out = _rotate_z_to(local, np.array([0.0, 0.0, -1.0], dtype=np.float32), np)
    assert math.isclose(float(out[0, 2]), -1.0, abs_tol=1e-4), out
    print("[OK] _rotate_z_to helper produces correct orientations")


def test_spark_emitter_config_custom_values() -> None:
    """SparkEmitterConfig는 모든 필드를 사용자 값으로 override 가능."""
    from fdw_sim.visualization.spark_emitter import SparkEmitterConfig

    cfg = SparkEmitterConfig(
        pool_size=32,
        spark_rate=50.0,
        lifetime_sec=0.6,
        initial_speed_min=0.3,
        initial_speed_max=2.0,
        spread_angle_deg=90.0,
        emit_direction=(0.0, 0.0, 1.0),
        gravity=(0.0, 0.0, -9.81),
        spark_size=0.008,
        spark_color=(1.0, 0.8, 0.2),
        floor_z=-1.0,
        verbose=False,
    )
    assert cfg.pool_size == 32
    assert cfg.spark_rate == 50.0
    assert cfg.lifetime_sec == 0.6
    assert cfg.spread_angle_deg == 90.0
    print("[OK] SparkEmitterConfig accepts custom values")


# -----------------------------------------------------------------------------
# 5. workshop_visualizer 통합 — Level 2.2 옵션이 회귀하지 않는지
# -----------------------------------------------------------------------------
def test_workshop_visualizer_with_level22_options() -> None:
    """Level 2.2 옵션을 켠 채 WorkshopVisualizer 초기화 (USD 없이)."""
    from fdw_sim.messaging.bus import InMemoryBus
    from fdw_sim.visualization.workshop_visualizer import (
        WorkshopVisualizer, WorkshopVizConfig,
    )

    bus = InMemoryBus()
    cfg = WorkshopVizConfig(
        use_real_robot=True,           # 실제로는 build_scene에서만 시도됨
        motion_mode="rmpflow",
        enable_sparks=True,
        spark_rate=45.0,
        spark_lifetime_sec=0.5,
        rmpflow_register_obstacles=True,
    )
    viz = WorkshopVisualizer(bus=bus, config=cfg)
    viz.register_cell("MAT_01", cell_type="material", position=(0, 0, 0))
    viz.register_cell("WELD_01", cell_type="welding", position=(5, 0, 0))
    assert "WELD_01" in viz.cell_positions
    # motion_mode 정규화 확인
    assert getattr(viz, "_motion_mode", None) == "rmpflow"
    print("[OK] WorkshopVisualizer with Level 2.2 options initializes")


def test_workshop_visualizer_motion_mode_fallback_unknown() -> None:
    """알 수 없는 motion_mode는 'auto'로 폴백되어야 한다."""
    from fdw_sim.messaging.bus import InMemoryBus
    from fdw_sim.visualization.workshop_visualizer import (
        WorkshopVisualizer, WorkshopVizConfig,
    )
    bus = InMemoryBus()
    cfg = WorkshopVizConfig(motion_mode="nonsense")
    viz = WorkshopVisualizer(bus=bus, config=cfg)
    assert viz._motion_mode == "auto", \
        f"unknown mode should fall back to auto, got {viz._motion_mode}"
    print("[OK] unknown motion_mode falls back to auto")


# -----------------------------------------------------------------------------
# Main runner
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    tests = [
        test_rmpflow_controller_imports,
        test_spark_emitter_imports,
        test_rmpflow_config_defaults,
        test_collision_sphere_dataclass,
        test_spark_emitter_config_defaults,
        test_workshop_viz_config_has_level22_fields,
        test_simulation_config_has_level22_fields,
        test_rmpflow_controller_locate_methods_safe,
        test_rmpflow_controller_diagnose_exists,
        test_spark_emitter_rotate_z_to_helper,
        test_spark_emitter_config_custom_values,
        test_workshop_visualizer_with_level22_options,
        test_workshop_visualizer_motion_mode_fallback_unknown,
    ]
    failed = []
    for t in tests:
        try:
            t()
        except Exception as e:
            print(f"[FAIL] {t.__name__}: {type(e).__name__}: {e}")
            failed.append(t.__name__)

    print(f"\n{'='*60}")
    print(f"Level 2.2 smoke: {len(tests) - len(failed)}/{len(tests)} passed")
    print(f"{'='*60}")
    if failed:
        print("Failed tests:", failed)
        sys.exit(1)
    print("\nAll Level 2.2 smoke tests passed!")
