"""Level 2.3 smoke tests — Real USD assets for non-welding cells.

Isaac Sim이 없는 환경(CI/로컬)에서도 다음을 확인한다:
 1) asset_catalog 모듈이 USD/Isaac 없이 import 가능
 2) ASSET_CATALOG에 기대 자산이 모두 등록되어 있고 UsdAssetSpec 인스턴스
 3) resolve/find/diagnose 헬퍼가 missing-asset에 대해 raise 하지 않음
 4) SceneBuilder가 새 USD 로딩 메서드 5개를 모두 노출
 5) WorkshopVizConfig가 Level 2.3 필드 7개를 모두 갖음
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


# -----------------------------------------------------------------------------
# 1. asset_catalog 모듈 import + 카탈로그 내용
# -----------------------------------------------------------------------------
def test_asset_catalog_imports() -> None:
    """asset_catalog 모듈은 Isaac Sim 없이 import 되어야 한다."""
    from fdw_sim.visualization import asset_catalog  # noqa: F401
    from fdw_sim.visualization.asset_catalog import (
        ASSET_CATALOG,
        UsdAssetSpec,
        resolve_asset_usd_path,
        find_local_asset,
        diagnose_catalog,
    )
    assert isinstance(ASSET_CATALOG, dict)
    assert UsdAssetSpec is not None
    assert callable(resolve_asset_usd_path)
    assert callable(find_local_asset)
    assert callable(diagnose_catalog)
    print("[OK] asset_catalog imports without Isaac Sim")


def test_asset_catalog_required_entries() -> None:
    """Level 2.3에서 사용하는 핵심 자산들이 모두 등록되어 있어야 한다."""
    from fdw_sim.visualization.asset_catalog import ASSET_CATALOG, UsdAssetSpec

    expected = [
        # AMRs
        "nova_carter", "jetbot", "iw_hub", "iw_hub_static",
        # Manipulators
        "ur10e", "ur5e", "ur16e",
        # Props
        "klt_bin", "cardboard_box",
        # Sensors
        "industrial_camera",
    ]
    missing = [n for n in expected if n not in ASSET_CATALOG]
    assert not missing, f"missing catalog entries: {missing}"

    for name, spec in ASSET_CATALOG.items():
        assert isinstance(spec, UsdAssetSpec), f"{name}: not a UsdAssetSpec"
        assert spec.name == name, f"{name}: spec.name mismatch"
        assert spec.usd_subpath.endswith(".usd"), \
            f"{name}: usd_subpath should end with .usd"
        assert spec.category in (
            "amr", "sensor", "rack", "prop", "fixture", "manipulator"
        ), f"{name}: unexpected category {spec.category}"

    print("[OK] asset_catalog has %d entries with expected types"
          % len(ASSET_CATALOG))


def test_asset_catalog_paths_match_isaac_5_1() -> None:
    """주요 자산 경로가 Isaac 5.1 공식 카탈로그 규칙을 따른다."""
    from fdw_sim.visualization.asset_catalog import ASSET_CATALOG

    # NVIDIA 공식 문서 기준 경로
    expected_paths = {
        "nova_carter":  "Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd",
        "jetbot":       "Isaac/Robots/NVIDIA/Jetbot/jetbot.usd",
        "iw_hub":       "Isaac/Robots/Idealworks/iwhub/iw_hub.usd",
        "ur10e":        "Isaac/Robots/UniversalRobots/ur10e/ur10e.usd",
    }
    for name, expected in expected_paths.items():
        actual = ASSET_CATALOG[name].usd_subpath
        assert actual == expected, \
            f"{name}: expected {expected!r}, got {actual!r}"
    print("[OK] catalog paths match Isaac 5.1 official paths")


# -----------------------------------------------------------------------------
# 2. Helper 함수 — missing asset에 대해 raise 하지 않음
# -----------------------------------------------------------------------------
def test_resolve_asset_usd_path_no_raise() -> None:
    """resolve_asset_usd_path는 자산 없어도 raise 하지 않고 문자열 반환."""
    from fdw_sim.visualization.asset_catalog import (
        ASSET_CATALOG, resolve_asset_usd_path,
    )
    for name, spec in ASSET_CATALOG.items():
        path = resolve_asset_usd_path(spec)
        assert isinstance(path, str) and path, \
            f"{name}: resolve returned {path!r}"
    print("[OK] resolve_asset_usd_path returns a string for all entries")


def test_find_local_asset_returns_none_when_missing() -> None:
    """find_local_asset은 로컬에 없으면 None을 반환 (raise X)."""
    from fdw_sim.visualization.asset_catalog import (
        ASSET_CATALOG, find_local_asset,
    )
    # 대부분의 CI 환경에서는 로컬 자산이 없을 것 → None 또는 str
    for name, spec in ASSET_CATALOG.items():
        result = find_local_asset(spec)
        assert result is None or isinstance(result, str), \
            f"{name}: find_local_asset returned {type(result)}"
    print("[OK] find_local_asset returns None or str (no raise)")


def test_diagnose_catalog_returns_dict() -> None:
    """diagnose_catalog는 모든 엔트리에 대한 report dict를 반환."""
    from fdw_sim.visualization.asset_catalog import (
        ASSET_CATALOG, diagnose_catalog,
    )
    report = diagnose_catalog()
    assert isinstance(report, dict)
    assert set(report.keys()) == set(ASSET_CATALOG.keys()), \
        "diagnose should cover every catalog entry"
    for name, info in report.items():
        for key in ("category", "subpath", "local", "resolved", "is_articulation"):
            assert key in info, f"{name}: missing key {key}"
    print("[OK] diagnose_catalog covers all %d entries" % len(report))


# -----------------------------------------------------------------------------
# 3. SceneBuilder — 새 USD 로딩 메서드 노출 확인
# -----------------------------------------------------------------------------
def test_scene_builder_has_new_usd_methods() -> None:
    """SceneBuilder가 Level 2.3 USD 로딩 메서드 5개를 노출해야 한다."""
    from fdw_sim.visualization.scene_builder import SceneBuilder

    new_methods = [
        "add_usd_reference",
        "add_amr_usd",
        "add_cell_prop",
        "add_inspection_camera_real",
        "add_smart_rack_real",
    ]
    missing = [m for m in new_methods if not hasattr(SceneBuilder, m)]
    assert not missing, f"SceneBuilder missing methods: {missing}"
    for m in new_methods:
        assert callable(getattr(SceneBuilder, m)), \
            f"SceneBuilder.{m} is not callable"
    print("[OK] SceneBuilder exposes all 5 Level 2.3 USD methods")


# -----------------------------------------------------------------------------
# 4. WorkshopVizConfig — Level 2.3 필드
# -----------------------------------------------------------------------------
def test_workshop_viz_config_has_level23_fields() -> None:
    """WorkshopVizConfig는 Level 2.3 필드 7개를 모두 갖는다."""
    from fdw_sim.visualization.workshop_visualizer import WorkshopVizConfig

    cfg = WorkshopVizConfig()
    expected_fields = {
        "use_real_inspection_cam": bool,
        "use_real_amr": bool,
        "amr_asset_name": str,
        "use_real_smart_rack": bool,
        "smart_rack_asset_name": str,
        "use_real_forming_arm": bool,
        "forming_robot_name": str,
    }
    for name, typ in expected_fields.items():
        assert hasattr(cfg, name), f"WorkshopVizConfig missing field: {name}"
        val = getattr(cfg, name)
        assert isinstance(val, typ), \
            f"{name}: expected {typ.__name__}, got {type(val).__name__}"

    # 기본값 의미 검증
    assert cfg.amr_asset_name == "nova_carter"
    assert cfg.smart_rack_asset_name == "klt_bin"
    assert cfg.forming_robot_name == "ur10"
    # Level 2.1 호환 필드도 보존됐는지
    assert cfg.show_smart_rack is True
    assert cfg.show_robot_arm is True
    assert cfg.show_camera is True
    print("[OK] WorkshopVizConfig has all 7 Level 2.3 fields with defaults")


def test_workshop_viz_config_level23_overridable() -> None:
    """Level 2.3 필드를 override 가능."""
    from fdw_sim.visualization.workshop_visualizer import WorkshopVizConfig

    cfg = WorkshopVizConfig(
        use_real_inspection_cam=False,
        use_real_amr=False,
        amr_asset_name="jetbot",
        use_real_smart_rack=False,
        smart_rack_asset_name="cardboard_box",
        use_real_forming_arm=False,
        forming_robot_name="ur10e",
    )
    assert cfg.use_real_inspection_cam is False
    assert cfg.amr_asset_name == "jetbot"
    assert cfg.smart_rack_asset_name == "cardboard_box"
    assert cfg.forming_robot_name == "ur10e"
    print("[OK] Level 2.3 fields are user-overridable")


# -----------------------------------------------------------------------------
# 5. WorkshopVisualizer — 통합 (Isaac 없이 build_scene 호출 X, register만)
# -----------------------------------------------------------------------------
def test_workshop_visualizer_with_level23_options() -> None:
    """Level 2.3 옵션을 켠 채 WorkshopVisualizer 초기화."""
    from fdw_sim.messaging.bus import InMemoryBus
    from fdw_sim.visualization.workshop_visualizer import (
        WorkshopVisualizer, WorkshopVizConfig,
    )

    bus = InMemoryBus()
    cfg = WorkshopVizConfig(
        use_real_inspection_cam=True,
        use_real_amr=True,
        amr_asset_name="nova_carter",
        use_real_smart_rack=True,
        smart_rack_asset_name="klt_bin",
        use_real_forming_arm=True,
        forming_robot_name="ur10",
    )
    viz = WorkshopVisualizer(bus=bus, config=cfg)
    viz.register_cell("MAT_01", cell_type="material", position=(0, 0, 0))
    viz.register_cell("WELD_01", cell_type="welding", position=(5, 0, 0))
    viz.register_cell("INSP_01", cell_type="inspection", position=(10, 0, 0))
    viz.register_cell("FORM_01", cell_type="forming", position=(15, 0, 0))
    viz.register_amr("AMR_01")
    assert "INSP_01" in viz.cell_positions
    assert "FORM_01" in viz.cell_positions
    assert len(viz._pending_amrs) == 1
    print("[OK] WorkshopVisualizer accepts Level 2.3 options for all cell types")


# -----------------------------------------------------------------------------
# Main runner
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    tests = [
        test_asset_catalog_imports,
        test_asset_catalog_required_entries,
        test_asset_catalog_paths_match_isaac_5_1,
        test_resolve_asset_usd_path_no_raise,
        test_find_local_asset_returns_none_when_missing,
        test_diagnose_catalog_returns_dict,
        test_scene_builder_has_new_usd_methods,
        test_workshop_viz_config_has_level23_fields,
        test_workshop_viz_config_level23_overridable,
        test_workshop_visualizer_with_level23_options,
    ]
    failed = []
    for t in tests:
        try:
            t()
        except Exception as e:
            print(f"[FAIL] {t.__name__}: {type(e).__name__}: {e}")
            failed.append(t.__name__)

    print(f"\n{'='*60}")
    print(f"Level 2.3 smoke: {len(tests) - len(failed)}/{len(tests)} passed")
    print(f"{'='*60}")
    if failed:
        print("Failed tests:", failed)
        sys.exit(1)
