"""AssetCatalog — Isaac Sim 5.1 일반 USD 자산(로봇팔 외) 카탈로그.

`robot_loader.py`가 로봇팔(Franka/UR10) 전용 카탈로그라면, 이 모듈은 셀
시각화에 사용되는 **그 외 자산**(AMR, 카메라/센서, 선반, 부품 등)을
다룬다. 모든 자산은 Isaac Sim 5.1 NVIDIA 공식 카탈로그 기준의 USD
subpath를 갖는다.

확인 소스: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_robots.html

이 모듈은 USD/Isaac Sim 없이 import 가능 (lazy 처리는 SceneBuilder 측에서).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import logging
import os
from pathlib import Path

logger = logging.getLogger(__name__)


# =============================================================================
# Asset descriptor
# =============================================================================
@dataclass
class UsdAssetSpec:
    """일반 USD 자산 메타데이터 (로봇 articulation이 아닌 reference 자산).

    variant_selection:
        USD variant set 선택 매핑 (variant_set_name -> variant_name).
        Reference attach 직후 ``prim.GetVariantSets().GetVariantSet(k)
        .SetVariantSelection(v)`` 형태로 적용된다.

        예) NovaCarter (NVIDIA 공식 Asset Structure 기준)::

            variant_selection = {
                "Configuration": "Base",          # Base / Fully Merged / No_Internals / Skirt_only
                "Physics":       "Physics_Base",  # No_Physics / Physics_Base
                "Sensors":       "All_Sensors",   # None / All_Sensors
            }

        None 또는 빈 dict이면 USD 기본(authored default) variant가 사용된다.
        Variant set이 USD에 정의되어 있지 않으면 SetVariantSelection은
        무시되며(no-op) 경고 로그만 남긴다.
    """

    name: str                          # "nova_carter", "klt_bin", ...
    usd_subpath: str                   # "Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
    category: str                      # "amr" | "sensor" | "rack" | "prop" | "fixture"
    scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    base_offset_z: float = 0.0         # ground (z=0) 위에 올릴 때 보정
    description: str = ""
    is_articulation: bool = False      # True면 SingleArticulation으로 wrap 시도
    variant_selection: Optional[Dict[str, str]] = None  # variant-set composition 자산용


# =============================================================================
# Isaac Sim 5.1 USD 자산 카탈로그
# =============================================================================
# 모든 경로는 NVIDIA 공식 문서(2025-10) 기준. 일부는 로컬 디스크에 없을 수
# 있으며, 미스 시 SceneBuilder는 placeholder로 자동 fallback 한다.

ASSET_CATALOG: Dict[str, UsdAssetSpec] = {
    # ========================================================================
    # AMR (Autonomous Mobile Robots)
    # ========================================================================
    "nova_carter": UsdAssetSpec(
        name="nova_carter",
        usd_subpath="Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd",
        category="amr",
        description="NVIDIA Nova Carter — full sensor stack AMR",
        is_articulation=True,
        # NovaCarter는 variant-set 컨테이너 (4 KB wrapper가 아니라 variant
        # composition arc 사용). variant 선택이 없으면 Configuration / Physics
        # / Sensors variant payload가 서로 다른 기본값으로 골라져 PhysicsJoint
        # body0/body1 prim이 stage에 없어 "CreateJoint - no bodies defined at
        # body0 and body1" 경고가 발생한다.
        #
        # 공식 옵션 (Asset Structure 문서):
        #   Configuration: Base / Fully Merged / No_Internals / Skirt_only
        #   Physics      : No_Physics / Physics_Base
        #   Sensors      : None / All_Sensors
        #
        # 2026-05 Thor 실측 분기: 첫 시도("Configuration"="Base")는 USD
        # composition 단계에서 ``arcNum < srcInfo.size()`` assertion 으로
        # 실패했고, 누락된 sublayer 경로가 ``Variants/
        # nova_carter_merged_no_internals.usd`` 로 보고됐다. 즉 실제 USD 에는
        # "Base" 라는 variant 가 없으며 옳은 이름은 "No_Internals" 계열이다.
        # 또한 Sensors="All_Sensors" 는 Hawk / Owl / RPLidar / XT-32 등 5 개
        # sub-USD payload 를 모두 요구해 download_isaac_assets.sh 가 별도로
        # 끌어오지 않으면 USD payload missing 경고로 이어진다. 따라서:
        #   * Configuration = "No_Internals" → CreateJoint body0/body1 누락
        #     경고를 제거하기 위해 internal joint 가 사라진 merged 변형 선택
        #   * Sensors       = "None"         → sub-USD payload 의존 제거
        #     (sensor 가 필요해지면 download_isaac_assets.sh 에 5 개 경로 추가
        #      후 Sensors = "All_Sensors" 로 다시 바꾼다)
        #   * Physics       = "Physics_Base" → joint/articulation 유지
        #
        # variant 이름 spelling이 실제 USD와 다를 경우 (예: "Fully Merged" vs
        # "Full_Merged"), scene_builder._apply_variant_selection 가
        # SetVariantSelection 실패를 감지하고 사용 가능한 variant 목록을 WARN
        # 로그로 출력한다. 더 정확한 이름이 필요하면 scripts/dump_usd_variants.py
        # (AppLauncher 기반) 로 라이브 덤프 가능.
        variant_selection={
            "Configuration": "No_Internals",
            "Physics": "Physics_Base",
            "Sensors": "None",
        },
    ),
    "jetbot": UsdAssetSpec(
        name="jetbot",
        usd_subpath="Isaac/Robots/NVIDIA/Jetbot/jetbot.usd",
        category="amr",
        description="NVIDIA Jetbot — compact 2-wheel AMR",
        is_articulation=True,
    ),
    "iw_hub": UsdAssetSpec(
        name="iw_hub",
        usd_subpath="Isaac/Robots/Idealworks/iwhub/iw_hub.usd",
        category="amr",
        description="Idealworks iw.hub — industrial AMR",
        is_articulation=True,
    ),
    "iw_hub_static": UsdAssetSpec(
        name="iw_hub_static",
        usd_subpath="Isaac/Robots/Idealworks/iwhub/iw_hub_static.usd",
        category="amr",
        description="Idealworks iw.hub (static / no actuation)",
        is_articulation=False,
    ),

    # ========================================================================
    # Inspection — manipulators light (UR series for forming/inspection)
    # ========================================================================
    "ur10e": UsdAssetSpec(
        name="ur10e",
        usd_subpath="Isaac/Robots/UniversalRobots/ur10e/ur10e.usd",
        category="manipulator",
        description="Universal Robots UR10e (E-series, 10 kg payload)",
        is_articulation=True,
    ),
    "ur5e": UsdAssetSpec(
        name="ur5e",
        usd_subpath="Isaac/Robots/UniversalRobots/ur5e/ur5e.usd",
        category="manipulator",
        description="Universal Robots UR5e (smaller arm, inspection 적합)",
        is_articulation=True,
    ),
    "ur16e": UsdAssetSpec(
        name="ur16e",
        usd_subpath="Isaac/Robots/UniversalRobots/ur16e/ur16e.usd",
        category="manipulator",
        description="Universal Robots UR16e (heavy-duty, forming 적합)",
        is_articulation=True,
    ),

    # ========================================================================
    # Props — 부품 / 컨테이너 (Isaac 5.1 표준 경로 추정 — 미스 시 fallback)
    # ========================================================================
    "klt_bin": UsdAssetSpec(
        name="klt_bin",
        usd_subpath="Isaac/Props/KLT_Bin/small_KLT_visual_collision.usd",
        category="prop",
        scale=(1.0, 1.0, 1.0),
        description="Small KLT bin (industrial logistics container)",
    ),
    "cardboard_box": UsdAssetSpec(
        name="cardboard_box",
        usd_subpath="Isaac/Props/Cardboard_Box/cardboard_box.usd",
        category="prop",
        description="Cardboard box (warehouse prop)",
    ),

    # ========================================================================
    # Sensors — inspection cell용
    # ========================================================================
    # NVIDIA Isaac에서 카메라는 Sensors 라이브러리에 prim으로 정의되어 있는
    # 경우가 많아 path 표준화가 어려움 → 1순위는 자체 UsdGeom.Camera로 폴백.
    "industrial_camera": UsdAssetSpec(
        name="industrial_camera",
        usd_subpath="Isaac/Sensors/Cameras/Industrial/CN_AMR_C320/CN_AMR_C320.usd",
        category="sensor",
        description="Generic industrial RGB camera body",
    ),
}


# =============================================================================
# 자산 경로 해석 (robot_loader.py와 동일한 전략)
# =============================================================================
def resolve_asset_usd_path(spec: UsdAssetSpec) -> str:
    """UsdAssetSpec → 절대 USD 경로 (로컬 우선, 원격 fallback).

    robot_loader.resolve_robot_usd_path()와 동일한 로직을 재사용한다.
    """
    # robot_loader가 갖고 있는 탐색 로직을 재사용 (subpath만 다름)
    from fdw_sim.visualization.robot_loader import (
        LOCAL_ASSET_CANDIDATES,
        _check_local_candidate,
        get_isaac_assets_root,
    )

    # 1) 직접 환경변수 (예: FDW_NOVA_CARTER_USD=/path/to/nova_carter.usd)
    env_key = f"FDW_{spec.name.upper()}_USD"
    direct = os.environ.get(env_key)
    if direct and Path(direct).is_file():
        logger.info("[AssetCatalog] %s resolved via %s=%s",
                    spec.name, env_key, direct)
        return direct

    # 2) ISAAC_NUCLEUS_DIR_LOCAL — 로컬 디스크 루트
    local_root = os.environ.get("ISAAC_NUCLEUS_DIR_LOCAL")
    if local_root:
        found = _check_local_candidate(local_root, spec.usd_subpath)
        if found:
            logger.info("[AssetCatalog] %s resolved via ISAAC_NUCLEUS_DIR_LOCAL: %s",
                        spec.name, found)
            return found

    # 3) 표준 후보 디렉토리
    for cand in LOCAL_ASSET_CANDIDATES:
        found = _check_local_candidate(cand, spec.usd_subpath)
        if found:
            logger.info("[AssetCatalog] %s resolved via local candidate: %s",
                        spec.name, found)
            return found

    # 4) 원격 (Nucleus / S3)
    remote_root = get_isaac_assets_root()
    return f"{remote_root}/{spec.usd_subpath}"


def find_local_asset(spec: UsdAssetSpec) -> Optional[str]:
    """로컬 디스크에서만 자산을 찾고, 없으면 None 반환 (원격 fallback X)."""
    from fdw_sim.visualization.robot_loader import (
        LOCAL_ASSET_CANDIDATES,
        _check_local_candidate,
    )

    env_key = f"FDW_{spec.name.upper()}_USD"
    direct = os.environ.get(env_key)
    if direct and Path(direct).is_file():
        return direct

    local_root = os.environ.get("ISAAC_NUCLEUS_DIR_LOCAL")
    if local_root:
        found = _check_local_candidate(local_root, spec.usd_subpath)
        if found:
            return found

    for cand in LOCAL_ASSET_CANDIDATES:
        found = _check_local_candidate(cand, spec.usd_subpath)
        if found:
            return found

    return None


def diagnose_catalog() -> Dict[str, Dict]:
    """카탈로그 전체에 대해 로컬/원격 해석 결과를 dict로 반환."""
    report: Dict[str, Dict] = {}
    for name, spec in ASSET_CATALOG.items():
        local = find_local_asset(spec)
        report[name] = {
            "category": spec.category,
            "subpath": spec.usd_subpath,
            "local": local,
            "resolved": resolve_asset_usd_path(spec),
            "is_articulation": spec.is_articulation,
        }
    return report


__all__ = [
    "UsdAssetSpec",
    "ASSET_CATALOG",
    "resolve_asset_usd_path",
    "find_local_asset",
    "diagnose_catalog",
]
