"""dump_usd_variants.py — USD 의 variant set / variant 이름을 실측 dump.

inspect_usd_refs.py 는 USDC 바이너리에서 regex 로 토큰을 뽑기 때문에 variant
이름 spelling 이 부분적으로만 보인다 (TOKENS 테이블의 압축/분할 영향).
이 스크립트는 pxr.Usd 로 stage 를 열어 라이브 composition 결과의 variant
이름을 한 번에 정확히 추출한다.

사용법 (반드시 Isaac Sim 환경 또는 pxr 가 설치된 conda env 에서 실행):

    cd ~/isaac_workspace/projects/fdw-sim
    conda activate isaac_sim   # 또는 source ~/isaac_workspace/.../setup.sh
    python scripts/dump_usd_variants.py \
        ~/isaac_assets/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd

    # JSON 으로 저장해서 asset_catalog 수정에 활용
    python scripts/dump_usd_variants.py --json \
        ~/isaac_assets/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd \
        > nova_carter_variants.json

출력 예 (NovaCarter — 가설):

    === /home/.../nova_carter.usd
    Root prim: /nova_carter  (default)

    Variant sets on /nova_carter (3):

      Configuration (default = 'Base')
        - Base
        - Fully Merged
        - No_Internals
        - Skirt_only

      Physics (default = 'Physics_Base')
        - No_Physics
        - Physics_Base

      Sensors (default = 'None')
        - None
        - All_Sensors

    Suggested asset_catalog mapping:
        variant_selection={
            "Configuration": "Base",
            "Physics":       "Physics_Base",
            "Sensors":       "All_Sensors",
        }

Importantly: 이 도구는 reference attach 를 하지 않고 ``Usd.Stage.Open()`` 으로
직접 USD layer 만 연다. 따라서 sub-USD payload 가 누락된 상태에서도 variant
set / variant 이름만은 안전하게 추출할 수 있다 (compose 실패 경고는 나올 수
있지만 variant 토큰은 메인 USD layer 에 박혀 있다).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List, Optional


def _import_pxr():
    """pxr.Usd 를 lazy import — Isaac Sim 환경이 아닐 때 친절한 에러."""
    try:
        from pxr import Usd  # type: ignore
        return Usd
    except ImportError as e:
        print("[FAIL] pxr.Usd import failed:", e, file=sys.stderr)
        print("", file=sys.stderr)
        print("이 스크립트는 USD 라이브러리(pxr) 가 필요합니다.", file=sys.stderr)
        print("다음 중 하나로 실행하세요:", file=sys.stderr)
        print("  1) conda activate isaac_sim", file=sys.stderr)
        print("  2) source ~/isaac_workspace/.../setup_python_env.sh", file=sys.stderr)
        print("  3) Isaac Sim 의 python.sh 로 직접 실행:", file=sys.stderr)
        print("       ~/isaac-sim/python.sh scripts/dump_usd_variants.py ...",
              file=sys.stderr)
        raise SystemExit(2)


def _collect_variants_on_prim(prim) -> Dict[str, Dict[str, object]]:
    """prim 에 정의된 variant set 들과 각 set 의 variant 이름 목록 수집."""
    out: Dict[str, Dict[str, object]] = {}
    try:
        vsets = prim.GetVariantSets()
        names = list(vsets.GetNames())
    except Exception as e:
        print(f"    [warn] GetVariantSets failed on {prim.GetPath()}: {e}",
              file=sys.stderr)
        return out

    for set_name in names:
        try:
            vset = vsets.GetVariantSet(set_name)
            variants = list(vset.GetVariantNames())
            current = vset.GetVariantSelection()
        except Exception as e:
            print(f"    [warn] failed to read variant set {set_name}: {e}",
                  file=sys.stderr)
            variants = []
            current = ""
        out[set_name] = {
            "default": current,
            "variants": variants,
        }
    return out


def dump_variants(usd_path: Path, recurse: bool = False) -> Dict[str, object]:
    """USD 의 variant 구조를 dict 로 반환.

    Args:
        usd_path: 분석할 USD 절대/상대 경로
        recurse:  default prim 만이 아니라 모든 prim 을 traverse 해서
                  variant set 이 있는 prim 을 모두 수집
    Returns:
        {
          "path"       : "<absolute path>",
          "default_prim": "/nova_carter",
          "prims": {
              "/nova_carter": { "<set>": {"default": "<name>",
                                            "variants": [...]} },
              ...
          }
        }
    """
    Usd = _import_pxr()

    if not usd_path.is_file():
        raise FileNotFoundError(f"USD not found: {usd_path}")

    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise RuntimeError(f"Usd.Stage.Open returned None for {usd_path}")

    result: Dict[str, object] = {
        "path": str(usd_path.resolve()),
        "default_prim": "",
        "prims": {},
    }

    default = stage.GetDefaultPrim()
    if default and default.IsValid():
        result["default_prim"] = str(default.GetPath())
        v = _collect_variants_on_prim(default)
        if v:
            result["prims"][str(default.GetPath())] = v

    if recurse:
        for prim in stage.TraverseAll():
            path_str = str(prim.GetPath())
            if path_str in result["prims"]:
                continue
            v = _collect_variants_on_prim(prim)
            if v:
                result["prims"][path_str] = v

    return result


def _suggest_nova_carter_mapping(prims: Dict[str, Dict[str, object]]
                                  ) -> Optional[Dict[str, str]]:
    """NovaCarter 처럼 보이는 variant 구조면 권장 매핑 자동 생성.

    선택 우선순위:
      Configuration : Base > No_Internals > Skirt_only > Full_Merged > (first)
      Physics       : Physics_Base > (first)
      Sensors       : All_Sensors > None > (first)
    """
    PREF = {
        "Configuration": ["Base", "No_Internals", "no_internals",
                          "Skirt_only", "skirt_only",
                          "Full_Merged", "Fully Merged", "full_merged"],
        "Physics":       ["Physics_Base", "physics_base", "Base"],
        "Sensors":       ["All_Sensors", "all_sensors", "None", "none"],
    }

    # 첫 번째 prim 의 variant set 만 본다 (보통 NovaCarter 의 root)
    if not prims:
        return None
    first_prim = next(iter(prims))
    sets_dict = prims[first_prim]

    has_nova = any(s in sets_dict for s in PREF)
    if not has_nova:
        return None

    mapping: Dict[str, str] = {}
    for set_name, prefs in PREF.items():
        info = sets_dict.get(set_name)
        if not info:
            continue
        variants: List[str] = list(info.get("variants", []))  # type: ignore
        if not variants:
            continue
        chosen = next((p for p in prefs if p in variants), variants[0])
        mapping[set_name] = chosen
    return mapping or None


def _print_human(report: Dict[str, object]) -> None:
    print(f"=== {report['path']}")
    default_prim = report.get("default_prim") or "(none)"
    print(f"Default prim: {default_prim}")
    prims = report.get("prims", {})  # type: ignore
    if not prims:
        print("\n  [INFO] No variant sets found on any prim.")
        print("    → 이 USD 는 variant-set 컨테이너가 아니거나, pxr 가")
        print("      composition 단계에서 variant metadata 를 읽지 못함.")
        print("    → inspect_usd_refs.py --dump-strings 로 raw 토큰을 확인하세요.")
        return

    for prim_path, sets_dict in prims.items():  # type: ignore
        print()
        print(f"Variant sets on {prim_path} ({len(sets_dict)}):")
        for set_name, info in sets_dict.items():
            default = info.get("default") or "(unset)"
            variants = info.get("variants") or []
            print(f"\n  {set_name} (default = '{default}')")
            if not variants:
                print("    (no variants found — may be defined in sublayer)")
                continue
            for v in variants:
                marker = "* " if v == default else "  "
                print(f"    {marker}{v}")

    suggested = _suggest_nova_carter_mapping(prims)  # type: ignore
    if suggested:
        print()
        print("Suggested asset_catalog mapping:")
        print("    variant_selection={")
        width = max(len(k) for k in suggested) + 2
        for k, v in suggested.items():
            key = f'"{k}":'.ljust(width + 3)
            print(f'        {key}"{v}",')
        print("    },")
        print()
        print("→ fdw_sim/visualization/asset_catalog.py 의 해당 항목을 위 값으로")
        print("  교체하면 됩니다. sensor sub-USD 가 누락된 경우 Sensors = 'None'")
        print("  으로 변경해서 payload 다운로드 부담을 피할 수도 있습니다.")


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description="USD variant set/variant 이름을 pxr.Usd 로 정확히 dump.")
    p.add_argument("usd", type=Path, help="분석할 USD 파일 경로")
    p.add_argument("--recurse", "-r", action="store_true",
                   help="default prim 외에 모든 prim 을 traverse")
    p.add_argument("--json", action="store_true",
                   help="결과를 JSON 으로 출력 (script-friendly)")
    args = p.parse_args(argv)

    try:
        report = dump_variants(args.usd, recurse=args.recurse)
    except FileNotFoundError as e:
        print(f"[FAIL] {e}", file=sys.stderr)
        return 2
    except RuntimeError as e:
        print(f"[FAIL] {e}", file=sys.stderr)
        return 3

    if args.json:
        suggested = _suggest_nova_carter_mapping(report.get("prims", {}))  # type: ignore
        report["suggested_mapping"] = suggested
        print(json.dumps(report, indent=2, ensure_ascii=False))
    else:
        _print_human(report)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
