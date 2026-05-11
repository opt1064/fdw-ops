"""dump_usd_variants.py — USD 의 variant set / variant 이름을 실측 dump.

inspect_usd_refs.py 는 USDC 바이너리에서 regex 로 토큰을 뽑기 때문에 variant
이름 spelling 이 부분적으로만 보인다 (TOKENS 테이블의 압축/분할 영향).
이 스크립트는 pxr.Usd 로 stage 를 열어 라이브 composition 결과의 variant
이름을 한 번에 정확히 추출한다.

사용법:

    # 가장 간단 — conda env 만 활성화하면 자동으로 isaacsim extscache 에서
    # pxr 를 찾아 sys.path 에 주입한다 (SimulationApp 부팅 불필요).
    conda activate isaac_sim
    cd ~/isaac_workspace/projects/fdw-sim
    python scripts/dump_usd_variants.py \
        ~/isaac_assets/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd

    # 자동 발견이 실패하면 환경변수로 직접 지정 가능:
    FDW_PXR_PATH=/path/to/pxr/parent_dir \
        python scripts/dump_usd_variants.py ...

    # JSON 으로 저장해서 asset_catalog 수정에 활용
    python scripts/dump_usd_variants.py --json \
        ~/isaac_assets/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd \
        > nova_carter_variants.json

Isaac Sim 5.1 wheel 설치 (pip install isaacsim) 의 경우 pxr 는
``<conda_env>/lib/pythonX.Y/site-packages/isaacsim/extscache/
omni.usd.libs-*/pxr/`` 안에 있다. 일반 ``import pxr`` 로는 잡히지
않으므로 이 스크립트가 해당 경로를 자동 탐색해 sys.path 에 추가한다.

Importantly: 이 도구는 reference attach 를 하지 않고 ``Usd.Stage.Open()`` 으로
직접 USD layer 만 연다. 따라서 sub-USD payload 가 누락된 상태에서도 variant
set / variant 이름만은 안전하게 추출할 수 있다 (compose 실패 경고는 나올 수
있지만 variant 토큰은 메인 USD layer 에 박혀 있다).
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import sysconfig
from pathlib import Path
from typing import Dict, List, Optional


def _candidate_extscache_roots() -> List[Path]:
    """isaacsim 의 extscache 가 있을 만한 디렉토리 후보 나열.

    Isaac Sim 5.1 wheel 설치 시 pxr 는 다음 위치에 있다:
        <site-packages>/isaacsim/extscache/omni.usd.libs-*/pxr/

    이 함수는 그 부모(`omni.usd.libs-*` 의 부모) 를 추출 가능한 모든
    site-packages 에서 모은다.
    """
    roots: List[Path] = []

    # 1) 현재 인터프리터의 site-packages
    try:
        purelib = Path(sysconfig.get_paths()["purelib"])
        roots.append(purelib / "isaacsim" / "extscache")
    except Exception:
        pass

    # 2) sys.path 에 있는 site-packages 모두
    for p in sys.path:
        if not p:
            continue
        pp = Path(p)
        if pp.name in ("site-packages", "dist-packages"):
            roots.append(pp / "isaacsim" / "extscache")

    # 3) CONDA_PREFIX 가 있으면 그 안의 표준 위치
    conda_prefix = os.environ.get("CONDA_PREFIX")
    if conda_prefix:
        for py in ("python3.11", "python3.10", "python3.12", "python3.9"):
            cand = (Path(conda_prefix) / "lib" / py / "site-packages"
                    / "isaacsim" / "extscache")
            roots.append(cand)

    # 4) 사용자 환경변수로 직접 지정 가능
    env = os.environ.get("FDW_PXR_PATH")
    if env:
        # FDW_PXR_PATH 는 pxr 의 부모 dir (= sys.path 에 추가될 경로) 를 직접 지정
        # 그러면 _find_pxr_dir 가 곧장 사용
        return [Path(env)]

    # 5) IsaacLab _isaac_sim 경로
    home = Path.home()
    for p in (home / "IsaacLab" / "_isaac_sim",
              home / "isaac-sim",
              Path("/opt/isaac-sim"),
              Path("/isaac-sim")):
        roots.append(p / "extscache")

    # 중복 제거 (순서 유지)
    seen: set = set()
    uniq: List[Path] = []
    for r in roots:
        s = str(r)
        if s in seen:
            continue
        seen.add(s)
        uniq.append(r)
    return uniq


def _find_pxr_dir() -> Optional[Path]:
    """pxr 패키지 부모 디렉토리(= sys.path 에 추가될 경로) 자동 탐색.

    Isaac Sim 5.1 extscache 구조:
        <extscache>/omni.usd.libs-<ver>/pxr/__init__.py
    sys.path 에 추가해야 할 건 ``<extscache>/omni.usd.libs-<ver>/`` 부모.

    FDW_PXR_PATH 환경변수가 있으면 그 값을 그대로 사용 (위 _candidate 가
    이미 단일 후보로 반환했다면 그 안의 ``pxr`` 존재만 확인).
    """
    env = os.environ.get("FDW_PXR_PATH")
    if env:
        p = Path(env)
        if (p / "pxr" / "__init__.py").is_file():
            return p
        # FDW_PXR_PATH 가 pxr 부모(=parent of parent) 인 경우도 허용
        for child in p.glob("omni.usd.libs*"):
            if (child / "pxr" / "__init__.py").is_file():
                return child
        return None

    for root in _candidate_extscache_roots():
        if not root.is_dir():
            continue
        # omni.usd.libs-<version>/pxr/__init__.py 패턴
        for child in sorted(root.glob("omni.usd.libs*")):
            init = child / "pxr" / "__init__.py"
            if init.is_file():
                return child
    return None


def _import_pxr():
    """pxr.Usd 를 import 시도. 실패 시 extscache 자동 발견 후 재시도.

    Isaac Sim 5.1 wheel 설치 환경에서는 pxr 가 sys.path 에 없으므로
    isaacsim/extscache/omni.usd.libs-*/ 를 sys.path 에 추가한다.
    """
    # 1) 정공법 — 이미 sys.path 에 있으면 OK
    try:
        from pxr import Usd  # type: ignore
        return Usd
    except ImportError:
        pass

    # 2) 자동 발견
    pxr_parent = _find_pxr_dir()
    if pxr_parent is not None:
        sys.path.insert(0, str(pxr_parent))
        # extscache 안의 omni.usd.libs 는 pxr 의 .so 들이 RPATH 로 같은 디렉토리
        # 의 다른 .so 들을 참조하는 경우가 많다. LD_LIBRARY_PATH 도 보강.
        ld = os.environ.get("LD_LIBRARY_PATH", "")
        if str(pxr_parent) not in ld:
            os.environ["LD_LIBRARY_PATH"] = (
                f"{pxr_parent}:{ld}" if ld else str(pxr_parent))
        try:
            from pxr import Usd  # type: ignore
            # 성공 — 사용자에게 어디서 찾았는지 알려준다
            print(f"[INFO] pxr loaded from: {pxr_parent}", file=sys.stderr)
            return Usd
        except ImportError as e2:
            print(f"[WARN] pxr 부모 디렉토리는 찾았지만 import 가 여전히 실패: {e2}",
                  file=sys.stderr)
            print(f"       경로: {pxr_parent}", file=sys.stderr)
            print("       LD_LIBRARY_PATH 보강 후에도 의존 .so 가 없을 수 있음.",
                  file=sys.stderr)

    # 3) 최종 실패 — 사용자에게 친절한 진단 메시지
    print("[FAIL] pxr.Usd 를 어떻게도 import 할 수 없습니다.", file=sys.stderr)
    print("", file=sys.stderr)
    print("탐색한 위치:", file=sys.stderr)
    for cand in _candidate_extscache_roots()[:6]:
        marker = "OK " if cand.is_dir() else "X  "
        print(f"  [{marker}] {cand}", file=sys.stderr)
    print("", file=sys.stderr)
    print("다음 중 하나를 시도하세요:", file=sys.stderr)
    print("  1) conda activate isaac_sim  (이미 했다면 다음 단계로)", file=sys.stderr)
    print("  2) FDW_PXR_PATH 로 pxr 부모 디렉토리 직접 지정:", file=sys.stderr)
    print("     find $CONDA_PREFIX -path '*/pxr/__init__.py' 2>/dev/null", file=sys.stderr)
    print("     → 출력된 경로의 ``../`` (pxr 의 부모) 를 FDW_PXR_PATH 로 export",
          file=sys.stderr)
    print("       예: export FDW_PXR_PATH=$CONDA_PREFIX/lib/python3.11/"
          "site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+...", file=sys.stderr)
    print("  3) SimulationApp 부팅을 거치는 wrapper 로 실행:", file=sys.stderr)
    print("     ~/IsaacLab/isaaclab.sh -p scripts/dump_usd_variants.py ...",
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
