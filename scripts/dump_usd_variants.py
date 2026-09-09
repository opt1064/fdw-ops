"""dump_usd_variants.py — USD 의 variant set / variant 이름을 실측 dump.

inspect_usd_refs.py 는 USDC 바이너리에서 regex 로 토큰을 뽑기 때문에 variant
이름 spelling 이 부분적으로만 보인다 (TOKENS 테이블의 압축/분할 영향). 이
스크립트는 **Isaac Sim 의 정공 boot 경로(IsaacLab AppLauncher)** 를 거쳐
pxr 가 완전히 로드된 상태에서 stage 를 열어 variant 이름을 한 번에 정확히
추출한다.

배경 (2026-05 Thor 진단 결과)::

    Isaac Sim 5.1 의 wheel install (pip install isaacsim) 환경에서는 pxr
    가 PEP 420 namespace package 로서 ``$CONDA_PREFIX/lib/python3.11/
    site-packages/isaacsim/extscache/`` 아래 **여러 개의** 패키지
    (omni.usd.libs-*, omni.usd.schema.physx-*, omni.anim.navigation.schema-*,
    ...) 에 ``pxr/`` 가 분산되어 있다.

    단순히 ``sys.path.insert`` + ``LD_LIBRARY_PATH`` 만으로는 .so 간 의존
    경로가 해결되지 않아 import 가 실패한다. 따라서 Isaac Sim 의 정식 boot
    sequence (SimulationApp / AppLauncher) 를 거치는 것이 유일하게 검증된
    방법이다. 이 스크립트는 그 패턴을 따른다.

사용법::

    # IsaacLab 의 conda env 가 활성화된 상태에서 (Thor 환경)
    conda activate isaac_sim
    cd ~/isaac_workspace/projects/fdw-sim

    # 기본 — AppLauncher headless 부팅 후 NovaCarter variant 덤프
    python scripts/dump_usd_variants.py \
        ~/isaac_assets/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd

    # JSON 으로 저장
    python scripts/dump_usd_variants.py --json \
        ~/isaac_assets/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd \
        > nova_carter_variants.json

    # default prim 만이 아니라 stage 전체를 traverse
    python scripts/dump_usd_variants.py --recurse \
        ~/isaac_assets/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd

    # IsaacLab 미설치 환경에서는 isaacsim 메타 패키지로 fallback (양쪽 모두
    # 없으면 마지막으로 plain ``from pxr import Usd`` 를 시도하지만, Isaac
    # Sim 5.1 wheel install 에서는 거의 항상 실패한다)
    ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
        python scripts/dump_usd_variants.py ...

오버헤드::

    AppLauncher 부팅은 첫 실행 시 ~15s (셰이더 캐시 미생성) ~ 30s 가 걸린다.
    이후 부팅은 ~10s 정도로 안정화. 일회성 진단용이므로 감내한다.

비-AppLauncher 사용 (legacy / 추출 전용)::

    스크립트가 ``USD_DUMP_NO_APP=1`` 환경변수를 인지하면 SimulationApp 부팅을
    건너뛰고 ``Usd.Stage.Open`` 만으로 시도한다 (sys.path 가 이미 pxr 를
    가진 환경 — 예: Isaac Sim deb install — 에서만 의미가 있음).

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
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


# =============================================================================
# pxr loader — AppLauncher (정공) → SimulationApp fallback → plain import
# =============================================================================
def _boot_isaac_app(headless: bool = True
                    ) -> Tuple[Optional[Any], Optional[Any]]:
    """Isaac Sim 의 SimulationApp 을 부팅한다.

    Isaac Sim 5.1 wheel install 환경에서 pxr 를 import 가능 상태로 만들기
    위한 **유일하게 검증된 절차**. 부팅 순서:

      1) ``from isaaclab.app import AppLauncher`` 가 성공하면 그 경로 사용
         (run_poc1_level2_2.py 와 동일한 패턴)
      2) ImportError 면 ``from isaacsim import SimulationApp`` 로 폴백
      3) 둘 다 실패 → (None, None) 반환 후 호출자가 plain import 시도

    Returns:
        (simulation_app, launcher) — launcher 는 AppLauncher 인스턴스 또는
        None (SimulationApp fallback path 인 경우)
    """
    # AppLauncher 부팅 전에 안전한 기본값 (EULA 동의)
    os.environ.setdefault("ACCEPT_EULA", "Y")
    os.environ.setdefault("PRIVACY_CONSENT", "Y")
    # Isaac Sim Kit 가 root 권한으로 실행되어도 막지 않음 (sandbox/CI)
    os.environ.setdefault("OMNI_KIT_ALLOW_ROOT", "1")

    # 1) IsaacLab AppLauncher — 정공
    try:
        from isaaclab.app import AppLauncher  # type: ignore

        launcher_args = {
            "headless": headless,
            # AppLauncher 는 livestream=0 이 기본. 진단용이므로 GUI 불필요.
        }
        print(f"[INFO] Booting Isaac Sim via IsaacLab AppLauncher "
              f"(headless={headless})…", file=sys.stderr)
        launcher = AppLauncher(launcher_args)
        app = launcher.app
        print("[INFO] AppLauncher boot complete — pxr should now be importable",
              file=sys.stderr)
        return app, launcher
    except ImportError as e:
        print(f"[INFO] isaaclab.app 가 없습니다 ({e}). "
              "SimulationApp fallback 시도…", file=sys.stderr)

    # 2) isaacsim.SimulationApp fallback
    try:
        from isaacsim import SimulationApp  # type: ignore

        app_kwargs = {"headless": headless}
        print(f"[INFO] Booting Isaac Sim via SimulationApp "
              f"(headless={headless})…", file=sys.stderr)
        app = SimulationApp(app_kwargs)
        print("[INFO] SimulationApp boot complete — pxr should now be importable",
              file=sys.stderr)
        return app, None
    except ImportError as e:
        print(f"[INFO] isaacsim 도 없습니다 ({e}). "
              "마지막 수단으로 plain ``import pxr`` 시도", file=sys.stderr)

    return None, None


def _import_pxr_after_boot() -> Any:
    """SimulationApp 부팅 이후 pxr.Usd 를 import.

    부팅 후에는 Isaac Sim 의 ext system 이 extscache 의 모든 pxr 분할 패키지
    경로를 sys.path / LD_LIBRARY_PATH 에 추가해 둔 상태이므로 표준 import 로
    충분하다.
    """
    try:
        from pxr import Usd  # type: ignore
        return Usd
    except ImportError as e:
        print(f"[FAIL] SimulationApp 부팅 이후에도 pxr import 실패: {e}",
              file=sys.stderr)
        print("       Isaac Sim 5.1 설치 / extscache 상태를 확인하세요.",
              file=sys.stderr)
        raise SystemExit(2)


def _import_pxr_no_boot() -> Any:
    """SimulationApp 부팅 없이 pxr.Usd 를 import.

    Isaac Sim 의 deb install 같은 환경, 또는 호출자가 명시적으로
    ``USD_DUMP_NO_APP=1`` 을 지정한 경우에만 시도한다. Isaac Sim 5.1 wheel
    install 에서는 거의 항상 실패한다 (namespace package + .so inter-dep).
    """
    try:
        from pxr import Usd  # type: ignore
        print("[INFO] pxr loaded without SimulationApp boot "
              "(sys.path already contains pxr).", file=sys.stderr)
        return Usd
    except ImportError as e:
        print(f"[FAIL] plain ``import pxr`` 실패: {e}", file=sys.stderr)
        print("", file=sys.stderr)
        print("USD_DUMP_NO_APP=1 을 지정하셨다면 해제 후 재시도하세요. "
              "Isaac Sim 5.1 wheel install 환경에서는 AppLauncher / "
              "SimulationApp boot 가 필수입니다.", file=sys.stderr)
        raise SystemExit(2)


# =============================================================================
# variant 수집
# =============================================================================
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


def _get_tf_error_mark_cls():
    """Isaac Sim 5.1 의 pxr.Tf 에서 ``ErrorMark`` 를 안전하게 가져온다.

    Thor 실측: 일부 Isaac Sim 5.1 wheel 빌드의 ``pxr.Tf`` 는 ``ErrorMark``
    심볼을 노출하지 않는다 (``module 'pxr.Tf' has no attribute 'ErrorMark'``).
    이런 환경에서도 동작하도록, fallback 으로 누구나 받아들이는 더미
    context manager 를 돌려준다.

    Returns:
        Tf.ErrorMark 호환 클래스 — IsClean()/GetErrors()/Clear() 시그니처를
        모두 충족. 진짜가 없는 경우엔 항상 IsClean()=True 를 반환하는 더미.
    """
    try:
        from pxr import Tf  # type: ignore
        em = getattr(Tf, "ErrorMark", None)
        if em is not None:
            return em
    except Exception:
        pass

    class _DummyErrorMark:
        """ErrorMark 가 없는 환경용 no-op shim.

        실제로는 USD composition error 가 발생해도 detect 할 수 없지만,
        ``Usd.Stage.Open`` 자체가 ``RuntimeError`` 를 던지지 않는다면
        그대로 stage 를 돌려주는 것으로 충분하다. 던지면 except 절이 잡는다.
        """
        def __init__(self):  # noqa: D401
            self._cleared = True
        def IsClean(self) -> bool:
            return True
        def GetErrors(self):
            return []
        def Clear(self) -> None:
            self._cleared = True
        def __enter__(self):
            return self
        def __exit__(self, *a):
            return False

    return _DummyErrorMark


def _open_stage_tolerant(Usd, usd_path: Path):
    """Composition error 가 발생해도 stage 객체를 돌려받기 위한 헬퍼.

    Thor 실측에서 NovaCarter 의 ``nova_carter.usd`` 는 variant 선택을
    명시하지 않은 상태로 ``Usd.Stage.Open`` 하면 sublayer 누락 →
    ``Error in 'operator()' at line 2549 ... 'arcNum < srcInfo.size()'``
    composition assertion 이 RuntimeError 로 Python 까지 전파되어
    stage 를 다루지도 못한 채 죽어버린다.

    하지만 variant set / variant 이름 메타데이터는 ``nova_carter.usd``
    의 root layer 에 직접 박혀 있으므로, **sublayer/payload 가 일부
    실패해도** root layer 만 열리면 variant 덤프는 가능하다.

    전략 (Thor 5.1 호환성 보강 후):
      1) ``Sdf.Layer.FindOrOpen`` + ``Stage.Open(layer, LoadNone)``
         — payload 따라가지 않음. Tf.ErrorMark 가 있으면 흡수, 없으면
         그대로 통과 (USD 가 RuntimeError 를 던지지 않는 한 OK).
      2) plain ``Stage.Open(str(path))`` — payload 까지 따라가지만
         variant metadata 가 더 풍부할 수 있음.
      3) Anonymous sublayer 트릭 — root layer 를 anon root 의 sublayer 로
         박아 composition 격리.

    2026-05-11 Thor 실측: 1) 단계가 ``Tf.ErrorMark`` AttributeError 로
    빠지면 2) 가 성공적으로 root layer 를 열어 variant 덤프 가능 (실측).
    이 경로를 빠르게 만들기 위해 1) 의 ``Tf.ErrorMark`` 가 없을 때엔
    no-op shim 으로 대체한다.
    """
    from pxr import Sdf  # type: ignore
    ErrorMarkCls = _get_tf_error_mark_cls()

    # 1) 가장 보수적: SessionLayer 없이 root layer 만 열고 payload load=None
    try:
        mark = ErrorMarkCls()
        layer = Sdf.Layer.FindOrOpen(str(usd_path))
        if layer is None:
            raise RuntimeError(f"Sdf.Layer.FindOrOpen returned None for {usd_path}")
        stage = Usd.Stage.Open(layer, load=Usd.Stage.LoadNone)
        # ErrorMark 안의 모든 USD error 는 stderr 로 한 줄씩만 흘리고 흡수.
        # 더미 shim 은 항상 IsClean()=True 라 이 블록은 그냥 통과한다.
        try:
            if not mark.IsClean():
                errors = list(mark.GetErrors())
                for e in errors[:6]:
                    try:
                        msg = e.commentary.splitlines()[0]
                    except Exception:
                        msg = str(e)
                    print(f"    [usd-warn] {msg}", file=sys.stderr)
                if len(errors) > 6:
                    print(f"    [usd-warn] ({len(errors) - 6} more errors hidden)",
                          file=sys.stderr)
                mark.Clear()
        except Exception:
            # ErrorMark 인터페이스가 예상 외로 다른 경우 — 무시
            pass
        if stage is not None:
            return stage
    except Exception as e1:
        print(f"    [warn] tolerant open (LoadNone) failed: {e1}", file=sys.stderr)

    # 2) Fallback — payload 까지 따라가는 일반 open. 일부 케이스에서는 더 많은
    #    variant 가 보이지만, composition assertion 으로 죽을 수 있다.
    try:
        stage = Usd.Stage.Open(str(usd_path))
        if stage is not None:
            return stage
    except Exception as e2:
        # 마지막 시도 — layer 단독 open 후 anonymous root 에 sublayer 로 끼움
        try:
            layer = Sdf.Layer.FindOrOpen(str(usd_path))
            if layer is not None:
                anon = Sdf.Layer.CreateAnonymous("dump_variants_root")
                anon.subLayerPaths.append(layer.identifier)
                return Usd.Stage.Open(anon, load=Usd.Stage.LoadNone)
        except Exception as e3:
            print(f"    [warn] anonymous-sublayer open failed: {e3}",
                  file=sys.stderr)
        raise RuntimeError(
            f"All Usd.Stage.Open attempts failed for {usd_path}: {e2}")

    # 2) 가 None 을 돌려준 경우 — 마지막 anonymous-sublayer 트릭
    try:
        layer = Sdf.Layer.FindOrOpen(str(usd_path))
        if layer is not None:
            anon = Sdf.Layer.CreateAnonymous("dump_variants_root")
            anon.subLayerPaths.append(layer.identifier)
            return Usd.Stage.Open(anon, load=Usd.Stage.LoadNone)
    except Exception as e3:
        print(f"    [warn] anonymous-sublayer open failed: {e3}",
              file=sys.stderr)
    raise RuntimeError(
        f"All Usd.Stage.Open attempts returned None for {usd_path}")


def dump_variants(Usd, usd_path: Path, recurse: bool = False
                  ) -> Dict[str, object]:
    """USD 의 variant 구조를 dict 로 반환.

    Args:
        Usd:      이미 import 된 pxr.Usd 모듈
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

    Note:
        Composition error 가 발생해도 root layer 만 열어 variant 메타데이터
        는 정상 추출한다 (``_open_stage_tolerant``). sublayer / payload 가
        없어도 variant 이름은 root layer 에 박혀 있으므로 OK.
    """
    if not usd_path.is_file():
        raise FileNotFoundError(f"USD not found: {usd_path}")

    stage = _open_stage_tolerant(Usd, usd_path)
    if stage is None:
        raise RuntimeError(f"Failed to open stage for {usd_path}")

    result: Dict[str, object] = {
        "path": str(usd_path.resolve()),
        "default_prim": "",
        "prims": {},
    }

    # default prim — composition 실패 시 None 일 수 있으므로 root layer 의
    # default prim spec 으로 fallback.
    default_prim_path: Optional[str] = None
    try:
        default = stage.GetDefaultPrim()
        if default and default.IsValid():
            default_prim_path = str(default.GetPath())
            v = _collect_variants_on_prim(default)
            if v:
                result["prims"][default_prim_path] = v  # type: ignore[index]
    except Exception as e:
        print(f"    [warn] GetDefaultPrim failed: {e}", file=sys.stderr)

    # composition 이 실패해 GetDefaultPrim 이 비어 있어도, root layer 의
    # defaultPrim 토큰으로 직접 prim 을 잡아본다.
    if default_prim_path is None:
        try:
            root_layer = stage.GetRootLayer()
            dp = root_layer.defaultPrim  # type: ignore[attr-defined]
            if dp:
                default_prim_path = f"/{dp}"
                prim = stage.GetPrimAtPath(default_prim_path)
                if prim and prim.IsValid():
                    v = _collect_variants_on_prim(prim)
                    if v:
                        result["prims"][default_prim_path] = v  # type: ignore[index]
        except Exception as e:
            print(f"    [warn] root-layer defaultPrim fallback failed: {e}",
                  file=sys.stderr)

    result["default_prim"] = default_prim_path or ""

    if recurse:
        try:
            for prim in stage.TraverseAll():
                path_str = str(prim.GetPath())
                if path_str in result["prims"]:  # type: ignore[operator]
                    continue
                v = _collect_variants_on_prim(prim)
                if v:
                    result["prims"][path_str] = v  # type: ignore[index]
        except Exception as e:
            print(f"    [warn] TraverseAll failed: {e}", file=sys.stderr)

    return result


def _suggest_nova_carter_mapping(prims: Dict[str, Dict[str, object]]
                                  ) -> Optional[Dict[str, str]]:
    """NovaCarter 처럼 보이는 variant 구조면 권장 매핑 자동 생성.

    선택 우선순위 (CreateJoint body0/body1 누락 회피 + sub-USD payload
    의존 최소화):

      Configuration : No_Internals > Base > Skirt_only > Fully Merged > (first)
      Physics       : Physics_Base > (first)
      Sensors       : None > All_Sensors > (first)   # ← payload 의존 회피
    """
    PREF = {
        "Configuration": [
            "No_Internals", "no_internals",
            "Base",
            "Skirt_only", "skirt_only",
            "Fully Merged", "Full_Merged", "full_merged",
        ],
        "Physics": ["Physics_Base", "physics_base", "Base"],
        # Sensors 는 "None" 우선 — Hawk/Owl/RPLidar/XT-32 sub-USD payload 회피
        "Sensors": ["None", "none", "All_Sensors", "all_sensors"],
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
        print("  으로 두면 payload 다운로드 부담을 피할 수 있습니다.")


# =============================================================================
# main
# =============================================================================
def _run(usd_path: Path, recurse: bool, want_json: bool,
         no_app: bool, headless: bool) -> int:
    """변환 가능한 pxr 모듈을 확보한 뒤 dump 를 실행한다.

    no_app=True : SimulationApp 부팅 건너뜀 (USD_DUMP_NO_APP=1 과 동일)
    """
    app = None
    launcher = None  # noqa: F841 (held for life-cycle reasons)

    try:
        if no_app:
            Usd = _import_pxr_no_boot()
        else:
            app, launcher = _boot_isaac_app(headless=headless)
            if app is None:
                # 부팅 자체가 불가능 → plain import 마지막 시도
                Usd = _import_pxr_no_boot()
            else:
                Usd = _import_pxr_after_boot()

        try:
            report = dump_variants(Usd, usd_path, recurse=recurse)
        except FileNotFoundError as e:
            print(f"[FAIL] {e}", file=sys.stderr)
            return 2
        except RuntimeError as e:
            print(f"[FAIL] {e}", file=sys.stderr)
            return 3

        if want_json:
            suggested = _suggest_nova_carter_mapping(report.get("prims", {}))  # type: ignore
            report["suggested_mapping"] = suggested  # type: ignore[index]
            print(json.dumps(report, indent=2, ensure_ascii=False))
        else:
            _print_human(report)

        return 0
    finally:
        # SimulationApp 은 명시적으로 close 해야 깔끔하게 종료
        if app is not None:
            try:
                app.close()
            except Exception:
                pass


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description="USD variant set/variant 이름을 pxr.Usd 로 정확히 dump "
                    "(IsaacLab AppLauncher 경로).")
    p.add_argument("usd", type=Path, help="분석할 USD 파일 경로")
    p.add_argument("--recurse", "-r", action="store_true",
                   help="default prim 외에 모든 prim 을 traverse")
    p.add_argument("--json", action="store_true",
                   help="결과를 JSON 으로 출력 (script-friendly)")
    p.add_argument("--no-app", action="store_true",
                   default=bool(os.environ.get("USD_DUMP_NO_APP")),
                   help="SimulationApp 부팅을 건너뛰고 plain ``import pxr`` "
                        "만 시도 (sys.path 에 pxr 가 이미 있는 환경 전용). "
                        "USD_DUMP_NO_APP=1 환경변수로도 동일하게 활성화.")
    p.add_argument("--no-headless", dest="headless", action="store_false",
                   default=True,
                   help="AppLauncher 를 GUI 모드로 부팅 (보통 불필요).")
    args = p.parse_args(argv)

    return _run(usd_path=args.usd, recurse=args.recurse, want_json=args.json,
                no_app=args.no_app, headless=args.headless)


if __name__ == "__main__":
    raise SystemExit(main())
