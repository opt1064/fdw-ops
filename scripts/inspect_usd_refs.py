"""inspect_usd_refs.py — USD 파일의 sub-USD 의존성 추출 진단 도구.

NovaCarter의 `PhysicsUSD::CreateJoint - no bodies defined at body0 and body1`
경고처럼 sub-USD 파일이 누락됐을 때, 어떤 sub-USD를 실제로 받아야 하는지
확인하기 위한 도구.

원리:
    Isaac Sim USD 파일은 거의 모두 USDC(바이너리)이지만, 그 안에 포함된
    sub-USD 참조 경로(`@.../foo.usd@`, `@.../bar.usd@</prim>`)는 일반 텍스트로
    저장되어 있어 strings/grep 으로 안정적으로 추출할 수 있다.

    또한 PhysicsJoint 가 참조하는 body prim path(rel rigidbody, body0, body1
    relationship target)도 텍스트로 박혀 있어 추출 가능.

사용법:
    # 기본 사용 (NovaCarter 의존성 분석)
    python scripts/inspect_usd_refs.py \
        ~/isaac_assets/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd

    # 재귀적으로 따라가며 모두 추출 (max-depth 4)
    python scripts/inspect_usd_refs.py --recursive \
        ~/isaac_assets/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd

    # 누락된 sub-USD 만 보기 (다운로드 누락 진단)
    python scripts/inspect_usd_refs.py --recursive --only-missing \
        ~/isaac_assets/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd

    # Joint body 참조도 함께 추출 (Physics 누락 body prim 진단)
    python scripts/inspect_usd_refs.py --joints \
        ~/isaac_assets/Isaac/Robots/NVIDIA/NovaCarter/nova_carter_physics.usd

    # download_isaac_assets.sh 가 받아야 할 subpath 형태로 출력
    python scripts/inspect_usd_refs.py --recursive --print-subpaths \
        ~/isaac_assets/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd

Isaac Sim/Omni 환경 없이 동작 — strings/grep 만 필요.
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path
from typing import Iterable, List, Optional, Set, Tuple


# =============================================================================
# 추출 패턴 — Isaac USDC 포맷은 다음 인코딩들을 혼용함:
#   1) `@<path>@`              — 표준 reference/payload 표기 (.usda 평문)
#   2) `@<path>@</Prim>`       — reference + target prim
#   3) `@./<path>@`            — 상대 경로 reference
#   4) USDC 바이너리: 경로 문자열이 길이-prefix 또는 null-terminated 로 박혀 있음
#      → `@` 래퍼 없이 그냥 `.usd`/`.usda`/`.usdc` 로 끝나는 ASCII 문자열만 추출
# =============================================================================

# Pattern 1: `@<path>@` 또는 `@<path>@</prim>` — 평문 USDA
REF_PATTERN_AT = re.compile(rb"@([^@\x00\n\r]{1,512}\.(?:usd|usda|usdc))(?:@|<)")

# Pattern 2: USDC 바이너리 — `.usd[a|c]` 로 끝나는 ASCII path
# 경로 문자: 알파벳/숫자/`. _ - / : #` 정도. 양 끝은 non-printable 로 끊어짐.
# `@` 는 제외 — Pattern AT 가 잡은 `@...@` 와 중복 매치 방지.
# 너무 짧은 매치(< 5 chars) 제외, omniverse:// 류 URI 도 포함.
REF_PATTERN_PATH = re.compile(
    rb"([./A-Za-z0-9_\-:#+]{5,512}\.(?:usd|usda|usdc))(?=[^A-Za-z0-9_\-./]|$)"
)

# Physics joint 가 참조하는 body relationship target — 보통 prim path 형태
# 예: physics:body0 = </NovaCarter/chassis>
JOINT_BODY_PATTERN = re.compile(
    rb"physics:body[01].*?</?([A-Za-z_][A-Za-z0-9_/]*)>", re.DOTALL)


# Variant-set composition 표식 — USDC TOKENS 테이블에 박혀 있는 키워드.
# 이 중 어느 하나라도 strings 에서 발견되면 해당 USD 는 variant-set 컨테이너
# 일 가능성이 높음 (NovaCarter 처럼 4 KB wrapper 가 아니라 variant payload
# 를 lazy-load 하는 구조).
VARIANT_KEYWORDS = (
    "variantSel",       # variantSelection (자식 토큰)
    "variantSet",       # variantSets
    "variantSetNames",
)

# NVIDIA 공식 NovaCarter variant 이름 (Asset Structure 문서 기준).
# inspect 결과에 이 토큰들이 보이면 NovaCarter-style 자산으로 판단 가능.
NOVA_CARTER_VARIANT_HINTS = (
    "Configuration",
    "Physics",
    "Sensors",
    "Physics_Base",
    "No_Physics",
    "All_Sensors",
    "Skirt_only",
    "No_Internals",
    "Full_Merged",     # USDC 토큰화로 공백이 _ 로 보일 수 있음
    "Fully Merged",
)


def extract_strings(usd_path: Path) -> bytes:
    """USDC 바이너리에서 printable strings 만 추출.

    strings(1) 가 있으면 사용하고, 없으면 자체적으로 bytes 처리.
    """
    if not usd_path.is_file():
        raise FileNotFoundError(f"USD not found: {usd_path}")

    # strings(1) 가 더 안정적 (USDC 패딩/오프셋 처리 깔끔)
    try:
        out = subprocess.run(
            ["strings", "-a", "-n", "4", str(usd_path)],
            capture_output=True, check=False, timeout=30,
        )
        if out.returncode == 0 and out.stdout:
            return out.stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass

    # Fallback: 파일 raw bytes (regex 가 binary 매칭 가능)
    return usd_path.read_bytes()


def _is_usdc(usd_path: Path) -> bool:
    """USDC 바이너리 여부 (매직 헤더 'PXR-USDC' 검사)."""
    try:
        with open(usd_path, "rb") as f:
            return f.read(8) == b"PXR-USDC"
    except Exception:
        return False


def _find_usdcat() -> Optional[str]:
    """Isaac Sim / Omniverse 의 usdcat 바이너리 자동 탐색.

    PATH → 환경변수 FDW_USDCAT → Isaac Sim 표준 설치 경로 순으로 검색.
    """
    import os
    import shutil

    # 1) PATH
    found = shutil.which("usdcat")
    if found:
        return found

    # 2) 환경변수 직접 지정
    direct = os.environ.get("FDW_USDCAT")
    if direct and Path(direct).is_file():
        return direct

    # 3) Isaac Sim 표준 설치 경로 (5.x) + 사용자 워크스페이스
    home = Path.home()
    isaac_roots = [
        home / "isaac-sim",
        home / "isaacsim",
        home / "isaac_workspace",                 # 사용자 prompt 에서 확인됨
        home / "isaac_workspace" / "isaac-sim",
        home / "isaac_workspace" / "isaacsim",
        home / "Omniverse",
        home / ".local" / "share" / "ov" / "pkg",
        Path("/opt/isaac-sim"),
        Path("/opt/nvidia/isaac-sim"),
        Path("/isaac-sim"),
    ]
    isaac_env = os.environ.get("ISAACSIM_PATH")
    if isaac_env:
        isaac_roots.insert(0, Path(isaac_env))
    # CONDA_PREFIX 도 시도 (isaac_sim conda env)
    conda_prefix = os.environ.get("CONDA_PREFIX")
    if conda_prefix:
        isaac_roots.insert(0, Path(conda_prefix))

    sub_candidates = [
        "kit/usdcat",
        "kit/python/bin/usdcat",
        "bin/usdcat",
        "extscache/usd.schema.usdShade/bin/usdcat",
        "exts/usd.schema.usdShade/bin/usdcat",
        "python.sh",   # 직접 binary 없으면 python.sh 로 usdcat 모듈 실행 가능
    ]

    candidates = []
    for root in isaac_roots:
        if not root.is_dir():
            continue
        for sub in sub_candidates:
            cand = root / sub
            if cand.is_file():
                # python.sh 는 사용자가 직접 -m usdcat 호출해야 함 → 표시만
                if cand.name != "python.sh":
                    candidates.append(str(cand))
        # 깊이-3 까지만 glob (재귀 무한 회피)
        try:
            for depth_pattern in ("usdcat", "*/usdcat", "*/*/usdcat",
                                  "*/*/*/usdcat"):
                for p in root.glob(depth_pattern):
                    if p.is_file() and p.name == "usdcat":
                        candidates.append(str(p))
                        if len(candidates) >= 3:
                            break
                if len(candidates) >= 3:
                    break
        except Exception:
            pass
        if candidates:
            break

    return candidates[0] if candidates else None


def extract_refs(usd_path: Path,
                 include_path_pattern: bool = True) -> List[str]:
    """단일 USD 파일에서 sub-USD 참조 경로 목록 추출 (중복 제거, 순서 유지).

    두 패턴을 병행:
        - REF_PATTERN_AT  : `@<path>@` (USDA 표준)
        - REF_PATTERN_PATH: `.usd` 로 끝나는 ASCII path (USDC 바이너리)

    include_path_pattern=False 면 strict 모드 (`@...@` 만).
    """
    blob = extract_strings(usd_path)
    # raw bytes 도 함께 스캔 (strings 가 USDC 의 중첩 문자열을 놓치는 경우 대비)
    raw = usd_path.read_bytes()

    seen: Set[str] = set()
    refs: List[str] = []

    def _add(path: str) -> None:
        path = path.strip().strip("\x00")
        if not path or path in seen:
            return
        # 너무 일반적인 매치 필터링
        if path in (".usd", ".usda", ".usdc"):
            return
        seen.add(path)
        refs.append(path)

    # Pattern 1: `@<path>@`
    for source in (blob, raw):
        for m in REF_PATTERN_AT.finditer(source):
            try:
                _add(m.group(1).decode("utf-8", errors="ignore"))
            except Exception:
                continue

    # Pattern 2: USDC 바이너리에서 `.usd` 로 끝나는 ASCII path
    # 자기 자신을 가리키는 ref 제외 — USDC 헤더에 박힌 asset path 메타데이터를
    # ref 로 오인하는 false positive 방지. 비교 대상:
    #   - 파일 이름 (nova_carter.usd)
    #   - 절대 경로 (/home/.../nova_carter.usd)
    #   - Isaac/... subpath (Isaac/Robots/.../nova_carter.usd)
    #   - 끝부분 일치 (cand 가 절대경로의 suffix)
    abs_str = str(usd_path.resolve())
    self_names: Set[str] = {usd_path.name, abs_str}
    isaac_self = to_isaac_subpath(usd_path.resolve())
    if isaac_self:
        self_names.add(isaac_self)

    def _is_self_ref(cand: str) -> bool:
        if cand in self_names:
            return True
        # cand 가 절대 경로의 suffix 인 경우 (예: cand="Isaac/.../foo.usd",
        # abs_str="/home/.../Isaac/.../foo.usd")
        if abs_str.endswith("/" + cand) or abs_str.endswith(cand):
            return True
        # cand 가 파일 이름으로 끝나는데 같은 dir 구조면 self
        if cand.endswith("/" + usd_path.name) or cand == usd_path.name:
            # 단, ./foo.usd 같은 정상 상대 참조는 그대로 통과
            if not cand.startswith("./") and not cand.startswith("../"):
                # subpath 일치 검사: cand 의 마지막 N 컴포넌트가 abs_str 끝과 같으면 self
                cand_parts = tuple(cand.split("/"))
                abs_parts = Path(abs_str).parts
                if (len(cand_parts) <= len(abs_parts)
                        and abs_parts[-len(cand_parts):] == cand_parts):
                    return True
        return False

    if include_path_pattern:
        for source in (blob, raw):
            for m in REF_PATTERN_PATH.finditer(source):
                try:
                    cand = m.group(1).decode("utf-8", errors="ignore")
                except Exception:
                    continue
                # `.usd` 만 들어있는 것 (확장자 토큰) 제외
                if cand in (".usd", ".usda", ".usdc"):
                    continue
                # 자기 자신 제외 (강화된 검사)
                if _is_self_ref(cand):
                    continue
                _add(cand)

    return refs


def detect_variant_composition(usd_path: Path) -> Tuple[bool, List[str], List[str]]:
    """USD 가 variant-set 컨테이너인지 감지.

    Returns:
        (is_variant_container, found_keywords, found_variant_names)

        is_variant_container : VARIANT_KEYWORDS 중 하나 이상이 strings 에서
                                발견되면 True
        found_keywords       : 실제로 발견된 키워드 부분집합
        found_variant_names  : NOVA_CARTER_VARIANT_HINTS 중 발견된 토큰
                                (NovaCarter 같은 알려진 자산 식별용)
    """
    blob = extract_strings(usd_path)
    try:
        text = blob.decode("utf-8", errors="ignore")
    except Exception:
        text = ""

    found_keywords = [kw for kw in VARIANT_KEYWORDS if kw in text]
    found_variant_names = [v for v in NOVA_CARTER_VARIANT_HINTS if v in text]
    return (bool(found_keywords), found_keywords, found_variant_names)


def extract_joint_bodies(usd_path: Path) -> List[str]:
    """Physics joint 가 참조하는 body prim path 목록 (중복 제거)."""
    blob = extract_strings(usd_path)
    seen: Set[str] = set()
    bodies: List[str] = []
    for m in JOINT_BODY_PATTERN.finditer(blob):
        try:
            path = m.group(1).decode("utf-8", errors="ignore").strip()
        except Exception:
            continue
        if not path or path in seen:
            continue
        seen.add(path)
        bodies.append(path)
    return bodies


def classify(ref: str) -> str:
    """참조 경로 종류 분류 (absolute/relative/url)."""
    low = ref.lower()
    if low.startswith(("omniverse://", "http://", "https://", "file://")):
        return "url"
    if ref.startswith("/"):
        return "absolute"
    return "relative"


def normalize_relative(ref: str, base_usd: Path,
                       assets_root: Optional[Path] = None) -> Optional[Path]:
    """상대 참조를 base_usd 기준으로 절대 경로로 정규화."""
    if classify(ref) != "relative":
        return None
    # USD 의 `./foo.usd` `../bar.usd` 같은 상대 경로
    try:
        resolved = (base_usd.parent / ref).resolve()
    except Exception:
        return None
    return resolved


def to_isaac_subpath(abs_path: Path,
                     assets_root: Optional[Path] = None) -> Optional[str]:
    """절대 경로 → Isaac/... 형태의 subpath (download_isaac_assets.sh 호환)."""
    parts = abs_path.parts
    if "Isaac" not in parts:
        return None
    idx = parts.index("Isaac")
    return "/".join(parts[idx:])


def walk_refs(root_usd: Path,
              max_depth: int = 4,
              follow: bool = True
              ) -> Tuple[List[Tuple[Path, str, str, bool]], Set[Path]]:
    """root_usd 부터 시작해서 sub-USD 를 재귀적으로 따라가며 모든 참조 수집.

    Returns:
        (results, visited)
        - results : [(parent_usd, ref_raw, kind, exists)] 튜플 리스트
        - visited : 실제로 방문(extract_refs 호출)한 USD 파일 set
        kind ∈ {url, absolute, relative}.
        exists 는 로컬 디스크 존재 여부.
    """
    results: List[Tuple[Path, str, str, bool]] = []
    visited: Set[Path] = set()
    queue: List[Tuple[Path, int]] = [(root_usd.resolve(), 0)]

    while queue:
        usd, depth = queue.pop(0)
        if usd in visited:
            continue

        if not usd.is_file():
            continue

        # 파일이 존재하고 처음 보는 경우만 visited 에 기록 (정확한 카운트)
        visited.add(usd)

        refs = extract_refs(usd, include_path_pattern=True)
        for ref in refs:
            kind = classify(ref)
            resolved = normalize_relative(ref, usd) if kind == "relative" else None
            exists = bool(resolved and resolved.is_file())
            results.append((usd, ref, kind, exists))

            if follow and depth < max_depth and resolved and resolved.is_file():
                queue.append((resolved, depth + 1))

    return results, visited


# =============================================================================
# CLI
# =============================================================================
def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description="USD 파일의 sub-USD 의존성을 추출/진단합니다.")
    p.add_argument("usd", type=Path, help="분석할 USD 파일 경로")
    p.add_argument("--recursive", "-r", action="store_true",
                   help="sub-USD 를 재귀적으로 따라가며 모두 수집")
    p.add_argument("--max-depth", type=int, default=4,
                   help="재귀 깊이 제한 (기본 4)")
    p.add_argument("--only-missing", action="store_true",
                   help="로컬 디스크에 없는 sub-USD 만 출력")
    p.add_argument("--joints", action="store_true",
                   help="Physics joint body relationship target 도 추출")
    p.add_argument("--print-subpaths", action="store_true",
                   help="Isaac/... subpath 형태로 출력 "
                        "(download_isaac_assets.sh download_subpath 인자용)")
    p.add_argument("--dump-strings", action="store_true",
                   help="파일 내부 모든 printable ASCII 문자열을 그대로 출력 "
                        "(USDC 인코딩 진단용 — 참조 추출 실패 시 사용)")
    p.add_argument("--dump-min-len", type=int, default=8,
                   help="--dump-strings 가 출력할 최소 문자열 길이 (기본 8)")
    p.add_argument("--strict", action="store_true",
                   help="REF_PATTERN_AT (`@...@`) 만 사용 — false positive 회피")
    p.add_argument("--usdcat", action="store_true",
                   help="usdcat(1) 으로 USDC→USDA 변환 후 분석 (Isaac Sim "
                        "환경 필요). 비표준 인코딩 시 가장 정확.")

    args = p.parse_args(argv)

    if not args.usd.is_file():
        print(f"[FAIL] USD not found: {args.usd}", file=sys.stderr)
        return 2

    print(f"=== Inspecting: {args.usd}")
    print(f"    size: {args.usd.stat().st_size:,} bytes")
    print(f"    USDC : {_is_usdc(args.usd)}")

    # Variant-set 컨테이너 감지 — NovaCarter 처럼 4 KB wrapper 가 아니라
    # variant payload 를 lazy-load 하는 구조라면 variant 선택 없이는
    # PhysicsJoint body0/body1 prim 이 stage 에 없을 수 있음.
    is_variant, var_keywords, var_names = detect_variant_composition(args.usd)
    if is_variant:
        print(f"    Variants: YES — keywords found: {', '.join(var_keywords)}")
        if var_names:
            print(f"              candidate variant names: {', '.join(var_names)}")
        print()
        print("    [VARIANT HINT] 이 USD 는 variant-set 컨테이너입니다.")
        print("      → AddReference 후 다음과 같이 variant 선택을 적용해야")
        print("        PhysicsJoint body0/body1 prim 이 stage 에 포함됩니다:")
        print("            prim.GetVariantSets().GetVariantSet(name)")
        print("                .SetVariantSelection(value)")
        print("      → fdw_sim/visualization/asset_catalog.py 의")
        print("        UsdAssetSpec.variant_selection 에 매핑을 정의하세요.")
        # NovaCarter 인 경우 권장 매핑 안내
        if any(n in var_names for n in ("Physics_Base", "All_Sensors",
                                          "Skirt_only", "Full_Merged")):
            print("      → NovaCarter 권장값 (NVIDIA Asset Structure 문서):")
            print("            Configuration = Base")
            print("            Physics       = Physics_Base")
            print("            Sensors       = All_Sensors")

    # --dump-strings : 파일 내부 모든 printable 문자열 그대로 출력
    if args.dump_strings:
        blob = extract_strings(args.usd)
        print(f"\n--- All printable strings (min-len={args.dump_min_len}) ---")
        # blob 은 strings(1) 출력이라 이미 줄바꿈 단위.
        # fallback (raw bytes) 인 경우 직접 분리.
        if b"\n" in blob:
            lines = blob.split(b"\n")
        else:
            # raw bytes 에서 길이 N+ 의 ASCII run 추출
            ascii_run = re.compile(
                rb"[\x20-\x7e]{%d,}" % max(1, args.dump_min_len))
            lines = [m.group(0) for m in ascii_run.finditer(blob)]
        for line in lines:
            try:
                s = line.decode("utf-8", errors="ignore").strip()
            except Exception:
                continue
            if len(s) >= args.dump_min_len:
                print(f"    {s}")
        return 0

    # --usdcat : Isaac Sim 환경의 usdcat 사용
    if args.usdcat:
        usdcat_bin = _find_usdcat()
        if usdcat_bin is None:
            print("[FAIL] usdcat not found.", file=sys.stderr)
            print("    Isaac Sim 의 usdcat 은 PATH 에 노출되지 않을 수 있습니다.",
                  file=sys.stderr)
            print("    다음 중 하나를 시도하세요:", file=sys.stderr)
            print("      1) find ~/isaac-sim -name 'usdcat' 2>/dev/null",
                  file=sys.stderr)
            print("      2) export PATH=\"$PATH:$(find ~/isaac-sim -name "
                  "usdcat -printf '%h\\n' | head -1)\"", file=sys.stderr)
            print("      3) FDW_USDCAT=/path/to/usdcat 환경변수로 직접 지정",
                  file=sys.stderr)
            return 3
        print(f"    usdcat : {usdcat_bin}")
        try:
            out = subprocess.run(
                [usdcat_bin, str(args.usd)],
                capture_output=True, check=False, timeout=60,
            )
            if out.returncode == 0 and out.stdout:
                text = out.stdout.decode("utf-8", errors="ignore")
                print("\n--- usdcat output (USDA equivalent) ---")
                # references / payload 줄만 강조 출력
                ref_lines = [ln for ln in text.splitlines()
                             if ("references" in ln or "payload" in ln
                                 or ("@" in ln and ".usd" in ln))]
                for ln in ref_lines:
                    print(f"    {ln.rstrip()}")
                if not ref_lines:
                    print("    (no reference / payload lines found)")
                    print("\n--- First 60 lines of full output ---")
                    for ln in text.splitlines()[:60]:
                        print(f"    {ln.rstrip()}")
                return 0
            print(f"[FAIL] usdcat failed: rc={out.returncode}", file=sys.stderr)
            if out.stderr:
                print(out.stderr.decode("utf-8", errors="ignore"),
                      file=sys.stderr)
            return 3
        except subprocess.TimeoutExpired:
            print("[FAIL] usdcat timed out (60s)", file=sys.stderr)
            return 3

    if not args.recursive:
        refs = extract_refs(args.usd, include_path_pattern=not args.strict)
        print(f"\n--- Direct sub-USD references ({len(refs)}) ---")
        for ref in refs:
            kind = classify(ref)
            resolved = normalize_relative(ref, args.usd) if kind == "relative" else None
            exists = bool(resolved and resolved.is_file())
            marker = "OK " if exists else ("? " if kind == "url" else "MISS")
            if args.only_missing and exists:
                continue
            if args.print_subpaths and resolved:
                sub = to_isaac_subpath(resolved)
                if sub:
                    print(f"    [{marker}] {sub}")
                else:
                    print(f"    [{marker}] {ref}  (→ {resolved})")
            else:
                if resolved:
                    print(f"    [{marker}] {ref}  → {resolved}")
                else:
                    print(f"    [{marker}] {ref}  ({kind})")
    else:
        # Recursive walk
        results, visited = walk_refs(args.usd, max_depth=args.max_depth, follow=True)
        # group by parent for readability
        from collections import defaultdict
        by_parent = defaultdict(list)
        for parent, ref, kind, exists in results:
            by_parent[parent].append((ref, kind, exists))

        # unique sub-USD 카운트
        unique_subs: Set[str] = set()
        missing_subs: Set[str] = set()
        for parent, ref, kind, exists in results:
            unique_subs.add(ref)
            if kind == "relative" and not exists:
                missing_subs.add(ref)

        print(f"\n--- Recursive walk (depth ≤ {args.max_depth}) ---")
        # visited 는 실제로 열어본 USD 파일 수 (참조가 0 개여도 카운트)
        print(f"    Total parents scanned : {len(visited)}")
        print(f"    Unique references     : {len(unique_subs)}")
        print(f"    Missing (relative)    : {len(missing_subs)}")

        # ref 가 너무 적으면 (0~2 개) 의심 — --only-missing 무시하고
        # 모든 ref 의 정체를 강제 출력해 사용자가 path/url 종류를 즉시 파악
        suspicious = len(unique_subs) <= 2
        if suspicious and args.only_missing:
            print()
            print(f"    [HINT] Only {len(unique_subs)} reference(s) found — "
                  f"--only-missing 을 무시하고 모두 출력합니다.")

        for parent in sorted(by_parent.keys(), key=lambda p: str(p)):
            entries = by_parent[parent]
            # suspicious 인 경우 --only-missing 무시
            if suspicious:
                shown = list(entries)
            else:
                shown = [(r, k, e) for r, k, e in entries
                         if not (args.only_missing and e)]
            if not shown:
                continue
            print(f"\n  [parent] {parent}")
            for ref, kind, exists in shown:
                marker = "OK " if exists else ("? " if kind == "url" else "MISS")
                if args.print_subpaths and kind == "relative":
                    resolved = normalize_relative(ref, parent)
                    sub = to_isaac_subpath(resolved) if resolved else None
                    if sub:
                        print(f"      [{marker}] {sub}")
                    else:
                        print(f"      [{marker}] {ref}")
                else:
                    print(f"      [{marker}] {ref}  ({kind})")

        # Summary: missing as subpaths (download_isaac_assets.sh 입력용)
        if missing_subs:
            print("\n--- Missing sub-USD subpaths (Isaac/...) ---")
            print("    → download_isaac_assets.sh 에 추가하거나, "
                  "extract_and_download_refs() 가 자동으로 받게 하세요.")
            unique_subpaths: Set[str] = set()
            for parent, ref, kind, exists in results:
                if kind != "relative" or exists:
                    continue
                resolved = normalize_relative(ref, parent)
                if resolved:
                    sub = to_isaac_subpath(resolved)
                    if sub:
                        unique_subpaths.add(sub)
            for sub in sorted(unique_subpaths):
                print(f"    {sub}")

        # 추가 진단: ref 가 0~2 개로 의심스러우면 wrapper 가능성 안내
        if suspicious:
            print()
            print("    [DIAG] 의심: 참조가 극소수 — wrapper 또는 비표준 인코딩 가능성.")
            if len(unique_subs) == 0:
                print("      → 0 refs : USDC 헤더만 있는 빈 wrapper 가능성 큼.")
            else:
                print(f"      → {len(unique_subs)} refs : 위 목록의 kind 확인.")
                print("        - kind=url     : Nucleus/HTTP 원격 — 로컬 캐시 필요")
                print("        - kind=absolute: 절대 경로 — 다른 머신의 경로일 수 있음")
                print("        - kind=relative + OK : 정상 (이미 존재)")
            print("    다음 진단 명령을 실행하세요:")
            print(f"      python {sys.argv[0]} --dump-strings {args.usd}")
            print(f"      python {sys.argv[0]} --usdcat {args.usd}")
            print("      ls -la $(dirname \"%s\")/.." % args.usd)

    if args.joints:
        bodies = extract_joint_bodies(args.usd)
        print(f"\n--- Physics joint body references ({len(bodies)}) ---")
        for b in bodies:
            print(f"    /{b}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
