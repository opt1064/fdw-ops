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
    if include_path_pattern:
        for source in (blob, raw):
            for m in REF_PATTERN_PATH.finditer(source):
                try:
                    cand = m.group(1).decode("utf-8", errors="ignore")
                except Exception:
                    continue
                # 자기 자신 제외
                if cand == usd_path.name:
                    continue
                # `.usd` 만 들어있는 것 (확장자 토큰) 제외
                if cand.startswith("."):
                    # `./foo.usd` 는 유효, `.usd` 는 무효
                    if cand in (".usd", ".usda", ".usdc"):
                        continue
                _add(cand)

    return refs


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
        try:
            out = subprocess.run(
                ["usdcat", str(args.usd)],
                capture_output=True, check=False, timeout=60,
            )
            if out.returncode == 0 and out.stdout:
                text = out.stdout.decode("utf-8", errors="ignore")
                print("\n--- usdcat output (USDA equivalent) ---")
                # references / payload 줄만 강조 출력
                ref_lines = [ln for ln in text.splitlines()
                             if ("references" in ln or "payload" in ln
                                 or "@" in ln and ".usd" in ln)]
                for ln in ref_lines:
                    print(f"    {ln.rstrip()}")
                if not ref_lines:
                    print("    (no reference / payload lines found)")
                return 0
            print(f"[FAIL] usdcat failed: rc={out.returncode}", file=sys.stderr)
            if out.stderr:
                print(out.stderr.decode("utf-8", errors="ignore"),
                      file=sys.stderr)
            return 3
        except FileNotFoundError:
            print("[FAIL] usdcat not found in PATH. Isaac Sim 환경에서 실행하세요.",
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

        # 참조 0 개일 때 진단 힌트
        if len(unique_subs) == 0:
            print()
            print("    [HINT] 0 references found. 가능한 원인:")
            print("      - 이 파일이 단순 wrapper(USDC 헤더만, 본문 0) 일 수 있음.")
            print("      - 참조 경로가 비표준 인코딩(USDC 압축 토큰)으로 저장됨.")
            print("    → '--dump-strings' 로 파일 내부 모든 ASCII 문자열을 확인하세요:")
            print(f"        python {sys.argv[0]} --dump-strings {args.usd}")
            print("    → 또는 'usdcat' 가 있으면:")
            print(f"        usdcat {args.usd} | head -200")

        for parent in sorted(by_parent.keys(), key=lambda p: str(p)):
            entries = by_parent[parent]
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

    if args.joints:
        bodies = extract_joint_bodies(args.usd)
        print(f"\n--- Physics joint body references ({len(bodies)}) ---")
        for b in bodies:
            print(f"    /{b}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
