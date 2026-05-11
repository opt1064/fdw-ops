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


# `@<path>@` 또는 `@<path>@</prim>` 패턴에서 path 추출
# - omniverse://, http(s)://, file:// 같은 절대 URI 는 별도 표시
# - .usd / .usda / .usdc 만 의미 있음
REF_PATTERN = re.compile(rb"@([^@\x00\n\r]+\.(?:usd|usda|usdc))(?:@|<)")

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


def extract_refs(usd_path: Path) -> List[str]:
    """단일 USD 파일에서 sub-USD 참조 경로 목록 추출 (중복 제거, 순서 유지)."""
    blob = extract_strings(usd_path)
    seen: Set[str] = set()
    refs: List[str] = []
    for m in REF_PATTERN.finditer(blob):
        try:
            path = m.group(1).decode("utf-8", errors="ignore").strip()
        except Exception:
            continue
        if not path or path in seen:
            continue
        seen.add(path)
        refs.append(path)
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
              ) -> List[Tuple[Path, str, str, bool]]:
    """root_usd 부터 시작해서 sub-USD 를 재귀적으로 따라가며 모든 참조 수집.

    Returns:
        [(parent_usd, ref_raw, kind, exists)] 튜플 리스트.
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
        visited.add(usd)

        if not usd.is_file():
            continue

        refs = extract_refs(usd)
        for ref in refs:
            kind = classify(ref)
            resolved = normalize_relative(ref, usd) if kind == "relative" else None
            exists = bool(resolved and resolved.is_file())
            results.append((usd, ref, kind, exists))

            if follow and depth < max_depth and resolved and resolved.is_file():
                queue.append((resolved, depth + 1))

    return results


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

    args = p.parse_args(argv)

    if not args.usd.is_file():
        print(f"[FAIL] USD not found: {args.usd}", file=sys.stderr)
        return 2

    print(f"=== Inspecting: {args.usd}")
    print(f"    size: {args.usd.stat().st_size:,} bytes")

    if not args.recursive:
        refs = extract_refs(args.usd)
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
        results = walk_refs(args.usd, max_depth=args.max_depth, follow=True)
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
        print(f"    Total parents scanned : {len(by_parent)}")
        print(f"    Unique references     : {len(unique_subs)}")
        print(f"    Missing (relative)    : {len(missing_subs)}")

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
