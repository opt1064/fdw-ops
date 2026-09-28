"""import_mir100_usd.py — MiR100 URDF -> USD 변환 (Isaac Sim 필요).

전제:
    scripts/prepare_mir100_urdf.sh를 먼저 실행해서 순수 URDF(mesh 경로가
    로컬 절대경로로 치환된 상태)를 만들어뒀어야 한다.

    MiR100 USD는 Isaac Sim 공식 카탈로그에 없다(fdw_sim/visualization/
    asset_catalog.py의 "mir100" 항목 주석 참고) — 이 스크립트가 그 갭을
    커뮤니티 URDF(DFKI-NI/mir_robot, BSD-3-Clause)로부터 직접 메운다.

API 확인 이력:
    2026-09-28 DGX Spark(Isaac Sim 6.1, isaacsim.asset.importer.urdf 3.11.10)
    에서 help()로 직접 확인한 클래스 기반 API를 사용한다:

        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
        config = URDFImporterConfig(urdf_path=..., usd_path=<dest DIRECTORY>, ...)
        importer = URDFImporter(config)
        output_usd_path = importer.import_urdf()   # 실제 생성된 usd 경로를 반환

    (예전 omni.importer.urdf._urdf.ImportConfig 방식은 이 Isaac Sim
    설치에는 존재하지 않음 — extension registry에 아예 없음이 확인됨.)

    ⚠️ URDFImporterConfig.usd_path는 "파일 경로"가 아니라 "USD를 저장할
    디렉토리" 다. 생성되는 파일명은 importer가 정하므로(보통 URDF의
    robot name 기반), --out으로 지정한 정확한 파일명이 그대로 나온다는
    보장이 없다 — 그래서 import 후 필요하면 그 디렉토리로 지정하고
    import_urdf()가 반환한 실제 경로를 최종적으로 --out 이름으로
    복사/링크한다.

사용법:
    "$ISAACSIM_PYTHON_EXE" scripts/import_mir100_usd.py \\
        --urdf ~/isaac_assets/mir100_urdf_src/mir100.urdf \\
        --out ~/isaac_assets/Robots/MiR/mir100/mir100.usd

    성공하면:
        export FDW_MIR100_USD=~/isaac_assets/Robots/MiR/mir100/mir100.usd
    로 fdw-ops가 바로 이 USD를 쓰게 할 수 있다
    (fdw_sim/visualization/asset_catalog.py의 resolve_asset_usd_path 참고).
"""
from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path


def main() -> int:
    p = argparse.ArgumentParser(description="MiR100 URDF -> USD (Isaac Sim URDF importer)")
    p.add_argument("--urdf", type=Path, required=True,
                   help="prepare_mir100_urdf.sh가 만든 mir100.urdf 경로")
    p.add_argument("--out", type=Path, required=True,
                   help="최종적으로 이 경로에 USD 파일을 두고 싶다 (예: "
                        "~/isaac_assets/Robots/MiR/mir100/mir100.usd) — 실제 "
                        "변환은 이 파일의 디렉토리에서 이뤄지고, importer가 "
                        "정한 파일명이 --out 이름과 다르면 복사해서 맞춘다.")
    p.add_argument("--fix-base", action="store_true",
                   help="베이스를 월드에 고정 (MiR100처럼 바닥 이동 로봇에는 지정하지 말 것)")
    p.add_argument("--headless", action="store_true", default=True,
                   help="헤드리스로 SimulationApp 실행 (기본 on — 변환만 하면 되므로)")
    args = p.parse_args()

    urdf_path = args.urdf.expanduser().resolve()
    out_path = args.out.expanduser().resolve()
    if not urdf_path.is_file():
        print(f"[FAIL] URDF not found: {urdf_path}", file=sys.stderr)
        print("       먼저 scripts/prepare_mir100_urdf.sh 를 실행하세요.", file=sys.stderr)
        return 1
    out_dir = out_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    # SimulationApp은 다른 omni.* import보다 먼저 인스턴스화해야 한다.
    from isaacsim import SimulationApp  # type: ignore
    simulation_app = SimulationApp({"headless": args.headless})

    try:
        import omni.kit.app  # type: ignore
        ext_mgr = omni.kit.app.get_app().get_extension_manager()
        ext_mgr.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)

        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig  # type: ignore

        config = URDFImporterConfig(
            urdf_path=str(urdf_path),
            usd_path=str(out_dir),
            merge_fixed_joints=True,   # IMU/라이다 마운트 등 순수 센서 프레임 정리
            merge_mesh=False,
            collision_from_visuals=False,
            fix_base=args.fix_base,   # MiR100은 바닥 이동 로봇 — 기본 False(floating-base)
        )

        print(f"[INFO] importing {urdf_path}")
        print(f"[INFO]   output dir: {out_dir}")
        importer = URDFImporter(config)
        generated_path = importer.import_urdf()

        if not generated_path:
            print("[FAIL] import_urdf() returned empty path", file=sys.stderr)
            return 1

        generated_path = Path(generated_path)
        if not generated_path.is_file():
            print(f"[FAIL] importer reported {generated_path} but it doesn't exist",
                  file=sys.stderr)
            return 1

        ok(f"USD generated: {generated_path}")

        if generated_path != out_path:
            print(f"[INFO] importer가 정한 파일명이 --out과 달라서 복사: "
                  f"{generated_path} -> {out_path}")
            shutil.copy2(generated_path, out_path)
            # USD가 상대경로로 sub-asset(mesh 등)을 참조하는 경우가 많으므로
            # 같은 디렉토리에 생성된 나머지 파일들도 함께 복사한다.
            if generated_path.parent != out_path.parent:
                for item in generated_path.parent.iterdir():
                    if item == generated_path:
                        continue
                    dest = out_path.parent / item.name
                    if item.is_dir():
                        shutil.copytree(item, dest, dirs_exist_ok=True)
                    else:
                        shutil.copy2(item, dest)

        print()
        print("=" * 70)
        print(f" export FDW_MIR100_USD={out_path}")
        print("=" * 70)
        return 0
    finally:
        simulation_app.close()


def ok(msg: str) -> None:
    print(f"[ OK ] {msg}")


if __name__ == "__main__":
    raise SystemExit(main())
