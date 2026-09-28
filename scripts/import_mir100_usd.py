"""import_mir100_usd.py — MiR100 URDF -> USD 변환 (Isaac Sim 필요).

전제:
    scripts/prepare_mir100_urdf.sh를 먼저 실행해서 순수 URDF(mesh 경로가
    로컬 절대경로로 치환된 상태)를 만들어뒀어야 한다.

    MiR100 USD는 Isaac Sim 공식 카탈로그에 없다(fdw_sim/visualization/
    asset_catalog.py의 "mir100" 항목 주석 참고) — 이 스크립트가 그 갭을
    커뮤니티 URDF(DFKI-NI/mir_robot, BSD-3-Clause)로부터 직접 메운다.

⚠️ 이 스크립트는 Isaac Sim 런타임(omni.kit.commands, URDF importer
   익스텐션)이 필요해서 GPU 없는 환경에서는 동작 확인을 못 했다.
   isaacsim.asset.importer.urdf(신규 네임스페이스, Isaac 4.5+)와
   omni.importer.urdf(레거시)를 둘 다 시도하도록 짰지만, 두 경로 모두
   미검증 상태 — 실행 중 ImportError/AttributeError가 나면 그 시점의
   실제 에러 메시지를 보고 API를 맞춰야 한다.

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
import sys
from pathlib import Path


def main() -> int:
    p = argparse.ArgumentParser(description="MiR100 URDF -> USD (Isaac Sim URDF importer)")
    p.add_argument("--urdf", type=Path, required=True,
                   help="prepare_mir100_urdf.sh가 만든 mir100.urdf 경로")
    p.add_argument("--out", type=Path, required=True,
                   help="출력 USD 경로 (예: ~/isaac_assets/Robots/MiR/mir100/mir100.usd)")
    p.add_argument("--fix-base", action="store_true",
                   help="베이스를 월드에 고정(바닥 이동 로봇에는 보통 off)")
    p.add_argument("--headless", action="store_true", default=True,
                   help="헤드리스로 SimulationApp 실행 (기본 on — 변환만 하면 되므로)")
    args = p.parse_args()

    urdf_path = args.urdf.expanduser().resolve()
    out_path = args.out.expanduser().resolve()
    if not urdf_path.is_file():
        print(f"[FAIL] URDF not found: {urdf_path}", file=sys.stderr)
        print("       먼저 scripts/prepare_mir100_urdf.sh 를 실행하세요.", file=sys.stderr)
        return 1
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # SimulationApp은 다른 omni.* import보다 먼저 인스턴스화해야 한다.
    from isaacsim import SimulationApp  # type: ignore
    simulation_app = SimulationApp({"headless": args.headless})

    try:
        import omni.kit.commands  # type: ignore
        import omni.usd  # type: ignore

        # 신규(isaacsim.asset.importer.urdf, Isaac 4.5+) 우선, 안 되면
        # 레거시(omni.importer.urdf)로 폴백 — robot_loader.py의 3-tier
        # 폴백 스타일과 동일한 이유(Isaac 버전마다 익스텐션 네임스페이스가
        # 다름).
        _urdf = None
        for ext_name, mod_path in [
            ("isaacsim.asset.importer.urdf", "isaacsim.asset.importer.urdf"),
            ("omni.importer.urdf", "omni.importer.urdf"),
        ]:
            try:
                import omni.kit.app  # type: ignore
                ext_mgr = omni.kit.app.get_app().get_extension_manager()
                ext_mgr.set_extension_enabled_immediate(ext_name, True)
                _urdf = __import__(mod_path, fromlist=["_urdf"])._urdf
                print(f"[INFO] using URDF importer extension: {ext_name}")
                break
            except Exception as e:  # noqa: BLE001
                print(f"[WARN] extension {ext_name} unavailable ({e}) — trying next")
                continue

        if _urdf is None:
            print("[FAIL] no URDF importer extension available "
                  "(tried isaacsim.asset.importer.urdf, omni.importer.urdf). "
                  "Isaac Sim 설치에 URDF importer 익스텐션이 포함돼 있는지 "
                  "확인하세요.", file=sys.stderr)
            return 1

        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = True   # IMU/라이다 마운트 등 순수 센서 프레임 정리
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = True
        import_config.fix_base = args.fix_base    # MiR100은 바닥 이동 로봇 — 기본 False
        import_config.make_default_prim = True
        import_config.self_collision = False
        import_config.create_physics_scene = False  # 씬 자체는 fdw-ops가 따로 만듦
        import_config.distance_scale = 1.0
        import_config.density = 0.0

        print(f"[INFO] importing {urdf_path} -> {out_path}")
        status, robot_prim_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=str(urdf_path),
            import_config=import_config,
            dest_path=str(out_path),
        )

        if not status:
            print(f"[FAIL] URDFParseAndImportFile returned status={status}",
                  file=sys.stderr)
            return 1

        if not out_path.is_file():
            print(f"[FAIL] import reported success but {out_path} does not exist",
                  file=sys.stderr)
            return 1

        print(f"[ OK ] USD written: {out_path}")
        print(f"[ OK ] robot prim path in imported stage: {robot_prim_path}")
        print()
        print("=" * 70)
        print(f" export FDW_MIR100_USD={out_path}")
        print("=" * 70)
        return 0
    finally:
        simulation_app.close()


if __name__ == "__main__":
    raise SystemExit(main())
