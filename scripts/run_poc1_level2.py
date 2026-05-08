"""PoC-1 Level 2 — Isaac Sim USD 시각화 데모.

run_poc1.py와 동일한 시뮬레이션을 Isaac Sim에서 시각적으로 보여준다.
셀, 작업대, 로봇팔, 카메라, AMR, 부품을 USD 스테이지에 배치하고
시뮬레이션 동안 부품의 이동을 애니메이션으로 표시한다.

실행 방법:

    # 본체 앞에서 GUI로 보기
    ACCEPT_EULA=Y PRIVACY_CONSENT=Y python scripts/run_poc1_level2.py --gui

    # 원격에서 WebRTC 스트리밍으로 보기
    ACCEPT_EULA=Y PRIVACY_CONSENT=Y python scripts/run_poc1_level2.py --livestream 2

    # 헤드리스(시각화 비활성)로 빠른 검증
    ACCEPT_EULA=Y PRIVACY_CONSENT=Y python scripts/run_poc1_level2.py --headless --no-viz

기본값은 --gui (모니터에 직접 표시).
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from scripts.run_poc1 import (
    build_simulation, load_config, print_report, submit_jobs,
)
from fdw_sim.utils.logging_setup import setup_logging


def main() -> int:
    p = argparse.ArgumentParser(
        description="PoC-1 Level 2 — Isaac Sim USD visualization demo")
    p.add_argument("--config", type=Path,
                   default=ROOT / "fdw_sim" / "config" / "poc1.yaml")

    display = p.add_mutually_exclusive_group()
    display.add_argument("--gui", action="store_true",
                         help="모니터 직접 표시 (기본)")
    display.add_argument("--headless", action="store_true",
                         help="헤드리스로 실행")
    display.add_argument("--livestream", type=int, choices=[1, 2],
                         help="원격 스트리밍: 1=Native, 2=WebRTC")

    p.add_argument("--no-viz", action="store_true",
                   help="USD 시각화 비활성 (Isaac Sim만 실행)")
    p.add_argument("--realtime", action="store_true",
                   help="시뮬레이션을 실시간 속도로 재생 (시각 확인용)")
    p.add_argument("--anim-duration", type=float, default=2.5,
                   help="셀 간 부품 이동 애니메이션 길이(초)")
    p.add_argument("--log-level", default="INFO",
                   choices=["DEBUG", "INFO", "WARNING", "ERROR"])

    args = p.parse_args()
    setup_logging(level=args.log_level)

    cfg = load_config(args.config)

    # 기본은 GUI 모드
    headless_override = False
    livestream_override = None
    if args.headless:
        headless_override = True
    elif args.livestream:
        headless_override = True
        livestream_override = args.livestream

    # 시뮬레이션 빌드 (Isaac 모드 강제)
    sim = build_simulation(cfg, mode_override="isaac",
                           headless_override=headless_override,
                           livestream_override=livestream_override)

    # 시각화 옵션 적용
    sim.config.enable_visualization = not args.no_viz
    sim.config.transfer_anim_duration_sec = args.anim_duration
    sim.config.realtime = args.realtime

    print("=" * 70)
    print(" FDW-OPS PoC-1 Level 2 — Isaac Sim 시각화 데모")
    print("=" * 70)
    print(f" Mode             : isaac")
    print(f" Headless         : {sim.config.headless}")
    print(f" Livestream       : {sim.config.livestream}")
    print(f" Visualization    : {sim.config.enable_visualization}")
    print(f" Realtime         : {sim.config.realtime}")
    print(f" Anim duration    : {args.anim_duration:.1f}s")
    print("=" * 70)
    print(" 첫 실행은 셰이더 컴파일로 1~3분 소요됩니다...")
    print("=" * 70 + "\n")

    t0 = time.time()
    sim.start()
    boot_time = time.time() - t0
    print(f"\n[INFO] Isaac Sim boot 완료 ({boot_time:.1f}s)")
    if sim.config.livestream == 2:
        print("[INFO] WebRTC 클라이언트 접속 URL:")
        print("       http://<AGX_THOR_IP>:8211/streaming/webrtc-client\n")
    elif sim.config.livestream == 1:
        print("[INFO] Native Streaming Client에서 <AGX_THOR_IP>로 접속\n")

    try:
        submit_jobs(sim, cfg)
        print("[INFO] 3 jobs 투입 — 시각화 시작\n")
        sim.run_until_done(all_jobs_done=True, extra_idle_sec=3.0)
        print_report(sim)
    finally:
        sim.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
