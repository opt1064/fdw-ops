"""PoC-1 Level 2.1 — 실 로봇팔(Franka Panda / UR10) + IK 데모.

Level 2의 placeholder 박스 로봇팔을 실제 7-DOF Franka Panda(또는 UR10)
USD 모델로 교체하고, 용접 셀이 PROCESSING 상태가 되면 IK(Lula 또는
휴리스틱 fallback)로 토치가 부품 위 용접 경로를 추적한다.

실행 예시:

    # GUI에서 Franka Panda + IK
    ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
        python scripts/run_poc1_level2_1.py --gui --real-robot --robot franka_panda

    # WebRTC 스트리밍 + UR10
    ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
        python scripts/run_poc1_level2_1.py --livestream 2 --real-robot --robot ur10

    # IK 비활성 (단순 USD 표시만)
    ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
        python scripts/run_poc1_level2_1.py --gui --real-robot --no-ik

    # 실 로봇팔 미사용 (Level 2 placeholder와 동일)
    ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
        python scripts/run_poc1_level2_1.py --gui

전제:
    * Isaac Sim 5.1.0 + Nucleus 접속 가능 (or 환경변수 ISAAC_ASSETS_ROOT 지정)
    * Franka USD : Isaac/Robots/Franka/franka.usd
    * UR10  USD : Isaac/Robots/UR10/ur10.usd
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


VALID_ROBOTS = ["franka_panda", "ur10", "franka_alt"]


def main() -> int:
    p = argparse.ArgumentParser(
        description="PoC-1 Level 2.1 — Franka/UR10 USD + IK demo")
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
                   help="시뮬레이션을 실시간 속도로 재생")
    p.add_argument("--anim-duration", type=float, default=2.5,
                   help="셀 간 부품 이동 애니메이션 길이(초)")

    # Level 2.1 전용 플래그
    p.add_argument("--real-robot", action="store_true",
                   help="실 로봇팔 USD 로드 (Franka/UR10). 미지정 시 placeholder 박스")
    p.add_argument("--robot", choices=VALID_ROBOTS, default="franka_panda",
                   help="로봇 모델 선택")
    p.add_argument("--no-ik", action="store_true",
                   help="IK 컨트롤러 비활성 (홈 자세 고정)")
    p.add_argument("--weld-offset", type=float, default=0.25,
                   help="용접 경로 길이의 절반 (m, 입력 버퍼 중심 ±값)")
    p.add_argument("--weld-height", type=float, default=0.05,
                   help="용접 경로의 부품 표면 위 높이 (m)")

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

    # 시각화 + Level 2.1 옵션 적용
    sim.config.enable_visualization = not args.no_viz
    sim.config.transfer_anim_duration_sec = args.anim_duration
    sim.config.realtime = args.realtime
    sim.config.use_real_robot = args.real_robot
    sim.config.robot_name = args.robot
    sim.config.enable_ik = not args.no_ik
    sim.config.weld_path_offset_y = args.weld_offset
    sim.config.weld_path_height = args.weld_height

    print("=" * 70)
    print(" FDW-OPS PoC-1 Level 2.1 — 실 로봇팔(Franka/UR10) + IK 데모")
    print("=" * 70)
    print(f" Mode             : isaac")
    print(f" Headless         : {sim.config.headless}")
    print(f" Livestream       : {sim.config.livestream}")
    print(f" Visualization    : {sim.config.enable_visualization}")
    print(f" Realtime         : {sim.config.realtime}")
    print(f" Anim duration    : {args.anim_duration:.1f}s")
    print(f" Real robot       : {sim.config.use_real_robot}")
    print(f" Robot model      : {sim.config.robot_name}")
    print(f" IK enabled       : {sim.config.enable_ik}")
    print(f" Weld path offset : ±{sim.config.weld_path_offset_y:.2f} m")
    print(f" Weld path height : {sim.config.weld_path_height:.3f} m")
    print("=" * 70)
    if sim.config.use_real_robot:
        print(" [Hint] Franka/UR10 USD가 NGC/Nucleus에서 처음 로드되면")
        print("        셰이더 캐시가 만들어지며 1~3분이 추가로 소요될 수 있습니다.")
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
