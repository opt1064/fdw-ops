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

    # RTX / Denoiser 안정성 옵션 (AGX Thor Blackwell GPU 호환)
    p.add_argument("--render-mode", choices=["RaytracedLighting", "PathTracing"],
                   default="RaytracedLighting",
                   help="RTX 렌더 모드 (기본: RaytracedLighting — NRD denoiser 불필요)")
    p.add_argument("--enable-denoiser", action="store_true",
                   help="NRD denoiser 강제 활성 (Blackwell GPU에서는 권장하지 않음)")
    p.add_argument("--show-rtx-log", action="store_true",
                   help="rtx.denoising 로그를 그대로 출력 (디버깅용)")

    # GPU device-lost 회피용 안전 모드 (VkResult: ERROR_DEVICE_LOST 대응)
    p.add_argument("--safe-mode", action="store_true",
                   help="AGX Thor Blackwell GPU device-lost 회피 — "
                        "auto camera/Aftermath/NGX 등 가장 보수적인 설정 일괄 적용")
    p.add_argument("--skip-auto-camera", action="store_true",
                   help="WorkshopVisualizer 카메라 자동 framing 건너뜀")
    p.add_argument("--disable-aftermath", action="store_true", default=True,
                   help="NV Aftermath GPU crash dumper 비활성 (기본: ON)")
    p.add_argument("--enable-aftermath", dest="disable_aftermath",
                   action="store_false",
                   help="Aftermath 강제 활성 (디버깅용 — device-lost 위험↑)")
    p.add_argument("--force-lighting-mode",
                   choices=["camera", "stage", "rig"], default=None,
                   help="라이팅 메뉴 모드 강제 — SetLightingMenuModeCommand 회피")
    p.add_argument("--disable-viewport-switch", action="store_true",
                   help="카메라 prim은 만들되 viewport 활성 카메라는 바꾸지 않음 "
                        "(omni.kit.viewport.utility 호출이 GPU crash trigger인 경우)")

    p.add_argument("--log-level", default="INFO",
                   choices=["DEBUG", "INFO", "WARNING", "ERROR"])

    args = p.parse_args()
    setup_logging(level=args.log_level)

    # RTX denoiser 환경변수 사전 설정 (SimulationApp 시작 전에 적용되어야 함)
    import os
    if not args.enable_denoiser:
        # carb 설정과 동일 키를 환경변수로도 주입 (가장 신뢰성 있는 방식)
        os.environ.setdefault("RTX_RENDERMODE", args.render_mode)
        os.environ.setdefault("OMNI_KIT_ALLOW_ROOT", "1")

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

    # RTX 안정성 옵션
    sim.config.render_mode = args.render_mode
    sim.config.disable_nrd_denoiser = not args.enable_denoiser
    sim.config.suppress_rtx_log_spam = not args.show_rtx_log

    # GPU device-lost 회피 옵션
    sim.config.safe_mode = args.safe_mode
    sim.config.skip_auto_camera = args.safe_mode or args.skip_auto_camera
    sim.config.disable_aftermath = args.disable_aftermath
    sim.config.force_lighting_mode = args.force_lighting_mode
    if args.disable_viewport_switch:
        os.environ["FDW_DISABLE_VIEWPORT_SWITCH"] = "1"

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
    print(f" Render mode      : {sim.config.render_mode}")
    print(f" NRD denoiser     : {'OFF (Blackwell-safe)' if sim.config.disable_nrd_denoiser else 'ON'}")
    print(f" RTX log spam     : {'suppressed' if sim.config.suppress_rtx_log_spam else 'visible'}")
    print(f" Safe mode        : {'ON (skip_auto_camera + aftermath_off)' if sim.config.safe_mode else 'OFF'}")
    print(f" Skip auto camera : {sim.config.skip_auto_camera}")
    print(f" Disable Aftermath: {sim.config.disable_aftermath}")
    print(f" Force lighting   : {sim.config.force_lighting_mode or '(Kit default)'}")
    print(f" Viewport switch  : {'DISABLED' if os.environ.get('FDW_DISABLE_VIEWPORT_SWITCH') == '1' else 'enabled'}")
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
