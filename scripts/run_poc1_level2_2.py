"""PoC-1 Level 2.2 — RMPflow 충돌회피 모션 + 용접 스파크 파티클 데모.

Level 2.1의 Lula IK 컨트롤러를 Lula **RMPflow**(Reactive Motion Policy flow)로
교체하여 작업대/부품을 장애물로 인식하고 회피 모션을 생성한다. 또한 용접
PROCESSING 단계에서 TCP(엔드 이펙터) 위치에 PointInstancer 기반 스파크
파티클을 분사하여 시각적 사실감을 향상시킨다.

3-tier 모션 백엔드 (자동 fallback):

    1. rmpflow   : Lula RMPflow + ArticulationMotionPolicy (장애물 회피 O)
    2. ik        : Lula Kinematics Solver (Level 2.1과 동일, 회피 X)
    3. heuristic : 외부 의존 0 — 어깨/팔꿈치 휴리스틱 + ±법선 repulsion

실행 예시:

    # GUI에서 Franka Panda + RMPflow + 스파크
    ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
        python scripts/run_poc1_level2_2.py --gui --real-robot --robot franka_panda

    # WebRTC 스트리밍 + RMPflow 강제 (실패시 RMPflowController 내부에서 fallback)
    ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
        python scripts/run_poc1_level2_2.py --livestream 2 --real-robot \
            --motion-mode rmpflow

    # RMPflow는 끄고 Level 2.1 IK로 — 스파크는 켜둠
    ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
        python scripts/run_poc1_level2_2.py --gui --real-robot --motion-mode ik

    # 스파크 비활성 + 휴리스틱 모션
    ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
        python scripts/run_poc1_level2_2.py --gui --real-robot \
            --motion-mode heuristic --no-sparks

    # 스파크 분사 비율 조정 (기본 30/s)
    ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
        python scripts/run_poc1_level2_2.py --gui --real-robot \
            --spark-rate 60 --spark-lifetime 0.6

    # Level 2.3: AMR을 NovaCarter 대신 Jetbot으로 (Props/* 404 우회)
    ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
        python scripts/run_poc1_level2_2.py --gui --real-robot \
            --amr-asset jetbot

    # Level 2.3: AMR/카메라/Rack 모두 placeholder로 (로봇팔만 실제)
    ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
        python scripts/run_poc1_level2_2.py --gui --real-robot \
            --no-real-amr --no-real-smart-rack --no-real-inspection-cam

전제:
    * Isaac Sim 5.1.0 + Nucleus 접속 가능 (or 환경변수 ISAAC_ASSETS_ROOT 지정)
    * Franka USD : Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd (Isaac 5.1 경로)
    * RMPflow config는 isaacsim.robot_motion.motion_generation의 표준 위치에서 자동 탐색
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
VALID_MOTION_MODES = ["auto", "rmpflow", "ik", "heuristic"]

# Level 2.3 USD 자산 카탈로그 선택지
VALID_AMR_ASSETS = ["nova_carter", "jetbot", "iw_hub", "iw_hub_static"]
VALID_SMART_RACKS = ["klt_bin", "cardboard_box"]
VALID_FORMING_ROBOTS = ["ur10", "ur10e", "ur5e", "ur16e"]


def main() -> int:
    p = argparse.ArgumentParser(
        description="PoC-1 Level 2.2 — RMPflow + welding spark particle demo")
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

    # Level 2.1 플래그 (재사용)
    p.add_argument("--real-robot", action="store_true",
                   help="실 로봇팔 USD 로드 (Franka/UR10). 미지정 시 placeholder 박스")
    p.add_argument("--robot", choices=VALID_ROBOTS, default="franka_panda",
                   help="로봇 모델 선택")
    p.add_argument("--no-ik", action="store_true",
                   help="모션 컨트롤러 전체 비활성 (홈 자세 고정)")
    p.add_argument("--weld-offset", type=float, default=0.25,
                   help="용접 경로 길이의 절반 (m, 입력 버퍼 중심 ±값)")
    p.add_argument("--weld-height", type=float, default=0.05,
                   help="용접 경로의 부품 표면 위 높이 (m)")

    # ----------------------------------------------------- Level 2.2 신규 플래그
    p.add_argument("--motion-mode", choices=VALID_MOTION_MODES, default="auto",
                   help="모션 백엔드: auto(=rmpflow→ik→heuristic) | "
                        "rmpflow | ik | heuristic (기본: auto)")
    p.add_argument("--no-sparks", dest="enable_sparks",
                   action="store_false", default=True,
                   help="용접 스파크 파티클 비활성")
    p.add_argument("--spark-rate", type=float, default=30.0,
                   help="스파크 분사 비율 (sparks/sec, 기본 30)")
    p.add_argument("--spark-lifetime", type=float, default=0.4,
                   help="스파크 평균 수명 (s, ±20%% jitter, 기본 0.4)")
    p.add_argument("--no-rmpflow-obstacles", dest="rmpflow_register_obstacles",
                   action="store_false", default=True,
                   help="RMPflow에 작업대/부품 장애물 등록 비활성")

    # ----------------------------------------------------- Level 2.3 신규 플래그
    # 셀별 실 USD 자산 선택. NovaCarter Props/* 404 이슈 우회를 위해
    # --amr-asset jetbot 또는 iw_hub_static 으로 즉시 스왑 가능.
    p.add_argument("--amr-asset", choices=VALID_AMR_ASSETS, default="nova_carter",
                   help="AMR USD 자산: nova_carter(기본) | jetbot | "
                        "iw_hub | iw_hub_static. "
                        "NovaCarter joint body 누락 경고 발생 시 "
                        "jetbot/iw_hub_static 권장 (self-contained)")
    p.add_argument("--no-real-amr", dest="use_real_amr",
                   action="store_false", default=True,
                   help="AMR을 USD 대신 placeholder 박스로 그림")
    p.add_argument("--smart-rack-asset", choices=VALID_SMART_RACKS,
                   default="klt_bin",
                   help="Material 셀 smart rack USD 자산 (기본 klt_bin)")
    p.add_argument("--no-real-smart-rack", dest="use_real_smart_rack",
                   action="store_false", default=True,
                   help="Smart rack을 USD 대신 placeholder 박스로 그림")
    p.add_argument("--forming-robot", choices=VALID_FORMING_ROBOTS,
                   default="ur10",
                   help="Forming 셀 로봇팔 (기본 ur10)")
    p.add_argument("--no-real-forming-arm", dest="use_real_forming_arm",
                   action="store_false", default=True,
                   help="Forming 셀 로봇팔을 placeholder 박스로 그림")
    p.add_argument("--no-real-inspection-cam", dest="use_real_inspection_cam",
                   action="store_false", default=True,
                   help="Inspection 셀 카메라를 USD 대신 placeholder로 그림")
    p.add_argument("--no-factory-walls", dest="show_factory_walls",
                   action="store_false", default=True,
                   help="작업장을 감싸는 4면 벽을 그리지 않음")
    p.add_argument("--no-ceiling-lights", dest="show_ceiling_lights",
                   action="store_false", default=True,
                   help="천장 조명 피팅을 그리지 않음")

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

    # Level 2.2 옵션
    sim.config.motion_mode = args.motion_mode
    sim.config.enable_sparks = args.enable_sparks
    sim.config.spark_rate = args.spark_rate
    sim.config.spark_lifetime_sec = args.spark_lifetime
    sim.config.rmpflow_register_obstacles = args.rmpflow_register_obstacles

    # Level 2.3 옵션 — 셀별 실 USD 자산 선택
    sim.config.use_real_amr = args.use_real_amr
    sim.config.amr_asset_name = args.amr_asset
    sim.config.use_real_smart_rack = args.use_real_smart_rack
    sim.config.smart_rack_asset_name = args.smart_rack_asset
    sim.config.use_real_forming_arm = args.use_real_forming_arm
    sim.config.forming_robot_name = args.forming_robot
    sim.config.use_real_inspection_cam = args.use_real_inspection_cam
    sim.config.show_factory_walls = args.show_factory_walls
    sim.config.show_ceiling_lights = args.show_ceiling_lights

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
    print(" FDW-OPS PoC-1 Level 2.2 — RMPflow + 용접 스파크 파티클 데모")
    print("=" * 70)
    print(f" Mode             : isaac")
    print(f" Headless         : {sim.config.headless}")
    print(f" Livestream       : {sim.config.livestream}")
    print(f" Visualization    : {sim.config.enable_visualization}")
    print(f" Realtime         : {sim.config.realtime}")
    print(f" Anim duration    : {args.anim_duration:.1f}s")
    print(f" Real robot       : {sim.config.use_real_robot}")
    print(f" Robot model      : {sim.config.robot_name}")
    print(f" Motion enabled   : {sim.config.enable_ik}")
    print(f" Motion mode      : {sim.config.motion_mode} "
          f"(auto=rmpflow→ik→heuristic)")
    print(f" RMPflow obstacles: {sim.config.rmpflow_register_obstacles}")
    print(f" Sparks enabled   : {sim.config.enable_sparks}")
    print(f" Spark rate       : {sim.config.spark_rate:.1f} /s")
    print(f" Spark lifetime   : {sim.config.spark_lifetime_sec:.2f} s")
    print(f" Weld path offset : ±{sim.config.weld_path_offset_y:.2f} m")
    print(f" Weld path height : {sim.config.weld_path_height:.3f} m")
    print(f" --- Level 2.3 USD assets ---")
    print(f" Real AMR         : {sim.config.use_real_amr} "
          f"({sim.config.amr_asset_name})")
    print(f" Real smart rack  : {sim.config.use_real_smart_rack} "
          f"({sim.config.smart_rack_asset_name})")
    print(f" Real forming arm : {sim.config.use_real_forming_arm} "
          f"({sim.config.forming_robot_name})")
    print(f" Real inspect cam : {sim.config.use_real_inspection_cam}")
    print(f" Factory walls    : {sim.config.show_factory_walls}")
    print(f" Ceiling lights   : {sim.config.show_ceiling_lights}")
    print(f" Render mode      : {sim.config.render_mode}")
    print(f" NRD denoiser     : {'OFF (Blackwell-safe)' if sim.config.disable_nrd_denoiser else 'ON'}")
    print(f" RTX log spam     : {'suppressed' if sim.config.suppress_rtx_log_spam else 'visible'}")
    print(f" Safe mode        : {'ON' if sim.config.safe_mode else 'OFF'}")
    print(f" Skip auto camera : {sim.config.skip_auto_camera}")
    print(f" Disable Aftermath: {sim.config.disable_aftermath}")
    print(f" Force lighting   : {sim.config.force_lighting_mode or '(Kit default)'}")
    print(f" Viewport switch  : {'DISABLED' if os.environ.get('FDW_DISABLE_VIEWPORT_SWITCH') == '1' else 'enabled'}")
    print("=" * 70)
    if sim.config.use_real_robot:
        print(" [Hint] Franka/UR10 USD가 NGC/Nucleus에서 처음 로드되면")
        print("        셰이더 캐시가 만들어지며 1~3분이 추가로 소요될 수 있습니다.")
    if sim.config.motion_mode in ("auto", "rmpflow"):
        print(" [Hint] RMPflow가 실패하면 자동으로 IK→heuristic으로 fallback 됩니다.")
        print("        실제 사용된 백엔드는 로그의 [VIS] 메시지에서 확인하세요.")
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
        print("[INFO] 3 jobs 투입 — RMPflow 모션 + 스파크 파티클 시각화 시작\n")
        sim.run_until_done(all_jobs_done=True, extra_idle_sec=3.0)
        print_report(sim)
    finally:
        sim.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
