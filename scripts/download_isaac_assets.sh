#!/usr/bin/env bash
# download_isaac_assets.sh — Isaac Sim 5.1 USD 자산 일괄 다운로드
#
# 배경:
#   AGX Thor (JetPack 7) 환경에서 Isaac Sim 5.1이 USD 자산을 NVIDIA S3
#   (https://omniverse-content-production.s3-us-west-2.amazonaws.com/...)
#   에서 직접 fetch 하려고 하면 Omni Client HTTPS resolver가 제대로
#   동작하지 않아 빈 prim만 만들어지고 자식이 0개가 되는 경우가 있다.
#
# 이 스크립트는 다음 자산들을 ~/isaac_assets/Isaac/ 아래에 받아둔다:
#   1) Franka Panda 로봇 (welding 셀 — Level 2.1)
#   2) Universal Robots UR10 / UR10e (forming 셀 — Level 2.3)
#   3) NVIDIA Nova Carter, Jetbot (AMR — Level 2.3)
#   4) Idealworks iw.hub / iw_hub_static (AMR — Level 2.3)
#   5) KLT Bin 등 부품 컨테이너 (material 셀 smart rack — Level 2.3)
#
# 시뮬레이터 실행 시 환경변수 한 줄로 사용 가능:
#
#     export ISAAC_NUCLEUS_DIR_LOCAL=~/isaac_assets
#
# 사용법:
#     bash scripts/download_isaac_assets.sh
#
# 옵션:
#     DEST=/path/to/dir bash scripts/download_isaac_assets.sh
#     ONLY=franka|robots|amr|props bash scripts/download_isaac_assets.sh
#         특정 카테고리만 받기 (기본: all)
#
# 참고:
#     https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_robots.html

set -u

DEST="${DEST:-$HOME/isaac_assets}"
ONLY="${ONLY:-all}"
BASE_URL="https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1"

color() { printf "\033[%sm%s\033[0m" "$1" "$2"; }
info()  { echo "$(color "36" "[INFO]") $*"; }
warn()  { echo "$(color "33" "[WARN]") $*"; }
ok()    { echo "$(color "32" "[ OK ]") $*"; }
err()   { echo "$(color "31" "[FAIL]") $*"; }
hdr()   { echo; echo "$(color "35" "=== $* ===")"; }

mkdir -p "$DEST"

info "Destination : $DEST"
info "Source root : $BASE_URL/"
info "Filter      : ONLY=$ONLY"

# wget 확인
if ! command -v wget >/dev/null 2>&1; then
    err "wget이 필요합니다. sudo apt install wget 후 다시 실행해주세요."
    exit 1
fi

# ---------------------------------------------------------------------
# 공통 다운로드 함수 — subpath = Isaac/... 형태의 상대 경로
# ---------------------------------------------------------------------
download_subpath() {
    local subpath="$1"
    local url="$BASE_URL/$subpath"
    local dst="$DEST/$subpath"
    mkdir -p "$(dirname "$dst")"
    if [ -f "$dst" ] && [ -s "$dst" ]; then
        ok "exists: $subpath"
        return 0
    fi
    info "downloading: $subpath"
    if wget -q --show-progress -O "$dst" "$url" && [ -s "$dst" ]; then
        ok "got: $subpath ($(du -h "$dst" | cut -f1))"
        return 0
    else
        warn "missing (404 or network): $subpath"
        rm -f "$dst"
        return 1
    fi
}

# 한 자산의 메인 USD + 자주 reference 되는 sub-USD를 함께 받기
# (sub-USD는 best-effort — 일부 자산은 self-contained)
download_asset_with_subusds() {
    local main_subpath="$1"
    shift
    local sub_subpaths=("$@")

    download_subpath "$main_subpath" || return 1

    for sub in "${sub_subpaths[@]}"; do
        download_subpath "$sub" || true
    done
}

# ---------------------------------------------------------------------
# 1) Franka Panda — welding 셀 (Level 2.1)
# ---------------------------------------------------------------------
download_franka() {
    hdr "1) Franka Panda (welding cell)"
    local franka_main="Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
    local franka_subs=(
        "Isaac/Robots/FrankaRobotics/FrankaPanda/configuration/franka_robot_schema.usd"
        "Isaac/Robots/FrankaRobotics/FrankaPanda/Props/panda_link0.usd"
        "Isaac/Robots/FrankaRobotics/FrankaPanda/Props/panda_link1.usd"
        "Isaac/Robots/FrankaRobotics/FrankaPanda/Props/panda_link2.usd"
        "Isaac/Robots/FrankaRobotics/FrankaPanda/Props/panda_link3.usd"
        "Isaac/Robots/FrankaRobotics/FrankaPanda/Props/panda_link4.usd"
        "Isaac/Robots/FrankaRobotics/FrankaPanda/Props/panda_link5.usd"
        "Isaac/Robots/FrankaRobotics/FrankaPanda/Props/panda_link6.usd"
        "Isaac/Robots/FrankaRobotics/FrankaPanda/Props/panda_link7.usd"
        "Isaac/Robots/FrankaRobotics/FrankaPanda/Props/panda_hand.usd"
        "Isaac/Robots/FrankaRobotics/FrankaPanda/Props/panda_leftfinger.usd"
        "Isaac/Robots/FrankaRobotics/FrankaPanda/Props/panda_rightfinger.usd"
    )
    download_asset_with_subusds "$franka_main" "${franka_subs[@]}"
}

# ---------------------------------------------------------------------
# 2) Universal Robots UR10 / UR10e — forming 셀 (Level 2.3)
# ---------------------------------------------------------------------
download_ur_robots() {
    hdr "2) Universal Robots (forming cell)"
    # UR10 (legacy, used by ROBOT_CATALOG['ur10'])
    download_subpath "Isaac/Robots/UniversalRobots/ur10/ur10.usd" || true
    # UR10e (E-series, used by ASSET_CATALOG['ur10e'])
    download_subpath "Isaac/Robots/UniversalRobots/ur10e/ur10e.usd" || true
    # 기타 변종 — best-effort
    download_subpath "Isaac/Robots/UniversalRobots/ur5e/ur5e.usd" || true
    download_subpath "Isaac/Robots/UniversalRobots/ur16e/ur16e.usd" || true
}

# ---------------------------------------------------------------------
# 3) AMR — NVIDIA Nova Carter / Jetbot / Idealworks iw.hub (Level 2.3)
# ---------------------------------------------------------------------
download_amr() {
    hdr "3) AMRs (Level 2.3 mobile robots)"
    # NVIDIA Nova Carter — 풀 센서 스택 AMR (기본)
    download_subpath "Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd" || true
    # NVIDIA Jetbot — 컴팩트 2륜 AMR
    download_subpath "Isaac/Robots/NVIDIA/Jetbot/jetbot.usd" || true
    # Idealworks iw.hub — 산업용 AMR
    download_subpath "Isaac/Robots/Idealworks/iwhub/iw_hub.usd" || true
    download_subpath "Isaac/Robots/Idealworks/iwhub/iw_hub_static.usd" || true
}

# ---------------------------------------------------------------------
# 4) Props — KLT bin, cardboard box (material 셀 smart rack, Level 2.3)
# ---------------------------------------------------------------------
download_props() {
    hdr "4) Props (material cell smart rack contents)"
    download_subpath "Isaac/Props/KLT_Bin/small_KLT_visual_collision.usd" || true
    download_subpath "Isaac/Props/Cardboard_Box/cardboard_box.usd" || true
    # 산업용 카메라 (inspection 셀 대체용 — 기본은 자체 UsdGeom.Camera 사용)
    download_subpath "Isaac/Sensors/Cameras/Industrial/CN_AMR_C320/CN_AMR_C320.usd" || true
}

# ---------------------------------------------------------------------
# 실행 — ONLY 필터에 따라 카테고리 선택
# ---------------------------------------------------------------------
case "$ONLY" in
    franka)  download_franka ;;
    robots)  download_ur_robots ;;
    amr)     download_amr ;;
    props)   download_props ;;
    all|"")
        download_franka
        download_ur_robots
        download_amr
        download_props
        ;;
    *)
        err "Unknown ONLY=$ONLY (expected: all|franka|robots|amr|props)"
        exit 2
        ;;
esac

# ---------------------------------------------------------------------
# 결과 요약
# ---------------------------------------------------------------------
hdr "결과 요약"
TOTAL=$(find "$DEST" -type f 2>/dev/null | wc -l)
SIZE=$(du -sh "$DEST" 2>/dev/null | cut -f1)
ok "다운로드된 파일 수: $TOTAL"
ok "총 크기            : $SIZE"
echo

cat <<EOF
==================================================================
 다음 단계 — 시뮬레이터가 로컬 USD를 사용하도록 환경변수 설정:

   export ISAAC_NUCLEUS_DIR_LOCAL="$DEST"

 영구 설정하려면 ~/.bashrc 에 추가:

   echo 'export ISAAC_NUCLEUS_DIR_LOCAL="$DEST"' >> ~/.bashrc

 그리고 Level 2.3 실행 (모든 셀이 실 USD 모델로 표시됨):

   ACCEPT_EULA=Y PRIVACY_CONSENT=Y \\
       python scripts/run_poc1_level2_2.py --gui --real-robot \\
       --motion-mode auto \\
       --disable-aftermath --disable-viewport-switch

 카탈로그 진단 (어떤 자산이 로컬에 있는지 확인):

   python -c "from fdw_sim.visualization.asset_catalog import diagnose_catalog; \\
       import json; print(json.dumps(diagnose_catalog(), indent=2))"
==================================================================
EOF
