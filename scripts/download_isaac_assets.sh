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

# ---------------------------------------------------------------------
# USD 안에서 reference / payload 되는 sub-USD를 자동 추출 → 재귀 다운로드
# ---------------------------------------------------------------------
# Isaac USD 는 대부분 USDC(바이너리)이지만 그 안에 sub-USD 경로 문자열은
# 일반 텍스트로 박혀 있어 strings/grep 으로 안정적으로 추출 가능.
#
# 추출 대상: @<path>@ 형태의 asset path 중 .usd / .usda / .usdc 확장자.
# 절대 경로(omniverse://, http(s)://, /로 시작) 는 제외 — 상대 경로만 처리.
#
# 글로벌 visited set 으로 무한루프(원형 reference) 방지.
declare -A USD_VISITED=()

extract_and_download_refs() {
    local subpath="$1"           # 예: Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd
    local depth="${2:-0}"
    local max_depth=4

    if [ "$depth" -gt "$max_depth" ]; then
        return 0
    fi

    local dst="$DEST/$subpath"
    if [ ! -f "$dst" ] || [ ! -s "$dst" ]; then
        return 0
    fi

    # 이미 처리했으면 skip
    if [ "${USD_VISITED[$subpath]:-}" = "1" ]; then
        return 0
    fi
    USD_VISITED[$subpath]=1

    local parent_dir
    parent_dir="$(dirname "$subpath")"

    # @<path>@ 형식 reference 추출 (USDC 도 평문 경로는 노출됨)
    # strings 가 없으면 grep -ao 로 fallback.
    local refs
    if command -v strings >/dev/null 2>&1; then
        refs=$(strings "$dst" 2>/dev/null \
               | grep -oE '@[^@[:space:]]+\.(usd|usda|usdc)@' \
               | sed 's/^@//; s/@$//' \
               | sort -u)
    else
        refs=$(grep -aoE '@[^@[:space:]]+\.(usd|usda|usdc)@' "$dst" 2>/dev/null \
               | sed 's/^@//; s/@$//' \
               | sort -u)
    fi

    if [ -z "$refs" ]; then
        return 0
    fi

    local indent=""
    for ((i=0; i<depth; i++)); do indent="$indent  "; done

    local sub
    while IFS= read -r sub; do
        [ -z "$sub" ] && continue
        # 절대 경로 / URL 은 그대로 두기 (받을 수 없음)
        case "$sub" in
            http://*|https://*|omniverse://*|/*)
                continue
                ;;
        esac

        # 상대 경로 → 부모 디렉토리 기준으로 normalize
        local rel="$sub"
        # ./xxx → xxx
        rel="${rel#./}"
        local target
        if [[ "$rel" == ../* ]]; then
            # 상위 디렉토리 참조 — 단순 처리 (한 단계만)
            local up="$(dirname "$parent_dir")"
            local clean="${rel#../}"
            target="$up/$clean"
            # 다중 .. 처리
            while [[ "$target" == */.. ]] || [[ "$target" == */../* ]]; do
                # bash 단순 정규화는 한계가 있음 — realpath 사용
                if command -v realpath >/dev/null 2>&1; then
                    target=$(realpath -m --relative-to=. "$DEST/$target" 2>/dev/null \
                             | sed "s|^$DEST/||")
                fi
                break
            done
        else
            target="$parent_dir/$rel"
        fi

        # Isaac/ 로 시작하지 않으면 무시 (다른 카테고리)
        case "$target" in
            Isaac/*) ;;
            *) continue ;;
        esac

        echo "${indent}[ref ↪] $target  (from $subpath)"
        if download_subpath "$target"; then
            # 재귀 — sub-USD 도 자기만의 reference 가 있을 수 있음
            extract_and_download_refs "$target" $((depth + 1)) || true
        fi
    done <<< "$refs"
}

# 한 자산의 메인 USD + 자주 reference 되는 sub-USD를 함께 받기.
# 명시한 sub_subpaths 를 먼저 받고, 그 다음 USD 안의 reference 를 자동 추출
# 해서 추가로 끌어온다 (NVIDIA 가 디렉토리를 바꿔도 안전).
download_asset_with_subusds() {
    local main_subpath="$1"
    shift
    local sub_subpaths=("$@")

    download_subpath "$main_subpath" || return 1

    for sub in "${sub_subpaths[@]}"; do
        download_subpath "$sub" || true
    done

    # 메인 + 명시한 sub 들 각각에서 내부 reference 추출 → 재귀 다운로드
    info "  → scanning $main_subpath for internal references..."
    extract_and_download_refs "$main_subpath" 0
    for sub in "${sub_subpaths[@]}"; do
        extract_and_download_refs "$sub" 1 || true
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
        # Materials.usd — panda_link*.usd 안의 Looks 가 reference 하는 머티리얼.
        # 누락 시 메쉬는 보이지만 색/광택이 빠짐 (사용자 보고 케이스).
        "Isaac/Robots/FrankaRobotics/FrankaPanda/Materials/Materials.usd"
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
    # UR 시리즈가 내부적으로 참조하는 sub-USD (Props/Materials)
    # NVIDIA Isaac 5.1에서 UR 로봇은 ur10.usd 단일 파일에 자체 포함되는 경우가 많지만
    # 일부 변종은 sub-USD를 가짐 — best-effort.
    for ur_name in ur10 ur10e ur5e ur16e; do
        for sub in Props Materials configuration; do
            # 디렉토리 단위 탐색은 안 되므로 추정 경로만 시도
            download_subpath "Isaac/Robots/UniversalRobots/${ur_name}/${sub}/${ur_name}_robot_schema.usd" || true
        done
    done
}

# ---------------------------------------------------------------------
# 3) AMR — NVIDIA Nova Carter / Jetbot / Idealworks iw.hub (Level 2.3)
# ---------------------------------------------------------------------
download_amr() {
    hdr "3) AMRs (Level 2.3 mobile robots)"
    # NVIDIA Nova Carter — 풀 센서 스택 AMR (기본)
    download_subpath "Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd" || true
    # Nova Carter 의 payload/reference sub-USD (사용자 보고 케이스 — 누락 시 children=0)
    download_subpath "Isaac/Robots/NVIDIA/NovaCarter/Variants/Physics/nova_carter_physics.usd" || true
    download_subpath "Isaac/Robots/NVIDIA/NovaCarter/Variants/Sensors/nova_carter_sensors.usd" || true
    download_subpath "Isaac/Robots/NVIDIA/NovaCarter/Variants/Sensors/nova_carter_no_sensors.usd" || true
    download_subpath "Isaac/Robots/NVIDIA/NovaCarter/Variants/Configuration/nova_carter_merged_no_internals.usd" || true
    download_subpath "Isaac/Robots/NVIDIA/NovaCarter/Variants/Configuration/nova_carter_merged_with_internals.usd" || true
    # 2026-05 Thor 실측 추가 — merged_no_internals 가 sublayer 로 요구하는 파일.
    # 누락 시 _ReportErrors 에서 "Could not load sublayer ...
    # nova_carter_sim_optimized.usd" 가 발생하고 reference 전체가 expire 된다.
    download_subpath "Isaac/Robots/NVIDIA/NovaCarter/Variants/nova_carter_sim_optimized.usd" || true
    # Nova Carter Props / Materials — 메쉬와 텍스처
    local nc_props=(
        "Isaac/Robots/NVIDIA/NovaCarter/Props/nova_carter_base.usd"
        "Isaac/Robots/NVIDIA/NovaCarter/Props/nova_carter_chassis.usd"
        "Isaac/Robots/NVIDIA/NovaCarter/Props/nova_carter_wheel.usd"
        "Isaac/Robots/NVIDIA/NovaCarter/Props/nova_carter_caster.usd"
        "Isaac/Robots/NVIDIA/NovaCarter/Materials/Materials.usd"
    )
    for p in "${nc_props[@]}"; do
        download_subpath "$p" || true
    done

    # Nova Carter sensor sub-USD payloads (Sensors variant != None 일 때 필요).
    # Thor 실측에서 14 개 "Could not open asset" payload missing warning 의
    # 원인. asset_catalog 가 Sensors="None" 이름으로 설정한 경우에도, variant
    # 이름이 USD 와 일치하지 않으면 default Sensors variant 가 적용되어
    # 이 payload 들을 여전히 요구한다. 따라서 best-effort 로 받아둔다.
    local nc_sensors=(
        "Isaac/Sensors/LeopardImaging/Hawk/hawk_v1.1_nominal.usd"
        "Isaac/Sensors/LeopardImaging/Owl/owl.usd"
        "Isaac/Sensors/Slamtec/RPLidar_S2e.usd"
        "Isaac/Sensors/HESAI/XT-32.usd"
    )
    for s in "${nc_sensors[@]}"; do
        download_subpath "$s" || true
    done

    # NVIDIA Jetbot — 컴팩트 2륜 AMR (대부분 self-contained)
    download_subpath "Isaac/Robots/NVIDIA/Jetbot/jetbot.usd" || true
    download_subpath "Isaac/Robots/NVIDIA/Jetbot/Props/jetbot_base.usd" || true
    download_subpath "Isaac/Robots/NVIDIA/Jetbot/Materials/Materials.usd" || true

    # Idealworks iw.hub — 산업용 AMR
    download_subpath "Isaac/Robots/Idealworks/iwhub/iw_hub.usd" || true
    download_subpath "Isaac/Robots/Idealworks/iwhub/iw_hub_static.usd" || true
    download_subpath "Isaac/Robots/Idealworks/iwhub/Props/iw_hub_base.usd" || true
    download_subpath "Isaac/Robots/Idealworks/iwhub/Materials/Materials.usd" || true
}

# ---------------------------------------------------------------------
# 4) Props — KLT bin, cardboard box (material 셀 smart rack, Level 2.3)
# ---------------------------------------------------------------------
download_props() {
    hdr "4) Props (material cell smart rack contents)"
    # KLT bin — small_KLT_visual_collision.usd 가 small_KLT_visual.usd 를
    # internal reference 하므로 둘 다 받아야 메쉬가 보임 (사용자 보고 케이스).
    download_subpath "Isaac/Props/KLT_Bin/small_KLT_visual_collision.usd" || true
    download_subpath "Isaac/Props/KLT_Bin/small_KLT_visual.usd" || true
    download_subpath "Isaac/Props/KLT_Bin/small_KLT.usd" || true
    # 변종/관련 파일 — best-effort
    download_subpath "Isaac/Props/KLT_Bin/big_KLT_visual_collision.usd" || true
    download_subpath "Isaac/Props/KLT_Bin/big_KLT_visual.usd" || true
    download_subpath "Isaac/Props/KLT_Bin/Materials/Materials.usd" || true

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
