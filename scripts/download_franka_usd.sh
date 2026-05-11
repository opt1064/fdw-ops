#!/usr/bin/env bash
# download_franka_usd.sh — Isaac Sim 5.1 Franka Panda USD 자산을 로컬에 다운로드
#
# 배경:
#   AGX Thor (JetPack 7) 환경에서 Isaac Sim 5.1이 USD 자산을 NVIDIA S3
#   (https://omniverse-content-production.s3-us-west-2.amazonaws.com/...)
#   에서 직접 fetch 하려고 하면 Omni Client HTTPS resolver가 제대로
#   동작하지 않아 빈 prim만 만들어지고 자식이 0개가 되는 경우가 있다.
#
# ⚠️ Isaac Sim 5.1에서 NVIDIA가 USD 자산 경로를 재구성했다.
#    옛 경로:  Isaac/Robots/Franka/franka.usd                          (4.x — 5.1에서 404)
#    새 경로:  Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd      (5.1 정식 경로)
#    참고:    https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_robots.html
#
# 이 스크립트는 Franka Panda USD 와 그것이 reference 하는 sub-USD/meshes/
# materials를 모두 ~/isaac_assets/Isaac/Robots/FrankaRobotics/FrankaPanda/
# 에 받아둔다. 시뮬레이터 실행 시 환경변수 한 줄로 사용 가능:
#
#     export ISAAC_NUCLEUS_DIR_LOCAL=~/isaac_assets
#
# 사용법:
#     bash scripts/download_franka_usd.sh
#
# 옵션:
#     DEST=/path/to/dir bash scripts/download_franka_usd.sh
#         기본 ~/isaac_assets 대신 다른 곳에 저장

set -u

DEST="${DEST:-$HOME/isaac_assets}"
BASE_URL="https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1"
# Isaac 5.1 신규 경로
FRANKA_SUBDIR="Isaac/Robots/FrankaRobotics/FrankaPanda"
FRANKA_DIR="$DEST/$FRANKA_SUBDIR"

color() { printf "\033[%sm%s\033[0m" "$1" "$2"; }
info()  { echo "$(color "36" "[INFO]") $*"; }
warn()  { echo "$(color "33" "[WARN]") $*"; }
ok()    { echo "$(color "32" "[ OK ]") $*"; }
err()   { echo "$(color "31" "[FAIL]") $*"; }

mkdir -p "$FRANKA_DIR"

info "Destination : $FRANKA_DIR"
info "Source root : $BASE_URL/$FRANKA_SUBDIR/"

# wget 확인
if ! command -v wget >/dev/null 2>&1; then
    err "wget이 필요합니다. sudo apt install wget 후 다시 실행해주세요."
    exit 1
fi

# ---------------------------------------------------------------------
# 1) 최상위 franka.usd 받기
# ---------------------------------------------------------------------
download_file() {
    local rel="$1"
    local url="$BASE_URL/$FRANKA_SUBDIR/$rel"
    local dst="$FRANKA_DIR/$rel"
    mkdir -p "$(dirname "$dst")"
    if [ -f "$dst" ] && [ -s "$dst" ]; then
        ok "exists: $rel"
        return 0
    fi
    info "downloading: $rel"
    if wget -q --show-progress -O "$dst" "$url"; then
        if [ ! -s "$dst" ]; then
            warn "empty file removed: $rel"
            rm -f "$dst"
            return 1
        fi
        ok "got: $rel ($(du -h "$dst" | cut -f1))"
        return 0
    else
        warn "missing (404 or network): $rel"
        rm -f "$dst"
        return 1
    fi
}

# Isaac 5.1 FrankaPanda 디렉토리의 알려진 파일 목록
# (NVIDIA S3는 listing이 안되므로 명시적으로 시도)
# franka_alt_fingers.usd 는 5.1에서 사라졌고, panda_instanceable.usd 는
# 형제 디렉토리 FrankaEmika/ 로 이동했다.
TOP_FILES=(
    "franka.usd"
)

# Isaac 5.1 형제 디렉토리에 있는 관련 변종들도 함께 받아둔다.
# 받기 실패해도 무시 (404 가능).
SIBLING_FILES=(
    "Isaac/Robots/FrankaRobotics/FrankaEmika/panda_instanceable.usd"
    "Isaac/Robots/FrankaRobotics/FrankaFR3/fr3.usd"
    "Isaac/Robots/FrankaRobotics/FactoryFranka/factory_franka.usd"
    "Isaac/Robots/FrankaRobotics/FactoryFranka/factory_franka_instanceable.usd"
)

download_sibling() {
    local subpath="$1"
    local url="$BASE_URL/$subpath"
    local dst="$DEST/$subpath"
    mkdir -p "$(dirname "$dst")"
    if [ -f "$dst" ] && [ -s "$dst" ]; then
        ok "exists: $subpath"
        return 0
    fi
    info "downloading sibling: $subpath"
    if wget -q --show-progress -O "$dst" "$url" && [ -s "$dst" ]; then
        ok "got: $subpath ($(du -h "$dst" | cut -f1))"
        return 0
    else
        warn "missing (optional): $subpath"
        rm -f "$dst"
        return 1
    fi
}

info "=== 1) Top-level USD 파일 ==="
GOT_ANY=0
for f in "${TOP_FILES[@]}"; do
    if download_file "$f"; then
        GOT_ANY=1
    fi
done

if [ "$GOT_ANY" -eq 0 ]; then
    err "최상위 franka.usd 파일을 하나도 못 받았습니다."
    err "URL이 바뀌었을 가능성이 있습니다. 다음을 직접 확인해주세요:"
    err "  $BASE_URL/$FRANKA_SUBDIR/franka.usd"
    err "참고 (Isaac 5.1 공식 자산 목록):"
    err "  https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_robots.html"
    exit 2
fi

# 형제 디렉토리(FrankaEmika/FrankaFR3/FactoryFranka)도 best-effort로 받음
info "=== 1b) Isaac 5.1 Franka 형제 자산 ==="
for f in "${SIBLING_FILES[@]}"; do
    download_sibling "$f" || true
done

# ---------------------------------------------------------------------
# 2) franka.usd 내부 reference 자동 추출 → sub-files 받기
# ---------------------------------------------------------------------
info "=== 2) franka.usd 내부 reference 추출 ==="

MAIN_USD="$FRANKA_DIR/franka.usd"
if [ ! -f "$MAIN_USD" ]; then
    warn "franka.usd가 없어 reference 추출을 건너뜁니다."
    exit 0
fi

# USD ASCII 파일 안의 @<path>@ 형식 reference 추출
# (binary USD인 경우 grep으로는 안 잡힘 — 그 경우 수동으로 chase 필요)
REFS=$(grep -oE '@[^@]+\.(usd|usda|usdc|usdz|png|jpg|jpeg|exr|mdl)@' "$MAIN_USD" 2>/dev/null \
       | sed 's/^@//; s/@$//' \
       | sort -u || true)

if [ -z "$REFS" ]; then
    # binary일 가능성 — file 명령어로 확인
    if file "$MAIN_USD" 2>/dev/null | grep -qi "data"; then
        warn "franka.usd가 binary 형식이라 reference를 텍스트로 추출할 수 없습니다."
        warn "흔히 쓰이는 Franka sub-asset 경로들을 휴리스틱으로 시도합니다."
    else
        info "franka.usd에 외부 reference가 없습니다 (self-contained)."
    fi

    # ⭐ Isaac 5.1 FrankaPanda 의 실제 sub-USD 목록
    # franka.usd (USDC 바이너리)가 internal로 참조하는 파일들 — Isaac Sim 실제 런타임
    # 경고 로그에서 추출한 정확한 경로 (2026-05-11 검증, 모두 HTTP 200 OK).
    # franka.usd 자체는 ~30 KB wrapper 일 뿐이고, 메쉬는 모두 여기에 있다.
    REQUIRED_FILES=(
        "configuration/franka_robot_schema.usd"   # 4 KB  — articulation schema
        "Props/panda_link0.usd"                   # 2.1 MB
        "Props/panda_link1.usd"                   # 502 KB
        "Props/panda_link2.usd"                   # 502 KB
        "Props/panda_link3.usd"                   # 868 KB
        "Props/panda_link4.usd"                   # 868 KB
        "Props/panda_link5.usd"                   # 780 KB
        "Props/panda_link6.usd"                   # 2.3 MB
        "Props/panda_link7.usd"                   # 1.3 MB
        "Props/panda_hand.usd"                    # 688 KB
        "Props/panda_leftfinger.usd"              # 74 KB
        "Props/panda_rightfinger.usd"             # 71 KB
    )
    info "Isaac 5.1 Franka sub-USD 자산 다운로드 (≈10 MB, 12개 파일):"
    MISSING=0
    for f in "${REQUIRED_FILES[@]}"; do
        if ! download_file "$f"; then
            MISSING=$((MISSING + 1))
        fi
    done
    if [ "$MISSING" -gt 0 ]; then
        warn "$MISSING 개의 필수 sub-USD를 못 받았습니다. GUI에서 메쉬가 안 보일 수 있습니다."
    else
        ok "모든 sub-USD 자산 확보 완료 — GUI 에서 Franka 메쉬가 표시됩니다."
    fi
else
    info "발견된 reference 수: $(echo "$REFS" | wc -l)"
    while IFS= read -r ref; do
        # 상대 경로만 처리 (절대 경로/URL은 건드리지 않음)
        if [[ "$ref" == http://* || "$ref" == https://* \
                || "$ref" == omniverse://* || "$ref" == /* ]]; then
            warn "skip absolute ref: $ref"
            continue
        fi
        # 상대 경로 → franka.usd 기준으로 download
        download_file "$ref" || true
    done <<< "$REFS"
fi

# ---------------------------------------------------------------------
# 3) 결과 요약
# ---------------------------------------------------------------------
echo
info "=== 3) 결과 요약 ==="
TOTAL=$(find "$FRANKA_DIR" -type f | wc -l)
SIZE=$(du -sh "$FRANKA_DIR" 2>/dev/null | cut -f1)
ok "다운로드된 파일 수: $TOTAL"
ok "총 크기            : $SIZE"
echo

cat <<EOF
==================================================================
 다음 단계 — 시뮬레이터가 로컬 USD를 사용하도록 환경변수 설정:

   export ISAAC_NUCLEUS_DIR_LOCAL="$DEST"

 영구 설정하려면 ~/.bashrc 에 추가:

   echo 'export ISAAC_NUCLEUS_DIR_LOCAL="$DEST"' >> ~/.bashrc

 그리고 Level 2.1 실행:

   ACCEPT_EULA=Y PRIVACY_CONSENT=Y \\
       python scripts/run_poc1_level2_1.py --gui --real-robot \\
       --robot franka_panda --safe-mode

 로드가 성공하면 다음 로그가 보입니다:
   [VIS] source   : LOCAL disk
   [VIS] <<< robot franka_panda loaded successfully (children=N>0)
==================================================================
EOF
