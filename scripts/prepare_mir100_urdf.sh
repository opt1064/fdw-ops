#!/usr/bin/env bash
# prepare_mir100_urdf.sh — MiR100 URDF를 로컬에 준비 (Isaac Sim/GPU 불필요)
#
# 배경:
#   NVIDIA Isaac Sim 6.1 공식 자산 카탈로그에는 MiR100 USD가 없다
#   (2026-09-28 omniverse-content-production S3 버킷 직접 확인 —
#   fdw_sim/visualization/asset_catalog.py의 "mir100" 항목 주석 참고).
#   대신 커뮤니티 ROS 패키지 DFKI-NI/mir_robot (BSD-3-Clause)의 URDF를
#   가져와 USD로 변환해야 한다. 이 스크립트는 그 변환의 "1단계"만 한다:
#
#     1) (이 스크립트, GPU 불필요) mir_robot 저장소를 clone하고,
#        xacro로 매크로를 전개해서 순수 URDF 한 장으로 만들고,
#        <mesh filename="package://...">를 로컬 절대경로로 바꾼다.
#     2) (scripts/import_mir100_usd.py, Isaac Sim 필요) 그 URDF를
#        isaacsim.asset.importer.urdf로 실제 USD로 변환한다.
#
# 사용법:
#     bash scripts/prepare_mir100_urdf.sh
#     DEST=/custom/path bash scripts/prepare_mir100_urdf.sh
#
#     # DGX Spark처럼 GitHub 아웃바운드가 막혀 있으면 zip을 미리 받아서:
#     MIR_ROBOT_ZIP=~/Downloads/mir_robot-noetic.zip bash scripts/prepare_mir100_urdf.sh
#     # 또는 이미 압축 해제된 디렉토리가 있으면:
#     MIR_ROBOT_SRC=~/mir_robot bash scripts/prepare_mir100_urdf.sh
#
# 출력:
#     $DEST/mir100.urdf         — 전개 완료된 최종 URDF (mesh 경로는 로컬 절대경로)
#     $DEST/meshes/...          — mir100.urdf가 참조하는 STL 메시들
#
# 다음 단계:
#     "$ISAACSIM_PYTHON_EXE" scripts/import_mir100_usd.py \
#         --urdf "$DEST/mir100.urdf" \
#         --out ~/isaac_assets/Robots/MiR/mir100/mir100.usd
#
#     export FDW_MIR100_USD=~/isaac_assets/Robots/MiR/mir100/mir100.usd

set -euo pipefail

DEST="${DEST:-$HOME/isaac_assets/mir100_urdf_src}"
MIR_REPO_URL="https://github.com/DFKI-NI/mir_robot"
MIR_TYPE="mir_100"

color() { printf "\033[%sm%s\033[0m" "$1" "$2"; }
info()  { echo "$(color "36" "[INFO]") $*"; }
warn()  { echo "$(color "33" "[WARN]") $*"; }
ok()    { echo "$(color "32" "[ OK ]") $*"; }
err()   { echo "$(color "31" "[FAIL]") $*"; }

info "Destination : $DEST"

# ---------------------------------------------------------------------
# 0) 의존성 확인 — xacro (standalone pip 패키지, 전체 ROS 설치 불필요)
# ---------------------------------------------------------------------
if ! command -v xacro >/dev/null 2>&1; then
    info "xacro CLI가 없어 pip로 설치합니다 (전체 ROS 설치는 필요 없음)"
    pip install --quiet xacro rospkg || {
        err "pip install xacro 실패 — 수동으로 'pip install xacro rospkg' 실행 후 재시도"
        exit 1
    }
fi
ok "xacro: $(command -v xacro)"

SRC_DIR="$DEST/mir_robot_src"

# ---------------------------------------------------------------------
# 1) mir_robot 소스 확보 — 3가지 방법 지원 (우선순위 순):
#      a) MIR_ROBOT_ZIP=/path/to/mir_robot-noetic.zip  — 로컬 zip 압축 해제
#         (DGX Spark처럼 GitHub 아웃바운드가 막힌 오프라인 환경용.
#         DFKI-NI/mir_robot의 노에틱/기본 브랜치 zip이면 그대로 동작함 —
#         2026-09-28에 두 소스를 diff -rq로 대조해 mir_description이
#         바이트 단위로 동일함을 확인함)
#      b) MIR_ROBOT_SRC=/path/to/already/extracted/mir_robot — 이미 풀린
#         디렉토리(mir_description을 포함하는 상위 폴더) 재사용
#      c) 위 둘 다 없으면 git clone (인터넷 필요)
# ---------------------------------------------------------------------
mkdir -p "$DEST"

if [ -n "${MIR_ROBOT_ZIP:-}" ]; then
    if [ ! -f "$MIR_ROBOT_ZIP" ]; then
        err "MIR_ROBOT_ZIP 파일을 찾을 수 없음: $MIR_ROBOT_ZIP"
        exit 1
    fi
    info "로컬 zip 사용: $MIR_ROBOT_ZIP"
    rm -rf "$SRC_DIR"
    mkdir -p "$SRC_DIR.unzip_tmp"
    unzip -q "$MIR_ROBOT_ZIP" -d "$SRC_DIR.unzip_tmp"
    # GitHub zip은 보통 <repo>-<branch>/ 한 겹으로 감싸져 있음 — 그 안에서
    # mir_description을 가진 디렉토리를 찾아 SRC_DIR로 삼는다.
    FOUND_DIR=$(find "$SRC_DIR.unzip_tmp" -maxdepth 3 -type d -name mir_description \
        -exec dirname {} \; | head -1)
    if [ -z "$FOUND_DIR" ]; then
        err "압축 안에서 mir_description 디렉토리를 못 찾음 — zip 내용 확인 필요"
        exit 1
    fi
    mv "$FOUND_DIR" "$SRC_DIR"
    rm -rf "$SRC_DIR.unzip_tmp"
    ok "압축 해제 완료: $SRC_DIR"
elif [ -n "${MIR_ROBOT_SRC:-}" ]; then
    if [ ! -f "$MIR_ROBOT_SRC/mir_description/package.xml" ]; then
        err "MIR_ROBOT_SRC/mir_description/package.xml 을 찾을 수 없음: $MIR_ROBOT_SRC"
        exit 1
    fi
    info "기존 디렉토리 재사용: $MIR_ROBOT_SRC"
    SRC_DIR="$MIR_ROBOT_SRC"
elif [ -d "$SRC_DIR/.git" ]; then
    info "이미 clone되어 있음 — 재사용: $SRC_DIR"
else
    if ! command -v git >/dev/null 2>&1; then
        err "git이 필요합니다 (또는 MIR_ROBOT_ZIP/MIR_ROBOT_SRC로 오프라인 소스 지정)."
        exit 1
    fi
    info "clone: $MIR_REPO_URL"
    rm -rf "$SRC_DIR"
    GIT_LFS_SKIP_SMUDGE=1 git clone --depth 1 "$MIR_REPO_URL" "$SRC_DIR"
fi

DESC_DIR="$SRC_DIR/mir_description"
if [ ! -f "$DESC_DIR/package.xml" ]; then
    err "$DESC_DIR/package.xml 을 찾을 수 없음 — 저장소 구조가 바뀐 것 같습니다"
    exit 1
fi
ok "mir_description: $DESC_DIR"

# ---------------------------------------------------------------------
# 2) xacro의 $(find mir_description)을 절대경로로 치환한 작업용 복사본 생성
#    (rospkg/ament 없이도 xacro가 돌게 하기 위함 — 2026-09-28 검증됨:
#    표준 ROS1 문법 $(find pkg)는 xacro include 단계에서만 쓰이므로
#    macro 전개 전에 텍스트로 치환해버리면 깔끔하게 해결된다)
# ---------------------------------------------------------------------
BUILD_DIR="$DEST/mir_description_build"
rm -rf "$BUILD_DIR"
cp -r "$DESC_DIR" "$BUILD_DIR"
grep -rl '\$(find mir_description)' "$BUILD_DIR" | \
    xargs -r sed -i "s#\$(find mir_description)#$BUILD_DIR#g"
ok "xacro \$(find) 치환 완료: $BUILD_DIR"

# ---------------------------------------------------------------------
# 3) xacro 전개 → 순수 URDF
# ---------------------------------------------------------------------
RAW_URDF="$DEST/mir100_raw.urdf"
info "xacro 전개 중 (mir_type=$MIR_TYPE)..."
xacro "$BUILD_DIR/urdf/mir.urdf.xacro" "mir_type:=$MIR_TYPE" > "$RAW_URDF"
ok "URDF 생성: $RAW_URDF ($(wc -l < "$RAW_URDF") lines)"

# ---------------------------------------------------------------------
# 4) <mesh filename="package://mir_description/...">를 로컬 절대경로로 치환
#    (Isaac Sim URDF importer는 ROS package:// resolver가 없으므로 필수)
# ---------------------------------------------------------------------
FINAL_URDF="$DEST/mir100.urdf"
sed "s#package://mir_description/#$BUILD_DIR/#g" "$RAW_URDF" > "$FINAL_URDF"
ok "메시 경로 로컬화 완료: $FINAL_URDF"

# 메시 참조 검증
MISSING=0
while IFS= read -r meshfile; do
    if [ ! -f "$meshfile" ]; then
        warn "메시 파일 없음: $meshfile"
        MISSING=$((MISSING + 1))
    fi
done < <(grep -o 'filename="[^"]*\.stl"' "$FINAL_URDF" | sed 's/filename="//;s/"//' | sort -u)

if [ "$MISSING" -eq 0 ]; then
    ok "모든 메시(STL) 참조 확인됨"
else
    warn "$MISSING 개 메시 파일이 없습니다 — import 시 해당 링크는 비주얼 없이 들어갈 수 있음"
fi

echo
echo "=================================================================="
echo " 1단계 완료 — URDF 준비됨: $FINAL_URDF"
echo "=================================================================="
echo " 다음 단계 (Isaac Sim 필요, GUI 켜서 실행 권장):"
echo
echo "   \"\$ISAACSIM_PYTHON_EXE\" scripts/import_mir100_usd.py \\"
echo "       --urdf \"$FINAL_URDF\" \\"
echo "       --out ~/isaac_assets/Robots/MiR/mir100/mir100.usd"
echo
echo "   export FDW_MIR100_USD=~/isaac_assets/Robots/MiR/mir100/mir100.usd"
echo "=================================================================="
