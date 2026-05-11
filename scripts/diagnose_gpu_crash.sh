#!/usr/bin/env bash
# diagnose_gpu_crash.sh — AGX Thor Blackwell GPU device-lost 자동 이등분 진단
#
# VkResult: ERROR_DEVICE_LOST 의 원인을 좁히기 위해 의심 요소를 하나씩
# 끄면서 6단계 시나리오를 차례로 실행한다. 각 단계가 통과/실패하는지
# 콘솔에 표시하고 로그를 logs/diag-NN.log 로 저장한다.
#
# 사용법:
#   ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
#       bash scripts/diagnose_gpu_crash.sh
#
# 각 단계 의미:
#   1) headless + no-viz       — 가장 가벼움, Isaac Sim 자체 부팅 가능한가?
#   2) headless + viz (no robot) — 시각화 빌드는 안전한가?
#   3) headless + viz + safe-mode + real-robot — Franka USD 로드 가능한가?
#   4) GUI + safe-mode (skip-auto-camera + aftermath off) — 전부 보수적
#   5) GUI + safe-mode + skip-auto-camera 만 해제 — 카메라가 trigger인가?
#   6) GUI 일반 모드 (baseline 재현) — 원래 크래시 시나리오
#
# 어디서 깨지는지 보면 원인이 자동으로 좁혀진다.

set -u

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$ROOT_DIR"

LOG_DIR="$ROOT_DIR/logs/diag-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$LOG_DIR"

RUN="python scripts/run_poc1_level2_1.py"
TIMEOUT="${FDW_DIAG_TIMEOUT:-180}"   # 각 단계 최대 180s

# AGX Thor 안정화 공통 환경변수
export NVDA_AFTERMATH=0
export RTX_AFTERMATH_ENABLED=0
export NSIGHT_AFTERMATH_ENABLED=0
export OMNI_KIT_CRASH_REPORT_DISABLED=1
export RTX_DENOISING_ENABLED=0
export RTX_NEWDENOISER_ENABLED=0

color() { printf "\033[%sm%s\033[0m" "$1" "$2"; }
ok()    { color "32" "PASS"; }
fail()  { color "31" "FAIL"; }
warn()  { color "33" "WARN"; }

run_step() {
    local idx="$1"; shift
    local desc="$1"; shift
    local log_file="$LOG_DIR/diag-$(printf '%02d' "$idx").log"

    echo
    echo "================================================================"
    echo " STEP $idx — $desc"
    echo "    cmd: $RUN $*"
    echo "    log: $log_file"
    echo "================================================================"

    # timeout: GPU crash가 일어나면 segfault로 종료되므로 timeout 이전에 끝남
    timeout "$TIMEOUT" $RUN "$@" \
        > "$log_file" 2>&1 &
    local pid=$!
    wait "$pid"
    local rc=$?

    local crashed="no"
    if grep -q "VkResult: ERROR_DEVICE_LOST" "$log_file" 2>/dev/null; then
        crashed="device-lost"
    elif grep -q "Segmentation fault" "$log_file" 2>/dev/null; then
        crashed="segfault"
    elif grep -q "GPU crash occurred" "$log_file" 2>/dev/null; then
        crashed="gpu-crash"
    fi

    if [ "$crashed" != "no" ]; then
        echo "  $(fail)  STEP $idx CRASHED ($crashed, rc=$rc)"
        return 1
    fi
    if [ "$rc" -eq 124 ]; then
        echo "  $(warn)  STEP $idx TIMEOUT after ${TIMEOUT}s — Isaac Sim 이 정상 동작 중일 수 있음"
        return 0
    fi
    if [ "$rc" -ne 0 ]; then
        echo "  $(warn)  STEP $idx exited rc=$rc (no crash signature) — 로그 확인 필요"
        return 0
    fi
    echo "  $(ok)  STEP $idx OK"
    return 0
}

echo
echo "FDW-OPS — GPU device-lost 진단 (AGX Thor Blackwell)"
echo "logs → $LOG_DIR"
echo

# Step 1 — Isaac Sim 부팅만 (시각화/로봇 전부 OFF)
run_step 1 "headless + no-viz  (Isaac Sim 부팅 자체 검증)" \
    --headless --no-viz --safe-mode || true

# Step 2 — Isaac Sim + USD 시각화 (로봇 OFF)
run_step 2 "headless + viz, no robot  (USD 빌드만)" \
    --headless --safe-mode || true

# Step 3 — Isaac Sim + 시각화 + Franka USD (헤드리스)
run_step 3 "headless + viz + Franka USD  (real-robot 로드 검증)" \
    --headless --safe-mode --real-robot --robot franka_panda || true

# Step 4 — GUI + safe-mode 일괄 (가장 보수적 GUI 모드)
run_step 4 "GUI + safe-mode  (skip_auto_camera + aftermath off)" \
    --gui --safe-mode --real-robot --robot franka_panda || true

# Step 5 — GUI + safe-mode 해제하고 auto camera만 켬 (이 단계에서 깨지면 카메라가 trigger)
run_step 5 "GUI + skip_auto_camera 해제  (auto-camera trigger 검증)" \
    --gui --real-robot --robot franka_panda \
    --disable-aftermath --force-lighting-mode camera || true

# Step 6 — GUI 일반 (baseline 재현)
run_step 6 "GUI baseline  (원래 크래시 시나리오 재현)" \
    --gui --real-robot --robot franka_panda || true

echo
echo "================================================================"
echo " 진단 완료. logs → $LOG_DIR"
echo
echo "  결과 해석 가이드:"
echo "    Step 1 FAIL ⇒ Isaac Sim 자체 문제 (드라이버/L4T)"
echo "    Step 2 FAIL ⇒ USD 시각화 빌드가 GPU에 부담"
echo "    Step 3 FAIL ⇒ Franka USD 로드가 trigger"
echo "    Step 4 FAIL ⇒ GUI 자체 / Kit lighting setup 문제"
echo "    Step 5 FAIL but Step 4 PASS ⇒ auto camera/viewport switch 가 trigger"
echo "    Step 6 FAIL but Step 4 PASS ⇒ Aftermath/auto-camera 조합 trigger"
echo "    Step 6 PASS ⇒ 일시적 GPU 상태 이슈, 재실행 시도"
echo "================================================================"
