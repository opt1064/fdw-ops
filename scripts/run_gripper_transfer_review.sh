#!/usr/bin/env bash
# Capture simulator output and its real exit status, including fast-shutdown failures.
set -uo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)" || exit 2
cd -- "$script_dir/.." || exit 2
: "${ISAACSIM_PYTHON_EXE:?Set ISAACSIM_PYTHON_EXE to the Isaac Sim Python executable}"
if pgrep -f '[r]un_gripper_motion_test.py' >/dev/null; then
    echo '[GRIPPER REVIEW] An existing gripper test is running. Close that viewer first.' >&2
    exit 2
fi
mkdir -p fdw_sim/logs || exit 2
console="fdw_sim/logs/gripper_transfer_console_$(date +%Y%m%d_%H%M%S)_$$.log"
echo "[GRIPPER REVIEW] console log: $console"
PYTHONUNBUFFERED=1 PYTHONFAULTHANDLER=1 "$ISAACSIM_PYTHON_EXE" scripts/run_gripper_motion_test.py \
    --mounted-on-fanuc --grasp-test --transfer-test --drive-damping-scale 10 --keep-alive "$@" \
    2>&1 | tee "$console"
pipeline_status=("${PIPESTATUS[@]}")
printf '\n[GRIPPER REVIEW] SIM_EXIT_CODE=%s TEE_EXIT_CODE=%s\n' \
    "${pipeline_status[0]}" "${pipeline_status[1]}" | tee -a "$console"
if (( pipeline_status[1] != 0 )); then
    exit 2
fi
exit "${pipeline_status[0]}"
