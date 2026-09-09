#!/bin/bash
# ============================================================================
# AGX Thor 환경 검증 스크립트
# Usage: bash scripts/check_agx_env.sh
# ============================================================================
set -e

echo "=========================================="
echo " FDW-SIM Environment Check (AGX Thor)"
echo "=========================================="

echo -e "\n[1] System Info"
uname -a
echo "---"
cat /etc/nv_tegra_release 2>/dev/null || echo "(no L4T info)"

echo -e "\n[2] GPU & CUDA"
nvidia-smi 2>/dev/null | head -15 || echo "(nvidia-smi not available)"
nvcc --version 2>/dev/null | tail -2 || ls /usr/local/ | grep cuda

echo -e "\n[3] Conda Environment"
conda --version 2>/dev/null || echo "(conda not found)"
echo "Current env: ${CONDA_DEFAULT_ENV:-none}"
which python
python --version

echo -e "\n[4] Isaac Sim / Isaac Lab Packages"
pip list 2>/dev/null | grep -iE "isaac|omni" | head -20 || echo "(no isaac packages)"

echo -e "\n[5] Project Dependencies"
python -c "import yaml; print('PyYAML:', yaml.__version__)" 2>&1
python -c "import isaaclab; print('Isaac Lab path:', isaaclab.__file__)" 2>&1

echo -e "\n[6] Disk Space"
df -h ~ | tail -2

echo -e "\n[7] Project Structure"
if [ -d "fdw_sim" ]; then
    echo "fdw_sim/ found at $(pwd)"
    ls fdw_sim/cells/
else
    echo "(run this from project root)"
fi

echo -e "\n=========================================="
echo " Check complete."
echo "=========================================="
