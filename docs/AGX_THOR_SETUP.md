# AGX Thor 환경 설정 가이드

이 문서는 NVIDIA Jetson AGX Thor (JetPack 7) + Isaac Sim 5.x 환경에서
FDW-OPS Virtual Workshop을 실행하기 위한 단계별 가이드입니다.

## 검증된 환경

* **Hardware** : NVIDIA Jetson AGX Thor (Blackwell GPU)
* **OS** : Ubuntu 24.04 (aarch64)
* **JetPack / L4T** : R38.2.0 (2025-08 빌드)
* **Driver** : 580.00, CUDA 13.0
* **Python** : conda env `isaac_sim` (Python 3.11)
* **Isaac Sim** : 5.1.0 (pip 설치, `isaacsim-kernel`, `isaacsim-app`,
  `isaacsim-core`, `isaacsim-extscache-physics/kit/kit-sdk`)
* **Isaac Lab** : 설치됨

## 1. 사전 점검

```bash
# JetPack 버전
cat /etc/nv_tegra_release

# GPU/CUDA
nvidia-smi

# conda 환경
conda env list                       # isaac_sim 이 보여야 함
conda activate isaac_sim
pip list | grep -iE "isaac|omni"     # isaacsim-* 패키지 다수 확인
```

## 2. 프로젝트 받기

```bash
mkdir -p ~/isaac_workspace/projects
cd ~/isaac_workspace/projects
git clone <repo-url> fdw_sim_workspace
cd fdw_sim_workspace
```

## 3. 의존성 설치 (최소)

```bash
conda activate isaac_sim
pip install -r requirements.txt    # PyYAML, numpy
```

## 4. Discrete 모드로 동작 확인 (Isaac Sim 미사용)

가장 빠르고 안전한 검증 방법입니다.

```bash
python scripts/run_poc1.py --mode discrete
```

성공 시 출력:
```
======================================================================
 FDW-OPS PoC-1 Simulation Report
======================================================================
 Total simulated time : 36.6 s
 Completed jobs       : 3
...
```

## 5. Isaac Sim 모드로 실행 (Level 2 진입)

```bash
ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    python scripts/run_poc1.py --mode isaac --headless
```

* 첫 실행 시 Kit 셰이더 컴파일로 1~5분 소요됩니다.
* GUI를 띄우려면 모니터가 직결된 상태에서 `--gui` 옵션을 사용하세요.

## 6. 환경 변수 영구 설정 (선택)

매번 `ACCEPT_EULA`를 입력하기 귀찮다면:

```bash
echo 'export ACCEPT_EULA=Y' >> ~/.bashrc
echo 'export PRIVACY_CONSENT=Y' >> ~/.bashrc
source ~/.bashrc
```

## 7. 자주 발생하는 문제

### 7.1 `ModuleNotFoundError: No module named 'omni.kit.usd'`

→ Isaac Sim 메타패키지가 누락된 경우. 다음으로 보충:
```bash
conda activate isaac_sim
pip install "isaacsim[all]==5.1.0" --extra-index-url https://pypi.nvidia.com
```

### 7.2 `IndentationError` from heredoc

→ heredoc(`<< 'EOF'`)으로 코드를 붙여넣으면 들여쓰기가 깨질 수 있습니다.
파일로 저장 후 실행하세요.

### 7.3 Docker `permission denied`

```bash
sudo usermod -aG docker $USER
newgrp docker     # 또는 재로그인
```

### 7.4 `pip install` 시 `externally-managed-environment` 에러

→ 시스템 Python(PEP 668 보호) 대신 conda 환경 안에서 `pip` 사용:
```bash
conda activate isaac_sim
pip install <package>     # sudo 없이
```

## 8. 부하 관리

AGX Thor에서는 동시에 너무 많은 셀/센서를 켜지 마세요. 권장 한계:

| 항목 | 권장 한계 |
|---|---|
| 셀 | ≤ 3 |
| AMR | 1 ~ 2 |
| 로봇팔 | 1 ~ 2 |
| 카메라/Depth | 셀당 1개 |
| LiDAR | 1개 |
| 물리 주기 | 30 ~ 60 Hz |

`fdw_sim/config/poc1.yaml`에서 `physics_hz`를 조정 가능합니다.

## 9. 다음 단계

### Level 2 (Isaac Sim 통합) 작업 항목
* `fdw_sim/assets/usd/` 에 셀별 USD 레이아웃 작성
* `cells/<x>/<x>_cell.py::configure()` 에서 USD prim 스폰
* `cells/welding/sensors.py` 등 센서 모듈 추가
* OmniGraph / Action Graph 연동

### Level 3 (Isaac Lab + RL) 작업 항목
* `agents/rl_router/` 에 GNN/MILP 라우터 추가
* `isaaclab.envs.ManagerBasedRLEnv` 로 학습환경 wrapping
* PPO/SAC 학습 스크립트 (`scripts/train_routing.py`)
* Replicator 합성데이터 (`scripts/generate_sdg.py`)
