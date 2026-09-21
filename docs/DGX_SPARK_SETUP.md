# DGX Spark 환경 설정 가이드

이 문서는 NVIDIA DGX Spark (GB10 Grace Blackwell) + Isaac Sim 5.x 환경에서
FDW-OPS Virtual Workshop의 **Level 2 이상(USD 시각화, 실로봇팔 IK, RMPflow)**을
실행하기 위한 단계별 가이드입니다.

> **AGX Thor 대신 DGX Spark를 고려하는 이유**
> Jetson AGX Thor는 전용 RT 코어가 없어 Isaac Sim의 렌더링 파이프라인이
> 공식 지원 플랫폼으로 명시되어 있지 않습니다. 실제로 이 프로젝트를
> Thor에서 실행하는 과정에서 `VkResult: ERROR_DEVICE_LOST` GPU 크래시가
> 재현된 바 있고(`scripts/diagnose_gpu_crash.sh`, `--safe-mode` 플래그로
> 대부분 회피), 이후 세션에서는 `--safe-mode` + RDP(GNOME Remote Desktop)
> 조합으로 GUI·실로봇팔·RMPflow까지 실제로 동작을 확인했습니다 — 즉 Thor가
> 완전히 막힌 것은 아니지만, 비공식 경로라 크래시 회피용 우회 설정에
> 계속 의존해야 합니다. DGX Spark의 GB10은 4세대 RT 코어를 탑재하고 있어
> Isaac Sim 5.1이 aarch64 공식 지원 플랫폼으로 명시하고 있으므로, Level 2.2
> 이상(RMPflow, 스파크 파티클 등 렌더링 부하가 큰 작업)을 더 안정적으로
> 늘려가기에 적합합니다. Thor는 실시간 제어/HIL(Hardware-in-the-Loop)
> 역할로 계속 활용합니다.

## 검증된 환경 (공식 문서 기준 — 실측 벤치마크는 아직 없음)

* **Hardware** : NVIDIA DGX Spark (GB10 Grace Blackwell Superchip)
* **OS** : NVIDIA DGX OS 7.2.3 (Ubuntu 24.04 기반, aarch64)
* **Driver** : 580.95.05 이상 (DGX Spark 전용 권장: 580.142)
* **CUDA** : 13.0 이상 필수 (GB10 아키텍처 요구사항)
* **Python** : 3.12
* **Isaac Sim** : 5.1.0 / Isaac Lab 최신

## 1. 사전 점검

```bash
# OS / 아키텍처
uname -a                 # aarch64 확인

# GPU / CUDA
nvidia-smi
nvcc --version            # CUDA 13.0 이상인지 확인

# 필수 개발 패키지
sudo apt install python3.12-dev libgl1-mesa-dev libx11-dev libxcursor-dev \
    libxi-dev libxinerama-dev libxrandr-dev
sudo apt install -y gcc-11 g++-11
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 200
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 200
```

## 2. 프로젝트 받기

```bash
mkdir -p ~/isaac_workspace/projects
cd ~/isaac_workspace/projects
git clone https://github.com/opt1064/fdw-ops.git
cd fdw-ops
git checkout claude/awesome-tesla-vbtpvo   # 현재 활발히 개발 중인 브랜치
```

## 3. Isaac Sim 설치

**실측 결론(2026-09): pip 설치는 쓰지 마세요 — 소스 빌드가 사실상 필수입니다.**

처음엔 이 문서도 "pip 먼저 시도"를 권했지만, 실제로 DGX Spark(Python 3.12,
`pip install "isaacsim[all]==6.1.0" --extra-index-url https://pypi.nvidia.com`)로
설치해보니 패키지 자체는 다 받아지고 `SimulationApp()`도 호출까지는 되는데,
Kit 확장 의존성 해석 단계에서 항상 다음 에러로 죽었습니다:

```
Failed to resolve extension dependencies. Failure hints:
  * No versions of isaacsim.anim.robot.schema that satisfies:
    isaacsim.exp.base-6.1.0 depends on isaacsim.anim.robot.schema version *
    - Available packages for isaacsim.anim.robot.schema version *:
      (none found)
```

`isaacsim.exp.base.kit`/`isaacsim.exp.base.python.kit`(기본 experience 파일)이
둘 다 이 extension에 의존하는데, aarch64용 확장 레지스트리에 이 패키지가
아예 없습니다(재시도해도 동일 — 네트워크 문제 아님). 동일 유형의 DGX Spark
이슈가 [isaac-sim/IsaacSim#732](https://github.com/isaac-sim/IsaacSim/issues/732)에도
보고돼 있음 — pip wheel의 aarch64 배포판이 아직 불완전한 것으로 보입니다.
[NVIDIA 공식 DGX Spark 플레이북](https://github.com/NVIDIA/dgx-spark-playbooks/blob/main/nvidia/isaac/README.md)도
pip가 아니라 소스 빌드를 표준 경로로 안내합니다. 아래를 바로 따라하세요.

```bash
cd ~/isaac_workspace   # 또는 원하는 위치, 50GB 이상 여유 공간 필요
git clone --depth=1 --recursive https://github.com/isaac-sim/IsaacSim
cd IsaacSim
git lfs install
git lfs pull

# 빌드 (약 30분 소요). "BUILD (RELEASE) SUCCEEDED" 메시지가 뜨면 성공
./build.sh

export ISAACSIM_PATH="${PWD}/_build/linux-aarch64/release"
export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"
echo 'export ISAACSIM_PATH="'"$ISAACSIM_PATH"'"' >> ~/.bashrc
echo 'export ISAACSIM_PYTHON_EXE="'"$ISAACSIM_PYTHON_EXE"'"' >> ~/.bashrc

# 일부 aarch64 환경은 libgomp preload 필요 (10.3 참고)
export LD_PRELOAD="$LD_PRELOAD:/lib/aarch64-linux-gnu/libgomp.so.1"
"${ISAACSIM_PATH}/isaac-sim.sh"   # 뷰어 창이 뜨면 성공
```

소스 빌드는 자체 Python(`python.sh`)을 갖고 있어서, 이 프로젝트 스크립트도
그 아래 4번부터는 `python` 대신 **`$ISAACSIM_PYTHON_EXE`**로 실행해야 합니다
(일반 venv의 `python`에는 Isaac Sim 모듈이 없습니다).

## 4. 프로젝트 의존성 설치

```bash
"$ISAACSIM_PYTHON_EXE" -m pip install -r requirements.txt    # PyYAML, numpy
```

## 5. Discrete 모드로 동작 확인 (Isaac Sim 미사용, 어디서든 가능)

Discrete 모드는 Isaac Sim이 전혀 필요 없으므로 일반 `python`(시스템/venv 아무거나)
으로 돌려도 됩니다:

```bash
python scripts/run_poc1.py --mode discrete
```

## 6. Level 2 실행 (USD 시각화 / 실로봇팔 IK / RMPflow)

Isaac Sim을 쓰는 모드는 전부 `$ISAACSIM_PYTHON_EXE`로 실행하세요:

```bash
export LD_PRELOAD="$LD_PRELOAD:/lib/aarch64-linux-gnu/libgomp.so.1"
ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    "$ISAACSIM_PYTHON_EXE" scripts/run_poc1_level2_2.py --gui --real-robot \
    --motion-mode auto --realtime --keep-alive
```

DGX Spark는 RT 코어가 있어 Thor용 `--safe-mode`(device-lost 크래시 회피)는
기본적으로 필요하지 않습니다. 문제가 생기면 옵션으로만 켜서 시도하세요.
`--keep-alive`는 모든 job이 끝나도 뷰어 창을 계속 띄워두는 옵션입니다
(창을 닫거나 Ctrl+C 할 때까지 유지).

## 7. 원격 접속 방식 — WebRTC 라이브스트리밍은 안 됨

**중요**: Isaac Lab 공식 문서 기준으로 `Livestream`과 `Hub Workstation Cache`는
DGX Spark에서 지원되지 않습니다. 즉 이 프로젝트의 `--livestream 1`/`--livestream 2`
옵션은 DGX Spark에서 동작하지 않습니다 (실행하면 경고만 출력됨). Thor에서도
동일한 이유(aarch64용 `omni.kit.livestream.webrtc` 빌드 없음)로 라이브스트림이
막혀서, 그 세션에서는 대신 RDP(원격 데스크톱)로 우회했습니다 — DGX Spark에서도
같은 우회가 통할 가능성이 높습니다.

원격에서 화면을 보려면 다음 중 하나를 쓰세요:

**A. 로컬 모니터 직결 GUI** (가장 간단)
```bash
"$ISAACSIM_PYTHON_EXE" scripts/run_poc1_level2_2.py --gui --real-robot
```

**B. RDP / 원격 데스크톱** (Thor에서 검증된 우회 경로)
```bash
# DGX Spark에서 GNOME Remote Desktop 등이 켜져 있는지 확인
systemctl --user status gnome-remote-desktop 2>&1 | head -5
# 켜져 있다면 Windows PC의 "원격 데스크톱 연결"에서 <DGX_SPARK_IP>:3389로 접속 후
# 그 데스크톱 세션 안에서 위 A의 GUI 명령을 실행
```

**C. Docker Compose 웹 뷰어** (브라우저로 원격 접속, DGX Spark 공식 지원)
```bash
cd docker/   # (docker compose 설정 추가 필요 시 별도 작업)
docker compose up --build -- --aarch64
# 로그에 뜨는 URL (예: http://<host-ip>:8210)을 Chromium 계열 브라우저에서 접속
```
> 웹 뷰어는 인증/암호화가 기본 내장돼 있지 않습니다. 사내망 밖에 노출할 경우
> nginx 등으로 HTTPS + 인증을 반드시 추가하세요.

## 8. 부하 관리

Thor의 "셀 ≤3" 같은 보수적 제한은 RT 코어가 없는 하드웨어 기준이었습니다.
DGX Spark(RT 코어 + 128GB 통합메모리)는 이보다 여유가 있을 것으로 예상되지만,
아직 이 프로젝트 기준 실측 벤치마크는 없습니다. 다음을 권장합니다:

| 항목 | 초기 권장 (실측 전) |
|---|---|
| 셀 | ≤ 5 (Thor 대비 여유 있게 시작, 점진적으로 늘리며 측정) |
| AMR | 2 ~ 4 |
| 로봇팔 | 2 ~ 3 |
| 카메라/Depth | 셀당 1 ~ 2개 |
| 물리 주기 | 30 ~ 60 Hz |

실측 후 이 표를 프로젝트 실제 벤치마크 수치로 교체하세요.

## 9. 알려진 aarch64/DGX Spark 제약 (Isaac Lab 공식 문서 기준)

* `SkillGen` 미지원 (cuRobo 네이티브 확장이 DGX Spark용으로 검증 안 됨)
* `OpenXR` 텔레오퍼레이션 미지원 (인코딩 성능 이슈)
* JAX 기반 SKRL 학습 — CPU 전용으로 동작 (aarch64용 사전빌드 CUDA wheel 없음)
* `Isaac Sim App Selector` (GUI 실행기) 미지원 — 터미널에서 직접 스크립트 실행
* `Cosmos Transfer1` 미지원

## 10. 자주 발생하는 문제

### 10.1 `ModuleNotFoundError: No module named 'omni.kit.usd'`
→ Isaac Sim 메타패키지 누락. 소스 빌드라면 `ISAACSIM_PATH`/`ISAACSIM_PYTHON_EXE`
환경변수 설정을 확인하세요. (이 프로젝트는 이 에러를 겪을 경우 IsaacLab의
headless experience 파일을 자동으로 찾아 대신 사용하도록
`SimulationManager._resolve_isaaclab_experience_file()`에서 이미 처리합니다 —
Thor에서 이 방식으로 해결됨. 단, isaaclab이 아예 설치 안 되어 있으면 이
fallback도 못 찾으므로 결국 10.5 문제로 이어질 수 있습니다.)

### 10.5 `Failed to resolve extension dependencies` / `isaacsim.anim.robot.schema` `(none found)`
→ **pip로 설치한 `isaacsim[all]` wheel을 쓰고 있다는 뜻입니다 — 3번 항목대로
소스 빌드로 전환하세요.** 실측(2026-09, isaacsim 6.1.0, Python 3.12)으로
확인된 내용: pip wheel의 기본 experience 파일(`isaacsim.exp.base*.kit`)이
요구하는 `isaacsim.anim.robot.schema` extension이 aarch64 확장 레지스트리에
없어서, 재시도해도 항상 exit code 55로 죽습니다. 네트워크 문제가 아니라
(같은 registry sync 로그가 매번 동일하게 502/379 패키지로 끝남) 이 wheel
자체의 aarch64 배포 누락으로 보입니다 — 소스 빌드본에는 이 문제가 없습니다.

### 10.2 `pip install` 시 `externally-managed-environment` 에러
→ 시스템 Python(PEP 668 보호) 대신 conda/venv 환경 안에서 `pip` 사용

### 10.3 라이브러리 로드 오류 (`libgomp` 관련)
→ `LD_PRELOAD="$LD_PRELOAD:/lib/aarch64-linux-gnu/libgomp.so.1"` 설정 후 재시도

### 10.4 `ModuleNotFoundError: No module named 'isaacsim.robot_motion'`
→ `isaacsim.robot_motion.motion_generation`은 pip 패키지가 아니라 Kit
extension이라 실행 중인 Kit 앱에 **enable** 되어 있어야 import 가능합니다.
이 프로젝트는 `rmpflow_controller.py`의 `_ensure_motion_generation_extension()`
에서 `enable_extension()`을 명시적으로 호출해 이미 처리합니다 — 별도 조치 불필요.
(AppLauncher를 우회해 IsaacLab의 headless experience 파일을 쓰는 경로에서는
이 extension이 기본 활성화 목록에 없어서, Thor에서 처음엔 계속 heuristic
백엔드로 fallback 되는 원인이었습니다.)

## 11. 다음 단계

* `docs/AGX_THOR_SETUP.md`는 실시간 제어(HIL) 노드 또는 Thor에서 계속
  개발/검증할 때 참고 (Thor도 `--safe-mode` + RDP 조합으로 Level 2.2까지
  실제로 동작 확인됨 — DGX Spark가 없거나 이동 중일 때는 Thor로 계속
  개발해도 무방)
* Level 2.2 이상(RMPflow, 스파크 파티클)도 이 문서 기준 DGX Spark에서 진행 가능
* Thor와의 HIL 연동(ROS 2 Bridge)은 별도 문서로 추가 예정
* `docker/` Compose 웹 뷰어 설정은 아직 이 저장소에 없음 — 실제로 DGX Spark를
  확보하면 우선순위로 추가
