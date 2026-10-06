# FDW-OPS Virtual Workshop

> 정적 랙·설비 경로 검사 추가: [정적 관통 방지와 검증 범위](docs/STATIC_NAVIGATION_KO.md). 실제 USD 경계 기반 경로 및 도크 검사이며 Isaac GUI/접촉 검증은 별도입니다.

> **2026-10-06 동작/검증 범위 정정:** 아래의 과거 GUI 동작 기록은 실제 파지·TCP 도달·충돌 안전 검증을 뜻하지 않습니다. 현재 기본 verified 모드는 검증된 파지 어댑터가 없으면 픽업을 보류합니다. 운영 데모는 명시적 `--motion-execution schematic`을 사용하며 실제 팔 집기/용접으로 표현하지 않습니다. [AMR·공정 동기화와 검증 한계](docs/MOTION_COORDINATION_KO.md)를 먼저 확인하세요.

> **Isaac Sim 기반 분산지능 셀 시뮬레이터** — 시뮬레이션은 DGX Spark, 실시간 제어(HIL)는 AGX Thor
>
> **DGX Spark**(GB10 Grace Blackwell, RT 코어 탑재 — Isaac Sim 공식 지원 플랫폼)가
> 현재 기본 실행 호스트입니다. Level 2.1~2.4(실 로봇팔+IK, RMPflow+스파크,
> 실 USD 자산, AMR 실주행+pick-and-place)까지 전부 DGX Spark 실기에서
> `--gui` 모드로 동작 확인했습니다.
>
> **AGX Thor**는 RT 코어가 없어 Isaac Sim 렌더링이 공식 지원 대상이 아닙니다
> (`--safe-mode` + RDP 우회로 과거 개발 초기에 동작을 확인한 적은 있으나,
> 지금은 DGX Spark를 기본으로 씁니다). Thor는 실시간 제어/HIL 노드로 계속
> 활용합니다.
>
> 설치 및 실행 환경: [docs/DGX_SPARK_SETUP.md](docs/DGX_SPARK_SETUP.md)
> (Thor에서 실행하려면 [docs/AGX_THOR_SETUP.md](docs/AGX_THOR_SETUP.md) 참고)

NVIDIA Isaac Sim 5.x/6.x 위에서 동작하는 FDW(Flexible Distributed Workshop) 운영 검증 플랫폼입니다.
공정별 분산지능 셀(소재·이송 / 용접 / 소성가공 / 적층 / 정밀가공 / 검사)을
Isaac Sim의 가상 작업장 안에 배치하고, 외부 FDW-OPS Orchestrator가
ROS 2 / DDS 등 미들웨어를 통해 전체 공정을 운영하는 구조를 그대로 코드로 옮겼습니다.

## 핵심 설계 원칙

```
[FDW-OPS Orchestrator]   ← 전체 공정 운영 AI (라우팅 / 스케줄링 / 복구)
        ↓ Middleware (ROS 2 / DDS / In-Memory Bus)
[분산지능 셀들]          ← 셀 단위 로컬 판단 (Edge AI Agent)
   Material / Welding / Forming / WAAM / Machining / Inspection
        ↓
[Isaac Sim]              ← 물리·로봇·센서·공간 검증 환경
```

* Isaac Sim 안에 모든 지능을 넣지 않습니다. Isaac Sim은 **가상 물리 공간**만 담당합니다.
* 각 셀은 **독립적인 분산지능 노드**로 동작하며 표준 메시지 스키마로만 통신합니다.
* FDW-OPS는 셀 내부 구현에 의존하지 않고 **상태 기반**으로만 의사결정합니다.

## 시뮬레이션 단계 (Level 1 → 3)

| Level | 목표 | 백엔드 | 구현 상태 |
|-------|------|--------|----------|
| **1. Discrete Event** | FDW-OPS 운영체계 검증 (라우팅/버퍼/병목) | Pure Python | ✅ PoC-1 완료 |
| **2.0. USD 시각화** | 셀 / 로봇팔 / AMR / 부품 USD 표현 + transfer 애니메이션 | Isaac Sim 5.x/6.x | ✅ 구현 완료 |
| **2.1. 실 로봇팔 + IK** | FANUC CRX-10iA(용접 셀 기본) / Franka Panda / UR10 USD 모델 + IK로 토치가 부품 추적 | Isaac Sim 5.x/6.x | ✅ 구현 완료 |
| **2.2. RMPflow + 스파크** | RMPflow 충돌회피 3-tier 폴백(rmpflow→ik→heuristic) + 용접 스파크 파티클 | Isaac Sim 5.x/6.x | ✅ 구현 완료 |
| **2.3. 실 USD 자산** | AMR(MiR100/NovaCarter 등)·Smart Rack·검사 카메라를 placeholder 대신 실 USD로 | Isaac Sim 5.x/6.x | ✅ 구현 완료 |
| **2.4. AMR 이동 + 공정 게이트** | 연속 위치·도크/통로 점유·명령별 배치/완료 게이트. schematic은 논리 데모, verified 물리 픽업은 검증된 어댑터까지 보류 | Isaac Sim 5.x/6.x | CPU 회귀 통과 / GUI 재검증 필요 |
| **3. AI / Learning-in-the-loop** | RL 라우팅 / 합성데이터 / 자율복구 | Isaac Lab + SDG | 🔭 다음 단계 |

## 디렉토리 구조

```
fdw_sim/
├── messaging/         # 공통 메시지 스키마 + Pub/Sub Bus 추상화
│   ├── schemas.py     #   CellStatusMessage, DispatchCommand, ...
│   └── bus.py         #   InMemoryBus (→ ROS2Bus 교체 가능)
├── cells/
│   ├── base/          # DistributedIntelligenceCell 베이스 + StateMachine
│   ├── material/      # Material / Transport (AMR + Smart Rack)
│   ├── welding/       # Welding (Gap/Tool/Quality Edge AI Agents)
│   ├── inspection/    # Inspection (Pass/Rework/Fail 판정)
│   ├── forming/       # (PoC-2에서 구현 예정)
│   ├── waam/          # (PoC-3에서 구현 예정)
│   └── machining/     # (PoC-3에서 구현 예정)
├── orchestrator/      # FDW-OPS 룰 기반 오케스트레이터
├── simulation/        # SimulationManager (discrete | isaac 두 모드)
├── kpi/               # KPILogger (JSONL + summary.json)
├── agents/            # (확장 예정) RL/MILP 라우팅 에이전트
├── utils/             # 로깅 등 헬퍼
├── config/poc1.yaml   # PoC-1 시나리오 정의
├── assets/            # USD/URDF 자산 (Level 2 이상)
├── visualization/     # USD 시각화 (Level 2)
│   ├── scene_builder.py        # USD prim 생성 (셀/AMR/부품/로봇팔)
│   ├── workshop_visualizer.py  # 셀 상태 → USD 매핑, AMR 실주행 + pick-and-place
│   ├── robot_loader.py         # ROBOT_CATALOG (CRX-10iA/Franka/UR10) + USD 경로 해석
│   ├── asset_catalog.py        # ASSET_CATALOG (AMR/Smart Rack/카메라 등 일반 USD 자산)
│   ├── ik_controller.py        # Lula IK 기반 토치 추적 (Level 2.1)
│   ├── rmpflow_controller.py   # RMPflow 3-tier 폴백 모션 컨트롤러 (Level 2.2)
│   └── spark_emitter.py        # 용접 스파크 PointInstancer (Level 2.2)
└── logs/              # KPI/시뮬레이션 로그 출력

scripts/run_poc1.py              # PoC-1 메인 실행 스크립트 (discrete/isaac)
scripts/run_poc1_level2.py       # PoC-1 Level 2.0 시각화 데모
scripts/run_poc1_level2_1.py     # PoC-1 Level 2.1 실 로봇팔 + IK 데모
scripts/run_poc1_level2_2.py     # PoC-1 Level 2.2~2.4 RMPflow/스파크/실USD/AMR pick-and-place 데모
scripts/prepare_mir100_urdf.sh   # MiR100 URDF 준비 (1단계, GPU 불필요)
scripts/import_mir100_usd.py     # MiR100 URDF -> USD 변환 (2단계, Isaac Sim 필요)
tests/test_poc1_smoke.py             # discrete 모드 회귀 테스트
tests/test_visualization_smoke.py    # 시각화 모듈 회귀 테스트
tests/test_robot_loader_smoke.py     # Level 2.1 로봇 로더/IK smoke test
tests/test_level2_2_smoke.py         # Level 2.2 RMPflow/스파크 smoke test
tests/test_level2_3_smoke.py         # Level 2.3 실 USD 자산 smoke test
tests/test_level2_4_amr_pickplace.py # Level 2.4 AMR 실주행/pick-and-place 회귀 테스트
```

## 빠른 시작

### 1. Discrete 모드 (Isaac Sim 없이도 동작 — 어디서든 가능)

```bash
pip install -r requirements.txt
python scripts/run_poc1.py --mode discrete
```

예상 결과:
```
======================================================================
 FDW-OPS PoC-1 Simulation Report
======================================================================
 Total simulated time : 36.6 s
 Completed jobs       : 3

  Job histories:
   - JOB_00001 (part=PART_A_001) makespan= 21.8s  route=[welding, inspection]
   - JOB_00003 (part=PART_B_001) makespan= 28.1s  route=[welding, inspection]
   - JOB_00002 (part=PART_A_002) makespan= 34.6s  route=[welding, inspection]
  ...
```

### 2. Isaac Sim 모드 — 헤드리스 (DGX Spark)

```bash
ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    "$ISAACSIM_PYTHON_EXE" scripts/run_poc1.py --mode isaac --headless
```

> 첫 실행 시 셰이더 컴파일 등으로 1~5분 소요될 수 있습니다.
>
> **`$ISAACSIM_PYTHON_EXE`를 꼭 쓰세요** — 일반 `python`으로 실행하면 `isaacsim`
> 모듈을 못 찾아 `ModuleNotFoundError`가 납니다. 설치/환경변수 설정은
> [docs/DGX_SPARK_SETUP.md](docs/DGX_SPARK_SETUP.md) 참고. AGX Thor에서
> 실행하려면 [docs/AGX_THOR_SETUP.md](docs/AGX_THOR_SETUP.md) 참고(설치
> 경로/원격 접속 방식이 다릅니다 — 코드/CLI 플래그는 동일).

### 3. Isaac Sim Level 2.0 — USD 시각화 (셀·로봇팔·AMR·부품)

```bash
# DGX Spark 본체 모니터에서 GUI 모드
ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    "$ISAACSIM_PYTHON_EXE" scripts/run_poc1_level2.py --gui --realtime
```

자세한 시각화 가이드: [docs/LEVEL2_VISUALIZATION.md](docs/LEVEL2_VISUALIZATION.md)

### 4. Isaac Sim Level 2.1 — 실 로봇팔(CRX-10iA / Franka / UR10) + IK

```bash
# FANUC CRX-10iA + IK (기본, 용접 셀 실 기체)
ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    "$ISAACSIM_PYTHON_EXE" scripts/run_poc1_level2_1.py --gui --real-robot

# Franka Panda로 되돌리기
ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    "$ISAACSIM_PYTHON_EXE" scripts/run_poc1_level2_1.py --gui --real-robot --robot franka_panda
```

> ✅ CRX-10iA USD 경로는 2026-09-28 DGX Spark 실측(S3 버킷 직접 리스팅)으로
> 확인됐습니다 (`Isaac/Robots/Fanuc/crx10ia/crx10ia.usd`). `home_joint_positions`은
> UR10 자세를 그대로 가져다 쓰다가 실측에서 팔이 얇은 막대처럼 접히는 걸 확인하고
> 제거했습니다 — 지금은 강제 홈포즈 없이 USD 자체의 rest pose를 씁니다.
> `end_effector_frame`도 여전히 추정치라 IK/RMPflow 정확도에 영향이 있을 수
> 있습니다(렌더링 자체는 무관). 자세한 내용은
> [fdw_sim/visualization/robot_loader.py](fdw_sim/visualization/robot_loader.py)의
> `ROBOT_CATALOG["fanuc_crx10ia"]` 주석 참고.

자세한 가이드: [docs/LEVEL2_1_REAL_ROBOT.md](docs/LEVEL2_1_REAL_ROBOT.md)

### 5. Isaac Sim Level 2.2~2.4 — RMPflow + 스파크 + 실 USD 자산 + AMR pick-and-place

```bash
# 기본: CRX-10iA + RMPflow(자동 폴백) + 스파크 + MiR100(AMR) + KLT bin rack
ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    "$ISAACSIM_PYTHON_EXE" scripts/run_poc1_level2_2.py --gui --real-robot --keep-alive

# 모션 백엔드 강제 / 스파크 조정 / 다른 AMR로 교체하는 예시는
# scripts/run_poc1_level2_2.py 상단 docstring 참고 (--motion-mode,
# --spark-rate, --amr-asset 등)
```

> ✅ MiR100은 Isaac Sim 공식 카탈로그엔 없지만(제조사 목록에 MiR/
> MobileIndustrialRobots 항목 자체가 없음), 커뮤니티 ROS 패키지
> `DFKI-NI/mir_robot`(BSD-3-Clause)의 URDF를 로컬에서 USD로 변환해서
> 씁니다 — 2026-09-28 DGX Spark 실측으로 `AMR ... placed via USD (mir100)`
> 정상 로드까지 확인했습니다.
>
> ```bash
> bash scripts/prepare_mir100_urdf.sh              # 1단계 — GPU 불필요
> "$ISAACSIM_PYTHON_EXE" scripts/import_mir100_usd.py \
>     --urdf ~/isaac_assets/mir100_urdf_src/mir100.urdf \
>     --out ~/isaac_assets/Robots/MiR/mir100/mir100.usd  # 2단계 — Isaac Sim 필요
> export FDW_MIR100_USD=~/isaac_assets/Robots/MiR/mir100/mir100.usd
> ```
>
> 이 경로가 없으면 자동으로 placeholder 박스로 대체되며 크래시하지
> 않습니다. `--amr-asset nova_carter` / `iw_hub_static`으로 다른 실존
> 카탈로그 자산으로 바로 바꿀 수도 있습니다. 자세한 내용은
> [fdw_sim/visualization/asset_catalog.py](fdw_sim/visualization/asset_catalog.py)의
> `ASSET_CATALOG["mir100"]` 주석 참고.
>
> **Level 2.4 시나리오**: AMR이 파이프(부품)를 싣고 소재 셀 → 용접 셀까지
> 실제로 주행하고, 도착하면 용접 로봇팔이 AMR 위 파이프를 집어(approach)
> 작업대로 옮긴 뒤(carry) 내려놓고(retreat), 곧바로 파이프를 따라 일직선
> 용접 모션이 시작됩니다. 로그에서 `[VIS] pick-and-place started/finished`
> 로 진행 상황을 확인할 수 있습니다.

자세한 가이드: [docs/LEVEL2_2_RMPFLOW_SPARKS.md](docs/LEVEL2_2_RMPFLOW_SPARKS.md),
[docs/LEVEL2_3_REAL_USD_ASSETS.md](docs/LEVEL2_3_REAL_USD_ASSETS.md)

### 6. 회귀 테스트

```bash
python tests/test_poc1_smoke.py              # discrete 모드 회귀
python tests/test_visualization_smoke.py     # 시각화 모듈 검증
python tests/test_robot_loader_smoke.py      # Level 2.1 로봇 로더 + IK smoke
python tests/test_level2_2_smoke.py          # Level 2.2 RMPflow/스파크 smoke
python tests/test_level2_3_smoke.py          # Level 2.3 실 USD 자산 smoke
python tests/test_level2_4_amr_pickplace.py  # Level 2.4 AMR 실주행/pick-and-place
```

## 표준 메시지 스키마

모든 셀은 다음 4개 메시지로 통신합니다 (`fdw_sim/messaging/schemas.py`):

```python
CellStatusMessage          # 셀 → FDW-OPS  (/fdw/cell_status)
DispatchCommand            # FDW-OPS → 셀  (/fdw/dispatch_command)
MaterialTransferCommand    # FDW-OPS → AMR (/fdw/material_transfer)
KPIRecord                  # 모든 노드     (/fdw/kpi_log)
```

이 스키마는 추후 ROS 2 `.msg` 또는 Protobuf로 1:1 변환 가능하도록 설계되었습니다.

## 분산지능 셀의 6대 구성요소

각 셀은 다음 컴포넌트를 갖는 `Cell Digital Asset`입니다:

| 구성요소 | 코드 위치 | 예시 |
|---|---|---|
| Physical Asset | (Level 2) `assets/usd/` | 로봇·지그·툴·버퍼 |
| Sensor Layer | (Level 2) `cells/<x>/sensors.py` | RGB/Depth/LiDAR/Force |
| Local Controller | `cells/<x>/<x>_cell.py::step_processing` | 모션·툴체인지 |
| Edge AI Agent | `cells/<x>/<x>_cell.py` (XxxAgent) | Gap·Tool·Quality 예측 |
| State Machine | `cells/base/state_machine.py` | IDLE→READY→PROCESSING→… |
| KPI Logger | `kpi/logger.py` | cycle_time, pass_rate |

## PoC 로드맵

* **PoC-1** ✅ Material + Welding + Inspection (Discrete Event)
* **PoC-2** 🚧 + Forming 셀, Isaac Sim Level 2 통합 (로봇 모션·충돌)
* **PoC-3** 🔭 + WAAM + Machining (전체 공정 / Tubular Frame 시나리오)
* **PoC-4** 🔭 AI 라우팅 / RL 학습 / 디지털트윈 KPI 대시보드

## 주요 KPI

PoC-1에서 자동 수집되는 지표:

* `job_makespan_sec` — Job별 전체 소요시간
* `cycle_time` — 셀별 단일 작업 시간
* `quality_score` — 셀 출력물의 품질 점수 (0~1)
* `quality_pass_rate` — Inspection 합격률
* `predicted_gap_mm` — 용접 갭 예측값
* `amr_delivery_count` — AMR 배달 횟수
* `rack_stock_count` — Smart Rack 재고

추후 추가될 KPI:
* `cell_utilization` / `amr_utilization`
* `buffer_occupancy`
* `routing_success_rate`
* `fault_recovery_time`

## DGX Spark 권장 부하 한계 (초기 추정치)

PoC-1 시나리오(셀 3개 — MATERIAL/WELDING/INSPECTION, AMR 2대, 실 로봇팔 1대,
RMPflow+스파크+실 USD AMR까지 전부 켠 상태)는 DGX Spark에서 실측으로 안정
동작을 확인했습니다. 아래는 그보다 여유를 둔 초기 권장 상한치이며, 정식
벤치마크 전까지는 추정치입니다. RT 코어가 없는 AGX Thor에서는 이보다 훨씬
보수적으로 잡아야 합니다(셀 ≤3, AMR 1~2, 로봇팔 1개 수준).

| 항목 | 권장 한계 |
|---|---|
| 셀 수 | ≤ 5 |
| AMR | 2 ~ 4 |
| 로봇팔 | 2 ~ 3 |
| 카메라/Depth 센서 | 셀당 1 ~ 2개 |
| 물리 주기 | 30 ~ 60 Hz |
| FDW-OPS 판단 주기 | 1 ~ 5 Hz |
| 셀 상태 업데이트 | 5 ~ 10 Hz |

## 새 셀 추가 가이드

1. `fdw_sim/cells/<name>/<name>_cell.py` 작성, `DistributedIntelligenceCell` 상속
2. 다음 3개 메서드 구현:
   * `configure()` — Isaac Sim 자산/컨트롤러 초기화
   * `on_command(command)` — DispatchCommand 해석 → True/False
   * `step_processing(dt)` — 진행률 업데이트, 완료 시 `_complete_job()`
3. `OrchestratorConfig.process_to_cell` 에 매핑 추가
4. `scripts/run_poc1.py` 또는 새 시나리오 스크립트에서 등록

## License

Internal R&D project. Contact maintainer for collaboration.
