# FDW-OPS Virtual Workshop

> **Isaac Sim 기반 분산지능 셀 시뮬레이터** for AGX Thor / JetPack 7

NVIDIA Isaac Sim 5.x 위에서 동작하는 FDW(Flexible Distributed Workshop) 운영 검증 플랫폼입니다.
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
| **2. USD 시각화 (Level 2.0)** | 셀 / 로봇팔 / AMR / 부품 USD 표현 + transfer 애니메이션 | Isaac Sim 5.x | ✅ 구현 완료 |
| **2.x Robot-in-the-loop** | 실 로봇 USD 모델 / IK / 충돌 / 접근성 | Isaac Sim 5.x | 🔧 다음 단계 |
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
└── logs/              # KPI/시뮬레이션 로그 출력

├── visualization/    # USD 시각화 (Level 2)
│   ├── scene_builder.py        # USD prim 생성 (셀/AMR/부품)
│   └── workshop_visualizer.py  # 셀 상태 → USD 매핑

scripts/run_poc1.py            # PoC-1 메인 실행 스크립트 (discrete/isaac)
scripts/run_poc1_level2.py     # PoC-1 Level 2 시각화 데모
tests/test_poc1_smoke.py       # discrete 모드 회귀 테스트
tests/test_visualization_smoke.py  # 시각화 모듈 회귀 테스트
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

### 2. Isaac Sim 모드 — 헤드리스 (AGX Thor + isaac_sim conda 환경)

```bash
conda activate isaac_sim
ACCEPT_EULA=Y PRIVACY_CONSENT=Y python scripts/run_poc1.py --mode isaac --headless
```

> 첫 실행 시 셰이더 컴파일 등으로 1~5분 소요될 수 있습니다.

### 3. Isaac Sim Level 2 — USD 시각화 (셀·로봇팔·AMR·부품)

```bash
# 본체 모니터에서 GUI 모드
ACCEPT_EULA=Y PRIVACY_CONSENT=Y python scripts/run_poc1_level2.py --gui --realtime

# 원격 PC에서 WebRTC 스트리밍 (브라우저)
ACCEPT_EULA=Y PRIVACY_CONSENT=Y python scripts/run_poc1_level2.py --livestream 2
```

자세한 시각화 가이드: [docs/LEVEL2_VISUALIZATION.md](docs/LEVEL2_VISUALIZATION.md)

### 4. 회귀 테스트

```bash
python tests/test_poc1_smoke.py            # discrete 모드 회귀
python tests/test_visualization_smoke.py   # 시각화 모듈 검증
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

## AGX Thor 권장 부하 한계 (초기 PoC)

| 항목 | 권장 한계 |
|---|---|
| 셀 수 | ≤ 3 |
| AMR | 1 ~ 2 |
| 로봇팔 | 1 ~ 2 |
| 카메라/Depth 센서 | 셀당 1개 |
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
