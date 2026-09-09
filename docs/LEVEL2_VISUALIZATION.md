# PoC-1 Level 2 — Isaac Sim USD 시각화 가이드

PoC-1 Discrete 모드에서 검증된 FDW-OPS 흐름을 Isaac Sim의 USD 스테이지 위에 시각적으로 표현한다.

## 무엇이 보이는가

| 요소 | 표현 | 색상 |
|------|------|------|
| 그라운드 | 30 m × 30 m 평면 | 어두운 회색 |
| Material Cell | 작업대 + Smart Rack (8슬롯) | 청회색 |
| Welding Cell | 작업대 + 로봇팔 placeholder + 토치 | 적갈색 / 주황 |
| Inspection Cell | 작업대 + 카메라 placeholder | 녹색 |
| AMR (×2) | 노란 박스 + 검은 바퀴 4개 | 황색 |
| 부품 (tubular_frame_A) | 작은 큐브 | 파랑 |
| 부품 (tubular_frame_B) | 작은 큐브 | 주황 |
| 입력 버퍼 마커 | 청색 슬랫 | 파랑 |
| 출력 버퍼 마커 | 주황 슬랫 | 주황 |

## 무엇이 움직이는가

1. **부품 spawn**: Material Cell의 Smart Rack에 입고되면 자동으로 큐브가 생성됨.
2. **부품 transfer**: 셀 간 이동 시 출력 버퍼 → 입력 버퍼로 부드럽게 보간 이동 (smoothstep).
3. **버퍼 위치 표시**: 각 셀의 입력/출력 버퍼가 색상으로 구분되어 부품의 진행 단계를 시각적으로 추적 가능.

## 실행 방법

### 1) AGX Thor 본체 앞에서 (모니터 직결)
```bash
cd ~/isaac_workspace/projects/fdw-sim
git pull origin genspark_ai_developer

conda activate isaac_sim
ACCEPT_EULA=Y PRIVACY_CONSENT=Y python scripts/run_poc1_level2.py --gui
```
- Isaac Sim 창이 모니터에 직접 표시됨
- 30~60 FPS

### 2) 원격 PC에서 WebRTC 스트리밍
```bash
ACCEPT_EULA=Y PRIVACY_CONSENT=Y python scripts/run_poc1_level2.py --livestream 2
```
브라우저로 접속:
```
http://<AGX_THOR_IP>:8211/streaming/webrtc-client
```
방화벽 개방:
```bash
sudo ufw allow 8211/tcp
sudo ufw allow 49100:49200/udp
```

### 3) 실시간 재생 모드 (눈으로 확인하기 좋음)
```bash
ACCEPT_EULA=Y PRIVACY_CONSENT=Y python scripts/run_poc1_level2.py --gui --realtime
```
- 시뮬레이션 시간 = 실제 시간으로 매핑됨 (60 FPS)
- 부품이 셀 사이를 약 2.5초에 걸쳐 이동하는 것을 볼 수 있음

### 4) 애니메이션 길이 조절
```bash
ACCEPT_EULA=Y PRIVACY_CONSENT=Y python scripts/run_poc1_level2.py --gui --realtime --anim-duration 4.0
```

### 5) 시각화 끄기 (헤드리스 검증용)
```bash
ACCEPT_EULA=Y PRIVACY_CONSENT=Y python scripts/run_poc1_level2.py --headless --no-viz
```

## CLI 옵션 요약

| 옵션 | 의미 |
|------|------|
| `--gui` (기본) | 모니터 직접 표시 |
| `--headless` | 화면 없이 실행 |
| `--livestream {1,2}` | 1=Native, 2=WebRTC |
| `--no-viz` | USD 시각화 스킵 (성능 비교용) |
| `--realtime` | sim_time = wall-clock |
| `--anim-duration N` | 셀 간 이동 애니메이션 길이(초) |

## 권장 시청 시나리오

1. `--gui --realtime --anim-duration 3.0`으로 실행
2. Isaac Sim Viewport에서 카메라를 위에서 비스듬히 내려다보는 각도로 조정
3. Stage 패널에서 `/World/FDW/Cells/`를 펼쳐 셀 계층 확인
4. 다음을 관찰:
   - 시뮬레이션 시작 직후 Material Cell의 랙에 부품 3개가 생성됨
   - 첫 부품이 Material → Welding으로 약 3초간 이동
   - Welding Cell에서 약 18초 정지(가공)
   - Welding → Inspection 이동
   - Inspection Cell에서 약 6초 정지(검사)
   - 작업 완료 후 인스펙션 출력 버퍼에서 사라짐

## 시각화 아키텍처

```
SimulationManager
  ├── start() → _start_isaac() → _build_visualization()
  │                                  ↓
  │                          WorkshopVisualizer
  │                                  ↓
  │                          SceneBuilder (USD prim 생성)
  │
  ├── step() → orchestrator.step() → bus.publish(MATERIAL_TRANSFER, cmd)
  │                                       ↓
  │                          on_transfer() callback
  │                                       ↓
  │                          viz.transfer_part(part_id, from, to)
  │                                       ↓
  │                          PartTween 등록
  │
  └── step() → visualizer.update(dt) → tween.elapsed += dt
                                            ↓
                                      scene.move_part(new_pos)
```

## 시각 요소 커스터마이즈

`fdw_sim/visualization/workshop_visualizer.py`:

```python
CELL_COLORS = {
    "material":   (0.50, 0.55, 0.65),   # 청회색
    "welding":    (0.75, 0.40, 0.35),   # 적갈색
    "inspection": (0.40, 0.70, 0.50),   # 녹색
    "forming":    (0.65, 0.55, 0.30),   # 황토
}

PART_COLORS = {
    "tubular_frame_A": (0.20, 0.60, 1.00),   # 파랑
    "tubular_frame_B": (1.00, 0.60, 0.20),   # 주황
}
```

`WorkshopVizConfig`:
```python
WorkshopVizConfig(
    cell_size=(1.8, 1.8, 0.8),         # 작업대 크기 (m)
    part_height_above_bench=0.2,        # 부품 높이
    transfer_duration_sec=2.0,          # 셀 간 이동 시간
    show_smart_rack=True,
    show_robot_arm=True,
    show_camera=True,
)
```

## 다음 단계 (Level 2.5+ 후보)

| 단계 | 내용 |
|------|------|
| Level 2.1 | Franka/UR10 USD 모델로 placeholder 교체 |
| Level 2.2 | NVIDIA Carter/Jetbot USD 모델로 AMR 교체 |
| Level 2.3 | AMR이 셀 사이를 실제로 이동하는 path-following |
| Level 2.4 | 로봇팔 IK + 가공 경로 애니메이션 |
| Level 2.5 | 카메라 센서 + USD Replicator 합성 데이터 생성 |

## 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| `omni.usd not found` | Isaac Sim 미시작 상태 | SimulationApp 인스턴스 후에만 SceneBuilder 사용 |
| 부품이 보이지 않음 | tween duration 너무 짧음 | `--anim-duration 3.0` 이상 |
| 시뮬레이션 종료 빨라 보이지 않음 | 비실시간 모드 | `--realtime` 추가 |
| 첫 실행 1~3분 멈춤 | 셰이더 컴파일 | 정상 — `~/.cache/ov` 크기 증가 모니터링 |
| WebRTC 연결 실패 | 방화벽 | 8211/tcp, 49100~49200/udp 개방 |

## 회귀 검증

다음 두 테스트가 통과하면 시각화 추가가 기존 기능에 영향이 없음:

```bash
python tests/test_poc1_smoke.py            # discrete mode 회귀
python tests/test_visualization_smoke.py   # 시각화 모듈 import 검증
```

기대 출력:
```
[OK] test_single_job_completes
[OK] test_multi_jobs_complete
All smoke tests passed!
---
[OK] workshop_visualizer module imports without USD
[OK] WorkshopVisualizer init without USD
[OK] discrete mode unaffected by visualization module
All visualization smoke tests passed!
```
