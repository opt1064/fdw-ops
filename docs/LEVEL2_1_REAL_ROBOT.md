# Level 2.1 — 실 로봇팔(Franka Panda / UR10) USD + IK 통합

> Level 2의 placeholder 박스 로봇팔을 NVIDIA Isaac Sim의 **실제 7-DOF Franka Panda**(또는 UR10) USD 모델로 교체하고, 용접 셀이 PROCESSING 상태가 되면 **IK 역기구학**으로 토치가 부품 위 용접 경로를 추적하도록 한 단계입니다.

---

## 1. 개요

| 항목 | Level 2 | **Level 2.1** |
|---|---|---|
| 로봇팔 | 박스 + 실린더 placeholder | **Franka Panda(7-DOF) / UR10(6-DOF) USD** |
| Articulation | 없음 | **isaacsim.core.prims.SingleArticulation** |
| 동작 | 정적 | **IK로 부품 용접 경로 추적** |
| 폴백 | — | placeholder 자동 fallback (USD 로드 실패 시) |
| IK 솔버 | — | **Lula** 우선, 실패 시 휴리스틱 fallback |

---

## 2. 새로 추가된 모듈

### `fdw_sim/visualization/robot_loader.py`
- `RobotSpec` 데이터클래스 — USD 경로 / EE 프레임 / 홈 자세 / base offset
- `ROBOT_CATALOG`: `franka_panda`, `ur10`, `franka_alt`(5.x 신규 경로 fallback)
- `get_isaac_assets_root()` — 우선순위:
  1. 환경변수 `ISAAC_NUCLEUS_DIR`
  2. `isaacsim.storage.native.get_assets_root_path()` (5.x)
  3. `omni.isaac.nucleus.get_assets_root_path()` (4.x fallback)
  4. 기본값 `omniverse://localhost/NVIDIA/Assets/Isaac`
- `RobotLoader.load_robot(robot_name, prim_path, position, orientation_deg_z)`
  - USD `AddReference()` 로 로봇 prim 추가
  - `SingleArticulation` (5.x) 또는 `Articulation` (4.x) 래핑 후 반환

### `fdw_sim/visualization/ik_controller.py`
- `WeldingPath` — `approach → weld → retreat → done` 4단계 상태머신
  - approach (0.5s) — `approach_height` 만큼 위에서 시작점으로 강하
  - weld (`travel_time_sec`) — start → end 직선 보간
  - retreat (0.5s) — 끝점에서 다시 위로 상승
- `IKConfig` — `use_lula`, `waypoint_blend_time`, `home_blend_time`
- `IKController(articulation, spec, config)`
  - `go_home()` — 스무드스텝 보간으로 홈 자세 복귀
  - `start_path(WeldingPath)` — 접근 자세 준비
  - `update(dt)` — Lula IK 또는 휴리스틱(어깨/팔꿈치 yaw 추정) fallback
  - `_apply_joints(q)` — 5.x `set_joint_positions` / 4.x `apply_action(ArticulationAction)`

---

## 3. 수정된 모듈

### `fdw_sim/visualization/scene_builder.py`
```python
def add_real_robot_arm(cell_id, robot_name="franka_panda", offset=(0,-0.3,0.85))
```
- 셀의 월드 좌표에 offset을 더해 로봇 베이스를 위치
- 기존 `add_robot_arm_placeholder()` 유지 (fallback용)

### `fdw_sim/visualization/workshop_visualizer.py`
- `WorkshopVizConfig` 확장:
  - `use_real_robot: bool = False`
  - `robot_name: str = "franka_panda"`
  - `enable_ik: bool = True`
  - `weld_path_offset_y: float = 0.25`
  - `weld_path_height: float = 0.05`
- `_spawn_robot_arm(cell_id, color)` — 옵션에 따라 실 로봇팔 또는 placeholder
- `_start_welding_motion(cell_id)` — CELL_STATUS = PROCESSING 감지 시 호출
- `update()` 루프에 IK 컨트롤러 step 추가
- `_on_cell_status()` — PROCESSING → 용접 경로 시작 / IDLE → 홈 복귀

### `fdw_sim/simulation/manager.py`
- `SimulationConfig`에 Level 2.1 5개 필드 추가
- `_build_visualization()`에서 `WorkshopVizConfig`로 전파

---

## 4. 실행 방법

### 사전 조건
```bash
# AGX Thor 본체에서 (Isaac Sim 5.1.0 + Nucleus 접근 가능 환경)
conda activate isaac_sim
cd ~/fdw-sim
git pull origin genspark_ai_developer
```

### 4.1 기본 실행 — Franka Panda + IK
```bash
ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    python scripts/run_poc1_level2_1.py \
        --gui \
        --real-robot \
        --robot franka_panda
```

### 4.2 원격 WebRTC 스트리밍 + UR10
```bash
ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    python scripts/run_poc1_level2_1.py \
        --livestream 2 \
        --real-robot \
        --robot ur10
# 브라우저: http://<AGX_THOR_IP>:8211/streaming/webrtc-client
```

### 4.3 IK 비활성 (USD 로드만 검증)
```bash
ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    python scripts/run_poc1_level2_1.py --gui --real-robot --no-ik
```

### 4.4 Level 2 호환 모드 (placeholder)
```bash
# --real-robot 미지정 시 Level 2와 동일
ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    python scripts/run_poc1_level2_1.py --gui
```

---

## 5. CLI 옵션

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--gui` / `--headless` / `--livestream {1,2}` | `--gui` | 디스플레이 모드 |
| `--no-viz` | off | USD 시각화 비활성 |
| `--realtime` | off | 시뮬레이션을 실시간 속도로 |
| `--anim-duration N` | 2.5 | 셀 간 부품 이동 애니메이션 길이(s) |
| **`--real-robot`** | off | 실 로봇팔 USD 로드 |
| **`--robot {franka_panda,ur10,franka_alt}`** | `franka_panda` | 로봇 모델 |
| **`--no-ik`** | off | IK 비활성 (홈 자세 고정) |
| **`--weld-offset N`** | 0.25 | 용접 경로 ±길이(m) |
| **`--weld-height N`** | 0.05 | 부품 표면 위 용접 높이(m) |

---

## 6. NGC / Nucleus 자산 요구사항

Isaac Sim 5.1.0에서 Franka/UR10 USD는 다음 경로에 있습니다.

```
omniverse://localhost/NVIDIA/Assets/Isaac/Robots/Franka/franka.usd
omniverse://localhost/NVIDIA/Assets/Isaac/Robots/UR10/ur10.usd
```

또는 로컬 캐시 경로:
```
~/Documents/Kit/shared/exts/.../Isaac/Robots/...
```

문제가 있을 경우 환경변수로 자산 루트를 직접 지정할 수 있습니다.

```bash
export ISAAC_NUCLEUS_DIR=/path/to/your/isaac_assets
```

> **참고**: Franka USD 경로가 5.x에서 변경된 경우 `--robot franka_alt`로 fallback 경로(`Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd`)를 시도해 보세요.

---

## 7. IK 동작 흐름

```
CELL_STATUS = PROCESSING  ─┐
                           │
                ┌──────────▼──────────┐
                │ _start_welding_motion │
                │  - input buffer pos 기준 │
                │  - start = (x, y-offset, z+height) │
                │  - end   = (x, y+offset, z+height) │
                └──────────┬──────────┘
                           │
                  ┌────────▼────────┐
                  │ ik.start_path()  │
                  └────────┬────────┘
                           │
       매 step ─→  ik.update(dt) ─→ WeldingPath.update(dt)
                           │              │
                           │              ├─ approach (0.5s)
                           │              ├─ weld     (travel_time_sec)
                           │              └─ retreat  (0.5s)
                           │
                  ┌────────▼────────┐
                  │ Lula IK 가능?    │
                  └────────┬────────┘
                       Yes │      │ No
                           ▼      ▼
                  joint q     yaw-based heuristic
                           │      │
                           └──┬───┘
                              ▼
                  _apply_joints(q)
                              │
CELL_STATUS = IDLE ─────────→ ik.go_home()
```

---

## 8. Lula IK fallback 동작

Lula 솔버가 사용 불가능한 경우 (예: robot description YAML 누락, NGC 미접속, 5.x 모듈 누락) 다음 휴리스틱이 적용됩니다.

- **Franka (7-DOF)**: 베이스 → 타겟 벡터의 yaw로 `joint1` 설정,
  거리/높이로 `joint2` (어깨), `joint4` (팔꿈치) 추정,
  나머지는 home 자세 유지
- **UR10 (6-DOF)**: 동일한 yaw 추정 + shoulder/elbow 보정

정확도는 떨어지지만 토치가 부품 위쪽 영역을 향하는 시각적 효과는 충분히 표현됩니다.

---

## 9. 검증된 테스트

| 테스트 | 위치 | 결과 |
|---|---|---|
| Level 2.1 smoke (12 cases) | `tests/test_robot_loader_smoke.py` | ✅ |
| PoC-1 discrete 회귀 (2 cases) | `tests/test_poc1_smoke.py` | ✅ |
| Visualization 회귀 (3 cases) | `tests/test_visualization_smoke.py` | ✅ |

```bash
cd ~/fdw-sim
python tests/test_robot_loader_smoke.py
python tests/test_poc1_smoke.py
python tests/test_visualization_smoke.py
```

---

## 10. 다음 단계 (Level 2.2 후보)

- [ ] **RMPflow 적용** — 충돌 회피 + 다이나믹 응답
- [ ] **용접 스파크/궤적 파티클** — 토치 끝점에 emission
- [ ] **다양한 부품 형상** — tubular frame USD 자산 교체
- [ ] **InspectionCell 카메라** — 실 비전 시뮬레이션 + AOV 캡처
- [ ] **AMR 실 USD** — Carter v1 / Jetbot 교체
