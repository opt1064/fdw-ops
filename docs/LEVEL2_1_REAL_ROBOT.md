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

## 10. AGX Thor Blackwell — GPU device-lost 트러블슈팅

부팅은 깨끗하지만 첫 GPU 리소스 업로드 단계에서 다음과 같이 죽는 경우:

```
[Error] [carb.graphics-vulkan.plugin] VkResult: ERROR_DEVICE_LOST
[Error] [omni.kit.renderer.plugin] uploadData: failed to end and submit ResourceLoader
[Error] [gpu.foundation.plugin] A GPU crash occurred. Exiting the application...
'lastCommand' = 'SetLightingMenuModeCommand(lighting_mode=stage,...)'
Segmentation fault (core dumped)
```

이는 RTX 셰이더 컴파일 spam과는 별개 문제로, **NVIDIA Aftermath GPU crash
dumper / `SetLightingMenuModeCommand` / `omni.kit.viewport.utility` 호출**
중 하나가 vk submit 시점에 device-lost를 유발하는 것으로 추정됩니다.

### 10.1 즉시 대응 — `--safe-mode`

가장 보수적인 설정을 한 번에 적용합니다:

```bash
ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    python scripts/run_poc1_level2_1.py --gui --real-robot --robot franka_panda \
    --safe-mode
```

`--safe-mode`는 다음을 일괄 적용합니다:
- `skip_auto_camera = True` — `WorkshopVisualizer._auto_frame_camera()` 건너뜀
- `disable_aftermath = True` — `NVDA_AFTERMATH=0` + carb `/rtx/aftermath/*` OFF
- `render_mode = RaytracedLighting` (NRD 불필요)
- `force_lighting_mode = camera` — Kit 기본 stage lighting setup 회피
- breakpad crash reporter 비활성

### 10.2 세부 토글

| 플래그 | 끄는 대상 |
|---|---|
| `--skip-auto-camera` | 카메라 자동 framing (SceneCamera prim 생성) |
| `--disable-aftermath` | Aftermath GPU dumper (기본 ON) |
| `--enable-aftermath` | Aftermath 강제 활성 (디버깅용) |
| `--force-lighting-mode camera` | stage lighting 자동 전환 회피 |
| `--disable-viewport-switch` | 카메라 prim은 만들되 viewport active camera 전환은 하지 않음 |
| `--headless` | viewport 자체를 안 띄움 (가장 안전) |
| `--no-viz` | USD 시각화 빌드 전체 건너뜀 |

### 10.3 자동 이등분 진단

원인이 어느 단계인지 좁히기 위한 6-step 스크립트:

```bash
ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    bash scripts/diagnose_gpu_crash.sh
```

순차적으로 6개 시나리오를 실행하며 logs/diag-* 에 결과를 저장합니다.

| Step | 시나리오 | 첫 FAIL의 의미 |
|---|---|---|
| 1 | `--headless --no-viz --safe-mode` | Isaac Sim 자체 / 드라이버 문제 |
| 2 | `--headless --safe-mode` | USD 시각화 빌드 자체 |
| 3 | `--headless --safe-mode --real-robot` | Franka USD 로드 |
| 4 | `--gui --safe-mode --real-robot` | GUI / Kit lighting setup |
| 5 | `--gui --real-robot --force-lighting-mode camera`<br>(skip-auto-camera OFF) | auto-camera/viewport switch trigger |
| 6 | `--gui --real-robot` (baseline) | 원본 크래시 재현 |

### 10.4 권장 fallback 시나리오

- **GUI가 꼭 필요 없다면**: `--headless --safe-mode --real-robot`
  → Franka USD 로드와 IK 모션 로직은 검증 가능, 시각적 확인은 KPI 로그로
- **GUI가 필요하면**: `--gui --safe-mode` 부터 시작 → 점진적으로 토글 해제
- **WebRTC 스트리밍**: `--livestream 2 --safe-mode` — 로컬 viewport를 띄우지
  않고 클라이언트에서만 렌더링되므로 device-lost 확률이 더 낮음

---

## 11. AGX Thor — Franka USD 로컬 자산 설치 (필수)

진단 결과 시스템 어디에도 `franka.usd`가 없고 `get_assets_root_path()`이
NVIDIA S3 URL을 반환하지만, AGX Thor에서는 Omni Client HTTPS resolver가
이를 fetch하지 못해 **빈 prim**(자식 0개)만 생기는 문제가 발견되었습니다.

### 11.1 증상

```
[VIS] >>> attempting to load real robot franka_panda for cell WELDING_CELL_01
[VIS] *** robot prim /World/FDW/Cells/WELDING_CELL_01/RobotArm has 0 children
     — USD likely failed to resolve (...
     path: https://omniverse-content-production.s3-us-west-2.amazonaws.com
           /Assets/Isaac/5.1/Isaac/Robots/Franka/franka.usd)
```

### 11.2 해결 — 로컬에 Franka USD 받기

```bash
# 1) Franka USD + sub-references 자동 다운로드
bash scripts/download_franka_usd.sh

# 2) 시뮬레이터가 로컬 USD를 사용하도록 환경변수 설정
export ISAAC_NUCLEUS_DIR_LOCAL=~/isaac_assets

# 영구 설정
echo 'export ISAAC_NUCLEUS_DIR_LOCAL="$HOME/isaac_assets"' >> ~/.bashrc

# 3) 실행
ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    python scripts/run_poc1_level2_1.py --gui --real-robot --robot franka_panda \
    --safe-mode
```

성공 시 로그:
```
[VIS]     USD path: /home/isweon/isaac_assets/Isaac/Robots/Franka/franka.usd
[VIS]     source   : LOCAL disk
[VIS] <<< robot franka_panda loaded successfully (children=N>0, articulation=True)
```

### 11.3 자산 경로 탐색 우선순위

`resolve_robot_usd_path()`는 다음 순서로 USD 경로를 결정:

1. **`FDW_<ROBOT>_USD` env** — 단일 파일 직접 지정 (예: `FDW_FRANKA_PANDA_USD=/path/franka.usd`)
2. **`ISAAC_NUCLEUS_DIR_LOCAL` env** — 루트 디렉토리, 그 아래 `Isaac/Robots/<...>/<...>.usd` 탐색
3. **표준 후보 디렉토리** — `LOCAL_ASSET_CANDIDATES`:
   - `~/isaac_assets`, `~/isaac_assets/Isaac/5.1`
   - `~/Documents/Omniverse/Library/Isaac-Sim Full/Assets/Isaac/5.1`
   - `~/.local/share/ov/data/assets/Isaac/5.1`
   - `/opt/nvidia/isaac-sim-assets/5.1`, `/opt/ov/assets/Isaac/5.1`
4. **`ISAAC_NUCLEUS_DIR` env** — 원격 Nucleus 경로
5. **`get_assets_root_path()`** — Isaac Sim 5.x 표준 API (S3 fallback 위험)
6. **하드코드** — `omniverse://localhost/NVIDIA/Assets/Isaac`

`fdw_sim.visualization.robot_loader.diagnose_robot_assets()` 함수로 현재
환경에서 어떻게 경로가 해석되는지 진단 정보를 얻을 수 있습니다.

### 11.4 로드 검증

`RobotLoader.load_robot()`은 `AddReference()` 후 다음을 수행:

- `omni.kit.app.update()` 를 최대 3회 호출 (비동기 payload resolve trigger)
- `prim.GetChildren()` 길이 검사
- 0이면 즉시 `RuntimeError` 발생 → `_spawn_robot_arm()`이 placeholder로 자동 fallback

이전에는 빈 prim이 조용히 남아 있어 IK 컨트롤러가 동작은 하지만 화면에는
아무것도 안 보이는 상태였는데, 이제는 placeholder가 명확히 대신 표시됩니다.

---

## 12. 다음 단계 (Level 2.2 후보)

- [ ] **RMPflow 적용** — 충돌 회피 + 다이나믹 응답
- [ ] **용접 스파크/궤적 파티클** — 토치 끝점에 emission
- [ ] **다양한 부품 형상** — tubular frame USD 자산 교체
- [ ] **InspectionCell 카메라** — 실 비전 시뮬레이션 + AOV 캡처
- [ ] **AMR 실 USD** — Carter v1 / Jetbot 교체
