# Level 2.2 — RMPflow 충돌회피 모션 + 용접 스파크 파티클

Level 2.1에서 도입한 Lula IK 컨트롤러를 **Lula RMPflow**(Reactive Motion
Policy flow)로 확장하고, 용접 PROCESSING 단계에서 TCP(엔드 이펙터)
위치에 PointInstancer 기반 스파크 파티클을 분사한다.

---

## 1. 개요

| 항목 | Level 2.1 | **Level 2.2** |
| --- | --- | --- |
| 모션 솔버 | Lula Kinematics (IK) | **Lula RMPflow** + IK + heuristic |
| 장애물 회피 | ❌ | ✅ CollisionSphere 동적 등록 |
| 폴백 체계 | IK ↔ heuristic | **rmpflow → ik → heuristic** 3-tier |
| 시각 효과 | (없음) | **PointInstancer 스파크** (TCP 추적) |
| CLI 플래그 | `--real-robot`, `--no-ik` 등 | `+--motion-mode`, `--no-sparks`, `--spark-rate` |

---

## 2. 모션 백엔드 자동 폴백

`RMPflowController`는 lazy 초기화 시 다음 순서로 시도한다:

```
preferred_backend="auto"  →  rmpflow  →  ik  →  heuristic
preferred_backend="rmpflow" →  rmpflow  → (실패 시) ik → heuristic
preferred_backend="ik"      →  ik       → (실패 시) heuristic
preferred_backend="heuristic" → heuristic (강제)
```

### 백엔드별 특성

| 백엔드 | 의존성 | 회피 | 비고 |
| --- | --- | --- | --- |
| `_RmpFlowBackend` | `isaacsim.robot_motion.motion_generation` (RmpFlow, ArticulationMotionPolicy) + RMP config YAML + URDF + robot description | ✅ SDF + repulsion | 권장 — Lula 표준 |
| `_LulaIKBackend` | Lula KinematicsSolver | ❌ (heuristic 후처리로 약한 회피) | Level 2.1과 동일 |
| `_HeuristicBackend` | numpy 만 | ✅ ±법선 repulsion (clearance 내) | 의존 0 — 안전망 |

선택된 백엔드는 시작 시 로그에서 확인할 수 있다:

```
[VIS] RMPflow controller attached to WELD_01 (robot=franka_panda, mode=auto)
[RMP] backend selected: rmpflow (RmpFlow+ArticulationMotionPolicy ready)
```

### RMPflow 설정 탐색

`_locate_rmp_config()`는 다음 순서로 RMP YAML/URDF/robot description 을 탐색:

1. `RMPFLOW_CONFIG_DIR` / `RMPFLOW_URDF_PATH` / `RMPFLOW_ROBOT_DESCRIPTION` 환경변수
2. `isaacsim.robot_motion.motion_generation` 패키지의 표준 위치
   - `isaacsim/robot_motion/motion_generation/mg_extension/motion_policy_configs/franka/`
3. `ISAAC_NUCLEUS_DIR/Isaac/Samples/motion_policy_configs/franka/...`
4. 사용자 정의 `RMPflowConfig.rmp_config_candidates` 리스트

탐색에 실패하면 자동으로 IK 백엔드로 폴백된다.

---

## 3. 장애물(CollisionSphere) 등록

용접 모션이 시작될 때 `WorkshopVisualizer._register_cell_obstacles()`가
호출되어, 셀별로 다음 2개의 Sphere 가 RMPflow에 자동 등록된다:

```python
CollisionSphere(name=f"{cell_id}_bench",
                center=(cx, cy, bench_top_z - r*0.5),
                radius=max(sx, sy) * 0.6, static=True)

CollisionSphere(name=f"{cell_id}_part",
                center=(bx, by, bz),     # 입력 버퍼 위치
                radius=0.08, static=True)
```

* **장점**: Sphere 표현은 RMPflow SDF, IK heuristic 모두에서 동일하게 동작
* **단점**: 입체 형상이 아니라 보수적인 회피 → 토치가 부품에서 약간 떨어진다
* **튜닝**: `RMPflowConfig.obstacle_clearance_m`, `obstacle_repulsion_gain` 조정

`--no-rmpflow-obstacles` 로 비활성 가능.

---

## 4. 용접 스파크 파티클

### 4.1 구조

```
{cell_root}/Sparks                    ← UsdGeom.PointInstancer
{cell_root}/Sparks/Proto              ← Xform
{cell_root}/Sparks/Proto/Sphere       ← UsdGeom.Sphere (반경 0.012m)
```

`WeldingSparkEmitter`가 매 step 다음을 수행:

1. **적분** — 활성 입자에 `vel += g·dt`, `pos += vel·dt`, `age += dt`
2. **만료/바닥 회수** — `age >= lifetime` 또는 `pos.z <= floor_z`
3. **신규 spawn** — `spark_rate·dt` (분수 누적), TCP + jitter
4. **USD 푸시** — `protoIndices`, `positions`, `scales`, `orientations`

스파크는 `set_active(True)` 인 동안만 생성되며, 컨트롤러의
`get_phase() == "weld"` 단계에서만 활성화된다.

### 4.2 파라미터 (`SparkEmitterConfig`)

| 필드 | 기본 | 설명 |
| --- | --- | --- |
| `pool_size` | 96 | 동시에 존재 가능한 최대 스파크 수 |
| `spark_rate` | 30.0 | 초당 생성 비율 |
| `lifetime_sec` | 0.4 | 평균 수명 (±20% jitter) |
| `initial_speed_min/max` | 0.6 / 1.6 m/s | 초기 속도 균일 샘플 |
| `spread_angle_deg` | 60° | 분사 콘 각도 |
| `emit_direction` | (0, 0, 1) | 분사 중심 방향 (월드) |
| `gravity` | (0, 0, -9.81) | 중력가속도 |
| `spark_size` | 0.012 m | Sphere 반경 |
| `spark_color` | (1.0, 0.55, 0.1) | displayColor (오렌지) |

수명 동안 scale 이 1.0 → 0.3 으로 축소되어 자연스러운 fade-out 효과를 준다.

---

## 5. 사용법

### 5.1 빠른 시작

```bash
# 기본: 자동 폴백 + 스파크 활성
ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    python scripts/run_poc1_level2_2.py --gui --real-robot

# WebRTC + RMPflow 강제
ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    python scripts/run_poc1_level2_2.py --livestream 2 --real-robot \
        --motion-mode rmpflow

# Level 2.1 동작 그대로 (IK + 스파크)
python scripts/run_poc1_level2_2.py --gui --real-robot --motion-mode ik

# 의존성 없이 휴리스틱만 + 스파크 OFF
python scripts/run_poc1_level2_2.py --gui --real-robot \
    --motion-mode heuristic --no-sparks

# 스파크 분사량/수명 조정
python scripts/run_poc1_level2_2.py --gui --real-robot \
    --spark-rate 60 --spark-lifetime 0.6
```

### 5.2 CLI 플래그 요약

| 플래그 | 기본 | 설명 |
| --- | --- | --- |
| `--motion-mode {auto,rmpflow,ik,heuristic}` | `auto` | 모션 백엔드 선택 |
| `--no-sparks` | (활성) | 스파크 비활성 |
| `--spark-rate FLOAT` | 30.0 | sparks / sec |
| `--spark-lifetime FLOAT` | 0.4 | 평균 수명 (s) |
| `--no-rmpflow-obstacles` | (활성) | 장애물 등록 비활성 |

이외 Level 2.1 플래그 (`--real-robot`, `--robot`, `--safe-mode`, 등) 는
그대로 사용 가능.

---

## 6. 코드 변경 요약

### 6.1 신규 파일

| 파일 | 줄 수 | 역할 |
| --- | --- | --- |
| `fdw_sim/visualization/rmpflow_controller.py` | 688 | 3-tier 모션 컨트롤러 (rmpflow/ik/heuristic) |
| `fdw_sim/visualization/spark_emitter.py` | 320 | PointInstancer 스파크 emitter |
| `scripts/run_poc1_level2_2.py` | 240 | Level 2.2 진입점 (`--motion-mode` 등) |
| `tests/test_level2_2_smoke.py` | 311 | Isaac-less 회귀 테스트 (13 cases) |

### 6.2 수정 파일

| 파일 | 변경 |
| --- | --- |
| `fdw_sim/visualization/workshop_visualizer.py` | WorkshopVizConfig +5 필드, `_spawn_robot_arm()` 백엔드 분기, `update()`에 스파크 통합, `_start_welding_motion()` RMP/IK 분기, `_register_cell_obstacles()` 신규 |
| `fdw_sim/simulation/manager.py` | SimulationConfig +5 필드, `_build_visualization()` 전파 |

### 6.3 API 호환성

`RMPflowController`는 `IKController`와 동일한 메서드를 노출:

```python
ctrl.start_path(start, end, travel_time_sec, approach_height)  # (RMP signature)
ctrl.update(dt)
ctrl.go_home()
ctrl.is_idle() -> bool
ctrl.get_phase() -> str   # "idle" | "approach" | "weld" | "retreat"
ctrl.get_tcp_position() -> Tuple[float, float, float]   # 스파크 emitter용
ctrl.add_obstacle(CollisionSphere)
ctrl.clear_obstacles()
ctrl.diagnose() -> Dict
```

`IKController.start_path(WeldingPath)`와 시그니처가 다르므로 호출부
(`_start_welding_motion`)에서 분기한다.

---

## 7. 트러블슈팅

### 7.1 RMPflow 자산이 안 보일 때

증상:
```
[RMP] RmpFlow init failed: cannot find rmp_config.yaml
[VIS] RMPflow controller attached to WELD_01 (mode=auto) — fell back to IK
```

조치:
1. `ls $ISAAC_PATH/exts/isaacsim.robot_motion.motion_generation/.../franka/`
2. 환경변수 설정:
   ```bash
   export RMPFLOW_CONFIG_DIR=/path/to/franka
   export RMPFLOW_URDF_PATH=/path/to/franka.urdf
   export RMPFLOW_ROBOT_DESCRIPTION=/path/to/robot_descriptor.yaml
   ```
3. 또는 그냥 `--motion-mode ik` 로 우회 — Level 2.1과 동일 동작

### 7.2 스파크가 안 보일 때

체크 순서:
1. `enable_sparks=True` 확인 (`--no-sparks` 안 줬는지)
2. 로그에 `[VIS] spark emitter attached to WELD_01` 출현
3. PROCESSING 상태 진입 + 컨트롤러 phase 가 "weld" 인지 (Approach 단계는 미발광)
4. `--spark-rate 60` 으로 분사량 ↑
5. USD 뷰어에서 `{cell_root}/Sparks/Proto/Sphere` 가시성 확인

### 7.3 토치가 부품/작업대를 통과할 때

* `obstacle_clearance_m`(기본 0.08) 키우기
* `obstacle_repulsion_gain`(기본 0.4) 키우기 — heuristic 백엔드에 큰 영향
* `--motion-mode rmpflow` 로 강제 후, RMPflow SDF 가 제대로 동작하는지 확인

### 7.4 GPU device-lost 가 재현될 때

Level 2.2 자체는 RTX 추가 요구 없음. Level 2.1과 동일하게:
```bash
python scripts/run_poc1_level2_2.py --safe-mode --disable-viewport-switch ...
```

---

## 8. 테스트

```bash
# Isaac-less 회귀
python tests/test_robot_loader_smoke.py     # 12 cases
python tests/test_visualization_smoke.py    # 3 cases
python tests/test_poc1_smoke.py             # 2 cases
python tests/test_level2_2_smoke.py         # 13 cases  ← Level 2.2 신규
# total: 30 cases passing
```

검증 범위:
- 모듈 import (USD/Isaac 없이도)
- Config dataclass 기본값 + override
- `RMPflowController` IK-호환 API 시그니처
- `_rotate_z_to` 수치 헬퍼 (3 케이스: +z, +x, -z)
- WorkshopVisualizer Level 2.2 옵션 전파 + unknown mode 폴백

실제 PointInstancer 생성 / Articulation set_joint_positions / RMPflow
ArticulationMotionPolicy 호출은 AGX Thor + Isaac Sim 환경에서만 검증.

---

## 9. 다음 단계 (Level 2.3+ 후보)

- [ ] Articulated obstacle (예: 부품 자체가 움직이는 동안 회피)
- [ ] 멀티 로봇팔 협조 — 인접 WeldingCell 간 충돌
- [ ] 진짜 GPU particles (Omni.Particles / Warp) 로 업그레이드
- [ ] RMPflow config 자동 학습 (작업 셀 형상별 튜닝)
- [ ] 스파크에 emissive material + bloom 후처리
