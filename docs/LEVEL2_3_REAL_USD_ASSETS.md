# Level 2.3 — Real USD Assets for Non-Welding Cells

## 배경

Level 2.1/2.2 까지는 welding 셀에만 실 로봇팔(Franka Panda) USD를 로드하고,
나머지 셀(material, inspection, forming)과 AMR은 단순 cube placeholder로
표시했다. 사용자가 GUI에서 cube만 보이는 문제로 인해, **모든 셀에 실 USD
모델**을 띄울 수 있도록 Level 2.3을 추가했다.

## 구성

### 1) `fdw_sim/visualization/asset_catalog.py` (신규)
Isaac Sim 5.1 일반 USD 자산 카탈로그 (로봇팔 외).

| name | category | usd_subpath |
|---|---|---|
| `nova_carter`     | amr         | `Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd` |
| `jetbot`          | amr         | `Isaac/Robots/NVIDIA/Jetbot/jetbot.usd` |
| `iw_hub`          | amr         | `Isaac/Robots/Idealworks/iwhub/iw_hub.usd` |
| `iw_hub_static`   | amr         | `Isaac/Robots/Idealworks/iwhub/iw_hub_static.usd` |
| `ur10e`           | manipulator | `Isaac/Robots/UniversalRobots/ur10e/ur10e.usd` |
| `ur5e`            | manipulator | `Isaac/Robots/UniversalRobots/ur5e/ur5e.usd` |
| `ur16e`           | manipulator | `Isaac/Robots/UniversalRobots/ur16e/ur16e.usd` |
| `klt_bin`         | prop        | `Isaac/Props/KLT_Bin/small_KLT_visual_collision.usd` |
| `cardboard_box`   | prop        | `Isaac/Props/Cardboard_Box/cardboard_box.usd` |
| `industrial_camera` | sensor    | `Isaac/Sensors/Cameras/Industrial/CN_AMR_C320/CN_AMR_C320.usd` |

헬퍼:
- `resolve_asset_usd_path(spec)` — 로컬 디스크 우선, 원격 fallback (robot_loader와 동일 전략)
- `find_local_asset(spec)` — 로컬에 없으면 `None` (raise X)
- `diagnose_catalog()` — 전체 카탈로그의 로컬/원격 해석 결과 dict

환경변수 override: `FDW_<NAME>_USD=/path/to/foo.usd`
예: `FDW_NOVA_CARTER_USD=/home/user/my_usds/nova_carter.usd`

### 2) `fdw_sim/visualization/scene_builder.py` (확장)
USD reference 로딩 메서드 5개 추가:

- `add_usd_reference(prim_path, usd_path, translation, scale, rotate_z_deg)` — 범용 USD reference 로더 (parent prim 자동 생성, 자식 수 검증)
- `add_amr_usd(amr_id, asset_name, position, rotate_z_deg)` — AMR USD 로드 (실패 시 None — placeholder fallback 신호)
- `add_cell_prop(cell_id, asset_name, offset, rotate_z_deg)` — 셀에 부착되는 USD prop
- `add_inspection_camera_real(cell_id, height, use_usd_camera)` — 카메라 하우징(cube) + 렌즈(cylinder) + 링 + 기둥 + `UsdGeom.Camera` prim (focal_length=35mm, clipping=0.05~100m)
- `add_smart_rack_real(cell_id, capacity, prop_asset)` — KLT bin USD 3-column 그리드 배치

모든 메서드는 USD 로드 실패 시 `None`을 반환해 호출측이 placeholder로
fallback 할 수 있게 한다.

### 3) `fdw_sim/visualization/workshop_visualizer.py` (수정)

`WorkshopVizConfig`에 Level 2.3 필드 7개 추가:
```python
use_real_inspection_cam: bool = True
use_real_amr: bool = True
amr_asset_name: str = "nova_carter"
use_real_smart_rack: bool = True
smart_rack_asset_name: str = "klt_bin"
use_real_forming_arm: bool = True
forming_robot_name: str = "ur10"
```

`build_scene()` 의 셀-타입별 분기를 USD 우선 + placeholder fallback 로 교체:

| 셀 타입 | 1순위 (USD) | 2순위 (Placeholder) |
|---|---|---|
| material   | `add_smart_rack_real(klt_bin)` | `add_smart_rack(cube)` |
| welding    | `_spawn_robot_arm(franka_panda)` | (기존 — Level 2.1) |
| inspection | `add_inspection_camera_real()` | `add_camera_placeholder()` |
| forming    | `_spawn_robot_arm(ur10)` | `add_robot_arm_placeholder()` |
| AMR        | `add_amr_usd(nova_carter)` | `add_amr(cube)` |

각 분기는 자산이 로컬에도 원격에도 없거나 reference 로딩이 빈 prim을
만들 경우 자동으로 placeholder로 떨어진다 → GUI는 항상 무엇인가를 보여줌.

### 4) `scripts/download_isaac_assets.sh` (신규)

기존 `download_franka_usd.sh`를 확장해 Level 2.3에 필요한 자산을 모두
로컬에 받는다.

**중요 — sub-USD 자동 추출**: Isaac Sim USD는 메인 USD가 내부적으로
sub-USD(메쉬/머티리얼/Variants)를 reference 한다. 메인만 받으면
`@small_KLT_visual.usd@ 못 찾음` 류 경고와 함께 메쉬가 안 보일 수 있다.
이 스크립트는 다운로드한 USD를 `strings|grep`으로 스캔해 `@...@` 형식
reference 를 자동으로 추출/재귀 다운로드 한다 (최대 깊이 4).

```bash
# 전부 받기 (기본) — 메인 + 명시한 sub + USD 안 자동 추출 reference
bash scripts/download_isaac_assets.sh

# 카테고리만 받기
ONLY=amr     bash scripts/download_isaac_assets.sh   # NovaCarter/Jetbot/iw_hub + sub-USDs
ONLY=robots  bash scripts/download_isaac_assets.sh   # UR10/UR10e/UR5e/UR16e + sub-USDs
ONLY=props   bash scripts/download_isaac_assets.sh   # KLT bin + small_KLT_visual.usd + ...
ONLY=franka  bash scripts/download_isaac_assets.sh   # Level 2.1 Franka + Materials.usd
```

`ISAAC_NUCLEUS_DIR_LOCAL=$HOME/isaac_assets` 설정 시 자동 사용.

### sub-USD 누락 시 증상 (실제 사용자 보고)

```
[Warning] Could not open asset @small_KLT_visual.usd@ for reference 
  introduced by @.../KLT_Bin/small_KLT_visual_collision.usd@</Root>

[Warning] Could not open asset @.../NovaCarter/Variants/Physics/nova_carter_physics.usd@ 
  for payload introduced by @.../NovaCarter/nova_carter.usd@
```

stage diagnose 출력:
```
amr:AMR_01    children=0 ref=...nova_carter.usd        ← ref는 attach, mesh 누락
cell:WELDING_CELL_01/RobotArm  children=12 ref=...franka.usd  ← OK
```

`children=0 + ref=...usd` 패턴은 **메인 USD는 attach 됐지만 sub-USD payload
가 없어 빈 prim**이라는 뜻 → 재다운로드 필요. 업데이트된 스크립트의 자동
추출 로직이 이 케이스를 해결한다.

## 실행

### GUI에서 모든 셀이 실 USD로 표시되도록 실행

```bash
# (1) 자산 다운로드 (한 번만)
bash scripts/download_isaac_assets.sh
export ISAAC_NUCLEUS_DIR_LOCAL=$HOME/isaac_assets

# (2) 실행 — Level 2.3 옵션 기본 켜져 있음
ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    python scripts/run_poc1_level2_2.py --gui --real-robot \
    --motion-mode auto \
    --disable-aftermath --disable-viewport-switch
```

성공 시 GUI에 다음이 표시됨:
- material 셀 위 KLT bin 6개 (rack 모양 그리드)
- welding 셀에 Franka Panda 아암
- inspection 셀에 산업용 카메라(하우징+렌즈+기둥+Camera prim)
- forming 셀에 UR10 아암
- AMR 위치에 Nova Carter (휠 + 센서 마운트 포함)

### 진단 — 로컬에 어떤 자산이 있는지 확인

```bash
python -c "from fdw_sim.visualization.asset_catalog import diagnose_catalog; \
    import json; print(json.dumps(diagnose_catalog(), indent=2))"
```

### Placeholder만 쓰고 싶을 때 (디버그용)

```python
cfg = WorkshopVizConfig(
    use_real_amr=False,
    use_real_inspection_cam=False,
    use_real_smart_rack=False,
    use_real_forming_arm=False,
)
```

또는 자산 부분 교체:
```python
cfg = WorkshopVizConfig(
    amr_asset_name="jetbot",         # 더 가벼운 AMR
    smart_rack_asset_name="cardboard_box",
    forming_robot_name="ur10e",      # E-series
)
```

## 검증

- `tests/test_level2_3_smoke.py` — **10/10 PASS**
  - asset_catalog import / 카탈로그 내용 (10 entries) / 경로 검증
  - resolve/find/diagnose 헬퍼 no-raise
  - SceneBuilder 5개 신규 메서드 노출 확인
  - WorkshopVizConfig 7개 Level 2.3 필드 (기본값 + override)
  - WorkshopVisualizer 통합 (4개 셀 타입 register + AMR)
- 회귀: Level 2.2 (13/13), visualization (3/3), POC1 (2/2), robot_loader (12/12) 모두 통과

## 호환성

- `WorkshopVizConfig.show_smart_rack` / `show_robot_arm` / `show_camera` 는
  그대로 보존 — Off 시 USD/placeholder 모두 스킵
- `use_real_robot` (Level 2.1) 은 welding 셀 전용으로 의미 유지
- forming 셀의 `use_real_forming_arm=True`는 내부적으로 `_spawn_robot_arm`을
  잠시 `robot_name=forming_robot_name`으로 스왑해 호출 → 끝나면 원복
