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

## 트러블슈팅 — NovaCarter `CreateJoint - no bodies defined at body0 and body1`

### 증상

NovaCarter AMR 로드 시 다음과 같은 5개 노란색 Physics USD 경고 팝업이 뜬다:

```
PhysicsUSD::CreateJoint - no bodies defined at body0 and body1, joint prim:
  /World/FDW/AMRs/AMR_01_USD/joint_swing_left
  /World/FDW/AMRs/AMR_01_USD/joint_swing_right
  /World/FDW/AMRs/AMR_01_USD/joint_caster_left
  /World/FDW/AMRs/AMR_01_USD/joint_caster_right
  /World/FDW/AMRs/AMR_02_USD/joint_swing_left
```

### 원인 (두 가지 중첩)

**원인 A — variant-set 컨테이너인데 variant 선택이 없음**

`nova_carter.usd` (4 KB) 는 wrapper 가 아니라 **variant-set 컨테이너**이다.
Configuration / Physics / Sensors variant set 을 각각 명시적으로 선택하지
않으면 USD 가 자체 default 를 사용하는데, 이 조합이 일관되지 않아 joint
가 참조하는 body prim 이 stage 에 compose 되지 않는다.

→ 해결: `asset_catalog.UsdAssetSpec.variant_selection` dict 로 매핑 지정.
  단, **variant 이름 spelling 이 USD 와 정확히 일치해야** AddReference 가
  성공한다. spelling 이 어긋나면 USD verification 실패
  (`arcNum < srcInfo.size()`) 로 reference 자체가 drop 된다.

> **2026-05 Thor 실측 분기 — 현재 카탈로그 추정값의 근거**
>
> 첫 시도 (`Configuration="Base"`, `Sensors="All_Sensors"`) 가 Thor 에서
> 다음 두 가지 동시 실패를 일으켰다:
>
>   1. USD composition `arcNum < srcInfo.size()` assertion
>      — 누락 sublayer 보고 경로: `Variants/nova_carter_merged_no_internals.usd`
>      → 실제 USD 에 **"Base" 라는 variant 가 존재하지 않으며**, 옳은
>      이름은 "No_Internals" 계열로 추정된다 (`merged_no_internals.usd`
>      라는 sublayer 이름 자체가 강력한 단서).
>
>   2. 5 개의 sensor sub-USD payload 누락 경고
>      (`Hawk/hawk_v1.1_nominal.usd`, `Owl/owl.usd`,
>      `Slamtec/RPLidar_S2e.usd`, `HESAI/XT-32.usd`,
>      `Variants/nova_carter_sim_optimized.usd`)
>      → `Sensors="All_Sensors"` 가 이 sub-USD 들을 모두 요구하지만
>      `download_isaac_assets.sh` 가 아직 끌어오지 않음.
>
> 따라서 현재 `asset_catalog.py` 의 NovaCarter 엔트리는:
>
> ```python
> variant_selection={
>     "Configuration": "No_Internals",   # CreateJoint 누락 회피
>     "Physics":       "Physics_Base",   # articulation 유지
>     "Sensors":       "None",           # sub-USD payload 의존 회피
> }
> ```
>
> 으로 설정되어 있다. **이 spelling 이 USD 와 한 글자라도 다르면**
> `scene_builder._apply_variant_selection` 가 WARN 로그로 사용 가능한
> variant 이름 목록을 출력하므로, 그 출력을 참고해 한 번 더 보정하면
> 된다 (또는 아래 절의 `dump_usd_variants.py` 로 라이브 덤프).

**원인 B — sub-USD payload 누락**

variant 가 정확히 골라져도, variant payload (예:
`Variants/Sensors/nova_carter_sensors.usd`) 가 다시 sensor sub-USD
(`Sensors/LeopardImaging/Hawk/hawk_v1.1_nominal.usd` 등) 를 payload 로
참조한다. 이 sub-USD 들이 디스크에 없으면 `Could not open asset` 경고
14개가 발생하고, 결과적으로 chassis_link 하위 sensor prim 이 빈 상태가
된다.

### 해결책 (우선순위 순)

**1) 가장 빠른 우회: 다른 AMR로 스왑**

`run_poc1_level2_2.py` 의 신규 `--amr-asset` 플래그로 즉시 변경 가능:

```bash
# Jetbot (compact 2-wheel AMR — self-contained 으로 확인됨)
python scripts/run_poc1_level2_2.py --gui --real-robot --amr-asset jetbot

# Idealworks iw_hub_static (no actuation, 가장 단순)
python scripts/run_poc1_level2_2.py --gui --real-robot --amr-asset iw_hub_static

# AMR 자체를 placeholder 박스로 (warning 0)
python scripts/run_poc1_level2_2.py --gui --real-robot --no-real-amr
```

**2) variant 이름을 정확히 dump — `scripts/dump_usd_variants.py` (권장, AppLauncher 기반)**

`inspect_usd_refs.py --dump-strings` 는 USDC TOKENS 테이블에서 regex 로
토큰을 뽑기 때문에 variant 이름이 부분적으로만 보일 수 있다. 정확한
variant 이름이 필요할 때는 pxr.Usd 로 stage 를 열어 라이브 composition
의 variant 메타데이터를 직접 dump 하는 다음 도구를 사용한다.

> **왜 AppLauncher 부팅이 필요한가** — Isaac Sim 5.1 wheel install
> (`pip install isaacsim`) 환경에서는 pxr 가 PEP 420 namespace package
> 로 `$CONDA_PREFIX/lib/python3.11/site-packages/isaacsim/extscache/`
> 아래 여러 패키지(`omni.usd.libs-*`, `omni.usd.schema.physx-*`,
> `omni.anim.navigation.schema-*`, …)에 분산되어 있다. 단순한
> `sys.path.insert` + `LD_LIBRARY_PATH` 추가만으로는 `.so` 간 의존이
> 풀리지 않아 import 가 실패한다 (`~/IsaacLab/isaaclab.sh -p` 도 같은
> 이유로 실패 — 단지 같은 conda python 을 호출할 뿐 SimulationApp 을
> 부팅하지 않기 때문). 따라서 `dump_usd_variants.py` 는 Isaac Sim 의
> 정공 boot 경로(IsaacLab `AppLauncher` → `isaacsim.SimulationApp`
> fallback)를 거친 뒤에 pxr 를 import 한다. 부팅 오버헤드 ~10–15s.

```bash
# Isaac Sim conda env 활성화 후 — headless 부팅으로 GUI 비용 회피
conda activate isaac_sim
ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    python scripts/dump_usd_variants.py \
    ~/isaac_assets/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd

# JSON 출력 (CI/스크립트 chaining)
ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    python scripts/dump_usd_variants.py --json \
    ~/isaac_assets/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd \
    > nova_carter_variants.json

# (드물게) Isaac Sim deb install 등 pxr 가 이미 sys.path 에 있는 환경
USD_DUMP_NO_APP=1 \
    python scripts/dump_usd_variants.py \
    ~/isaac_assets/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd
```

출력 예 (실측 후 채워질 부분):

```
=== /home/.../nova_carter.usd
Default prim: /nova_carter

Variant sets on /nova_carter (3):

  Configuration (default = '...')
    - <name1>
    - <name2>
    ...

  Physics (default = '...')
    - <name1>
    - <name2>

  Sensors (default = '...')
    - <name1>
    - <name2>

Suggested asset_catalog mapping:
    variant_selection={
        "Configuration": "...",
        "Physics":       "...",
        "Sensors":       "...",
    },
```

dump 결과의 정확한 spelling 으로
`fdw_sim/visualization/asset_catalog.py` 의 NovaCarter 엔트리 mapping
을 교체하면 AddReference 가 USD verification 단계에서 실패하지 않는다.

옵션:
- `--recurse` / `-r` : default prim 외 모든 prim 까지 traverse
- `--json` : script-friendly JSON 출력 (CI/스크립트 chaining 용)

**3) 실제 의존성 추출 (NovaCarter Props 경로 정정)**

NVIDIA S3 의 실제 sub-USD 경로를 찾기 위해 `scripts/inspect_usd_refs.py`
사용:

```bash
# 직접 참조만 확인
python scripts/inspect_usd_refs.py \
    ~/isaac_assets/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd

# 재귀 + 누락된 sub-USD 만 Isaac/... subpath 형태로 출력
python scripts/inspect_usd_refs.py --recursive --only-missing --print-subpaths \
    ~/isaac_assets/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd

# Physics joint 의 body relationship target 도 추출
python scripts/inspect_usd_refs.py --joints \
    ~/isaac_assets/Isaac/Robots/NVIDIA/NovaCarter/Variants/Physics/nova_carter_physics.usd
```

출력의 `Missing sub-USD subpaths (Isaac/...)` 섹션을
`scripts/download_isaac_assets.sh` 의 `download_subpath` 호출에 추가하면
완전한 의존성 트리를 구성할 수 있다.

**3) 다른 AMR 자산 카탈로그 사용 — CLI 플래그 매트릭스**

| AMR | 자체 포함도 | CreateJoint 경고 | 권장 용도 |
|---|---|---|---|
| `nova_carter` | Props/* 404 (현재) | 5개 (swing/caster) | 본격 사용 전 sub-USD 검증 필요 |
| `jetbot` | ✅ self-contained | 0 | **임시 대체 권장** |
| `iw_hub_static` | ✅ no articulation | 0 | 가장 안전, 정지 모델 |
| `iw_hub` | ? | 미검증 | 추가 검증 필요 |

### 진단 도구 — `scripts/inspect_usd_refs.py`

Isaac Sim / Omni 의존성 없이 동작 (strings/grep 만 사용). USDC 바이너리
안에 텍스트로 박혀 있는 `@<path>@` 참조와 `physics:body0/body1`
relationship target 을 추출한다.

```bash
python scripts/inspect_usd_refs.py --help
```

옵션:
- `--recursive` / `-r` : sub-USD 를 따라가며 재귀 스캔
- `--max-depth N` : 재귀 깊이 제한 (기본 4)
- `--only-missing` : 로컬에 없는 sub-USD 만 출력
- `--joints` : Physics joint body 참조 prim path 추출
- `--print-subpaths` : `Isaac/...` 형태로 출력 (download script 입력용)
