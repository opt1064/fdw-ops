# FANUC 공구 TCP·그리퍼 구성 확인

사용자가 시뮬레이션 팔 움직임을 확인한 뒤 진행하는 읽기 전용 단계다.
용접 토치와 그리퍼를 교체 사용한다는 요구에 맞춰 두 공구 설정을 분리했다.
현재 코드는 공구를 장착·교환하거나 그리퍼를 구동하지 않는다.

## 실행

검토 checkout에서 미커밋 변경과 브랜치를 확인한 뒤:

```bash
git status -sb
git pull --ff-only origin fix/fdw-quality-disposition-base-pose
"$ISAACSIM_PYTHON_EXE" scripts/run_poc1_level2_2.py \
  --gui --tool-diagnostics --keep-alive
```

`--tool-diagnostics`는 FANUC 로딩과 기존 로봇 진단을 활성화한다.
생산 작업, 팔·그리퍼 명령과 스파크를 실행하지 않는다. 수집 후 물리를 pause하고,
`--keep-alive`는 화면 갱신만 한다. `--show-robot-motion` 및
`--joint-feedback-test`와 함께 사용하면 시작 전에 거부한다.

별도 설정 파일은 `--tool-config /절대/경로/tooling.yaml`로 지정한다.
기본 파일은 `fdw_sim/config/fanuc_tooling.yaml`이다. 이 파일의 미확인 값은
`null`로 유지하며, 실행 중 자산을 추가하거나 컨트롤러에 TCP를 적용하지 않는다.

## 수집 내용

- `flange`와 `ee_link`의 고유 경로 및 플랜지 기준 상대 위치·방향
- 토치와 그리퍼 각각의 TCP 프레임, 장착 공구 식별 및 미확인 설정
- 로봇 subtree와 말단에 고정 joint로 연결된 외부 공구, 바깥 방향 joint로 연결된 finger body
- 관절 이름·종류·한계·실제 피드백, drive·mimic 관련 USD 속성
- collision 활성 상태, 관련 접촉 보고 속성 및 material binding 근거
- surface gripper schema/type 후보, 교환기 mounting joint 및 lock 피드백 설정

보고서는 `fdw_sim/logs/tool_diagnostics_<시간>_<식별자>.json`에 저장한다.
같은 실행의 `robot_diagnostics_<시간>.json`도 보관한다.

`NEEDS_TOOL_CONFIGURATION`은 자료 수집이 끝났지만 공구 보정·설정이 남아 있음을 뜻한다.
누락·중복 프레임이나 잘못된 설정은 `FAILED`로 기록하며 종료 시 코드 2를 반환한다.
후보 이름, 충돌 형상, schema 또는 여섯 축의 피드백만으로 그리퍼 유무나 파지 성공을 판정하지 않는다.
외부에 있지만 아직 장착되지 않은 공구는 자동으로 선택하지 않는다.

## 공구별 TCP와 교환기 설정

앞선 사용자 실행 로그에서 `flange`와 `ee_link`의 위치·방향은 같았다.
따라서 `ee_link`는 현재 말단 참조 프레임이고, 토치 끝 또는 그리퍼 파지 중심으로
보정됐다는 증거는 없다. 진단은 이 상대 변환을 새 실행에서 다시 측정한다.

기본 설정의 `welding_torch`와 `gripper`에 다음 값을 실제 모델·도면에서 확인해야 한다:

1. 공구 모델명 및 USD/URDF 경로, 장착 root prim 경로
2. 공구별 TCP 프레임 및 플랜지 기준 `translation_m` / `orientation_wxyz`
3. 공구 TCP 치수·방향을 확인한 도면 또는 측정 근거
4. 그리퍼 방식과 구동 관절·개폐 위치·접촉 피드백 근거
5. 교환기 모델, mounting joint, 잠금/해제 상태 피드백

위치는 m, 방향은 단위 quaternion `[w,x,y,z]`이다. offset을 입력하지 않았을 때
자동으로 0 또는 추정 치수를 대신 넣지 않는다. 평행 그리퍼의 개폐 값은
실제 관절 종류와 stage 단위를 확인한 뒤 지정한다. 그리퍼 종류도 아직 미정이다.

현재 설정은 준비용이며 `active_tool`은 null이다. 추후 교환 완료와 잠금 확인을
받은 뒤 해당 공구의 보정 TCP를 선택하는 로직을 구현해야 한다. 설정에 이름을
넣었다는 이유만으로 실제 공구 전환이나 보정 완료로 간주하지 않는다.

## 다음 판정

보고서로 현재 자산에 공구와 그리퍼 구성이 들어 있는지 먼저 확인한다.
공구 모델이 없다면 실제 모델·치수 확보 후 장착 설정을 만든다. 구성이 있으면
공구별 TCP 좌표 대조와 그리퍼 무부하 개폐 시험을 별도로 진행한다.
그 뒤 부품 접촉·유지력 및 공구 교환/잠금 검증을 수행한다.
CPU/OpenUSD 테스트는 변환·진단·실패 처리를 검증하며 실제 Isaac 접촉을 검증하지 않는다.

API 근거: [NVIDIA Isaac Sim 6.1 Manipulators](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/deprecated/isaacsim.robot.manipulators/docs/index.html),
[OpenUSD Collision API](https://openusd.org/dev/api/class_usd_physics_collision_a_p_i.html),
[OpenUSD Drive API](https://openusd.org/dev/api/class_usd_physics_drive_a_p_i.html).
