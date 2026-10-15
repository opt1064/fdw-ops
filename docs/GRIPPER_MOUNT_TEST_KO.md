# FANUC 말단 장착 상태의 무부하 그리퍼 시험

## 목적과 선행 결과

단독 Robotiq 2F-140 시험은 사용자 Spark 실행 로그
`gripper_motion_20261014_205955_90f1bece.json`에서
`PASSED_UNLOADED_GRIPPER_CYCLE`, 오류 없음, 4/4 구간, 시작 위치 복귀로 확인됐다.
이 결과는 중력 0의 부분 개폐이며 전체 stroke나 접촉 파지 검증이 아니다.

이번 단계는 FANUC가 정지 자세를 유지하는 동안 장착된 그리퍼가 같은 시험을
수행하는지 확인한다. **실제 어댑터·툴체인저 CAD와 장착 치수는 아직 제공되지 않았다.**
따라서 실제 설치의 교정값으로 간주하지 않는 임시 직접 장착을 사용한다.

## 실행

기존 Isaac Sim 창을 종료한 뒤 검토 worktree에서 실행한다.

```bash
cd ~/isaac_workspace/projects/fdw-ops-review
git pull --ff-only origin fix/fdw-quality-disposition-base-pose
git log -1 --oneline
"$ISAACSIM_PYTHON_EXE" scripts/run_gripper_motion_test.py \
  --mounted-on-fanuc --drive-damping-scale 10 --keep-alive
```

FANUC 경로는 기존 로더의 로컬/환경변수/Isaac 자산 경로를 사용한다.
필요하면 `--fanuc-usd <실제_USD_경로>`로 명시할 수 있다.
그리퍼는 기존 `--usd`와 기본 자산 해석을 유지한다.
장착 옵션 없이 실행하는 기존 단독 시험의 동작과 성공 토큰은 유지한다.

## 장착·관절 처리

- 실험 anonymous root layer에서만 변경하며 두 원본 자산을 저장하지 않는다.
- FANUC의 유일한 `flange` 위치에 Robotiq의 실제 rigid body `robotiq_base_link` 원점을 둔다.
  원본 FANUC `J6_link/flange/ee_link`는 +X가 손목 연장 방향이며,
  Robotiq base에서 grasp_frame으로 향하는 손가락 방향은 +Z다.
  그리퍼 +Z를 EE +X에, 그리퍼 +Y를 EE +Y에 맞춘다(EE 기준 +Y축 90° 회전에 해당).
  flange와 base의 회전을 그대로 일치시키면 손가락이 위를 향하므로 사용하지 않는다.
  별도 위치 오프셋이나 어댑터 길이를 추정하지 않는다.
- flange의 rigid-body 조상과 그리퍼 base를 fixed joint로 연결한다.
  연결된 두 body 사이 collision만 fixed joint 설정으로 비활성화한다.
- 기존 그리퍼의 world fixed joint는 실험에서 비활성화하고, 그리퍼 articulation root는
  제거한다. FANUC의 articulation root를 유지해 하나의 관절 트리로 읽는다.
- FANUC default prim 안에 world anchor가 없을 때는 실제 root body의 현재 월드 자세를
  유지하는 world fixed joint를 추가한다. 임의 home 관절 자세는 넣지 않는다.
- 합쳐진 관절 이름에서 `J1`~`J6`와 `finger_joint`를 확인한다.
  팔은 초기 실측 위치를 hold하며 gripper/passive target을 덮지 않는다.
- FANUC의 gain·질량·관성과 기존 root의 self-collision 설정은 유지한다.
  합쳐진 트리의 self-collision 정책은 FANUC root에서 상속하며 로그에 기록한다.
  이 시험으로 접촉·충돌 회피가 검증됐다고 주장하지 않는다.
- 단위 불일치, 모호한 프레임·관절·root, 외부 body 관계, 비유한 피드백은 차단한다.
  물리 단위를 임의 scaling으로 맞추지 않는다.
  단위 비교는 float32 상대 정밀도(약 1.1921e-7) 이내의 메타데이터 반올림만 허용한다.
  실제 Robotiq 원본의 `metersPerUnit=0.9999999776482582`와 FANUC의 `1.0`은
  약 2.2352e-8의 상대 차이이므로 이 범위에 해당한다. cm/m 등의 실제 단위 차이는 차단한다.
  `setup.mount.unit_compatibility`에 두 값·상대 차이·허용 정밀도를 기록한다.
  이 처리는 원본 메타데이터·형상·관성·gain을 변경하거나 scaling을 적용하지 않는다.

## 화면과 TCP 후보

왼쪽 손끝 BLUE, 오른쪽 ORANGE는 기존과 같다. CYAN 표시는 자산에 실제로 있는
`robotiq_base_link/grasp_frame`의 현재 월드 위치다.
이 프레임은 **TCP 후보**이며 캘리브레이션 완료나 실제 접촉 중심을 의미하지 않는다.
`setup.mount_initial_observation`과 `mount_final_observation`에 flange에서
TCP 후보로의 4x4 상대 변환 행렬(위치 m)을 기록한다.
EE/그리퍼의 월드 forward 벡터와 `forward_alignment_dot`도 기록한다.
정상 정렬이면 두 벡터 내적은 1에 가깝다. 장착 감시의 방향 오차는 identity가 아니라
실제로 적용한 `desired_flange_to_base_matrix_m`의 회전을 기준으로 측정한다.
`tcp_calibration_verified`, `mount_calibration_verified`는 계속 false다.
기존 `fanuc_tooling.yaml`의 교정값이나 `active_tool`도 채우지 않는다.

## 중단·성공 기준

4° 준비 자세 안정화 후 실제 측정된 시작 자세에서 추가 10° 닫힘·복귀를 두 번 수행한다.
기존 ±0.5° 위치·±0.5°/s 속도 안정화, 양쪽 독립 이동·복귀 기준을 유지한다.
팔 6개 관절 중 하나라도 시작 대비 0.5°를 넘게 변하면 중단한다.
flange/base 상대 위치가 2 mm, 상대 방향이 0.01 rad를 넘게 어긋나도 중단한다.
그리퍼 base의 월드 자세 이동 감시도 유지한다. 실패 시 가능한 위치 hold와 pause를 수행한다.

성공 토큰은 아래와 같다. 실제 Spark 실행 결과를 확보하기 전에는 통과로 기록하지 않는다.

```text
PASSED_MOUNTED_UNLOADED_GRIPPER_CYCLE
passed_legs: 4
returned_to_start: true
mounted_on_fanuc: true
```

최종 결과 출력까지 창을 유지한다. `setup.arm_hold`에서 초기 팔 관절값과 최대 편차를,
`setup.mount`에서 fixed joint·비활성화한 world anchor·관절 root를 확인한다.
출력 JSON과 시작부터 종료까지 화면 영상을 보관한다.
TCP 교정·하중·접촉 파지·공구 자동 교환·팔의 운송 동작은 후속 검증 대상이다.

## 근거와 검증 구분

[NVIDIA Isaac Sim 6.1 Robot Assembler](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/robot_setup/assemble_robots.html)의
fixed joint 연결·attach-side world 고정 해제·단일 articulation root 원칙을 따른다.
이번 코드는 외부 자산을 저장하지 않는 USD 실험 조합으로 이를 적용하며
Assembler의 session 저장/variant API는 호출하지 않는다.
CPU/OpenUSD 테스트는 좌표·관절 연결·원본 보존·드리프트 중단 논리를 검증한다.
실제 PhysX 구동·화면 확인은 사용자 Spark 결과로 별도 판정한다.
