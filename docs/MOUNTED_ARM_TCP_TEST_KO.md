# 그리퍼 장착 상태의 소폭 팔 동작·TCP 후보 좌표 시험

## 목적과 검증 범위

장착 상태 개폐는 사용자 로그 `gripper_motion_20261015_163645_16dcb097.json`에서
`PASSED_MOUNTED_UNLOADED_GRIPPER_CYCLE`, 4개 구간 통과로 확인됐다.
그 실행의 팔 최대 편차는 약 0.000425°였고, 앞서 사용자가 장착 방향도 화면으로 확인했다.
이 결과를 팔 동작 중의 장착 안정성이나 TCP 교정 결과로 확대하지 않는다.

새 시험은 같은 임시 직접 장착과 무부하·중력 0 조건에서 팔을 소폭 움직인다.
기존 `fanuc_tooling.yaml`의 active_tool·교정값은 채우지 않으며, 실제 어댑터·툴체인저와
접촉 파지·하중·충돌 회피는 검증하지 않는다. self-collision 설정도 기존과 같다.

## 실행

기존 Isaac Sim 창을 종료한 뒤 실행한다.

```bash
cd ~/isaac_workspace/projects/fdw-ops-review
git pull --ff-only origin fix/fdw-quality-disposition-base-pose
git log -1 --oneline
"$ISAACSIM_PYTHON_EXE" scripts/run_gripper_motion_test.py \
  --mounted-on-fanuc --mounted-arm-motion --drive-damping-scale 10 --keep-alive
```

기본 진폭은 각 관절 ±1°다. `--arm-amplitude-deg`로 0° 초과 2° 이하의 시험 진폭만
지정할 수 있다. 이 옵션은 원본 관절 limits/gain/물성을 변경하지 않는다.

## 실행 순서

1. 기존 mounted 개폐 시험의 준비 자세 안정화와 닫힘·복귀 2회(4개 구간)를 수행한다.
2. 4개 구간이 모두 통과한 경우에만 팔 동작을 준비한다. 실패하면 arm 시험은 건너뛴다.
3. 장착된 스테이지·native tensor view를 유지하며 FK 구조와 초기 피드백을 대조한다.
4. J1, J5, J6 순서로 각각 `+1° → 시작 자세 → -1° → 시작 자세`를 수행한다(12개 구간).
   각 구간은 4초 quintic ramp를 사용해 시작·종료 시 목표 속도·가속도를 0으로 둔다.
5. 마지막 관절 자세와 TCP 후보가 시작 위치·방향으로 돌아왔는지 확인한 뒤 pause한다.

그리퍼는 개폐 시험 종료 시의 실측 개방 자세를 피드백 제어로 유지한다.
팔의 여섯 target만 명시된 관절 이름으로 쓰고, 그리퍼·수동 관절 target은 보존한다.
관절 위치를 순간 설정하거나, 움직이는 동안 그리퍼를 xform으로 따라붙이지 않는다.
구동 중 play/reset/rebind를 추가하지 않고 이미 검증한 명시적 PhysX simulate/fetch 경로를 사용한다.

## 좌표 비교

USD `J1`~`J6`의 `body0/body1`, `axis`, `localPos0/1`, `localRot0/1`을 읽어
serial-chain FK를 구성한다. 로컬 joint 위치는 stage 단위에서 m로 변환하고,
native 관절 실측값은 rad로 사용한다. 초기 root body 월드 자세와 고정된
terminal→flange, 장착 회전, base→grasp_frame을 스냅샷으로 보관한다.

예상 TCP 후보는 **실측 관절값과 이 USD 관절 모델**로 계산한다. 구동 중 링크의
USD 월드 위치를 다시 읽어 예측에 넣지 않는다. 관측 TCP 후보는 매 물리 step 뒤
USD에 반영된 `grasp_frame`의 월드 변환이다. 두 위치·방향을 매 step 비교한다.
이것은 같은 USD 모델을 기반으로 하는 일관성 검사이며 독립적인 Lula 검증이나
실측 TCP 캘리브레이션은 아니다.

청록색 점은 기존과 같은 실제 자산의 `grasp_frame` 후보 위치다.
J6처럼 TCP 후보가 회전축 위에 있는 경우 위치 변화가 거의 없어도 방향은 변할 수 있다.
따라서 위치와 회전을 함께 검사한다.

## 중단·통과 기준

- 초기 예측/관측 TCP가 위치 2 mm·방향 0.01 rad 범위를 벗어나면 팔 명령 전에 차단한다.
- 원본에서 읽은 유한 관절 한계가 없거나 계획 목표가 그 범위를 벗어나면 차단한다.
- 선택 관절은 시작 대비 진폭+0.5°, 나머지 관절은 0.5° 초과 이탈 시 중단한다.
- 명령 대비 관절 실측 오차 2° 초과, 비유한 피드백, viewer 종료 시 중단한다.
- flange/base 장착 오차 위치 2 mm·방향 0.01 rad 초과 시 중단한다.
- 그리퍼 primary 자세가 시작 대비 0.5° 넘게 변하거나, base 기준 손끝 위치가 2 mm 넘게
  변하면 중단한다. 정상 팔 동작으로 변하는 그리퍼 월드 위치에는 정지 조건을 적용하지 않는다.
- 예상/관측 TCP 후보의 위치 오차 2 mm·방향 오차 0.01 rad 초과 시 중단한다.
- 각 구간은 목표 관절 오차 ±0.1°·속도 ±0.5°/s 이내를 연속 0.5초 만족해야 한다.
  닫힘/복귀가 아닌 팔 이동 구간에서는 요청 진폭의 80% 이상 실측 이동도 요구한다.
- 각 구간 제한은 8초다. 실패 시 가능한 현재 실측 위치 hold 후 pause하며 자동 복귀하지 않는다.
- 최종 관절은 시작 대비 ±0.1°, TCP 후보는 시작 대비 2 mm·0.01 rad 이내로 돌아와야 한다.

전체 성공 토큰은 `PASSED_MOUNTED_ARM_TCP_TEST`다. 상위 report의
`gripper_cycle_status`는 `PASSED_MOUNTED_UNLOADED_GRIPPER_CYCLE`,
`gripper_passed_legs: 4`, `arm_motion_passed_legs: 12`, `returned_to_start: true`를 확인한다.
`arm_motion` 하위의 `passed_legs: 12`, `errors: []`와 최대 장착/TCP 오차도 확인한다.
기존 상위 `passed_legs`는 개폐 시험의 4개 구간을 가리킨다.

## 결과 기록과 현장 확인

기존 `fdw_sim/logs/gripper_motion_<시간>_<식별자>.json`에 결과를 저장한다.
`arm_motion.legs`에는 각 구간의 관절값·예상/관측 TCP 행렬(m)·오차를,
`arm_motion.samples`에는 약 0.1초 간격 명령·실측 관절값과 TCP를 기록한다.
콘솔에는 samples를 제외한 판정·최종 피드백을 출력한다.
최종 결과가 나올 때까지 창을 유지하고, 시작부터 종료까지 영상과 로그를 확보한다.

CPU/OpenUSD 회귀 검증은 FK 계산·목표 보존·오류 차단을 확인한다.
실제 mounted 팔 구동과 안정성은 사용자 Spark 실행 로그·영상으로 별도 판정한다.

관절 프레임 근거:
[OpenUSD RevoluteJoint](https://openusd.org/release/api/class_usd_physics_revolute_joint.html).
