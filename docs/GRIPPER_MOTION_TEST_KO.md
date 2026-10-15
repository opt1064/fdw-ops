# Robotiq 2F-140 단독 소범위 개폐·양쪽 손가락 검증

사용자 환경에서 `Robotiq_2F_140_physics_edit.usd` 단독 자산 조사 결과
오류 없이 USD 가동 관절 10개, 강체 9개, 충돌 형상 11개가 확인됐다.
이번 단계는 그 자산을 별도 Isaac GUI에 열고 실제 관절 drive로 움직인다.
FANUC 장착, 부품 파지, 공구 교환은 실행하지 않는다.

## 실행과 화면

검토 checkout에서 미커밋 변경이 없는지 먼저 확인한 후:

```bash
git status -sb
git pull --ff-only origin fix/fdw-quality-disposition-base-pose
"$ISAACSIM_PYTHON_EXE" scripts/run_gripper_motion_test.py --keep-alive
```

그리퍼만 근접 화면에 표시하고 5초 카운트다운 후
`10° 부분 닫힘 → 시작 위치 복귀`를 두 번 실행한다. 각 이동 목표는 4초 ramp로
변화시키고, 목표 오차 0.5° 이내가 0.2초 지속된 뒤 1.5초 더 유지한다.
각 구간은 최대 14초이며, 전체 시험 시간은 실제 drive 정착에 따라 달라진다.

왼쪽 손끝의 파란 구와 오른쪽 손끝의 주황 구는 실제 USD 손끝 형상의 측정 중심을
따라가는 표시다. 구에는 충돌·강체 속성을 넣지 않는다. 손가락 형상과 관절은
물리로 움직이며, marker를 움직였다는 이유로 성공 판정을 내리지 않는다.
자동 카메라 전환을 피해야 하는 환경은 `--skip-auto-camera`로 실행한 뒤 수동 확대한다.
자산 경로는 `--usd /절대/경로/모델.usd`로 지정할 수 있고, 다른 자산으로 자동 대체하지 않는다.

## 명령 전 확인

다음 실제 설정을 읽어 확인한 후에만 명령을 시작한다:

- 고유한 `finger_joint`와 `right_outer_knuckle_joint`, 두 `Fingertip_01`
- 오른쪽 mimic의 실제 `referenceJoint` 관계가 왼쪽 `finger_joint`를 가리킴
- 현재 자산에서 확인한 gearing -1, offset 0, 오른쪽 직접 drive의 stiffness/damping 0
- 왼쪽 양수 position-drive stiffness 및 유한한 관절 한계
- 기존의 enabled world-to-base 고정 joint
- live articulation의 `finger_joint` 이름·index 및 유한 피드백

실제 index로 선택한 `finger_joint`의 목표만 변경한다. 기존 SingleArticulation 래퍼 대신
새 `omni.physics.tensors` view의 실제 DOF 목록과 `get_dof_positions()`를 직접 사용한다.
이 API는 DOF 일부만 쓰는 기능이 없어 현재 `get_dof_position_targets()`를 복사하고,
primary 열 하나만 바꿔 `set_dof_position_targets()`로 보낸다. 오른쪽 follower와
수동 관절의 기존 목표값은 보존하며, 값이 없거나 유한하지 않으면 중단한다.
USD 각도 한계는 degree, 런타임 명령과 피드백은 rad다.
질량·관성·한계·gain·mimic 설정을 조정하거나 `set_joint_positions`로 순간이동시키지 않는다.
피드백의 관절 한계 경계 roundoff가 1e-4 rad 이하라면 시작 목표만 한계 안으로 제한하고 기록한다.
5° 이상 닫혀 있는 예상 밖의 시작 상태나 목표 한계 초과는 이동 없이 실패한다.

카메라·조명·표시 생성과 카운트다운을 먼저 마친다. 마지막 Play 전환과 한 번의
물리 step을 처리한 뒤 새 NumPy PhysX simulation view를 만들고 articulation을 연결한다.
이때 `IPhysxSimulation.get_attached_stage()`와 현재 USD stage ID를 대조한다.
PhysX 연결이 비어 있으면 `attach_stage()`로 현재 시험 스테이지를 파싱하고
`flush_changes()` 후 ID가 일치하는지 확인한다. 다른 스테이지가 이미 연결됐거나
연결 실패·ID 불일치가 발생하면 중단한다. 새 관절 view는 이 확인 뒤에 만든다.
로그 `PhysX attached stage=...`와 보고서의 연결 전후 ID에 실제 상태를 기록한다.
화면 준비 이전에 만든 tensor view를 재사용하지 않는다. 첫 관절 명령 전에
이 새 연결로 이름·index와 유한 피드백을 다시 확인하며,
보고서 `setup.fresh_physics_view_after_final_play`에 기록한다.
`setup.control_backend`는 `direct_omni_physics_tensors`다. legacy 래퍼가 전역
SimulationManager에 저장된 오래된 view를 조회하는 경로를 거치지 않는다.
관절 adapter가 상위 `SimulationView`와 하위 관절 view를 모두 보관해 연결 함수가
반환된 뒤에도 객체 수명이 유지되도록 한다. 보고서
`setup.simulation_view_owner_retained`가 true여야 한다.

GUI의 `World.step(render=True)`는 app 갱신 경로만 호출할 수 있으므로 이번 독립 시험에서는
최종 바인딩 전에 timeline을 pause하고 pause 이벤트를 처리한 뒤 PhysX를 연결한다.
반복 구동은 `IPhysxSimulation.simulate(1/120, 경과시간)` → `fetch_results()` →
`update_transformations(False, True)` → GUI 갱신 순서로 직접 진행한다.
USD 결과는 별도 anonymous root layer로 지정하며 원본 자산을 저장하지 않는다.
timeline이 Play 상태이거나 PhysX의 연결 stage가 바뀌면 중단하여 중복 계산을 막는다.
`setup.physics_step_backend`, `completed_physics_steps`, `explicit_physics_time_sec`에
계산 경로·완료한 호출 횟수·경과 시간을 기록한다. 호출 횟수만으로 실제 관절 움직임을
통과 판정하지 않는다. 실패 구간의 `last_feedback`에도 마지막 명령·실측값·양쪽 손끝을 남긴다.

## 검증 범위와 결과

이번 시험은 **물체 없는 무부하·중력 0 조건**이다. 원본 자산은 저장·수정하지 않고,
원본 자산 전체를 별도 메모리 스테이지의 sublayer로 읽는다. 최상위 `/FixedJoint`도
유지한다. 실험의 gravity·물리 씬·조명·카메라·표시 변경은 별도 anonymous root layer에
작성하며 저장하지 않는다. reset에서 지워질 수 있는 session layer에 물리 씬을 두지 않는다.
물리 씬은 표시 root와 분리된 `/World/GripperPhysicsScene`에 두고, reset 직후와
마지막 Play 전후에 실제 `PhysicsScene` prim이 존재하는지 검사한다.
실제 하중·중력·파지 유지력 또는 전체 140 mm stroke를 검증하는 시험은 아니다.
현재 기본 실행은 primary drive의 런타임 damping만 원래 값의 10배로 설정하는
무부하 수렴 실험이다. 실제 하드웨어에 교정된 gain으로 취급하지 않는다.
`--drive-damping-scale 1`로 원래 감쇠값을 사용해 비교할 수 있다(허용 범위 1~100).
Tensor API에서 읽은 원래 감쇠값에 비율을 적용하므로 USD 각도 단위를 추측해 변환하지 않는다.
stiffness·maxForce·질량·관성·mimic·수동 관절 gain은 유지한다. primary velocity target은
정지 목표인 0으로 설정하고 다른 관절의 velocity target은 유지한다.
변경 전후 값과 readback 검사를 `setup.drive_experiment`에 기록한다.
설정이 없거나 읽기/반영에 실패하면 임의 대체값이나 자동 재시도로 진행하지 않는다.
이 런타임 변경은 USD 원본이나 저장 파일에 적용하지 않는다.

원본 inner finger의 양쪽 drive는 stiffness가 0이 아니며 목표가 -45°다.
감쇠 증가만으로 폐쇄 링크의 정지 편차가 제거된다고 가정하지 않는다.
그 drive는 유지하고 primary의 실제 관절 오차를 적분해 drive 목표에 제한된 보정을 더한다.
보정 gain은 1/s, 크기는 ±5°로 제한하며, 실제 drive 목표도 원본 관절 한계 안으로 제한한다.
관절 한계 밖으로 오차가 누적되지 않도록 anti-windup을 적용한다.
이는 실험용 피드백 제어이며 원본 stiffness/질량 등을 추정하거나 변경하지 않는다.
통과 판정은 보정된 drive 목표가 아니라 원래 요청한 **실측 자세 목표**와 비교한다.
`drive_command`에 요청 자세·적용 drive 목표·보정값을 별도로 기록한다.

처음에 partial-open 목표를 4°로 명령하며 최대 8초 안에 위치 오차 ±0.5°와
속도 ±0.5°/s 이내를 연속 0.5초 만족해야 한다. 그때의 관절·손끝 위치를 복귀 기준으로 삼는다.
초기 구간은 4° 목표에서 5° 넘게 이탈하면 중단한다. 4°는 실험의 시작 자세이며
전체 개방이나 TCP 교정값이 아니다. 그 시작 자세에서 추가 10° 닫힘과 복귀를 두 번 수행한다.
이후 각 구간에도 위치 ±0.5°와 속도 ±0.5°/s 조건을 함께 적용하며,
기존 14초 제한과 양쪽 손끝 이동·복귀 기준은 완화하지 않는다.
실패 시 마지막 위치와 속도 및 약 0.1초 간격 samples를 기록한다.

닫힘 구간에서 base 기준으로 측정한 왼쪽 손끝은 오른쪽 방향으로 1 mm 이상,
오른쪽은 왼쪽 방향으로 1 mm 이상 움직여야 하고, 두 중심 간 거리가 2 mm 이상 줄어야 한다.
복귀 구간에는 양쪽 측정 중심이 각각 시작 위치 2 mm 이내로 돌아와야 한다.
여기서 손끝 중심 간격은 실제 접촉면 사이 clearance 또는 교정된 개구 폭이 아니다.

base 이동, 피드백 누락, 시작 대비 15° 초과 이탈, 명령 추종 오차 8° 초과,
목표 시간 초과, 한쪽 손가락 정지 또는 USD 형상 정지는 실패다.
실패 시 가능한 경우 현재 primary joint 피드백으로 hold하고 pause한다.
성공한 경우에도 시험 종료 후 pause하여 마지막 모습을 유지한다.
`--keep-alive`는 화면만 갱신하며 자동으로 물리를 재시작하지 않는다.

성공 기준은 `PASSED_UNLOADED_GRIPPER_CYCLE`, `passed_legs: 4`, `returned_to_start: true`다.
`fdw_sim/logs/gripper_motion_<시간>_<식별자>.json`에는 실제 mimic 연결·DOF 목록,
관절 명령과 피드백, 양쪽 측정 위치·중심 간격, 구간별 판정 및 실패 원인을 저장한다.
JSON의 `samples`는 구간별 약 0.1초 간격 기록이다. GUI 시작부터 종료까지 영상과 로그를 보관한다.
실패 실행은 창 종료 후 코드 2를 반환한다.

CPU/OpenUSD 회귀 테스트는 제어·형상 판정과 실패 처리를 검증한다.
사용자 spark에서의 실제 PhysX 구동 결과는 실행 로그·영상으로 별도 확인해야 한다.

API 근거: [NVIDIA Isaac 6.1 closed-loop/mimic 구성](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/robot_setup_tutorials/rig_closed_loop_structures.html),
[그리퍼 제어 예제](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/robot_setup_tutorials/tutorial_pickplace_example.html),
[PhysX stage 연결 API](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/extensions/runtime/source/omni.physx/docs/api/python.html).
[Tensor articulation API](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/108.0/extensions/runtime/source/omni.physics.tensors/docs/api/python.html),
[legacy 래퍼의 공개 초기화 소스](https://github.com/isaac-sim/IsaacSim/blob/develop/source/deprecated/isaacsim.core.prims/python/impl/articulation.py).

## 시험 물체 파지 단계

장착 상태의 무부하 개폐·팔 소폭 이동 확인 후에는 [시험 블록 접촉·파지·유지·해제 검증](CONTACT_GRASP_TEST_KO.md)을 실행한다. `--mounted-on-fanuc --grasp-test`로 양쪽 접촉, 받침대 분리, 3초 유지, 개방 후 낙하·안착을 검사한다.
