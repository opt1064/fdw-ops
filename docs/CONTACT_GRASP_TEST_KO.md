# 시험 블록 접촉·파지·유지·해제 검증

기존 FANUC 장착 그리퍼의 무부하 닫힘·복귀 2회(4구간)가 성공한 뒤, 독립된 동적 시험 블록으로 접촉 검증을 수행한다. 실제 Isaac Sim에서 성공 로그와 화면을 함께 확인해야 한다. CPU 테스트는 판정 로직과 USD 구성을 검증하며, 물리 파지 성공의 증거가 아니다.

## 실행

기존 시뮬레이터 창을 닫은 뒤 검토 worktree에서 실행한다.

```bash
cd ~/isaac_workspace/projects/fdw-ops-review
git status -sb
git pull --ff-only origin fix/fdw-quality-disposition-base-pose
"$ISAACSIM_PYTHON_EXE" scripts/run_gripper_motion_test.py --mounted-on-fanuc --grasp-test --drive-damping-scale 10 --keep-alive
```

`--grasp-test`에는 `--mounted-on-fanuc`가 필요하며 `--mounted-arm-motion`과 동시에 지정하지 않는다. 시작 전 5초 동안 화면을 준비할 수 있다. 먼저 기존 무부하 개폐 2회가 진행되고, 이후 노란 시험 블록과 파란 받침대가 생성된다. 창의 타임라인 Play를 누르지 않는다. 정지 타임라인에서 명시적인 PhysX 스텝으로 진행한다.

## 화면과 통과 조건

1. `SETTLE_ON_SUPPORT`: 노란 블록이 파란 받침대에 안착하고 양쪽 손가락과 떨어져 있어야 한다.
2. `CLOSE_CONTACT`: 양쪽 손가락이 블록에 닿은 후 각각 1 N 이상, 팔 속도 0.2도/초 이하, 그리퍼 속도 0.5도/초 이하, 물체 속도 2 mm/초 이하인 상태를 1초 유지해야 한다. 이 시간 동안 그리퍼 기준 물체 위치 변화 0.5 mm/자세 변화 0.5도 이내, 양쪽 접촉력 변화 20% 이내여야 한다. 안정화가 완료되기 전에는 들어 올리지 않는다.
3. `LIFT`: 블록이 녹색으로 표시된다. J5를 최대 2도, 6초 동안 움직여 받침대에서 5 mm 이상 분리하고, 받침대 접촉이 사라져야 한다. 이동 방향은 실제 시작 관절 좌표와 USD 기구학으로 선택하며 예측 상승량이 8 mm 미만이면 시작하지 않는다.
4. `HOLD_UNSUPPORTED`: 양쪽 접촉과 받침대 분리를 3초 유지한다. 그리퍼 기준 물체 위치가 파지 시점에서 3 mm, 자세가 5도 이상 변하면 실패한다.
5. `OPEN_RELEASE`: 블록이 주황색으로 표시된다. 손가락을 벌리면 블록이 최소 5 mm 낙하하여 받침대에 안착하고 양쪽 손가락 접촉이 사라져야 한다.
6. `RETURN_EMPTY`: 블록은 받침대에 남고, 열린 그리퍼와 팔이 시작 관절 자세로 돌아온다. 블록은 하늘색으로 표시된다.

전체 성공 표시는 `PASSED_SIMULATED_CONTACT_GRASP_RELEASE`, `grasp_test.passed_phases: 6`이다. 실패 시 블록을 빨간색으로 표시하고 물리 진행을 멈춘다. 자동으로 개방하거나 물체를 다른 장소로 옮기지 않는다. JSON은 `fdw_sim/logs/gripper_motion_*.json`에 저장된다. 위상별 마지막 좌표·속도·접촉력, 약 0.1초 간격 측정값, 장착부와 TCP 후보 좌표 검사를 남긴다. 화면만 움직이거나 한쪽 접촉만 검출된 경우에는 통과하지 않는다.

## 시험 조건과 범위

- 시험 블록 질량은 **50 g**이며 실제 손가락 형상 경계에서 크기와 초기 위치를 계산한다. 열린 안쪽 간격의 85%를 블록 폭으로 사용한다. 받침대는 손가락과 간섭하지 않도록 블록보다 좁게 만든다.
- 시험 블록·받침대 재질의 정지/동적 마찰계수는 0.8, 반발계수는 0이다. 이 값은 합성 시험 물체의 설정이며 실제 제품의 측정 물성이 아니다. 원본 로봇과 손가락 재질·질량·관성·토크 제한은 바꾸지 않는다.
- 기존 검증 조건을 유지하여 장면 중력은 0이다. 블록에만 매 물리 스텝마다 **0.4905 N**의 아래 방향 힘을 가한다. 물체의 중력 상당 하중에 대한 파지 시험이며 로봇 전체의 중력 보상·정격 하중 시험은 아니다.
- 블록은 독립 동적 강체다. 초기 생성 이후 좌표를 쓰거나, 그리퍼 아래에 붙이거나, 고정 조인트를 만들어 따라오게 하지 않는다. 접촉 보고와 네이티브 강체 좌표를 이용한다.
- 새 물체 생성 후 물리 장면 변경을 반영하고 관절·강체 뷰를 새로 만든다. 이때 기존 관절 좌표와 강성·감쇠·힘 제한이 보존되었는지 확인한다. 타임라인 재생/리셋과 감쇠 배율 재적용을 하지 않는다.
- 접촉 관통 2 mm 초과, 시험 접촉력 50 N 초과, 예상하지 않은 그리퍼 몸체 접촉, 좌표·속도 피드백 손실은 실패로 처리한다. 이 수치는 이 소형 시험의 중단 기준이다.
- 장착부 상대 오차 2 mm/0.01 rad 및 실제 관절 기반 USD FK와 TCP 후보 좌표의 일치 여부를 계속 검사한다. TCP 실측 보정, 교환장치 검증, 생산 부품 파지, 실기 로봇 안전성 또는 작업장 `verified` 모드의 파지 어댑터 검증으로 확대 해석하지 않는다. 설정의 활성 공구나 생산 모드 어댑터를 자동 변경하지 않는다.

API 참고: [PhysX 접촉 보고](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/110.1/extensions/runtime/source/omni.physx/docs/dev_guide/contact_reports.html), [Physics tensors](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/108.0/extensions/runtime/source/omni.physics.tensors/docs/api/python.html). 설치된 Isaac Sim 6.1에서 최종 실행 확인이 필요하다.

## 다음 단계

제자리 6단계가 통과한 뒤에는 [수평 이송·목표 받침대 배치 시험](GRASP_TRANSFER_TEST_KO.md)을 사용한다. 기존 명령에 `--transfer-test`를 추가하면 별도 목표 받침대로 옮겨 내려놓고 빈 그리퍼를 복귀시키는 11단계를 수행한다.
