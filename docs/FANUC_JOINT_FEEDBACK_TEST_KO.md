# FANUC 소각도 관절 구동·피드백 시험

이 단계는 Isaac Sim FANUC CRX-10iA 관절을 실제 시뮬레이션 drive로 움직이고,
관절 피드백 및 USD 말단 프레임을 Lula FK와 대조한다. 실물 FANUC 장비에 접속하지 않는다.
생산 작업, 부품 투입, 용접 스파크 및 기존 IK/RMPflow 명령은 실행하지 않는다.
질량·관성·drive gain·관절 한계는 기존 자산 값을 유지한다.

## 실행

### 화면에서 팔 움직임 먼저 보기

사용자 환경에서 ±1° 관절 시험이 통과한 뒤, 눈으로 움직임을 확인하려면:

```bash
"$ISAACSIM_PYTHON_EXE" scripts/run_poc1_level2_2.py \
  --gui --show-robot-motion --keep-alive
```

이 옵션은 FANUC 진단·내보내기·Lula 검증·실시간 관절 시험을 활성화한다.
모델 검증 통과 후 기존 SceneCamera를 로봇 base와 말단의 중간 위치에 맞춰 가까이 배치한다.
5초 카운트다운 뒤 J1만 `+5° → 시작 자세 → -5° → 시작 자세`로 움직이며,
각 이동은 10초, 총 약 40초다. 다른 관절은 시작 자세 목표를 유지한다.
생산 작업과 부품 투입은 없다. 보고서의 `selected_joints`는 `["J1"]`,
`amplitude_deg`는 5, `planned_legs` 및 `passed_legs`는 성공 시 4이다.
이 결과로 J2~J6 시연이나 충돌 검증을 주장하지 않는다.
선택한 관절의 허용 이탈은 시작 자세에서 5.5°, 나머지는 기존 1.5°다.
관절 한계·추종·말단 좌표 검사와 실패 시 정지는 계속 적용한다.

`--safe-mode`, `--skip-auto-camera`, `--disable-viewport-switch`의 기존 카메라
회피 설정은 존중한다. 카메라 자동 전환을 끈 환경에서는 로봇을 수동으로 확대한다.
GUI 전용이며 `--headless` 및 `--livestream`과 함께 사용하지 않는다.

### 전체 관절 ±1° 피드백 시험

검토 checkout에서 `git status -sb`로 미커밋 변경이 없고 브랜치가
`review/fdw-pr1`인지 먼저 확인한다. 이후:

```bash
git pull --ff-only origin fix/fdw-quality-disposition-base-pose
git log -1 --oneline
"$ISAACSIM_PYTHON_EXE" scripts/run_poc1_level2_2.py \
  --gui --real-robot --robot fanuc_crx10ia \
  --robot-diagnostics --export-robot-model --validate-lula-model \
  --joint-feedback-test --realtime --keep-alive --motion-execution schematic
```

`schematic`은 기존 생산 흐름 설정이다. 명시적인 `--joint-feedback-test` 옵션은
진단 분기에서 별도의 소각도 drive 시험을 실행한다.
진단·모델 내보내기·계산 자세 검증 옵션과 FANUC 모델 지정이 필수다.
동일 실행에서 `PASSED_COMPUTED_MULTI_POSE`가 나와야 관절 명령을 시작한다.

## 시험 순서와 판정

현재 관절 피드백을 시작 자세로 기록한다. 모든 목표를 관절 한계와 먼저 대조하고,
초기 말단 좌표가 Lula와 일치하는지 확인한다. 시작 자세가 한계에 가까워 ±1° 목표 중
하나라도 한계를 넘으면 이동 없이 실패한다. 자동으로 다른 자세로 이동하지 않는다.

J1~J6에 각각 `+1° → 시작 자세 → -1° → 시작 자세`를 실행한다. 총 24구간이며
각 구간은 2초에 걸쳐 목표를 변화시키고 최대 3초 더 기다린다.
다른 관절의 목표는 시작 자세로 유지하며 feedback을 모든 관절에서 확인한다.
`set_joint_position_targets` 또는 명시적 관절 index를 넣은 `ArticulationAction`을 사용한다.
`set_joint_positions`를 통한 순간이동은 사용하지 않는다.

구간 종료 시 모든 관절의 목표 오차가 0.1° 이하인 상태가 최소 0.1초 지속되어야 한다.
매 물리 step에서 실제 피드백으로 계산한 Lula 말단 자세와 USD 말단 자세를 대조한다.
허용 오차는 위치 2 mm, 방향 0.01 rad이다. 시작 자세 대비 1.5° 초과 이탈,
명령 추종 오차 2° 초과, 피드백 누락, 말단 오차 초과 또는 시간 초과는 실패다.
실패하면 가능한 경우 현재 피드백 위치를 hold하고 물리 시뮬레이션을 pause한다.
실패 후 원위치로 자동 이동하지 않는다. `--keep-alive`는 실패 시 화면 갱신만 한다.
창 종료 또는 Ctrl+C 후 실패 실행은 종료 코드 2를 반환한다.

성공 결과:

```text
status: PASSED_SIMULATED_JOINT_FEEDBACK
planned_legs: 24
passed_legs: 24
returned_to_start: true
```

새 `fdw_sim/logs/fanuc_model_<시간>_<식별자>/joint_feedback_test.json`에
시작 관절/말단 자세, 구간 목표, 최종 관절/말단 자세, 최대 추종·말단 오차,
성공 구간 수와 실패 원인을 저장한다. 모델 내보내기 및 Lula 검증 보고서도 같은 폴더에 있다.
시작부터 종료까지 로그와 FANUC을 가까이 촬영한 영상을 보관한다.

## 검증 범위

CPU 회귀 테스트는 제어 로직과 실패 처리를 확인한다. Isaac의 실제 drive 응답은
사용자 환경에서 위 시험을 실행한 결과로 판정한다. 피드백이 움직여도 USD 말단이
갱신되지 않으면 통과시키지 않는다. 이런 경우 transform 동기화 문제를 먼저 조사한다.

이 시험은 작은 범위의 시뮬레이션 구동과 모델 일관성 검증이다. TCP 공구 오프셋,
실물 로봇 보정, 충돌 회피, 실제 파지, 용접, 전체 작업영역의 궤적 검증을 의미하지 않는다.
FK 전용 Lula 설정의 빈 collision spheres로 충돌 검증을 주장하지 않는다.
후속 단계는 결과 확인 후 공구 TCP·그리퍼 접촉 검증으로 진행한다.

API 근거: [NVIDIA Isaac Sim 6.1 Articulation Controller](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/robot_simulation/articulation_controller.html),
[Core Prims API](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/deprecated/isaacsim.core.prims/docs/index.html).
