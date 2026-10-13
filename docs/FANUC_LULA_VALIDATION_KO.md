# FANUC Lula FK 설정과 다중 자세 모델 대조

단일 자세 비교를 통과한 URDF에서 FK 전용 Lula 설정을 생성하고,
여러 자세에 대해 USD 관절 앵커, URDF, 실제 Lula FK 결과를 대조한다.
이번 단계는 계산 검증이다. 실제 팔을 여러 자세로 움직이지 않는다.

## 실행

기존 Isaac 창을 닫고 검토 작업본에 미커밋 변경이 없는지 확인한 뒤 실행한다.

```bash
git pull --ff-only origin fix/fdw-quality-disposition-base-pose

"$ISAACSIM_PYTHON_EXE" scripts/run_poc1_level2_2.py \
  --gui --real-robot --robot fanuc_crx10ia \
  --robot-diagnostics --export-robot-model --validate-lula-model \
  --keep-alive --motion-execution schematic
```

자동으로 작업을 투입하거나 관절 명령을 보내지 않는다. 기존 제어 환경변수,
홈 자세와 파지 어댑터를 변경하지 않는다. 매 실행은 새 모델 출력 폴더를 사용한다.

## 생성 파일

`fdw_sim/logs/fanuc_model_시간_고유번호/`에 다음 파일이 추가된다.

- `fanuc_robot_description_fk_only.yaml`: 실제 관절 순서 J1~J6와
  내보낸 URDF의 root link를 사용한다. `default_q`는 읽어온 현재 관절값이다.
  이는 설정 입력이며 실제 홈 이동 명령이 아니다.
- `lula_model_validation.json`: Lula 관절 매핑, USD 관절의 양쪽 앵커,
  자세별 관절값, 세 모델 간 위치·방향 차이와 실패 사유.

설정은 기구학 전용이다. 충돌 구는 빈 목록이며, 근거 없는 속도·가속도·jerk
한계나 RMPflow 정책값을 넣지 않는다. 충돌 회피가 가능한 운영 설정으로 사용하지 않는다.
실제 Lula가 파일을 거부하면 오류를 기록하고 실패로 표시하며 heuristic으로 대체하지 않는다.

## 계산하는 자세

- 현재 자세와 영점: 한계를 만족하는 경우.
- 각 관절을 현재 값에서 -15°, -5°, +5°, +15° 바꾼 자세.
- 각 관절에 교대로 ±10° 오프셋을 적용한 복합 자세 2개.

6관절에서 최대 28개이며 관절 한계를 벗어나는 샘플은 제외한다. 현재 자세가
한계를 벗어나면 검증을 실패시킨다. 샘플을 임의로 한계에 맞춰 잘라내지 않는다.

USD 기준 계산은 양쪽 관절 앵커를 사용한다:

`T_child = T_parent × T_local0 × Motion(q) × inverse(T_local1)`

말단 프레임 오프셋도 실제 USD에서 읽는다. URDF 계산은 joint origin과 axis를
사용하며, Lula는 실제 `LulaKinematicsSolver.compute_forward_kinematics`를 호출한다.
Lula가 반환한 관절 순서가 다르면 이름으로 재정렬한다. 모든 결과에 동일한
root link의 월드 변환과 길이 단위를 적용한다.

모든 자세·모든 모델 쌍에서 위치 차이 2mm와 방향 차이 0.01rad 이내일 때만
`PASSED_COMPUTED_MULTI_POSE`를 기록한다. 이 허용 오차는 모델 대조 기준이며
제조 공정 정밀도를 의미하지 않는다. 실패하면 자세별 차이를 보존한다.

## 다음 단계

생성된 YAML과 `lula_model_validation.json`, 같은 실행 로그를 확보한다.
계산 검증이 통과한 뒤 실제 관절 구동과 다중 자세의 피드백 대조를 별도 진행한다.
작업대 도달 가능성·이동 경로 충돌·TCP 보정·그리퍼 접촉·파지는 아직 미검증이다.

## 공식 참고

- Lula 설정 형식 예제:
  https://github.com/isaac-sim/IsaacSim/blob/main/source/deprecated/isaacsim.robot_motion.motion_generation/motion_policy_configs/franka/rmpflow/robot_descriptor.yaml
- FK API:
  https://docs.isaacsim.omniverse.nvidia.com/6.1.0/manipulators/manipulators_lula_kinematics.html
- 순수 기구학 설정에서 충돌 구를 생략할 수 있다는 설명:
  https://docs.isaacsim.omniverse.nvidia.com/5.1.0/manipulators/manipulators_robot_description_editor.html
