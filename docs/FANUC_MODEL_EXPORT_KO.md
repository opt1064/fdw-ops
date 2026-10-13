# FANUC USD → URDF 내보내기와 현재 자세 대조

실제 USD의 관절 J1~J6와 말단 `J6_link/flange/ee_link`를 확인했지만,
현재 컨트롤러 탐색에서는 URDF·Lula robot description·RMPflow 설정을 찾지 못했다.
같은 USD에서 URDF를 추출해 모델 구조와 현재 자세의 말단 좌표부터 대조한다.

Isaac 공식 API `isaacsim.asset.exporter.urdf.UsdToUrdfConverter`를 사용한다.
이 확장은 사용자 Isaac 6.1 실행 로그에서 이미 시작된 것으로 확인됐다.
참고: https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.asset.exporter.urdf/docs/api.html

## 실행

기존 Isaac 창을 닫고, 검토 작업본의 미커밋 변경을 확인한다. 변경이 없고
브랜치가 예상과 같으면 다음 명령으로 실행한다.

```bash
git pull --ff-only origin fix/fdw-quality-disposition-base-pose

"$ISAACSIM_PYTHON_EXE" scripts/run_poc1_level2_2.py \
  --gui --real-robot --robot fanuc_crx10ia \
  --robot-diagnostics --export-robot-model --keep-alive \
  --motion-execution schematic
```

작업 투입·관절 명령은 하지 않는다. 현재 로드된 FANUC 범위만 내보낸다.
원본 USD 파일, 홈 관절값, 물성값, 컨트롤러 환경변수를 덮어쓰지 않는다.
매 실행은 `fdw_sim/logs/fanuc_model_시간_고유번호/`라는 새 폴더에 저장한다.

## 산출물과 판정

- `fanuc_crx10ia.urdf` 및 `meshes/`: 공식 exporter가 추출한 모델.
- exporter가 비물리 `ee_link` 프레임을 생략하면
  `fanuc_crx10ia_with_end_frame.urdf`를 별도로 생성한다. 실제 USD에서
  `ee_link`를 읽고, 설치된 exporter의 `build_urdf_frames`가 정의한 상위
  URDF 링크 좌표계와의 고정 변환을 계산해 추가한다. URDF 링크 좌표계는
  관절 기준으로 재정의될 수 있으므로 USD 몸체 좌표계로 대체하지 않는다.
  원본 URDF는 그대로 보존하며 질량·관성·그리퍼 보정값은 추가하지 않는다.
- `model_comparison.json`: 원본 진단, 관절 이름·종류·부모/자식·한계 대조,
  현재 피드백 관절값으로 계산한 말단 위치·방향 차이. `fixed_end_frame`에는
  고정 프레임을 만든 USD 경로·변환과 검증 한계가 기록된다.
  `exporter_parent_world_matrix_m`과 `usd_body_parent_world_matrix_m`도 남겨
  exporter 링크 좌표계와 USD 몸체 좌표계의 차이를 추적할 수 있다.
- 내보내기 API 또는 변환 실패 시 `export_error.json`과 traceback을 보존한다.

URDF는 미터·라디안을 사용한다. USD revolute 한계는 도에서 라디안으로
변환해서 비교한다. USD 부모 변환과 길이 단위도 반영한다. 현재 자세 비교의
허용 오차는 위치 2mm, 방향 0.01rad이다. 이는 모델 대조 기준이며 파지·용접
정밀도 요구사항을 의미하지 않는다.

`comparison.status = PASSED_SINGLE_POSE`는 구조·한계와 현재 한 자세에서의
말단 FK 일치를 뜻한다. 관절 전체 범위, 도달 가능성, 경로 충돌, TCP 보정,
그리퍼 접촉 또는 파지 성공을 뜻하지 않는다. 영점 부근에서는 축 방향 오류가
잘 드러나지 않을 수 있으므로 이후 다중 자세의 FK 대조가 필요하다.

`FAILED`라면 `comparison.errors`를 검토한다. USD에서 프레임을 찾지 못하거나
상위 링크 이름이 중복되면 자동 추가하지 않는다. 누락된 실제 rigid body도
고정 프레임으로 대체하지 않는다. 링크·축·물성값을 임의로 보충하지 않는다.
설치된 exporter의 링크 프레임을 읽지 못해도 USD 몸체 프레임으로 대신 계산하지 않는다.

프레임 정의 참고:
https://github.com/isaac-sim/IsaacSim/blob/main/source/extensions/isaacsim.asset.exporter.urdf/isaacsim/asset/exporter/urdf/converter/urdf_frames.py

URDF와 `model_comparison.json` 및 같은 실행 로그를 확보한다. 이 결과를
검토한 뒤 URDF와 일치하는 Lula robot description을 만들고, 다중 자세 대조와
제한된 빈손 이동을 별도로 준비한다. RMPflow 충돌 형상과 그리퍼 모델은
확인된 근거가 생긴 뒤 추가한다.
