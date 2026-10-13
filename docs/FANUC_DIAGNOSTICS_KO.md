# FANUC 관절·TCP 수동 진단

AMR 교통과 품질 분기 확인 다음 단계는 실제 FANUC 모델의 관절·링크와
제어용 프레임을 확인하는 것이다. `tool0`는 아직 검증되지 않은 카탈로그 값이다.
임의로 TCP 이름이나 홈 관절값을 바꾸지 않는다.

검토 작업본에 미커밋 변경이 없는지 확인하고 작업 브랜치를 받은 뒤 실행한다.

```bash
git pull --ff-only origin fix/fdw-quality-disposition-base-pose

"$ISAACSIM_PYTHON_EXE" scripts/run_poc1_level2_2.py \
  --gui --real-robot --robot fanuc_crx10ia \
  --robot-diagnostics --keep-alive --motion-execution schematic
```

진단은 기존 작업장과 로봇을 로드하되 작업을 투입하지 않는다. 모션 컨트롤러와
스파크를 비활성화하고, 관절 명령·홈 명령·픽업·가공 명령을 보내지 않는다.
USD가 가진 물리 동작이나 중력 영향까지 비활성화하는 모드는 아니다.

출력은 화면과 `fdw_sim/logs/robot_diagnostics_YYYYMMDD_HHMMSS.json`에 저장된다.
생성된 JSON과 같은 실행의 전체 로그를 확보한다.

수집 정보:

- 실제 USD 관절 이름, 종류, 연결 링크, 작성된 한계와 축.
- articulation API에서 읽을 수 있는 관절 이름·위치와 USD 이름 대응.
- 부모 변환과 USD 길이 단위를 반영한 로봇 베이스 좌표.
- 설정된 TCP 이름의 존재·중복 여부, 후보 링크 경로와 좌표.
- 명시적으로 지정한 FANUC URDF·robot description·RMPflow 설정 파일의 존재 여부.

USD revolute 한계는 도(degrees), articulation revolute 위치는 라디안이다.
좌표 출력은 미터이며 USD up-axis도 함께 기록한다. 후보 링크가 있거나
`unique_link_found`가 나오는 것만으로 TCP 보정·그리퍼 접촉이 검증되지는 않는다.
명시적 설정이 비어 있다고 Isaac 번들에도 해당 파일이 없다고 단정하지 않는다.

진단 결과에서 실제 관절 피드백과 관절 이름 대응을 확인하고, TCP 후보를 실제
툴 플랜지·그리퍼 구조 및 보정 정보와 대조한다. 이후 매칭되는 URDF·Lula·RMPflow
설정을 확인한 뒤 빈손 접근 시험을 구성한다. 그리퍼 접촉·부품 고정·들기·놓기
검증을 완료하기 전에는 물리 파지 어댑터를 verified로 표시하지 않는다.
