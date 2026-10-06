# FDW 우선 수정: 품질 전달·격리·로봇 기준좌표

기준: `claude/awesome-tesla-vbtpvo`, `69eb529598a92528e35e46c2e4e24daeca24620f`.
`main`은 별도 이력이므로 이 패치를 main에 적용하지 마세요.

## 변경 내용

- AMR이 출발 셀의 해당 부품 품질을 복사하여 운반하고, 검사 셀에 전달합니다. 입력 버퍼가 가득 차 재시도해도 같은 품질을 유지하며, 다음 부품의 품질과 섞이지 않습니다.
- 검사에 품질이 없거나 유효하지 않으면 FAIL로 처리합니다. 이전 부품의 PASS를 재사용하지 않습니다. 셀 상태 메시지도 버퍼/품질 스냅샷으로 발행합니다.
- 검사 PASS만 다음 공정 또는 출하로 진행합니다. FAIL, REWORK, 판정 누락/부품 불일치는 논리 격리 목록에 보관하고 출력 버퍼를 비웁니다. 완료/출하 이벤트와 makespan KPI에는 포함하지 않습니다. 별도 `job_quarantined_count`와 `/fdw/job_quarantined` 이벤트를 기록합니다.
- `orchestrator.inspection_nonpass_disposition: quarantine`이 유일한 지원 정책이며 생략 시에도 동일합니다. 자동 재작업은 **0회**입니다. `rework` 등 다른 값은 설정 오류입니다. 재작업은 운영자가 원인을 확인한 뒤 별도 작업으로 결정해야 합니다.
- 여러 AMR이 같은 가득 찬 입력으로 몰려 출력 이송 차량까지 소진하지 않도록, 적재 전에 목적지 입력을 예약합니다.
- 제어기는 로봇의 실제 월드 위치 XYZ 및 자세 WXYZ를 사용합니다. 로더의 셀-로컬 오프셋은 유지하고, 부모 USD 변환 또는 articulation에서 월드 자세를 읽습니다. 휴리스틱/Lula/RMPflow에 반영하며 움직이는 베이스도 갱신합니다.

## 검증

```sh
python -m pip install -r requirements-dev.txt
python -m pytest -q
python -m compileall -q fdw_sim scripts tests
python scripts/run_poc1.py --mode discrete --log-level WARNING
```

검증 환경: Python 3.12, PyYAML 6.0.3, NumPy 2.5.3, pytest 9.1.1.
전체 92 tests + 14 subtests 통과. 기존 6개 smoke 스크립트도 직접 실행 통과.
회귀 테스트는 실제 discrete AMR→용접→검사 흐름의 PASS/FAIL/REWORK, 품질 누락,
혼합 4개 작업, 느린 상태 발행, 이송 재시도, 잘못된 부품 ID 및 격리 집계를 포함합니다.
로봇 테스트는 CPU와 엄격한 USD/Isaac API 대역을 사용합니다.

현재 기본 YAML의 레시피는 자동 선택 도구와 출력 조건이 맞지 않아 실제 불량 판정이
드러날 수 있습니다. 검증 실행에서는 1개 출하, 2개 격리, 미완료 0개였습니다.
검사 노이즈가 있으므로 경계 점수의 구체적 판정은 실행마다 달라질 수 있습니다.
좋은 경로 smoke fixture만 레이저에 맞는 작은 gap으로 고정했고,
생산 YAML의 레시피나 판정 임계값을 통과시키기 위해 변경하지 않았습니다.

## 한계

- 논리 격리는 시뮬레이터 메모리/이벤트/KPI상의 보관입니다. 물리 격리 구역,
  AMR 격리 이동, 재작업 공정, 설비 안전 인증 또는 영구 MES 저장을 구현하지 않습니다.
- 실제 Isaac Sim GUI/PhysX, USD 자산 로딩, 충돌/토치 접촉, 실제 로봇은 검증하지 않았습니다.
- inspection을 포함하지 않는 공정 경로에 새로운 검사 공정을 자동 삽입하지 않습니다.
- USD 자산 재구축, 그리퍼/물리 결합 또는 다른 이슈는 변경 범위가 아닙니다.
