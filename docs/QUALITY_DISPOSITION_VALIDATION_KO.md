# PASS / FAIL / REWORK 분기 검증

`fdw_sim/config/poc1_quality_disposition.yaml`은 기존 품질 예측 모델과
AMR 운송 경로를 사용해 PASS 1건, FAIL 1건, REWORK 1건을 재현하는 설정이다.
판정을 강제로 주입하지 않는다. 부품 ID와 용접 레시피로 예측 점수를 만들고,
이 설정에서만 검사 측정 노이즈를 0으로 둔다. 기존 `poc1.yaml`은 변경하지 않는다.

예상 결과:

- `JOB_DEMO_PASS` / `PART_DEMO_PASS`: 점수 약 0.930, PASS, 논리 출하 완료.
- `JOB_DEMO_FAIL` / `PART_DEMO_FAIL`: 점수 약 0.239, FAIL, 논리 격리.
- `JOB_DEMO_REWORK` / `PART_DEMO_REWORK`: 점수 약 0.732, REWORK, 논리 격리.
- AMR 전달 6건, 출하 완료 1건, 격리 2건, 남은 작업 없음.

레시피는 surrogate 모델의 분기 검증 입력이다. 실제 용접 조건의 적합성이나
품질을 입증하지 않는다. 물리 출하 이송, 물리 격리 구역 이송, 자동 재작업은
이 시나리오에서 구현하지 않는다.

## CPU 검증

```bash
python scripts/run_poc1.py --mode discrete \
  --config fdw_sim/config/poc1_quality_disposition.yaml
```

## Isaac GUI 검증

검토 작업본의 미커밋 변경을 먼저 확인한다. 변경이 없고 브랜치가 예상과 같으면
작업 브랜치를 받아 아래 명령으로 실행한다.

```bash
git pull --ff-only origin fix/fdw-quality-disposition-base-pose

"$ISAACSIM_PYTHON_EXE" scripts/run_poc1_level2_2.py \
  --config fdw_sim/config/poc1_quality_disposition.yaml \
  --gui --real-robot --realtime --keep-alive \
  --motion-execution schematic
```

이 설정은 전체 3건 검증이므로 `--single-amr-demo`와 함께 쓰면 실행을 거부한다.
`verified` 모드는 검증된 물리 파지 어댑터가 없으면 픽업을 보류한다.

보고서의 `Completed jobs: 1`, `Quarantined jobs: 2`, `Active jobs: (none)`과
`[VALIDATION] PASSED logical disposition scenario`를 확인한다. 자동 검증은
부품·작업 ID, 검사까지의 이력, 출하·격리 사유, 남은 운송·셀 작업을 검사한다.
예상과 다르면 성공 메시지 대신 예외로 실행이 종료된다.

시작부터 보고서 출력까지 영상을 확보한다. 별도로 확인할 항목은 AMR 간 겹침,
랙·파이프·기둥 관통, 도크 대기 및 주차 복귀, 배치 완료 후 가공 시작 순서다.
기존처럼 전체 로그와 KPI JSONL은 `fdw_sim/logs/`에 기록된다.

CPU 통과는 GUI 충돌·물리 접촉·파지·용접 성공의 증거가 아니다.
`schematic`은 팔 명령과 스파크를 비활성화한 논리 데모다.
