[← 프로젝트 개요](../README.ko.md)

# dataset — 학습·검증 데이터

SUMO 시나리오 6종에서 추출한 궤적 feature와 `scripts/` 파이프라인이 붙인 라벨(위험도·이벤트)입니다. 좌표는 SUMO 로컬 좌표계(미터 단위)입니다.

## 구성

| 경로 | 내용 |
| --- | --- |
| `scenario_000/` ~ `scenario_005/` | 시나리오별 `feature.csv`(궤적) / `risk_result.csv`(위험도) / `event_result.csv`(이벤트) |
| `master_dataset.csv` | 6개 시나리오 병합 — **78,853행 × 24열** |
| `labeled_master_dataset.csv` | Transformer 학습 입력 — **12,621행 × 23열** |
| `feature_with_zone.csv`, `event_result.csv` | 파이프라인 중간 산출물 |

## 라벨 분포 (labeled_master_dataset 기준)

| risk_level | 행 수 | 비율 |
| --- | --- | --- |
| 0 (Normal) | 10,796 | 85.5% |
| 1 (Caution) | 1,523 | 12.1% |
| 2 (Warning) | 272 | 2.2% |
| 3 (Near Miss) | 30 | 0.24% |

심한 클래스 불균형 때문에 학습 시 클래스 가중치 CrossEntropy를 사용합니다 (`AI_Model/` 참고). TTC 미접근 행(센티널 `9999`)은 전체의 61.8%입니다.
