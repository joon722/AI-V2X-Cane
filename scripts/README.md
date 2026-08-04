[← 프로젝트 개요](../README.ko.md)

# scripts — SUMO 출력 → 라벨링 데이터셋 파이프라인

SUMO 시뮬레이션에서 뽑은 feature CSV에 **zone 판정 → 위험도 점수/등급 → 이벤트 유형**을 순서대로 붙여 학습 데이터셋(`dataset/`)을 만드는 배치 파이프라인입니다.

## 파일 구성

| 파일 | 역할 |
| --- | --- |
| `zone_detector.py` | `zones/zone_definition.csv`의 원형 반경(30m) 안에 있는지 판정 |
| `risk_calculator.py` | **팀 위험도 점수표의 정본.** 거리 30 + TTC 35 + 상대속도 20 + 차량속도 10 + zone 5 = 100점, 컷오프 ≥70→3 / ≥45→2 / ≥20→1. TTC 미접근 센티널 `9999.0` |
| `event_classifier.py` | Near Miss / Parking Exit Risk / Blind Spot Risk / Vehicle Yield / Safe Pass 등 이벤트 유형 라벨링 |
| `event_analyzer.py` | 이벤트 통계 분석 |
| `run_scenario.py` | 시나리오 하나에 대해 위 과정을 자동 실행 |
| `merge_dataset.py` | 시나리오별 결과를 마스터 데이터셋으로 병합 |
| `backup/` | 이전 버전 보관 |

## 점수표 정본 규칙

`risk_calculator.py`가 위험도 점수표의 **단일 정본(source of truth)** 입니다. Jetson 배포용 복사본(`lux/risk_scoring.py`)이 이 파일과 달라지면 drift 감지 테스트가 실패하도록 되어 있으므로, 점수표를 바꿀 때는 반드시 이 파일을 먼저 수정하세요.
