# Simulation — SUMO 시뮬레이션 작업 기록

SUMO(Simulation of Urban Mobility) 환경 구축과 데이터 생성 과정의 작업 기록입니다.

## 수행 내용 (작업 기록 요약)

- OSM 기반 실제 도로망 import → `.net.xml` 생성
- 차량 Flow/Route 설정(`route.rou.xml`), `config.sumocfg` 구성
- 차량 20대 이상 동시 주행 교통 시뮬레이션
- FCD(Floating Car Data) 출력 → XML → CSV 변환 (`preprocess.py`)
- TraCI 연동으로 차량 ID·속도 실시간 수집
- 보행자 route 생성 및 테스트

상세 기록: [`20260511`](20260511), [`20240503 최민서`](<20240503 최민서>)

## 산출물 위치

| 산출물 | 위치 |
| --- | --- |
| 시나리오별 궤적 CSV (6종) | [`dataset/`](../dataset/) |
| zone·위험도·이벤트 라벨링 파이프라인 | [`scripts/`](../scripts/) |

> **참고:** SUMO 프로젝트 원본(`.net.xml`, `.rou.xml`, `.sumocfg`, `preprocess.py`)은 시뮬레이션 담당 팀원의 로컬 환경에 있으며 이 저장소에는 산출 데이터(CSV)만 포함되어 있습니다.
