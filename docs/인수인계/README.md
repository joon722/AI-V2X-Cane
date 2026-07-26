# 인수인계 자료

신규 팀원이 프로젝트 구조를 빠르게 파악하기 위한 설명 자료입니다.
저장소의 실제 코드(`arduino/`, `lux/`, `scripts/`, `AI_Model/`)를 근거로 작성했습니다.

## 읽는 순서

| 순서 | 파일 | 내용 |
| --- | --- | --- |
| 1 | `AI_V2X_연결구조_다이어그램_report.pptx` | 하드웨어·SUMO/AI·서버가 어떻게 이어지는지 다이어그램 중심 (편집 가능) |
| 2 | `AI_V2X_시스템_인수인계_report.pdf` | 시스템 전체 — 9단계 구조, RSU 상세, 위험도 판정 위치, 서버·SUMO 연동 |
| 3 | `AI_V2X_코드_설명서_report.pdf` | 파이썬 파일별 설명 — 입력·출력·핵심 동작·다음 단계 |

## 핵심 요약

- **RSU 노드 = RSU ESP32(무선 중계) + Jetson(판정 두뇌)**
  ESP32는 중계만 하고, 거리·TTC 계산·AI(ONNX) 추론·`max` 융합·최종 risk 0~3 결정은 **Jetson**이 수행합니다.
- **세 살림은 두 개의 다리로 이어진다**
  - 로컬 PC(SUMO·AI 학습) → **모델 파일 `.onnx` 이식** → 현장(Jetson)
  - 현장·로컬 PC → **이벤트/결과 업로드** → 서버(위험지도)
  - SUMO는 현장 장비와 실시간 통신하지 않습니다.
- **역할 경계**: 위험의 *기준·라벨·모델*은 오프라인(SUMO/AI)에서 정의하고,
  그 기준으로 *실시간 판정*하는 것은 현장(Jetson)입니다.

## 함께 볼 문서

- `docs/integration_contract.md` — 모듈 간 규격 합의서(좌표계·feature·risk 정의)
- `lux/v2x_session_summary_next_steps.md` — 단계별 진행·결정 기록

## 알려진 정합 이슈

- 위험도 정의가 두 벌(점수표 방식 A / 거리 임계 방식 B) — Transformer가 A로 학습되어 **A로 통일** 필요
- 점수표가 `scripts/risk_calculator.py`(정의)와 `lux/risk_scoring.py`(실행)에 이중 존재 — 단일 소스 관리 필요
- 좌표계 3종(실시간 위경도 / SUMO 로컬 미터 / 지도 상도동 위경도) 미통일
- RSU(`arduino/10_...`)는 V2X version 1, 지팡이(`09_...`)·차량(`06_...`)은 version 2 — RSU 경유 경고 전달을 위해 정합 필요
