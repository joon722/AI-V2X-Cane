[← 프로젝트 개요](../README.ko.md)

# lux — Jetson 실시간 위험도 엔진

RSU 브리지 ESP32가 USB 시리얼로 올려주는 V2X JSON을 받아 **파싱 → 상태 저장 → 운동학 계산 → 위험도 등급화 → RSU 다운링크 전송**까지 담당하는 Jetson(Linux) 배포용 패키지입니다.

## 파이프라인

```
parse_v2x → state_store → kinematics(칼만 필터: 거리·접근속도·TTC·DCPA)
          → risk_scoring(팀 점수표 100점 → 등급 0~3) + predict_risk(ONNX, 선택)
          → stability(히스테리시스) → send_risk(트러스트 게이팅 + 레이트 리밋) → RSU
```

## 모듈 구성

| 파일 | 역할 |
| --- | --- |
| `parse_v2x.py` | RSU 시리얼 JSON 한 줄 → 정규화된 상태 레코드 |
| `state_store.py` | 노드별 최신 상태 저장, 신선도(0.5s) 판정 |
| `kinematics.py` | raw/칼만 두 트랙으로 거리·closing speed·TTC·DCPA 계산 (GPS σ=2.5m) |
| `risk_scoring.py` | 팀 점수표(거리 30 + TTC 35 + 상대속도 20 + 차량속도 10 + zone 5) 기반 등급화. 정본은 `scripts/risk_calculator.py` |
| `predict_risk.py` | ONNX Transformer 추론 슬롯 — 모델/런타임이 없으면 자동 폴백 |
| `stability.py` | 위험도 히스테리시스(순간 튐 억제) |
| `send_risk.py` | RSU 다운링크 — GPS 미고정 시 위험 경고 억제, 변화 시 + heartbeat만 전송 |
| `risk_engine.py` | 위 모듈을 묶는 실행 진입점 |
| `jetson_rsu_bridge.py` | RSU 시리얼 연결 유틸 |
| `sim_vehicle.py` / `probe_risk_downlink.py` | 하드웨어 없이 가상 차량·다운링크 검증 도구 |

데이터 출처 구분(real / test / fallback / simulation)은 [SOURCE_MODE.md](SOURCE_MODE.md), 단계별 실기 검증 기록은 [v2x_session_summary_next_steps.md](v2x_session_summary_next_steps.md) 참고.

## 테스트

하드웨어 없이 전 구간을 단위 테스트로 검증할 수 있습니다:

```bash
cd lux
python -m unittest discover -p "test_*.py"
```

(main 기준 113개 테스트 통과 확인)

## 진행 중

zone 위험도 모듈과 `final = max(rule, zone, AI)` 융합은 `feat/lux-fusion-zone` 브랜치에서 통합 작업 중입니다.
