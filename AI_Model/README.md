[← 프로젝트 개요](../README.ko.md)

# AI_Model — Transformer 위험도 분류기

SUMO 라벨 데이터로 시퀀스 분류 Transformer를 학습하고, ONNX로 내보내 Jetson 온디바이스 추론에 사용합니다.

## 무엇을 하는 모델인가

보행자·차량 궤적의 **최근 10프레임**을 입력받아 현재 시점의 **위험 등급 0~3** (Normal / Caution / Warning / Near Miss)을 분류합니다.

```
Linear(11 → 64) → TransformerEncoder(2층, d_model 64, head 4, FFN 128, dropout 0.1)
                → 마지막 프레임 추출 → LayerNorm → Linear(64 → 4클래스)
```

- 입력: `[batch, 10, 11]` — 위치, 속도, 거리, TTC, 규칙 점수, zone 위험도 등 11개 feature (z-score 정규화, 파라미터는 `models/scaler.json`)
- 출력: `[batch, 4]` logits, argmax = risk_level
- TTC 미접근 센티널 값: `9999`
- 클래스 가중치 CrossEntropy로 불균형 대응 (위험 프레임은 전체의 0.24%)

## 파일 구성

| 파일 | 역할 |
| --- | --- |
| `transformer/merge_labels.py` | `master_dataset.csv`(feature) + `event_result.csv`(label) 조인 |
| `transformer/train_transformer.py` | 학습 파이프라인 (seq 10, batch 64, epoch 20, Adam 1e-3, 계층적 분할 64/16/20) |
| `transformer/export_onnx.py` | 학습된 `.pt` → `risk_transformer.onnx` 변환 |
| `transformer/test_onnx.py` | ONNX 추론 결과 검증 |
| `transformer/models/` | 학습 산출물: `.onnx`(325KB) / `.pt` / `scaler.json` / `training_report.txt` |

## 성능 (테스트셋 2,523 시퀀스)

accuracy **99.3%**, macro F1 **0.898** — 상세 지표는 [`transformer/models/training_report.txt`](transformer/models/training_report.txt) 참고. Normal 클래스가 85.5%를 차지하는 불균형 데이터라 macro F1이 대표 지표입니다.

## 알려진 한계

현재 학습 데이터에서 보행자 위치·속도가 상수로 고정되어 있어, 움직이는 보행자 시나리오로 재학습하기 전까지 실기 운용에서는 AI 슬롯을 기본 OFF로 두고 rule + zone 경로가 안전 기능을 담당합니다 (`lux/predict_risk.py`의 폴백 설계 참고).
