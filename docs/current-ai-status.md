# Current AI operating status / 현재 AI 운용 상태

Reviewed: 2026-09-28. Latest field session examined: 2026-09-19.

## What is implemented / 구현 상태

**AI contributes to field alerts. / AI가 현장 경보에 기여합니다.**

The field runner is [`step8_send_risk.py`](../rsu/v2x/03_jetson/step8_send_risk.py). It loads [`ModelGate`](../rsu/v2x/03_jetson/model_gate.py) unless `--no-model` is set. The [deployment wrapper](../rsu/v2x/03_jetson/deploy/run_v2x_risk_engine.sh) disables it only when `V2X_NO_MODEL=1`. Device-specific environment settings can override the default; this is not a live check of the hardware.

현장 step8은 기본적으로 모델을 적재합니다. `--no-model` 또는 배포 래퍼의 `V2X_NO_MODEL=1`은 비활성화 설정입니다. 소스의 기본값과 실제 장비의 환경 설정은 구별해야 합니다.

[`TreeEnsemble`](../rsu/v2x/03_jetson/model_runtime.py) performs inference on exported gradient-boosted trees using Python's standard library. `ModelGate` uses 15 trajectory features, raises a rule level below 2 to level 2 when the score meets the model threshold, and never lowers a rule-generated level. Missing/failed models fall back to rules. Shared sensor errors remain possible.

15개 궤적 특징을 학습된 트리 앙상블에 입력하고, 문턱을 넘으면 규칙 등급을 2로 상향합니다. 기존 규칙 등급을 낮추지 않으며 모델 오류 시 규칙으로 복귀합니다. 두 판단이 공유하는 센서 입력 오류까지 해결하는 보장은 아닙니다.

## Direct field evidence / 공개 로그의 직접 근거

Source: [`risk_tx_20260919_095416.csv`](../data/젯슨로그_20260919/risk_tx_20260919_095416.csv), with [session conditions and limitations](../data/젯슨로그_20260919/README.md).

| Measure / 집계 | Value |
| --- | ---: |
| Valid transmission rows / 유효 송신 기록 | 11,951 |
| `level_source=model` rows | 39 |
| Of those: `reason=change` | 9 |
| Of those: `reason=heartbeat` | 30 |
| Model-score range in those rows | 0.9099–0.9779 |

These rows demonstrate that model decisions entered the transmission path. They do **not** establish 39 independent hazard detections, successful user responses, or prevented accidents. Even change-triggered rows are not independently labeled events. The session includes controlled input degradation and approach trials; see its README before interpreting alert correctness.

모델 판정이 송신 경로에 반영됐다는 근거입니다. **39개의 독립 위험 검출·사용자 대응 성공·사고 예방을 의미하지 않습니다.** 변화 송신 9행도 정답 라벨이 붙은 독립 사건 수는 아닙니다. 입력 열화 시험과 접근 주행을 포함하므로 해당 세션 설명과 함께 해석해야 합니다.

Run this read-only count from the repository root (the raw file has a trailing NUL-only line):

```python
import csv
from collections import Counter
from pathlib import Path

p = Path('data/젯슨로그_20260919/risk_tx_20260919_095416.csv')
with p.open(encoding='utf-8-sig', newline='') as f:
    rows = [r for r in csv.DictReader(f)
            if r.get('level_source') in {'table', 'model'}]
model_rows = [r for r in rows if r['level_source'] == 'model']
print(len(rows), len(model_rows))             # 11951 39
print(Counter(r['reason'] for r in model_rows))  # heartbeat: 30, change: 9
```

## Model identity / 모델 버전 구분

Both artifacts below are in the repository. They are different models; **do not replace the default silently** when reproducing a recorded session.

| Artifact | Trees | Threshold | Metadata: training scenarios |
| --- | ---: | ---: | ---: |
| [Default `risk_model.json`](../rsu/v2x/03_jetson/risk_model.json) | 91 | 0.1400672321608068 | 1,200 |
| [v2 export `risk_model_streams_12k.json`](../rsu/v2x/03_jetson/auto_pipeline/risk_model_streams_12k.json) | 200 | 0.9078839313447887 | 8,400 |

SHA-256 (file bytes):

```text
Default: 8c7aa76de51781013dbaf7b10684da977b5d1181d0ade6a3047963b9e5c8a42d
v2:      26fd2d25da26285ddb07f4aa44798d0f7e84cc01e37d47442c04636d94d635b5
```

The v2 export matched the locally retained September 19 device-model snapshot byte for byte during this review. That device snapshot is separate from the public TX CSV; the CSV alone does not encode a model fingerprint. For a reproducible run, record the commit, explicit `--model` path or installed artifact hash, environment flags, and matching session log.

이번 점검에서 v2 export와 로컬에 보관된 9월 19일 장치 모델 스냅샷의 바이트 지문이 일치했습니다. 이 장치 스냅샷과 공개 TX CSV는 별도 근거이며, CSV 자체에는 모델 지문이 없습니다. 재현 시 코드 커밋·모델 경로/해시·실행 환경 설정·세션 로그를 함께 기록해야 합니다.

The v2 metadata describes simulated scenarios with field-like noise passed through the runtime pipeline (`sources=["scenario_sim"]`). This is not evidence of training on labeled real collisions. The filename `12k` is not the training-count metric; the artifact metadata reports 8,400 training scenarios.

v2는 현장과 유사한 잡음을 적용한 합성 시나리오를 실행 파이프라인에 통과시켜 학습한 모델입니다. 실제 사고 정답 데이터로 학습했다는 의미가 아닙니다. 학습 시나리오 수는 파일명의 `12k`가 아니라 메타데이터의 8,400개를 기준으로 합니다.

## Separate Transformer results / 별도 Transformer 결과

[`AI_Model/`](../AI_Model/) and [`lux/predict_risk.py`](../lux/predict_risk.py) contain a separate Transformer/ONNX route. The [existing training report](../AI_Model/transformer/models/training_report.txt) reports accuracy 0.9929 and macro F1 0.8978 on 2,523 sequences, including only six Near Miss examples. These metrics must not be assigned to the field tree model or presented as real-road safety accuracy. No distillation relationship between the Transformer and tree artifacts was established by this review.

Transformer와 현장 트리는 별도 경로입니다. Transformer의 99.3%를 현장 경보 정확도로 설명하거나, 트리가 Transformer를 증류한 모델이라고 단정하지 않습니다.

The field-tree [dataset builder](../rsu/v2x/03_jetson/auto_pipeline/build_dataset_from_streams.py) distinguishes the evaluation window `[t, t+3]` from the lead-shifted training window `[t+2, t+5]`. A prediction-window definition is not a guarantee that every real hazard is detected three seconds early.

트리 데이터빌더는 평가 창 `[t,t+3]`과 학습 창 `[t+2,t+5]`를 구분합니다. 이를 현장 최소 3초 선행 경보 보장으로 표현하지 않습니다.

## Scope of this update / 수정 범위

This update corrects documentation only. It changes neither the deployed device nor runtime code, model artifacts or configuration. Historical August field aggregates remain dated separately from September AI-use evidence. Event-level field accuracy, missed warnings, unnecessary warnings and end-to-end actuation latency require their own measurements.

문서만 수정했으며 장비·런타임 코드·모델·설정을 변경하지 않았습니다. 8월 집계와 9월 AI 사용 근거는 시점을 구분했습니다. 사건 단위 현장 검출률·미경보·불필요 경보·실제 경고 출력까지의 지연은 별도 측정이 필요합니다.
