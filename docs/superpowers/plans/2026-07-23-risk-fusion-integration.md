# 위험도 융합 통합 (Zone + AI 예측 + Rule) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 최민서 파트가 넘겨준 Transformer 예측(.onnx, 미래 risk_level 0~3 직접 출력)과 지도 zone 위험도를, 이미 완성된 강현준 실시간 rule 파이프라인(step7)에 `final_risk = max(rule, zone, predicted)` 구조로 합류시켜 지팡이까지 한 번에 흐르게 한다.

**Architecture:** 기존 단계 파일(step2~8)은 한 줄도 수정하지 않고 import만 하는 확립된 컨벤션을 그대로 따른다. 세 개의 순수 모듈(`zone_risk.py`, `predict_risk.py`, `fusion.py`)을 새로 만들고, 이들을 묶는 통합 실시간 러너(`step_fused_send.py`)가 step8의 전송·트러스트·heartbeat 로직을 재사용한다. 모든 위험원은 최종적으로 risk_level(0~3)로 통일된 뒤 max로 합쳐지고, reason에 어느 위험원이 이겼는지 기록한다.

**Tech Stack:** Python 3.12 표준 라이브러리(csv, json, math, collections.deque, dataclasses, unittest), 선택적 onnxruntime(import 가드로 없으면 자동 폴백), 선택적 pyserial(실기 전송 시).

## Global Constraints

- **기존 파일 불변**: step2~step9 및 그 test_*.py는 수정하지 않는다. 새 기능은 새 파일에서 import로만 얹는다. (프로젝트 확립 원칙 — 세션 요약 "단계마다 파일을 새로 만들고 이전 파일은 고치지 않는다")
- **모든 위험원은 risk_level 0~3으로 통일**한 뒤 융합한다. score(0~100)가 아니라 level로 max한다.
- **순수 함수/상태기계는 시리얼·시계 없이 테스트 가능**하게 분리한다. `now`는 인자로 주입한다. (step8 `RiskTransmitter` 패턴)
- **하드웨어 없이 검증**: 모든 러너는 `--stdin` 모드를 지원하고, 명령을 `[WIRE]`로 stderr에 찍는다.
- **의존성 최소화**: numpy/onnxruntime가 없어도 파이프라인이 죽지 않아야 한다. 없으면 예측은 "값 없음(None)"으로 폴백하고 rule+zone만으로 동작한다.
- **테스트 러너**: `python3 -m unittest discover -p "test_*.py"` 가 계속 초록이어야 한다(현재 85개 통과 → 태스크마다 증가).
- **작업 폴더**: `C:\Users\user\OneDrive\바탕 화면\goag\` (Windows 개발 사본) / `~/v2x/03_jetson` (Jetson 실기). 경로는 리포 루트 기준 상대.

---

## ⚠️ 통합을 막는 미확정 항목 (Task 0에서 반드시 먼저 닫는다)

이 세 가지가 확정되지 않으면 아래 코드 태스크는 "인터페이스는 맞지만 값이 틀린" 상태가 된다. 코드로 우회할 수 없고 **최민서와의 합의**가 필요하다.

1. **좌표계 불일치.** 실시간 파이프라인은 GPS 위경도(37.0/127.0) → ENU 평면(원점=첫 cane 좌표). 민서 `zone_definition.csv`는 SUMO 로컬좌표(center_x=3602.48 등 수천 단위). 그대로 붙이면 보행자가 **항상 OUT zone**으로 판정된다.
2. **AI 입력 feature 규격.** 사용자는 "AI엔 GPS 위경도를 넣겠다"고 했으나, 모델이 SUMO 로컬좌표·정규화된 feature로 학습됐다면 위경도(37/127 스케일)를 그대로 넣으면 예측이 무의미하다. onnx 입력 텐서의 **feature 이름·순서·윈도우 길이·정규화 방식**을 민서 학습 코드와 1:1로 맞춰야 한다.
3. **zone base_risk(0~5) → risk_level(0~3) 매핑.** zone csv의 base_risk는 2~5. 이를 융합용 level로 바꾸는 규칙을 확정한다.

---

## 전체 남은 로드맵 (요약)

통합(이 플랜)은 아래 로드맵의 **B 구간**이다. 앞뒤 맥락:

| 구간 | 내용 | 상태 | 담당 |
|---|---|---|---|
| A | 실시간 rule 파이프라인 step1~8 (수신→파싱→상태→거리/TTC/CPA/칼만→score→RSU 전송) | ✅ 완료 (테스트 85, Jetson 실기 7/23) | 강현준 |
| A' | SUMO 데이터셋·zone 정의·Transformer 학습·ONNX 변환 | ✅ 완료 | 최민서 |
| **B0** | **통합 규격 3종 합의 (좌표계 / onnx 입력 feature / zone level 매핑)** | ⏳ 이 플랜 Task 0 | 공동 |
| **B1** | **zone 판정 모듈 (zone_risk.py)** | ⏳ 이 플랜 Task 1 | 강현준 |
| **B2** | **AI 예측 추론 슬롯 (predict_risk.py) — 미래 RISK 직접** | ⏳ 이 플랜 Task 2 | 강현준 |
| **B3** | **max 융합 엔진 (fusion.py)** | ⏳ 이 플랜 Task 3 | 강현준 |
| **B4** | **통합 실시간 러너 (step_fused_send.py) — end-to-end** | ⏳ 이 플랜 Task 4 | 강현준 |
| C | 9단계: test vehicle → 실차 ESP32 신호 교체 | ⏭ 차량 ESP32 신호 대기 | 강현준+차량담당 |
| D | Jetson TensorRT 변환(.onnx 최적화), 실외 GPS 로그로 임계값(DCPA near/far, zone 반경) 재조정 | ⏭ 8월, Jetson·실외 데이터 후 | 공동 |
| E | 발표용 시각화 3종(rule vs 예측 / 궤적 / RISK 추이), 위험지도 자동 연동 | ⏭ | 최민서 중심 |

이 플랜은 B0~B4를 다룬다. C/D/E는 별도 플랜.

---

## File Structure

새로 만드는 파일 (리포 루트 = `goag/`):

- `zone_risk.py` — zone_definition.csv 로드 + 보행자 위치 → zone_base_risk → zone_level. detect 로직은 민서 `zone_detector.py`의 원형-반경 판정을 벤더(복사)하되, 좌표 변환 어댑터를 앞에 둔다.
- `predict_risk.py` — 과거 궤적 버퍼(`TrajectoryBuffer`) + 예측기 인터페이스(`RiskPredictor`). onnxruntime 있으면 `OnnxPredictor`, 없으면 `NullPredictor`(항상 None). 미래 risk_level 0~3을 직접 반환.
- `fusion.py` — `fuse_risk(rule_level, zone_level, predicted_level) -> FusedRisk(level, reason, sources)`.
- `step_fused_send.py` — 통합 실시간 러너. step6 `KinematicsPipeline`, step7 `assess_risk`, step8 `RiskTransmitter`/transport를 재사용해 rule+zone+predicted를 융합·전송·로그.
- `docs/zones_latlng.csv` (Task 0 산출물, 좌표계 합의 결과에 따라) — 위경도 기준 zone 정의. 좌표계 합의가 "위경도 통일"로 나면 생성.

각 파일마다 `test_<name>.py`.

---

### Task 0: 통합 규격 3종 합의 (코드 아님, 선행 필수)

**산출물:** `docs/integration_contract.md` — 아래 3개 표를 채운 합의 문서. 이 표의 값이 Task 1~4의 상수·매핑의 근거가 된다.

**Interfaces:**
- Produces: 좌표계 결정, onnx 입력 feature 명세, zone level 매핑표. Task 1(zone 매핑), Task 2(onnx feature/shape), Task 4(좌표 변환)가 이 문서를 참조한다.

- [ ] **Step 1: 민서에게 확인할 3개 질문을 문서로 정리**

`docs/integration_contract.md` 생성, 아래 골격:

```markdown
# 통합 규격 합의 (강현준 ↔ 최민서)

## 1. 좌표계
- [ ] zone 판정 기준 좌표계: (위경도 / SUMO로컬) 중 택1
- [ ] 결정 시 변환식: SUMO(x,y) ↔ 위경도(lat,lng) 매핑 파라미터 (원점, 스케일)
- [ ] zone_definition.csv를 위경도로 재발행할지 여부

## 2. onnx 입력 feature (모델 input 텐서)
- [ ] 입력 shape: (window, n_features) = (?, ?)
- [ ] feature 순서/이름: 예) [ped_x, ped_y, veh_x, veh_y, ped_speed, veh_speed, distance, rel_speed]
- [ ] 좌표계/단위/정규화: (raw / min-max / z-score, 파라미터 값)
- [ ] window 길이(프레임 수)와 샘플링 주기(Hz)
- [ ] 출력: 미래 risk_level 0~3 (확정됨). 몇 초 선행(1s/2s)인지, 스칼라인지 로짓 벡터인지

## 3. zone base_risk(0~5) → risk_level(0~3) 매핑
- [ ] 매핑표 확정 (아래 잠정안 검토)
```

- [ ] **Step 2: zone level 매핑 잠정안을 문서에 기입 (민서 검토용)**

```markdown
| base_risk | risk_level | 근거 |
|---|---|---|
| 5 (주차장 출구) | 3 | 시야 차단 + 상시 차량 진출 |
| 4 (중문/blind spot) | 2 | 건물 사각 |
| 3 (정문) | 1 | 보행 밀집 |
| 2 (학생회관) | 1 | 보행 밀집 |
| 0 (OUT) | 0 | 등록 구역 밖 |
```

- [ ] **Step 3: onnx 파일과 메타데이터 요청 항목 명시**

문서에 "민서에게 받을 것" 체크리스트: `model.onnx` 파일, 학습 시 feature 추출 스크립트(전처리 정규화 파라미터 포함), 샘플 입력/출력 한 쌍(회귀 테스트 고정용).

- [ ] **Step 4: 커밋**

```bash
git add docs/integration_contract.md
git commit -m "docs: integration contract skeleton for zone/onnx/coord"
```

**완료 기준:** 3개 표의 빈칸이 민서 답변으로 채워지고, `model.onnx` + 샘플 입출력 한 쌍을 확보. (이 완료 없이 Task 2의 OnnxPredictor는 인터페이스만 맞는 stub 상태로 남는다 — 그래도 Task 1/3/4는 진행 가능.)

---

### Task 1: zone 판정 모듈 (`zone_risk.py`)

**Files:**
- Create: `zone_risk.py`
- Test: `test_zone_risk.py`
- Read: `tmp/AI-V2X-Cane-audit/scripts/zone_detector.py` (벤더 원본), `tmp/AI-V2X-Cane-audit/zones/zone_definition.csv` (스키마)

**Interfaces:**
- Consumes: 보행자 위치 `(x, y)` — 좌표계는 Task 0 결정에 따름. zone csv 스키마: `zone_id,zone_name,zone_type,center_x,center_y,radius_m,base_risk,speed_limit,description`.
- Produces:
  - `load_zones(csv_path) -> list[dict]`
  - `detect_zone(x, y, zones) -> dict`  (zone_id, zone_base_risk 포함; OUT이면 base_risk 0)
  - `BASE_RISK_TO_LEVEL: dict[int,int]` (Task 0 매핑표)
  - `zone_level(x, y, zones) -> int`  (0~3). Task 3이 소비.

- [ ] **Step 1: 실패 테스트 작성**

`test_zone_risk.py`:

```python
import unittest
from pathlib import Path
import tempfile

from zone_risk import load_zones, detect_zone, zone_level, BASE_RISK_TO_LEVEL

ZONE_CSV = (
    "zone_id,zone_name,zone_type,center_x,center_y,radius_m,base_risk,speed_limit,description\n"
    "Z04,Parking Exit,Parking,100.0,100.0,30,5,10,exit\n"
    "Z03,Student Center,Pedestrian Area,300.0,300.0,30,2,10,center\n"
)


def _write(tmp):
    p = Path(tmp) / "zones.csv"
    p.write_text(ZONE_CSV, encoding="utf-8-sig")
    return p


class ZoneRiskTest(unittest.TestCase):
    def test_inside_zone_returns_base_risk(self):
        with tempfile.TemporaryDirectory() as tmp:
            zones = load_zones(_write(tmp))
            hit = detect_zone(105.0, 105.0, zones)  # within 30m of Z04
            self.assertEqual(hit["zone_id"], "Z04")
            self.assertEqual(hit["zone_base_risk"], 5)

    def test_outside_all_zones_is_out(self):
        with tempfile.TemporaryDirectory() as tmp:
            zones = load_zones(_write(tmp))
            hit = detect_zone(1000.0, 1000.0, zones)
            self.assertEqual(hit["zone_id"], "OUT")
            self.assertEqual(hit["zone_base_risk"], 0)

    def test_zone_level_maps_base_risk(self):
        with tempfile.TemporaryDirectory() as tmp:
            zones = load_zones(_write(tmp))
            self.assertEqual(zone_level(105.0, 105.0, zones), BASE_RISK_TO_LEVEL[5])
            self.assertEqual(zone_level(1000.0, 1000.0, zones), 0)

    def test_nearest_zone_wins_on_overlap(self):
        with tempfile.TemporaryDirectory() as tmp:
            zones = load_zones(_write(tmp))
            # equidistant-ish point closer to Z03
            hit = detect_zone(295.0, 295.0, zones)
            self.assertEqual(hit["zone_id"], "Z03")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `python3 -m unittest test_zone_risk -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'zone_risk'`

- [ ] **Step 3: `zone_risk.py` 구현**

```python
#!/usr/bin/env python3
"""Map a pedestrian position to a static hazard-zone risk level (0-3).

The circular-zone detection is vendored verbatim from the team's
tmp/AI-V2X-Cane-audit/scripts/zone_detector.py so the two stay consistent; the
audit folder is throwaway and importing across it is fragile. This module adds
two things the realtime pipeline needs and the audit script does not: an
explicit base_risk(0-5) -> risk_level(0-3) table, and a single zone_level()
entry point for the fusion stage.

Coordinate frame is whatever docs/integration_contract.md settles on. Callers
must hand in (x, y) already in the same frame as the zone csv's center_x/center_y.
"""

import csv
import math
from pathlib import Path


# base_risk (0-5, from the team zone csv) -> fusion risk_level (0-3).
# Provisional; confirm against docs/integration_contract.md section 3.
BASE_RISK_TO_LEVEL = {0: 0, 1: 0, 2: 1, 3: 1, 4: 2, 5: 3}


def load_zones(csv_path):
    zones = []
    with open(csv_path, "r", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            zones.append(
                {
                    "zone_id": row["zone_id"],
                    "zone_name": row["zone_name"],
                    "zone_type": row["zone_type"],
                    "center_x": float(row["center_x"]),
                    "center_y": float(row["center_y"]),
                    "radius_m": float(row["radius_m"]),
                    "base_risk": int(row["base_risk"]),
                }
            )
    return zones


def _distance(x1, y1, x2, y2):
    return math.hypot(x1 - x2, y1 - y2)


def detect_zone(x, y, zones):
    """Nearest zone whose radius contains (x, y); OUT sentinel if none."""
    nearest = None
    nearest_distance = float("inf")
    for zone in zones:
        d = _distance(x, y, zone["center_x"], zone["center_y"])
        if d <= zone["radius_m"] and d < nearest_distance:
            nearest, nearest_distance = zone, d
    if nearest is None:
        return {"zone_id": "OUT", "zone_base_risk": 0, "zone_distance_m": None}
    return {
        "zone_id": nearest["zone_id"],
        "zone_base_risk": nearest["base_risk"],
        "zone_distance_m": round(nearest_distance, 2),
    }


def zone_level(x, y, zones):
    """Static zone risk as a 0-3 level for the fusion stage."""
    base = detect_zone(x, y, zones)["zone_base_risk"]
    return BASE_RISK_TO_LEVEL.get(base, 0)
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `python3 -m unittest test_zone_risk -v`
Expected: PASS (4 tests)

- [ ] **Step 5: 커밋**

```bash
git add zone_risk.py test_zone_risk.py
git commit -m "feat: zone_risk module (circular zone detection + base_risk->level)"
```

---

### Task 2: AI 예측 추론 슬롯 (`predict_risk.py`)

**Files:**
- Create: `predict_risk.py`
- Test: `test_predict_risk.py`

**Interfaces:**
- Consumes: 과거 궤적 샘플의 시퀀스. 한 샘플 = `dict`(최소 `ped_x, ped_y, veh_x, veh_y, ped_speed, veh_speed, distance_m, rel_speed`; 최종 feature 목록은 Task 0 확정). 좌표계/정규화도 Task 0.
- Produces:
  - `TrajectoryBuffer(window)` — `.append(sample: dict)`, `.ready() -> bool`, `.features() -> list[list[float]]`
  - `RiskPredictor` (ABC): `.predict(buffer) -> int|None`  (미래 risk_level 0~3, 값 없으면 None)
  - `NullPredictor` — 항상 None (onnxruntime/모델 없음 폴백)
  - `OnnxPredictor(model_path, feature_order, window)` — onnxruntime 추론, 출력이 로짓이면 argmax, 스칼라면 round·clamp
  - `make_predictor(model_path=None, ...) -> RiskPredictor` — 팩토리(가드)

- [ ] **Step 1: 실패 테스트 작성 (모델 없이 인터페이스 계약만 검증)**

`test_predict_risk.py`:

```python
import unittest

from predict_risk import TrajectoryBuffer, NullPredictor, make_predictor, RiskPredictor


def _sample(d):
    return {"ped_x": 0.0, "ped_y": 0.0, "veh_x": d, "veh_y": 0.0,
            "ped_speed": 0.0, "veh_speed": 5.0, "distance_m": d, "rel_speed": 5.0}


class TrajectoryBufferTest(unittest.TestCase):
    def test_not_ready_until_window_filled(self):
        buf = TrajectoryBuffer(window=3)
        self.assertFalse(buf.ready())
        for d in (30.0, 25.0):
            buf.append(_sample(d))
        self.assertFalse(buf.ready())
        buf.append(_sample(20.0))
        self.assertTrue(buf.ready())

    def test_features_are_window_by_nfeatures(self):
        buf = TrajectoryBuffer(window=2, feature_order=["distance_m", "rel_speed"])
        buf.append(_sample(30.0))
        buf.append(_sample(25.0))
        feats = buf.features()
        self.assertEqual(len(feats), 2)
        self.assertEqual(feats[0], [30.0, 5.0])


class PredictorContractTest(unittest.TestCase):
    def test_null_predictor_returns_none(self):
        buf = TrajectoryBuffer(window=1)
        buf.append(_sample(10.0))
        self.assertIsNone(NullPredictor().predict(buf))

    def test_factory_falls_back_to_null_without_model(self):
        pred = make_predictor(model_path=None)
        self.assertIsInstance(pred, RiskPredictor)
        buf = TrajectoryBuffer(window=1)
        buf.append(_sample(10.0))
        self.assertIsNone(pred.predict(buf))

    def test_predict_returns_none_when_buffer_not_ready(self):
        pred = make_predictor(model_path=None)
        self.assertIsNone(pred.predict(TrajectoryBuffer(window=5)))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `python3 -m unittest test_predict_risk -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'predict_risk'`

- [ ] **Step 3: `predict_risk.py` 구현**

```python
#!/usr/bin/env python3
"""Future-risk prediction slot for the fusion stage.

Minseo's Transformer is exported to ONNX and outputs a future risk_level (0-3)
directly (confirmed 2026-07-23). This module is the adapter that feeds a recent
trajectory window into that model and returns the level, or None when there is
nothing to say (model absent, or not enough history yet).

The exact feature list, coordinate frame and normalisation must match Minseo's
training pipeline -- see docs/integration_contract.md section 2. They are passed
in (feature_order, plus any scaling done by the caller) so this file does not
hard-code a guess. Without onnxruntime or a model file the factory returns a
NullPredictor and the whole pipeline degrades to rule+zone gracefully.
"""

from abc import ABC, abstractmethod
from collections import deque

# Confirmed with Minseo: match training window/features in the contract.
DEFAULT_WINDOW = 10  # ~1 s at 10 Hz
DEFAULT_FEATURE_ORDER = [
    "ped_x", "ped_y", "veh_x", "veh_y",
    "ped_speed", "veh_speed", "distance_m", "rel_speed",
]


class TrajectoryBuffer:
    """Rolling window of recent samples, newest last."""

    def __init__(self, window=DEFAULT_WINDOW, feature_order=None):
        self.window = window
        self.feature_order = feature_order or DEFAULT_FEATURE_ORDER
        self._samples = deque(maxlen=window)

    def append(self, sample):
        self._samples.append(sample)

    def ready(self):
        return len(self._samples) >= self.window

    def features(self):
        """window x n_features float matrix in feature_order."""
        return [
            [float(sample.get(name, 0.0)) for name in self.feature_order]
            for sample in self._samples
        ]


class RiskPredictor(ABC):
    @abstractmethod
    def predict(self, buffer):
        """Future risk_level 0-3, or None when unavailable."""


class NullPredictor(RiskPredictor):
    """Used when onnxruntime or the model file is missing."""

    def predict(self, buffer):
        return None


class OnnxPredictor(RiskPredictor):
    """Runs Minseo's exported model. Output is a future risk_level 0-3.

    Handles both output shapes: a length-4 logit/prob vector (argmax) or a
    single scalar (round + clamp). Which one it is comes from the contract; the
    code tolerates either so a shape surprise fails soft, not hard.
    """

    def __init__(self, session, feature_order, window):
        self._session = session
        self._input_name = session.get_inputs()[0].name
        self.feature_order = feature_order
        self.window = window

    def predict(self, buffer):
        if not buffer.ready():
            return None
        import numpy as np

        matrix = np.asarray(buffer.features(), dtype=np.float32)  # (window, n_feat)
        batch = matrix[np.newaxis, :, :]  # (1, window, n_feat)
        outputs = self._session.run(None, {self._input_name: batch})
        arr = np.asarray(outputs[0]).reshape(-1)
        if arr.size >= 4:
            level = int(arr.argmax())
        else:
            level = int(round(float(arr[0])))
        return max(0, min(3, level))


def make_predictor(model_path=None, feature_order=None, window=DEFAULT_WINDOW):
    """Factory with an onnxruntime guard.

    Returns a working predictor only when both onnxruntime and a model file are
    present; otherwise NullPredictor, so callers never branch on availability.
    """
    if not model_path:
        return NullPredictor()
    try:
        import onnxruntime as ort
    except ImportError:
        print("[WARN] onnxruntime not installed; prediction disabled")
        return NullPredictor()
    from pathlib import Path

    if not Path(model_path).exists():
        print(f"[WARN] model not found: {model_path}; prediction disabled")
        return NullPredictor()
    session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
    return OnnxPredictor(session, feature_order or DEFAULT_FEATURE_ORDER, window)
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `python3 -m unittest test_predict_risk -v`
Expected: PASS (5 tests). onnxruntime 미설치 환경에서도 통과(팩토리가 NullPredictor 폴백).

- [ ] **Step 5: 커밋**

```bash
git add predict_risk.py test_predict_risk.py
git commit -m "feat: predict_risk onnx inference slot (future risk 0-3, null fallback)"
```

---

### Task 3: max 융합 엔진 (`fusion.py`)

**Files:**
- Create: `fusion.py`
- Test: `test_fusion.py`

**Interfaces:**
- Consumes: `rule_level: int`(step7 risk_level), `zone_level: int`(Task 1), `predicted_level: int|None`(Task 2).
- Produces:
  - `FusedRisk(level: int, reason: str, sources: dict)` (frozen dataclass)
  - `fuse_risk(rule_level, zone_level, predicted_level) -> FusedRisk`

- [ ] **Step 1: 실패 테스트 작성**

`test_fusion.py`:

```python
import unittest

from fusion import fuse_risk, FusedRisk


class FusionTest(unittest.TestCase):
    def test_max_wins(self):
        fused = fuse_risk(rule_level=1, zone_level=3, predicted_level=2)
        self.assertEqual(fused.level, 3)
        self.assertEqual(fused.reason, "zone")

    def test_none_prediction_is_ignored(self):
        fused = fuse_risk(rule_level=2, zone_level=0, predicted_level=None)
        self.assertEqual(fused.level, 2)
        self.assertEqual(fused.reason, "rule")

    def test_all_zero_is_zero_and_reason_none(self):
        fused = fuse_risk(rule_level=0, zone_level=0, predicted_level=0)
        self.assertEqual(fused.level, 0)
        self.assertEqual(fused.reason, "none")

    def test_tie_lists_all_winners_in_priority_order(self):
        fused = fuse_risk(rule_level=2, zone_level=2, predicted_level=0)
        self.assertEqual(fused.level, 2)
        # rule listed before zone before predicted on a tie
        self.assertEqual(fused.reason, "rule+zone")

    def test_sources_recorded(self):
        fused = fuse_risk(rule_level=1, zone_level=2, predicted_level=3)
        self.assertEqual(
            fused.sources, {"rule": 1, "zone": 2, "predicted": 3}
        )


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `python3 -m unittest test_fusion -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'fusion'`

- [ ] **Step 3: `fusion.py` 구현**

```python
#!/usr/bin/env python3
"""Combine the three risk sources into one level, safety-first.

final = max(rule, zone, predicted). The highest wins so a miss on any single
path cannot silence the warning (the team's agreed structure). reason records
which source(s) produced the winning level, in priority order rule > zone >
predicted, so a log line explains itself. A None prediction (model absent or
buffer not ready) simply drops out of the max.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class FusedRisk:
    level: int
    reason: str
    sources: dict


def fuse_risk(rule_level, zone_level, predicted_level):
    sources = {"rule": rule_level, "zone": zone_level}
    if predicted_level is not None:
        sources["predicted"] = predicted_level

    level = max(sources.values())
    if level == 0:
        return FusedRisk(level=0, reason="none", sources=sources)

    order = ("rule", "zone", "predicted")
    winners = [name for name in order if sources.get(name) == level]
    return FusedRisk(level=level, reason="+".join(winners), sources=sources)
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `python3 -m unittest test_fusion -v`
Expected: PASS (5 tests)

- [ ] **Step 5: 커밋**

```bash
git add fusion.py test_fusion.py
git commit -m "feat: fusion engine max(rule,zone,predicted) with reason"
```

---

### Task 4: 통합 실시간 러너 (`step_fused_send.py`)

step6 파이프라인 + step7 rule 채점 + Task1 zone + Task2 예측 + Task3 융합 + step8 전송을 한 흐름으로 연결한다. step7/step8을 재사용하고 수정하지 않는다.

**설계 결정 (중복 제거):** step7 `assess_risk`는 `zone_base_risk` 인자를 "점수 가점"으로 쓴다. 융합에서 zone을 level로 max하므로 **가점을 이중 계산하지 않도록 `assess_risk(..., zone_base_risk=0)`로 호출**한다. zone은 오직 Task1 `zone_level`을 통해 융합 단계에서만 반영된다.

**Files:**
- Create: `step_fused_send.py`
- Test: `test_step_fused_send.py`
- Read (import): `step6_kinematics.py`, `step7_risk.py`, `step8_send_risk.py`, `step3_parse_v2x.py`, `step4_state_store.py`, `step5_test_vehicle.py`, `zone_risk.py`, `predict_risk.py`, `fusion.py`

**Interfaces:**
- Consumes: `KinematicsPipeline.compute() -> (now, raw, filtered)`; `assess_risk(filtered, vehicle_speed, zone_base_risk=0) -> RiskAssessment`; `RiskTransmitter.consider(level, cane_gps_valid, now) -> TxDecision`; `zone_level(x,y,zones)`; `make_predictor(...)`, `TrajectoryBuffer`; `fuse_risk(...)`.
- Produces: `FusedRiskSender` (step8 `RiskSender`와 동형이나 융합 삽입), CSV `step_fused_log.csv` (fields: pc_time, cane_seq, vehicle_seq, rule_level, zone_level, predicted_level, final_level, reason, effective_level, trusted, tx_reason).

- [ ] **Step 1: 실패 테스트 작성 (stdin/stdout, 하드웨어 없이 end-to-end)**

`test_step_fused_send.py`:

```python
import io
import json
import unittest
from contextlib import redirect_stderr

from step4_state_store import StateStore
from step6_kinematics import KinematicsPipeline
from step8_send_risk import RiskTransmitter
from zone_risk import load_zones
from predict_risk import make_predictor, TrajectoryBuffer
from step_fused_send import FusedRiskSender


def _cane(seq, lat=37.0, lng=127.0, t=1000.0):
    return {"type": "cane", "seq": seq, "gps_valid": 1, "lat": lat, "lng": lng,
            "speed_mps": 0.0, "heading_deg": 0.0, "pc_time": t}


def _vehicle(seq, lat, lng, speed, t):
    return {"type": "vehicle", "seq": seq, "gps_valid": 1, "lat": lat, "lng": lng,
            "speed_mps": speed, "heading_deg": 180.0, "pc_time": t}


class FusedSenderTest(unittest.TestCase):
    def _make_sender(self, commands, zones=None):
        store = StateStore(fresh_window_s=10.0)
        pipeline = KinematicsPipeline(store)
        transmitter = RiskTransmitter(allow_untrusted=True)
        return FusedRiskSender(
            pipeline=pipeline,
            transmitter=transmitter,
            transport=lambda c: commands.append(c),
            csv_path=None,               # None => skip CSV in tests
            gate_params={},
            zones=zones or [],
            predictor=make_predictor(model_path=None),   # NullPredictor
            buffer=TrajectoryBuffer(window=1),
        )

    def test_rule_only_flow_transmits_on_approach(self):
        commands = []
        sender = self._make_sender(commands)
        # cane first (anchors frame), then a close approaching vehicle
        sender.process_line(json.dumps(_cane(1, t=1000.0)), "test")
        sender.process_line(json.dumps(_vehicle(1, 37.00003, 127.0, 5.0, 1000.1)), "test")
        # a nonzero risk command should have gone out (target_id/risk json)
        self.assertTrue(commands, "expected at least one downlink command")
        payload = json.loads(commands[-1])
        self.assertIn("risk", payload)

    def test_zone_raises_final_level(self):
        # A zone covering the cane origin (0,0 in ENU) forces zone_level high
        zones = [{"zone_id": "Z", "zone_name": "z", "zone_type": "t",
                  "center_x": 0.0, "center_y": 0.0, "radius_m": 50.0,
                  "base_risk": 5}]
        commands = []
        sender = self._make_sender(commands, zones=zones)
        with redirect_stderr(io.StringIO()):
            sender.process_line(json.dumps(_cane(1, t=1000.0)), "test")
            sender.process_line(json.dumps(_vehicle(1, 37.0009, 127.0, 0.0, 1000.1)), "test")
        # even with a far/idle vehicle, zone base_risk 5 -> level 3 must dominate
        payload = json.loads(commands[-1])
        self.assertEqual(payload["risk"], 3)


if __name__ == "__main__":
    unittest.main()
```

> 주: zone 판정에 넘길 보행자 좌표는 러너가 ENU로 넘긴다(테스트의 zone center도 ENU 0,0). Task 0에서 좌표계가 "위경도 통일"로 확정되면 러너의 좌표 변환과 이 테스트의 center 값을 그에 맞게 바꾼다.

- [ ] **Step 2: 테스트 실패 확인**

Run: `python3 -m unittest test_step_fused_send -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'step_fused_send'`

- [ ] **Step 3: `step_fused_send.py` 구현**

```python
#!/usr/bin/env python3
"""End-to-end fused risk: rule (step7) + zone (zone_risk) + prediction
(predict_risk), combined by max (fusion) and transmitted through step8's
downlink policy. step2-8 are imported, not modified.

zone is deliberately fed to assess_risk as zone_base_risk=0 so it is not double
counted: the static zone contribution enters only once, as a level, in the
fusion max. The pedestrian position handed to the zone detector and the feature
window handed to the predictor use the frame agreed in
docs/integration_contract.md; until that lands, both use the pipeline's ENU frame.
"""

import argparse
import csv
import json
import sys
import time
from pathlib import Path

from step3_parse_v2x import SOURCE_MODES, normalize_record
from step4_state_store import FRESH_WINDOW_S, StateStore, has_position
from step5_test_vehicle import SPEED_MPS, START_DISTANCE_M, TestVehicle
from step6_kinematics import KinematicsPipeline, to_float
from step7_risk import DCPA_FAR_M, DCPA_FLOOR, DCPA_NEAR_M, assess_risk
from step8_send_risk import (
    HEARTBEAT_S,
    RSU_ACK_TYPES,
    RiskTransmitter,
    serial_transport,
    stdout_transport,
)
from zone_risk import load_zones, zone_level
from predict_risk import TrajectoryBuffer, make_predictor
from fusion import fuse_risk


CSV_FIELDS = (
    "pc_time", "cane_seq", "vehicle_seq",
    "rule_level", "zone_level", "predicted_level", "final_level", "reason",
    "effective_level", "trusted", "tx_reason",
)


def append_row(csv_path, row):
    path = Path(csv_path)
    needs_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        if needs_header:
            writer.writeheader()
        writer.writerow(row)


class FusedRiskSender:
    """RiskSender (step8) with zone + prediction folded into the level."""

    def __init__(self, pipeline, transmitter, transport, csv_path, gate_params,
                 zones, predictor, buffer):
        self.pipeline = pipeline
        self.transmitter = transmitter
        self.transport = transport
        self.csv_path = csv_path
        self.gate_params = gate_params
        self.zones = zones
        self.predictor = predictor
        self.buffer = buffer

    def process_line(self, raw_line, source_mode):
        line = raw_line.strip()
        if not line:
            return
        try:
            payload = json.loads(line)
            if not isinstance(payload, dict):
                raise ValueError("top-level JSON must be an object")
            row = normalize_record(payload, source_mode)
        except (json.JSONDecodeError, ValueError) as exc:
            print(f"[WARN] parse_failed error={exc} source={source_mode}", file=sys.stderr)
            return

        if row["type"] in RSU_ACK_TYPES:
            return
        if not self.pipeline.store.update(row):
            print(f"[WARN] ignored_type type={row['type']!r}", file=sys.stderr)
            return

        self.pipeline.observe(row)
        result = self.pipeline.compute()
        if result is None:
            return

        now, _raw, filtered = result
        store = self.pipeline.store
        vehicle_speed = to_float(store.latest["vehicle"]["speed_mps"])

        # 1. rule level (zone_base_risk=0: zone enters via fusion, not as a bonus)
        assessment = assess_risk(filtered, vehicle_speed, zone_base_risk=0, **self.gate_params)
        rule_level = assessment.risk_level

        # 2. zone level from the cane position in the pipeline's ENU frame
        cane = store.latest["cane"]
        ped_xy = self.pipeline.frame.to_enu(to_float(cane["lat"]), to_float(cane["lng"]))
        z_level = zone_level(ped_xy[0], ped_xy[1], self.zones) if self.zones else 0

        # 3. predicted level (None until model + full window)
        self.buffer.append(self._sample(store, filtered, ped_xy))
        p_level = self.predictor.predict(self.buffer)

        # 4. fuse
        fused = fuse_risk(rule_level, z_level, p_level)

        # 5. transmit through step8's trust + heartbeat policy
        cane_gps_valid = cane["gps_valid"]
        decision = self.transmitter.consider(fused.level, cane_gps_valid, now)
        if decision.should_send:
            self.transport(self.transmitter.command(decision.effective_level))
            print(
                f"[FUSED] level={fused.level} reason={fused.reason} "
                f"(rule={rule_level} zone={z_level} pred={p_level}) "
                f"tx={decision.effective_level}/{decision.reason}",
                flush=True,
            )
        if self.csv_path:
            append_row(self.csv_path, {
                "pc_time": round(now, 3),
                "cane_seq": store.latest["cane"]["seq"],
                "vehicle_seq": store.latest["vehicle"]["seq"],
                "rule_level": rule_level,
                "zone_level": z_level,
                "predicted_level": "" if p_level is None else p_level,
                "final_level": fused.level,
                "reason": fused.reason,
                "effective_level": decision.effective_level,
                "trusted": int(decision.trusted),
                "tx_reason": decision.reason,
            })

    def _sample(self, store, filtered, ped_xy):
        veh = store.latest["vehicle"]
        veh_xy = self.pipeline.frame.to_enu(to_float(veh["lat"]), to_float(veh["lng"]))
        return {
            "ped_x": ped_xy[0], "ped_y": ped_xy[1],
            "veh_x": veh_xy[0], "veh_y": veh_xy[1],
            "ped_speed": to_float(store.latest["cane"]["speed_mps"]),
            "veh_speed": to_float(veh["speed_mps"]),
            "distance_m": filtered.distance_m,
            "rel_speed": filtered.closing_los,
        }


def inject_vehicle(vehicle, sender, source_mode, now):
    cane = sender.pipeline.store.latest.get("cane")
    if cane is None or not has_position(cane) or not vehicle.is_due(now):
        return
    payload, distance_m = vehicle.record(to_float(cane["lat"]), to_float(cane["lng"]), now)
    print(f"[TESTVEH] seq={payload['seq']} distance_m={distance_m:.1f}", flush=True)
    sender.process_line(json.dumps(payload), source_mode)


def parse_args():
    parser = argparse.ArgumentParser(description="Fused (rule+zone+ai) realtime risk sender.")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--source-mode", choices=SOURCE_MODES, default="test")
    parser.add_argument("--csv", default="step_fused_log.csv")
    parser.add_argument("--fresh-window-ms", type=int, default=int(FRESH_WINDOW_S * 1000))
    parser.add_argument("--stdin", action="store_true")
    parser.add_argument("--test-vehicle", action="store_true")
    parser.add_argument("--vehicle-speed", type=float, default=SPEED_MPS)
    parser.add_argument("--vehicle-start-m", type=float, default=START_DISTANCE_M)
    parser.add_argument("--target-id", type=int, default=0)
    parser.add_argument("--tx-heartbeat-s", type=float, default=HEARTBEAT_S)
    parser.add_argument("--tx-untrusted", action="store_true")
    parser.add_argument("--zones-csv", default=None, help="zone_definition.csv path (optional)")
    parser.add_argument("--model", default=None, help=".onnx model path (optional)")
    parser.add_argument("--predict-window", type=int, default=10)
    parser.add_argument("--dcpa-near-m", type=float, default=DCPA_NEAR_M)
    parser.add_argument("--dcpa-far-m", type=float, default=DCPA_FAR_M)
    parser.add_argument("--dcpa-floor", type=float, default=DCPA_FLOOR)
    return parser.parse_args()


def main():
    args = parse_args()
    store = StateStore(fresh_window_s=args.fresh_window_ms / 1000)
    pipeline = KinematicsPipeline(store)
    transmitter = RiskTransmitter(
        target_id=args.target_id, heartbeat_s=args.tx_heartbeat_s,
        allow_untrusted=args.tx_untrusted,
    )
    gate_params = {"near_m": args.dcpa_near_m, "far_m": args.dcpa_far_m, "floor": args.dcpa_floor}
    zones = load_zones(args.zones_csv) if args.zones_csv else []
    predictor = make_predictor(model_path=args.model, window=args.predict_window)
    buffer = TrajectoryBuffer(window=args.predict_window)

    vehicle = TestVehicle(start_distance_m=args.vehicle_start_m, speed_mps=args.vehicle_speed) \
        if args.test_vehicle else None

    if args.stdin:
        sender = FusedRiskSender(pipeline, transmitter, stdout_transport, args.csv,
                                 gate_params, zones, predictor, buffer)
        _run(sender, sys.stdin, args.source_mode, vehicle)
        return

    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial required for serial mode: pip3 install pyserial") from exc

    with serial.Serial(args.port, args.baud, timeout=1) as connection:
        connection.reset_input_buffer()
        sender = FusedRiskSender(pipeline, transmitter, serial_transport(connection),
                                 args.csv, gate_params, zones, predictor, buffer)
        _run(sender, _serial_lines(connection), args.source_mode, vehicle)


def _serial_lines(connection):
    while True:
        raw = connection.readline()
        if raw:
            yield raw.decode("utf-8", errors="replace")


def _run(sender, lines, source_mode, vehicle):
    try:
        for line in lines:
            sender.process_line(line, source_mode)
            if vehicle is not None:
                inject_vehicle(vehicle, sender, source_mode, time.time())
    except KeyboardInterrupt:
        print("\n[INFO] stopped", file=sys.stderr)


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `python3 -m unittest test_step_fused_send -v`
Expected: PASS (2 tests)

- [ ] **Step 5: 전체 회귀 확인 (기존 85개 + 신규 안 깨짐)**

Run: `python3 -m unittest discover -p "test_*.py"`
Expected: OK (기존 85 + zone 4 + predict 5 + fusion 5 + fused 2 = 101)

- [ ] **Step 6: 인프로세스 스모크 (rule+zone, 모델 없이)**

Run:
```bash
printf '%s\n' \
  '{"type":"cane","seq":1,"gps_valid":1,"lat":37.0,"lng":127.0,"speed_mps":0,"heading_deg":0,"pc_time":1000.0}' \
  '{"type":"vehicle","seq":1,"gps_valid":1,"lat":37.00003,"lng":127.0,"speed_mps":5,"heading_deg":180,"pc_time":1000.2}' \
  | python3 step_fused_send.py --stdin --source-mode test --tx-untrusted --csv /dev/null
```
Expected: stderr에 `[WIRE] {"target_id":0,"risk":N}` (N>=1), stdout에 `[FUSED] level=... reason=rule ...`

- [ ] **Step 7: 커밋**

```bash
git add step_fused_send.py test_step_fused_send.py
git commit -m "feat: fused realtime sender (rule+zone+ai max, reuses step6-8)"
```

---

## Self-Review

**1. 스펙 커버리지 (회의에서 정한 강현준 통합 할 일 vs 태스크):**
- "zone 판정 함수 추가" → Task 1 ✅
- "max 병합 로직 (reason에 셋 다 기록)" → Task 3 ✅
- ".onnx 추론 슬롯 (빈 함수 먼저)" → Task 2 ✅ (NullPredictor 폴백 = 빈 슬롯, 모델 오면 OnnxPredictor)
- "입력 연결 (시리얼 수신)" → Task 4가 step6 파이프라인 재사용, 시리얼/stdin 모두 지원 ✅
- "confirm/cancel 송신" → step8 `RiskTransmitter`(change/heartbeat/trust=0으로 해제) 재사용 ✅
- 좌표계·onnx feature·zone 매핑 미확정 → Task 0으로 명시 ✅

**2. 플레이스홀더 스캔:** 각 코드 스텝에 완전한 코드/테스트 포함. Task 0만 합의 문서라 코드가 없으나, 이는 인간 합의 태스크로 성격이 다름을 명시. "TODO/추후" 문구는 Task 0 산출물(계약 문서)로 수렴시킴.

**3. 타입 일관성 확인:**
- `zone_level(x,y,zones) -> int` (Task1) → Task4에서 `z_level`로 소비 ✅
- `RiskPredictor.predict(buffer) -> int|None` (Task2) → Task4 `p_level`, `fuse_risk`가 None 허용 ✅
- `fuse_risk(rule, zone, predicted) -> FusedRisk(level, reason, sources)` (Task3) → Task4에서 `.level`/`.reason` 사용 ✅
- `assess_risk(filtered, vehicle_speed, zone_base_risk=0, **gate_params) -> RiskAssessment(.risk_level)` (기존 step7) → Task4에서 정확히 호출, zone 이중계산 방지 위해 `zone_base_risk=0` 명시 ✅
- `RiskTransmitter.consider(level, cane_gps_valid, now) -> TxDecision(.should_send/.effective_level/.reason)` (기존 step8) → Task4 사용 ✅
- `KinematicsPipeline.frame.to_enu(lat,lng) -> (e,n)` (기존 step6) → Task4 zone/predict 좌표 ✅

**주의로 남기는 리스크:**
- Task 4 테스트 `test_zone_raises_final_level`은 zone center를 ENU (0,0)로 둔 것으로, Task 0에서 좌표계가 위경도로 확정되면 러너와 테스트를 함께 갱신해야 한다 (테스트 주석에 명시).
- OnnxPredictor의 실제 정확도는 Task 0의 feature/정규화 합의 없이는 검증 불가 — 모델+샘플 입출력 확보 후 회귀 테스트 1건 추가 권장.
