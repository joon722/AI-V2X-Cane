#!/usr/bin/env python3
"""step9: AI 3초 선행 예측을 실시간 위험 판정에 병합 — 초안 (리뷰용)

역할
----
step7(규칙 판정)과 step8(지팡이 회신) 사이에 AI 항을 추가한다:

    final_level = max(step7 규칙 판정,  AI의 "3초 내 예상 위험")

- AI는 경고를 앞당기거나 강화만 할 수 있고, 규칙이 올린 경고를 내릴 수 없다
  (규칙 안전망 유지 — 안전 우선 병합).
- 모델은 auto_pipeline의 Transformer(v4 가중치, 19개 특징, 시퀀스 10초)를
  그대로 사용한다. SUMO 배치 파이프라인과 동일 모델 = 검증 결과 공유.
- 워밍업 패딩: 차량 등장 직후 히스토리가 10초 미만이면 첫 관측을 반복해
  창을 채워 즉시 예측을 시작한다 (기존 "첫 9초 공백" 해소).

의존성: onnxruntime(설치됨), numpy(설치됨), pyproj(pip install pyproj 필요)

step8 통합 예시 (강현준님 확인 후 적용 — 3줄)
---------------------------------------------
    from step9_ai_risk import AiRiskPredictor
    ai = AiRiskPredictor()                       # 시작 시 1회
    ...
    # 매 판정 주기에서 (assess_risk 직후):
    ai_level = ai.update(vehicle_id, cane_state, veh_state,
                         assessment.base_score, now)
    effective_level = max(assessment.level, ai_level)   # ← 기존 level 대신 전달

미해결 확인 사항 (팀 논의 필요)
------------------------------
1. cane_state/veh_state 인터페이스: 아래 update()는 (lat, lng, speed_mps)를
   가진 객체를 가정 — step6 NodeTracker의 칼만 필터 통과 상태를 넘기는 것을
   권장 (GPS 노이즈 완화). 실제 필드명에 맞춰 조정 필요.
2. 호출 주기: 모델은 1초 간격 학습이므로 update()는 초당 1회만 반영한다
   (내부에서 1초 미만 중복 호출은 무시). 10Hz 루프에서 그대로 불러도 안전.
3. step8 히스테리시스(LevelStabilizer)는 max() 결과에 그대로 적용하면 됨.
"""
import collections
import json
import math
import time
from pathlib import Path

import numpy as np

try:
    import onnxruntime as ort
except ImportError:  # 개발 PC 등 미설치 환경
    ort = None

try:
    from pyproj import Proj
    _UTM52 = Proj(proj="utm", zone=52, ellps="WGS84")
except ImportError:
    _UTM52 = None

# SUMO 지도(net_v2)의 투영 정보 — 학습 데이터와 같은 좌표계로 변환하기 위함
_NET_OFFSET = (-315516.76, -4150401.46)

_AUTO = Path(__file__).resolve().parent / "auto_pipeline"
MODEL_PATH = _AUTO / "risk_transformer_v3.onnx"   # 파일명 v3, 가중치는 최신(v4)
SCALER_PATH = _AUTO / "scaler_v3.json"

SEQ_LEN = 10
HORIZON_S = 3.0
ZONE_BASE_RISK = 0.0  # Zone 위험은 별도 항으로 병합됨 (RISK-03)


def latlng_to_sumo_xy(lat, lng):
    """위경도 -> SUMO 지도 좌표 (학습 데이터와 동일한 절대 좌표계)."""
    if _UTM52 is None:
        raise RuntimeError("pyproj가 필요합니다: pip install pyproj")
    e, n = _UTM52(lng, lat)
    return e + _NET_OFFSET[0], n + _NET_OFFSET[1]


class AiRiskPredictor:
    """차량별 최근 10초 특징 창을 유지하며 '3초 내 예상 위험'을 출력한다."""

    STALE_S = 5.0  # 이 시간 넘게 관측이 없으면 차량 히스토리 리셋

    def __init__(self, model_path=MODEL_PATH, scaler_path=SCALER_PATH):
        scaler = json.loads(Path(scaler_path).read_text(encoding="utf-8"))
        self.cols = scaler["feature_columns"]
        self.mean = np.array(scaler["mean"], dtype=np.float32)
        scale = np.array(scaler["scale"], dtype=np.float32)
        self.scale = np.where(scale == 0, 1.0, scale).astype(np.float32)
        if ort is None:
            raise RuntimeError("onnxruntime가 필요합니다")
        self.sess = ort.InferenceSession(
            str(model_path), providers=["CPUExecutionProvider"])
        self._in = self.sess.get_inputs()[0].name
        # vehicle_id -> deque of 특징 행 (최대 10개)
        self._hist = {}
        # vehicle_id -> (마지막 반영 시각, 직전 상대좌표)
        self._meta = {}
        # vehicle_id -> 마지막 AI 판정 (1초 미만 재호출 시 반환)
        self._last = {}

    # ── 특징 계산 (auto_pipeline/process_scenarios.py와 동일 정의) ──
    def _features(self, vid, ped_xy, ped_speed, veh_xy, veh_speed, base_score,
                  now):
        px, py = ped_xy
        vx, vy = veh_xy
        rx, ry = vx - px, vy - py
        dist = math.hypot(rx, ry)

        last_t, last_rel = self._meta.get(vid, (None, None))
        if last_rel is None:
            dvx = dvy = 0.0
            rel_speed = 0.0
        else:
            dt = max(now - last_t, 1e-3)
            dvx = (rx - last_rel[0]) / dt
            dvy = (ry - last_rel[1]) / dt
            last_dist = math.hypot(*last_rel)
            rel_speed = (last_dist - dist) / dt
        self._meta[vid] = (now, (rx, ry))

        ttc = dist / rel_speed if rel_speed > 0 else 9999.0

        # DCPA (최근접 예상 거리)
        v2 = dvx * dvx + dvy * dvy
        if v2 > 1e-6:
            t_cpa = -(rx * dvx + ry * dvy) / v2
        else:
            t_cpa = -1.0
        if t_cpa > 0:
            dcpa = math.hypot(rx + dvx * t_cpa, ry + dvy * t_cpa)
        else:
            dcpa = dist

        # 물리 외삽 (3초 내 최소 거리/시점 + 그 시점의 채점표 점수)
        t_hit = min(max(t_cpa, 0.0), HORIZON_S)
        phys_dist = math.hypot(rx + dvx * t_hit, ry + dvy * t_hit)
        # base_score는 step7이 이미 계산한 채점표 점수를 재사용한다.
        phys_score = base_score  # 초안: 근사. 정밀 버전은 미래 상태로 재채점.

        return [px, py, vx, vy, ped_speed, veh_speed,
                dist, rel_speed, ttc, base_score, ZONE_BASE_RISK,
                rx, ry, dvx, dvy, dcpa,
                phys_dist, max(t_hit, 0.05), phys_score]

    def update(self, vehicle_id, cane_state, veh_state, base_score,
               now=None):
        """관측 1회 반영 후 AI 예상 위험(0~3)을 반환.

        cane_state / veh_state: .lat, .lng, .speed_mps 를 가진 객체
        base_score: step7의 채점표 점수 (assessment.base_score)
        """
        now = time.time() if now is None else now

        hist = self._hist.get(vehicle_id)
        last_t, _ = self._meta.get(vehicle_id, (None, None))
        if hist is None or (last_t and now - last_t > self.STALE_S):
            hist = collections.deque(maxlen=SEQ_LEN)
            self._hist[vehicle_id] = hist
            self._meta.pop(vehicle_id, None)
        # 1초 간격으로만 반영 (모델 학습 간격과 일치)
        if last_t is not None and now - last_t < 0.95:
            return self._last_level(vehicle_id)

        ped_xy = latlng_to_sumo_xy(cane_state.lat, cane_state.lng)
        veh_xy = latlng_to_sumo_xy(veh_state.lat, veh_state.lng)
        row = self._features(vehicle_id, ped_xy,
                             float(getattr(cane_state, "speed_mps", 0.0)),
                             veh_xy,
                             float(getattr(veh_state, "speed_mps", 0.0)),
                             float(base_score), now)
        hist.append(row)

        # 워밍업 패딩: 첫 관측을 반복해 10초 창을 즉시 확보
        window = list(hist)
        while len(window) < SEQ_LEN:
            window.insert(0, window[0])

        x = (np.asarray([window], dtype=np.float32) - self.mean) / self.scale
        logits = self.sess.run(None, {self._in: x})[0][0]
        level = int(np.argmax(logits))
        self._last[vehicle_id] = level
        return level

    def _last_level(self, vehicle_id):
        return self._last.get(vehicle_id, 0)


if __name__ == "__main__":
    # 간단 동작 확인: 정면 접근 차량 시뮬레이션
    class _S:
        def __init__(self, lat, lng, speed):
            self.lat, self.lng, self.speed_mps = lat, lng, speed

    ai = AiRiskPredictor()
    cane = _S(37.4956, 126.9547, 1.2)
    print("정면 접근 시뮬레이션 (60m -> 3m):")
    for k in range(12):
        d = 60 - k * 5
        veh = _S(37.4956 + d / 111320.0, 126.9547, 8.0)
        lv = ai.update("test_car", cane, veh,
                       base_score=30 if d < 40 else 10, now=1000.0 + k)
        print(f"  t={k:2d}s 거리={d:2d}m -> AI 예상 위험: {lv}")
