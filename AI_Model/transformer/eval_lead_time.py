# -*- coding: utf-8 -*-
"""
v3 선행 예측 종합 평가 — "실제 미래와 비교"

평가 시나리오(학습에 미사용 60개)의 차량별로:
  1. 실제 위험 발생 시각: 현재 라벨(risk_level)이 처음 2 이상이 된 순간
  2. v3 경고 시각: 모델(과거 10초 입력만)이 처음 L2+ 예측한 순간
  3. 물리 기준선 경고 시각: 물리+DCPA가 처음 L2+ 예측한 순간
  -> 선행 시간 = 실제 위험 시각 - 경고 시각 (양수 = 미리 경고)
"""
import random
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import onnxruntime as ort

sys.stdout.reconfigure(encoding="utf-8")
HERE = Path(__file__).parent
from train_transformer_v3 import FEATURE_COLUMNS, SEQ_LEN, SEED, TEST_SCENARIO_RATIO
import json

def gate(d):
    if d <= 2.5: return 1.0
    if d >= 7.5: return 0.2
    return 1.0 + (d - 2.5) / 5.0 * (0.2 - 1.0)

def classify(s):
    return 3 if s >= 70 else 2 if s >= 45 else 1 if s >= 20 else 0

df = pd.read_csv(HERE / "training_dataset_v3.csv")
random.seed(SEED)
sc = sorted(df["scenario_id"].unique()); random.shuffle(sc)
test = df[df["scenario_id"].isin(set(sc[:max(1, int(len(sc) * TEST_SCENARIO_RATIO))]))]

s = json.load(open(HERE / "models_v3" / "scaler_v3.json"))
mean = np.array(s["mean"], np.float32); scale = np.array(s["scale"], np.float32)
sess = ort.InferenceSession(str(HERE / "models_v3" / "risk_transformer_v3.onnx"))

leads_ai, leads_phys = [], []
ai_caught = phys_caught = n_danger = 0
fp_ai = fp_phys = n_safe = 0

for (sc_id, vid), g in test.groupby(["scenario_id", "vehicle_id"], sort=False):
    if len(g) < SEQ_LEN:
        continue
    g = g.sort_values("timestep_time").reset_index(drop=True)
    t = g["timestep_time"].to_numpy()
    now = g["risk_level"].to_numpy()

    f = ((g[FEATURE_COLUMNS].to_numpy(np.float32) - mean) / scale)
    seqs = np.stack([f[i - SEQ_LEN + 1:i + 1] for i in range(SEQ_LEN - 1, len(g))])
    pred = sess.run(None, {"input": seqs})[0].argmax(1)  # t[9:] 에 대응
    phys = np.array([classify(sv * gate(dv)) for sv, dv in
                     zip(g["phys_score_3s"], g["dcpa_m"])])[SEQ_LEN - 1:]

    covered_now = now[SEQ_LEN - 1:]
    danger_idx = np.where(covered_now >= 2)[0]
    if len(danger_idx) == 0:
        n_safe += 1
        if (pred >= 2).any(): fp_ai += 1
        if (phys >= 2).any(): fp_phys += 1
        continue

    n_danger += 1
    t_danger = t[SEQ_LEN - 1 + danger_idx[0]]
    ai_idx = np.where(pred >= 2)[0]
    if len(ai_idx) and t[SEQ_LEN - 1 + ai_idx[0]] <= t_danger:
        ai_caught += 1
        leads_ai.append(t_danger - t[SEQ_LEN - 1 + ai_idx[0]])
    ph_idx = np.where(phys >= 2)[0]
    if len(ph_idx) and t[SEQ_LEN - 1 + ph_idx[0]] <= t_danger:
        phys_caught += 1
        leads_phys.append(t_danger - t[SEQ_LEN - 1 + ph_idx[0]])

print(f"실제 위험(L2+) 차량: {n_danger}대 / 안전 차량: {n_safe}대\n")
print(f"[하이브리드 AI (v3)]")
print(f"  위험 발생 전(또는 동시) 경고 성공: {ai_caught}/{n_danger}대 ({ai_caught/n_danger*100:.1f}%)")
if leads_ai:
    la = np.array(leads_ai)
    print(f"  선행 시간: 평균 {la.mean():.2f}초 / 중앙값 {np.median(la):.1f}초 / 3초 이상 먼저 {np.mean(la>=3)*100:.0f}%")
print(f"  안전 차량 오경보: {fp_ai}/{n_safe}대 ({fp_ai/n_safe*100:.1f}%)\n")
print(f"[물리 외삽 기준선]")
print(f"  위험 발생 전 경고 성공: {phys_caught}/{n_danger}대 ({phys_caught/n_danger*100:.1f}%)")
if leads_phys:
    lp = np.array(leads_phys)
    print(f"  선행 시간: 평균 {lp.mean():.2f}초 / 중앙값 {np.median(lp):.1f}초")
print(f"  안전 차량 오경보: {fp_phys}/{n_safe}대 ({fp_phys/n_safe*100:.1f}%)")
