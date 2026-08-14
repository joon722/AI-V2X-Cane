# -*- coding: utf-8 -*-
"""v4.1 vs v5 비교 평가 — 동일한 교내(v3 생성기) 테스트 시나리오에서.

테스트 셋 = train_v5.py와 동일 분할(seed 42, 15%)의 45개 시나리오 (학습 미사용).
두 모델 모두 각자의 scaler로 정규화해 ONNX로 추론한다.
"""
import json
import random
import sys
from pathlib import Path

import numpy as np
import onnxruntime as ort
import pandas as pd
from sklearn.metrics import classification_report, confusion_matrix

sys.stdout.reconfigure(encoding="utf-8")
HERE = Path(__file__).parent
SEQ_LEN = 10
TARGET = "risk_level_future3"

df = pd.read_csv(HERE / "training_dataset_v5.csv")
scenarios = sorted(df["scenario_id"].unique())
random.seed(42)
random.shuffle(scenarios)
n_test = max(1, int(len(scenarios) * 0.15))
test_df = df[df["scenario_id"].isin(set(scenarios[:n_test]))]
print(f"테스트: 시나리오 {n_test}개, {len(test_df):,}행")


def evaluate(tag, model_dir):
    scaler = json.loads((model_dir / "scaler_v3.json").read_text(encoding="utf-8"))
    cols = scaler["feature_columns"]
    mean = np.array(scaler["mean"], np.float32)
    scale = np.array(scaler["scale"], np.float32)
    feats = (test_df[cols].to_numpy(np.float32) - mean) / scale
    labels = test_df[TARGET].to_numpy()
    xs, ys = [], []
    for _, idx in test_df.groupby(["scenario_id", "vehicle_id"],
                                  sort=False).indices.items():
        idx = np.sort(idx)
        f, l = feats[idx], labels[idx]
        for end in range(SEQ_LEN - 1, len(idx)):
            xs.append(f[end - SEQ_LEN + 1:end + 1])
            ys.append(l[end])
    x = np.asarray(xs, np.float32)
    y = np.asarray(ys, np.int64)
    sess = ort.InferenceSession(str(model_dir / "risk_transformer_v3.onnx"))
    preds = []
    for i in range(0, len(x), 4096):
        preds.append(sess.run(None, {"input": x[i:i + 4096]})[0].argmax(1))
    p = np.concatenate(preds)
    acc = (p == y).mean()
    cm = confusion_matrix(y, p, labels=[0, 1, 2, 3])
    l3 = y == 3
    l2p = y >= 2
    print(f"\n===== {tag} =====")
    print(f"정확도: {acc*100:.1f}%")
    print(f"미래 L3 재현율: {(p[l3]>=2).mean()*100:.1f}% (경고 이상으로 잡음)"
          f" / 정확 L3: {(p[l3]==3).mean()*100:.1f}%")
    print(f"미래 L2+ 재현율: {(p[l2p]>=2).mean()*100:.1f}%")
    print(f"L3를 안전(L0)으로 놓침: {(p[l3]==0).mean()*100:.2f}%")
    print(f"안전(L0) 오경보율(L2+ 판정): {(p[y==0]>=2).mean()*100:.1f}%")
    print(confusion_matrix(y, p, labels=[0, 1, 2, 3]))
    return acc


evaluate("v4.1 (옛 데이터로 학습, TTC 수정판)", HERE / "models_v4")
evaluate("v5 (진짜 교내 주행 데이터로 재학습)", HERE / "models_v5")
