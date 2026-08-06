# -*- coding: utf-8 -*-
"""
Transformer v3 학습 — 3초 선행 예측 (하이브리드)

v2와의 차이:
  - 목표(정답)가 "현재 위험"이 아니라 "향후 3초 내 최대 위험" (risk_level_future3)
  - 특징 19개 = v2의 16개 + 물리 외삽 3개 (phys_min_dist_3s, phys_t_cpa, phys_score_3s)
    -> 물리 계산 결과를 힌트로 주고, 물리가 못 잡는 패턴을 모델이 보정하는 구조
"""
import json
import random
import time
from pathlib import Path

import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from sklearn.metrics import classification_report, confusion_matrix
from torch.utils.data import DataLoader, Dataset

SEED = 42
HERE = Path(__file__).parent
DATA_FILE = HERE / "training_dataset_v3.csv"
OUT_DIR = HERE / "models_v3"

FEATURE_COLUMNS = [
    "ped_x", "ped_y", "veh_x", "veh_y",
    "ped_speed_mps", "veh_speed_mps",
    "distance_m", "rel_speed_mps", "ttc",
    "risk_score", "zone_base_risk",
    "dx", "dy", "dvx", "dvy", "dcpa_m",
    "phys_min_dist_3s", "phys_t_cpa", "phys_score_3s",
]
TARGET = "risk_level_future3"
SEQ_LEN = 10
BATCH_SIZE = 128
EPOCHS = 12
LR = 0.001
TEST_SCENARIO_RATIO = 0.15
L0_CAP = {"train": 250_000, "test": 60_000}


class RiskTransformer(nn.Module):
    def __init__(self, num_features, num_classes=4, d_model=64,
                 nhead=4, num_layers=2):
        super().__init__()
        self.input_projection = nn.Linear(num_features, d_model)
        layer = nn.TransformerEncoderLayer(
            d_model=d_model, nhead=nhead, dim_feedforward=128,
            dropout=0.1, batch_first=True)
        self.encoder = nn.TransformerEncoder(layer, num_layers=num_layers)
        self.classifier = nn.Sequential(
            nn.LayerNorm(d_model), nn.Linear(d_model, num_classes))

    def forward(self, x):
        x = self.input_projection(x)
        x = self.encoder(x)
        return self.classifier(x[:, -1, :])


class SeqDataset(Dataset):
    def __init__(self, x, y):
        self.x = torch.tensor(x, dtype=torch.float32)
        self.y = torch.tensor(y, dtype=torch.long)

    def __len__(self):
        return len(self.y)

    def __getitem__(self, i):
        return self.x[i], self.y[i]


def make_sequences(df, mean, scale):
    xs, ys = [], []
    feats = ((df[FEATURE_COLUMNS].to_numpy(np.float32) - mean) / scale)
    labels = df[TARGET].to_numpy()
    for _, idx in df.groupby(["scenario_id", "vehicle_id"],
                             sort=False).indices.items():
        idx = np.sort(idx)
        f, l = feats[idx], labels[idx]
        for end in range(SEQ_LEN - 1, len(idx)):
            xs.append(f[end - SEQ_LEN + 1:end + 1])
            ys.append(l[end])
    return np.asarray(xs, np.float32), np.asarray(ys, np.int64)


def downsample_l0(x, y, cap):
    idx0 = np.where(y == 0)[0]
    rest = np.where(y != 0)[0]
    if len(idx0) > cap:
        idx0 = np.random.choice(idx0, cap, replace=False)
    keep = np.sort(np.concatenate([idx0, rest]))
    return x[keep], y[keep]


def main():
    random.seed(SEED); np.random.seed(SEED); torch.manual_seed(SEED)
    OUT_DIR.mkdir(exist_ok=True)
    print("1. 데이터 로드...")
    df = pd.read_csv(DATA_FILE)
    print(f"   {len(df):,}행, 시나리오 {df['scenario_id'].nunique()}개")

    scenarios = sorted(df["scenario_id"].unique())
    random.shuffle(scenarios)
    n_test = max(1, int(len(scenarios) * TEST_SCENARIO_RATIO))
    test_set = set(scenarios[:n_test])
    train_df = df[~df["scenario_id"].isin(test_set)]
    test_df = df[df["scenario_id"].isin(test_set)]

    mean = train_df[FEATURE_COLUMNS].mean().to_numpy(np.float32)
    scale = train_df[FEATURE_COLUMNS].std().to_numpy(np.float32)
    scale = np.where(scale == 0, 1.0, scale).astype(np.float32)

    print("2. 시퀀스 생성...")
    x_tr, y_tr = make_sequences(train_df, mean, scale)
    x_te, y_te = make_sequences(test_df, mean, scale)
    x_tr, y_tr = downsample_l0(x_tr, y_tr, L0_CAP["train"])
    x_te, y_te = downsample_l0(x_te, y_te, L0_CAP["test"])
    print(f"   학습 {len(y_tr):,} / 평가 {len(y_te):,}")
    print(f"   학습 라벨: {np.bincount(y_tr, minlength=4).tolist()}")

    counts = np.bincount(y_tr, minlength=4)
    weights = torch.tensor(counts.sum() / (4 * np.maximum(counts, 1)),
                           dtype=torch.float32)

    model = RiskTransformer(len(FEATURE_COLUMNS))
    print(f"3. 학습 ({sum(p.numel() for p in model.parameters()):,} 파라미터)...")
    loader = DataLoader(SeqDataset(x_tr, y_tr), batch_size=BATCH_SIZE,
                        shuffle=True)
    opt = torch.optim.Adam(model.parameters(), lr=LR)
    crit = nn.CrossEntropyLoss(weight=weights)
    for ep in range(1, EPOCHS + 1):
        model.train(); t0 = time.time(); tot = cor = 0; ls = 0.0
        for xb, yb in loader:
            opt.zero_grad()
            out = model(xb); loss = crit(out, yb)
            loss.backward(); opt.step()
            ls += loss.item() * len(yb)
            cor += (out.argmax(1) == yb).sum().item(); tot += len(yb)
        print(f"   epoch {ep:2d}/{EPOCHS}  loss {ls/tot:.4f}  "
              f"acc {cor/tot:.4f}  ({time.time()-t0:.0f}s)", flush=True)

    print("4. 평가 (미래 3초 라벨 기준)...")
    model.eval(); preds = []
    with torch.no_grad():
        for i in range(0, len(x_te), 4096):
            preds.append(model(torch.tensor(x_te[i:i+4096])).argmax(1).numpy())
    preds = np.concatenate(preds)
    report = classification_report(y_te, preds, labels=[0, 1, 2, 3],
                                   target_names=["안전0", "주의1", "경고2", "위험3"],
                                   digits=4)
    cm = confusion_matrix(y_te, preds, labels=[0, 1, 2, 3])
    print(report); print(cm)

    print("5. 저장·ONNX 변환...")
    torch.save(model.state_dict(), OUT_DIR / "risk_transformer_v3.pt")
    (OUT_DIR / "scaler_v3.json").write_text(json.dumps({
        "feature_columns": FEATURE_COLUMNS,
        "mean": [float(v) for v in mean],
        "scale": [float(v) for v in scale],
        "sequence_length": SEQ_LEN,
        "predicts": "max risk level within next 3 seconds",
    }, indent=2), encoding="utf-8")

    dummy = torch.randn(1, SEQ_LEN, len(FEATURE_COLUMNS))
    torch.onnx.export(model, dummy, OUT_DIR / "risk_transformer_v3.onnx",
                      input_names=["input"], output_names=["logits"],
                      dynamic_axes={"input": {0: "batch"},
                                    "logits": {0: "batch"}},
                      opset_version=17, dynamo=False)
    import onnxruntime as ort
    sess = ort.InferenceSession(str(OUT_DIR / "risk_transformer_v3.onnx"))
    sample = x_te[:256]
    with torch.no_grad():
        t_out = model(torch.tensor(sample)).numpy()
    diff = float(np.abs(t_out - sess.run(None, {"input": sample})[0]).max())
    print(f"   ONNX 오차 {diff:.2e} ({'OK' if diff < 1e-4 else 'FAIL'})")

    (OUT_DIR / "training_report_v3.txt").write_text(
        f"target: {TARGET} (3s lookahead)\nfeatures({len(FEATURE_COLUMNS)})\n"
        f"train {len(y_tr):,} / test {len(y_te):,} (scenario split {n_test})\n\n"
        f"{report}\n{cm}\nONNX diff {diff:.2e}\n", encoding="utf-8")
    print("완료:", OUT_DIR)


if __name__ == "__main__":
    main()
