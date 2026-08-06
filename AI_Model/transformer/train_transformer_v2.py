# -*- coding: utf-8 -*-
"""
Transformer 위험 예측 모델 v2 학습

v1(train_transformer.py) 대비 개선:
  1. 특징 16개 — 기존 11개(보행자 실제 좌표) + 벡터 5개(dx, dy, dvx, dvy, dcpa_m)
  2. 시퀀스를 (시나리오, 차량) 단위로 생성 — 서로 다른 차량이 한 시퀀스에 섞이지 않음
  3. 학습/평가를 '시나리오' 단위로 분리 — 겹치는 창(window)으로 인한 평가 누수 제거
  4. 안전(L0) 시퀀스 다운샘플링 + 클래스 가중치로 불균형 완화
  5. 학습 후 ONNX 변환과 PyTorch-ONNX 출력 일치 검증까지 한 번에

출력: models_v2/risk_transformer_v2.pt / .onnx / scaler_v2.json / training_report_v2.txt
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
DATA_FILE = Path(__file__).parent / "training_dataset_v2.csv"
OUT_DIR = Path(__file__).parent / "models_v2"

FEATURE_COLUMNS = [
    "ped_x", "ped_y", "veh_x", "veh_y",
    "ped_speed_mps", "veh_speed_mps",
    "distance_m", "rel_speed_mps", "ttc",
    "risk_score", "zone_base_risk",
    "dx", "dy", "dvx", "dvy", "dcpa_m",
]
TARGET = "risk_level"
SEQ_LEN = 10
BATCH_SIZE = 128
EPOCHS = 12
LR = 0.001
TEST_SCENARIO_RATIO = 0.15
L0_MAX_PER_SPLIT = {"train": 250_000, "test": 60_000}


def set_seed(s):
    random.seed(s); np.random.seed(s); torch.manual_seed(s)


class RiskTransformer(nn.Module):
    """v1과 동일 구조 (입력 차원만 16으로 확장)"""
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
    """(시나리오, 차량) 그룹 안에서만 슬라이딩 윈도우 생성"""
    xs, ys = [], []
    feats = ((df[FEATURE_COLUMNS].to_numpy(np.float32) - mean) / scale)
    labels = df[TARGET].to_numpy()
    starts = df.groupby(["scenario_id", "vehicle_id"], sort=False).indices
    for _, idx in starts.items():
        idx = np.sort(idx)
        f, l = feats[idx], labels[idx]
        for end in range(SEQ_LEN - 1, len(idx)):
            xs.append(f[end - SEQ_LEN + 1:end + 1])
            ys.append(l[end])
    return np.asarray(xs, np.float32), np.asarray(ys, np.int64)


def downsample_l0(x, y, cap):
    idx0 = np.where(y == 0)[0]
    idx_rest = np.where(y != 0)[0]
    if len(idx0) > cap:
        idx0 = np.random.choice(idx0, cap, replace=False)
    keep = np.sort(np.concatenate([idx0, idx_rest]))
    return x[keep], y[keep]


def main():
    set_seed(SEED)
    OUT_DIR.mkdir(exist_ok=True)
    print("1. 데이터 로드...")
    df = pd.read_csv(DATA_FILE)
    print(f"   {len(df):,}행, 시나리오 {df['scenario_id'].nunique()}개")

    # 시나리오 단위 학습/평가 분리 (누수 방지)
    scenarios = sorted(df["scenario_id"].unique())
    random.shuffle(scenarios)
    n_test = max(1, int(len(scenarios) * TEST_SCENARIO_RATIO))
    test_set = set(scenarios[:n_test])
    train_df = df[~df["scenario_id"].isin(test_set)]
    test_df = df[df["scenario_id"].isin(test_set)]
    print(f"   학습 시나리오 {len(scenarios)-n_test} / 평가 {n_test}")

    # scaler는 학습 데이터로만 fit
    mean = train_df[FEATURE_COLUMNS].mean().to_numpy(np.float32)
    scale = train_df[FEATURE_COLUMNS].std().to_numpy(np.float32)
    scale = np.where(scale == 0, 1.0, scale).astype(np.float32)

    print("2. 시퀀스 생성...")
    x_tr, y_tr = make_sequences(train_df, mean, scale)
    x_te, y_te = make_sequences(test_df, mean, scale)
    x_tr, y_tr = downsample_l0(x_tr, y_tr, L0_MAX_PER_SPLIT["train"])
    x_te, y_te = downsample_l0(x_te, y_te, L0_MAX_PER_SPLIT["test"])
    print(f"   학습 {len(y_tr):,}개 / 평가 {len(y_te):,}개")
    print(f"   학습 라벨: {np.bincount(y_tr, minlength=4).tolist()}")
    print(f"   평가 라벨: {np.bincount(y_te, minlength=4).tolist()}")

    counts = np.bincount(y_tr, minlength=4)
    weights = torch.tensor(counts.sum() / (4 * np.maximum(counts, 1)),
                           dtype=torch.float32)
    print(f"   클래스 가중치: {weights.numpy().round(2).tolist()}")

    model = RiskTransformer(len(FEATURE_COLUMNS))
    n_params = sum(p.numel() for p in model.parameters())
    print(f"3. 학습 시작 (파라미터 {n_params:,}개, {EPOCHS} epoch)...")
    loader = DataLoader(SeqDataset(x_tr, y_tr), batch_size=BATCH_SIZE,
                        shuffle=True, num_workers=0)
    optimizer = torch.optim.Adam(model.parameters(), lr=LR)
    criterion = nn.CrossEntropyLoss(weight=weights)

    for epoch in range(1, EPOCHS + 1):
        model.train()
        t0, total, correct, loss_sum = time.time(), 0, 0, 0.0
        for xb, yb in loader:
            optimizer.zero_grad()
            out = model(xb)
            loss = criterion(out, yb)
            loss.backward()
            optimizer.step()
            loss_sum += loss.item() * len(yb)
            correct += (out.argmax(1) == yb).sum().item()
            total += len(yb)
        print(f"   epoch {epoch:2d}/{EPOCHS}  loss {loss_sum/total:.4f}  "
              f"acc {correct/total:.4f}  ({time.time()-t0:.0f}s)", flush=True)

    print("4. 평가...")
    model.eval()
    preds = []
    with torch.no_grad():
        for i in range(0, len(x_te), 4096):
            out = model(torch.tensor(x_te[i:i + 4096]))
            preds.append(out.argmax(1).numpy())
    preds = np.concatenate(preds)
    report = classification_report(
        y_te, preds, labels=[0, 1, 2, 3],
        target_names=["안전0", "주의1", "경고2", "위험3"], digits=4)
    cm = confusion_matrix(y_te, preds, labels=[0, 1, 2, 3])
    print(report)
    print("혼동 행렬 (행=실제, 열=예측):")
    print(cm)

    print("5. 저장 및 ONNX 변환...")
    torch.save(model.state_dict(), OUT_DIR / "risk_transformer_v2.pt")
    scaler = {
        "feature_columns": FEATURE_COLUMNS,
        "mean": [float(v) for v in mean],
        "scale": [float(v) for v in scale],
        "sequence_length": SEQ_LEN,
    }
    (OUT_DIR / "scaler_v2.json").write_text(
        json.dumps(scaler, indent=2), encoding="utf-8")

    dummy = torch.randn(1, SEQ_LEN, len(FEATURE_COLUMNS))
    torch.onnx.export(
        model, dummy, OUT_DIR / "risk_transformer_v2.onnx",
        input_names=["input"], output_names=["logits"],
        dynamic_axes={"input": {0: "batch"}, "logits": {0: "batch"}},
        opset_version=17)

    # PyTorch-ONNX 출력 일치 검증
    import onnxruntime as ort  # noqa
    sess = ort.InferenceSession(str(OUT_DIR / "risk_transformer_v2.onnx"))
    sample = x_te[:256]
    with torch.no_grad():
        torch_out = model(torch.tensor(sample)).numpy()
    onnx_out = sess.run(None, {"input": sample})[0]
    max_diff = float(np.abs(torch_out - onnx_out).max())
    print(f"   ONNX-PyTorch 최대 오차: {max_diff:.2e} "
          f"({'OK' if max_diff < 1e-4 else '불일치!'})")

    (OUT_DIR / "training_report_v2.txt").write_text(
        f"dataset: {DATA_FILE.name} ({len(df):,} rows)\n"
        f"features({len(FEATURE_COLUMNS)}): {FEATURE_COLUMNS}\n"
        f"train seq {len(y_tr):,} / test seq {len(y_te):,} "
        f"(scenario-level split, test={n_test})\n\n"
        f"{report}\n혼동 행렬:\n{cm}\nONNX max diff: {max_diff:.2e}\n",
        encoding="utf-8")
    print("완료. 결과:", OUT_DIR)


if __name__ == "__main__":
    main()
