#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""경보 transformer — 학습 공장 CSV(build_dataset_from_streams) 위에서 최근 10프레임 시퀀스로 학습.

팀 요청(2026-08-18 현준) 그대로:
  입력  CSV 의 FEATURE_COLUMNS 15열 그대로 (다른 특징 만들지 않음 — 젯슨 실행 특징과 동일)
  정답  y_train (2 초 뒤 창 = "지금 울려야 피할 수 있는 위험")
  채점  시나리오 단위 적시경보율(ttc_study/safety_floor.timely_alarm_rate). 행 단위 재현율 X.
  비교  규칙(팀 점수표)과 같은 오경보율에서 임계값을 잡고 적시경보율을 나란히 본다.
  산출  ONNX (입력 [batch, 10, 15]) + scaler JSON → 젯슨 onnxruntime 에서 바로 사용.

시퀀스는 시나리오 안에서만 만든다(경계 넘지 않음). 시나리오 앞 9프레임은 첫 프레임을
왼쪽 패딩해 채워서 첫 판정부터 시퀀스 예측이 나오게 한다(젯슨 실행도 같은 규칙을 쓰면 됨).

사용
    python train_alarm_transformer.py --csv training_dataset_streams.csv --out models_alarm_tf
"""
import argparse
import json
import random
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

HERE = Path(__file__).resolve().parent
JETSON = HERE.parent
for p in (HERE, JETSON, JETSON / "ttc_study"):
    sys.path.insert(0, str(p))
sys.stdout.reconfigure(encoding="utf-8")

from baselines import ALARM_LEVEL, team_table_levels  # noqa: E402
from dataset import split_by_scenario  # noqa: E402
from features import FEATURE_COLUMNS  # noqa: E402
from model import threshold_at_false_alarm_rate  # noqa: E402
from safety_floor import T_FLOOR_S, timely_alarm_rate  # noqa: E402

SEQ_LEN = 10
SEED = 42
FEATS = list(FEATURE_COLUMNS)


class AlarmTransformer(nn.Module):
    """risk_transformer_v3 와 같은 뼈대(d_model 64, 2층), 출력은 이진 logit 하나."""

    def __init__(self, num_features, d_model=64, nhead=4, num_layers=2):
        super().__init__()
        self.input_projection = nn.Linear(num_features, d_model)
        layer = nn.TransformerEncoderLayer(d_model=d_model, nhead=nhead, dim_feedforward=128,
                                           dropout=0.1, batch_first=True)
        self.encoder = nn.TransformerEncoder(layer, num_layers=num_layers)
        self.head = nn.Sequential(nn.LayerNorm(d_model), nn.Linear(d_model, 1))

    def forward(self, x):
        x = self.encoder(self.input_projection(x))
        return self.head(x[:, -1, :]).squeeze(-1)


def make_sequences(df, mean, scale):
    """시나리오 안에서만 길이 SEQ_LEN 창. 앞은 첫 프레임 반복 패딩. 행 순서 = df 순서 유지."""
    feats = ((df[FEATS].to_numpy(np.float32) - mean) / scale).astype(np.float32)
    xs = np.empty((len(df), SEQ_LEN, len(FEATS)), np.float32)
    pos = 0
    for _, idx in df.groupby("scenario_id", sort=False).indices.items():
        idx = np.sort(idx)
        f = feats[idx]
        pad = np.concatenate([np.repeat(f[:1], SEQ_LEN - 1, axis=0), f], axis=0)
        for i in range(len(idx)):
            xs[idx[i]] = pad[i:i + SEQ_LEN]
        pos += len(idx)
    return xs


def predict(model, x, bs=4096):
    model.eval()
    out = []
    with torch.no_grad():
        for i in range(0, len(x), bs):
            out.append(torch.sigmoid(model(torch.from_numpy(x[i:i + bs]))).numpy())
    return np.concatenate(out) if out else np.empty(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--out", default="models_alarm_tf")
    ap.add_argument("--epochs", type=int, default=8)
    ap.add_argument("--seed", type=int, default=SEED)
    args = ap.parse_args()
    random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed)
    out_dir = Path(args.out); out_dir.mkdir(exist_ok=True)

    df = pd.read_csv(args.csv, low_memory=False)
    # scenario_sim(정수 id)과 sumo(문자 id)를 섞으면 split_by_scenario 의 sorted() 가 죽는다 → 문자열 통일
    df["scenario_id"] = df["scenario_id"].astype(str)
    df = df.sort_values(["scenario_id", "t"], kind="stable").reset_index(drop=True)
    print(f"1. 로드: {len(df):,}행, 시나리오 {df.scenario_id.nunique()}개, "
          f"원천 {sorted(df.source.unique().tolist())}")

    train_df, test_df = split_by_scenario(df, test_ratio=0.3, seed=args.seed)
    train_df = train_df.reset_index(drop=True); test_df = test_df.reset_index(drop=True)
    mean = train_df[FEATS].mean().to_numpy(np.float32)
    scale = train_df[FEATS].std().replace(0, 1.0).to_numpy(np.float32)
    x_tr = make_sequences(train_df, mean, scale); y_tr = train_df.y_train.to_numpy(np.float32)
    x_te = make_sequences(test_df, mean, scale)
    print(f"2. 시퀀스: 학습 {len(x_tr):,} (y_train=1 {y_tr.mean()*100:.1f}%) / 평가 {len(x_te):,}")

    pos_w = torch.tensor([(1 - y_tr.mean()) / max(y_tr.mean(), 1e-6)], dtype=torch.float32)
    model = AlarmTransformer(len(FEATS))
    opt = torch.optim.Adam(model.parameters(), lr=1e-3)
    crit = nn.BCEWithLogitsLoss(pos_weight=pos_w)
    loader = DataLoader(TensorDataset(torch.from_numpy(x_tr), torch.from_numpy(y_tr)),
                        batch_size=256, shuffle=True)
    print(f"3. 학습 ({sum(p.numel() for p in model.parameters()):,} 파라미터, "
          f"pos_weight {pos_w.item():.1f})")
    for ep in range(1, args.epochs + 1):
        model.train(); t0 = time.time(); ls = 0.0; n = 0
        for xb, yb in loader:
            opt.zero_grad(); loss = crit(model(xb), yb); loss.backward(); opt.step()
            ls += loss.item() * len(yb); n += len(yb)
        print(f"   epoch {ep}/{args.epochs}  loss {ls/n:.4f}  ({time.time()-t0:.0f}s)", flush=True)

    # ---- 채점: 규칙과 같은 오경보율에서 적시경보율 ----
    print("4. 채점 (시나리오 단위 적시경보율, 규칙과 같은 오경보율)")
    levels = team_table_levels({c: test_df[c].to_numpy() for c in
                                ("distance_m", "closing_los", "ttc", "veh_speed_mps", "dcpa_m")})
    rule_alarm = levels >= ALARM_LEVEL
    y = test_df.y.to_numpy()
    target_far = float(rule_alarm[y == 0].mean())
    proba = predict(model, x_te)
    thr = threshold_at_false_alarm_rate(proba, y, target_far)
    model_alarm = proba >= thr
    timely_rule = timely_alarm_rate(test_df, rule_alarm)
    timely_model = timely_alarm_rate(test_df, model_alarm)
    far_model = float(model_alarm[y == 0].mean())
    # 정지 오경보(둘 다 거의 정지) — 팀이 따로 보는 지표
    still = (test_df.veh_speed_mps.to_numpy() < 0.3) & (test_df.ped_speed_mps.to_numpy() < 0.3) & (y == 0)
    still_rule = float(rule_alarm[still].mean()) if still.any() else float("nan")
    still_model = float(model_alarm[still].mean()) if still.any() else float("nan")
    per_source = {}
    for src in sorted(test_df.source.unique()):
        m = (test_df.source == src).to_numpy()
        sub = test_df[m].reset_index(drop=True)
        per_source[src] = {"timely_rule": timely_alarm_rate(sub, rule_alarm[m]),
                           "timely_model": timely_alarm_rate(sub, model_alarm[m]),
                           "scenarios": int(sub.scenario_id.nunique())}
    print(f"   임계값 {thr:.4f}  (규칙 오경보 {target_far*100:.2f}% 기준 → 모델 오경보 {far_model*100:.2f}%)")
    print(f"   적시경보(T_floor {T_FLOOR_S:.1f}s 앞)  규칙 {timely_rule*100:.1f}% → 모델 {timely_model*100:.1f}%")
    print(f"   정지 오경보                    규칙 {still_rule*100:.2f}% → 모델 {still_model*100:.2f}%")
    for src, r in per_source.items():
        print(f"   [{src}] ({r['scenarios']} 시나리오) 적시경보 규칙 {r['timely_rule']*100:.1f}% → 모델 {r['timely_model']*100:.1f}%")

    # ---- 저장: ONNX + scaler ----
    print("5. 저장·ONNX")
    torch.save(model.state_dict(), out_dir / "alarm_transformer.pt")
    dummy = torch.randn(1, SEQ_LEN, len(FEATS))
    torch.onnx.export(model, dummy, out_dir / "alarm_transformer.onnx",
                      input_names=["input"], output_names=["logit"],
                      dynamic_axes={"input": {0: "batch"}, "logit": {0: "batch"}},
                      opset_version=17, dynamo=False)
    import onnxruntime as ort
    sess = ort.InferenceSession(str(out_dir / "alarm_transformer.onnx"))
    sample = x_te[:256]
    ref = predict(model, sample)
    got = 1 / (1 + np.exp(-sess.run(None, {"input": sample})[0]))
    diff = float(np.abs(ref - got).max())
    meta = {
        "feature_columns": FEATS, "mean": [float(v) for v in mean], "scale": [float(v) for v in scale],
        "sequence_length": SEQ_LEN, "pad": "repeat first frame on the left",
        "output": "logit; alarm if sigmoid(logit) >= threshold",
        "threshold": float(thr), "target_false_alarm_rate": target_far,
        "timely_rule": timely_rule, "timely_model": timely_model,
        "still_false_alarm_rule": still_rule, "still_false_alarm_model": still_model,
        "per_source": per_source, "t_floor_s": T_FLOOR_S,
        "trained_rows": int(len(train_df)), "trained_scenarios": int(train_df.scenario_id.nunique()),
        "label": "y_train", "onnx_max_abs_diff": diff,
    }
    (out_dir / "alarm_transformer_scaler.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False),
                                                           encoding="utf-8")
    print(f"   ONNX 오차 {diff:.2e} ({'OK' if diff < 1e-4 else 'FAIL'}) → {out_dir}")


if __name__ == "__main__":
    main()
