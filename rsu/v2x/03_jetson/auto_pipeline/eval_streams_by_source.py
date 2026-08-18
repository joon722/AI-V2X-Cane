#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""risk_model_streams.json 을 원천(sumo / scenario_sim)별로 나눠 적시경보율 채점.

train_and_export 와 같은 분할(seed)·같은 임계값(저장된 값)을 써서, 전체 숫자를
원천별로 쪼개 본다. "실차 스케일(SUMO)에서 규칙 대비 얼마나 위인가"에 답하기 위함.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
JETSON = HERE.parent
for p in (HERE, JETSON, JETSON / "ttc_study"):
    sys.path.insert(0, str(p))
sys.stdout.reconfigure(encoding="utf-8")

from baselines import ALARM_LEVEL, team_table_levels  # noqa: E402
from dataset import split_by_scenario  # noqa: E402
from model_runtime import TreeEnsemble  # noqa: E402
from safety_floor import timely_alarm_rate  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="training_dataset_streams.csv")
    ap.add_argument("--model", default="risk_model_streams.json")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    df = pd.read_csv(args.csv, low_memory=False)
    df["scenario_id"] = df["scenario_id"].astype(str)
    _, test_df = split_by_scenario(df, test_ratio=0.3, seed=args.seed)
    test_df = test_df.reset_index(drop=True)
    runtime = TreeEnsemble.load(args.model)
    thr = json.loads(Path(args.model).read_text(encoding="utf-8")).get("threshold", 0.5)
    rows = test_df[list(runtime.features)].to_numpy(dtype=float)
    proba = np.array([runtime.predict_proba(dict(zip(runtime.features, r))) for r in rows])
    model_alarm = proba >= thr
    levels = team_table_levels({c: test_df[c].to_numpy() for c in
                                ("distance_m", "closing_los", "ttc", "veh_speed_mps", "dcpa_m")})
    rule_alarm = levels >= ALARM_LEVEL
    y = test_df.y.to_numpy()
    print(f"임계값 {thr:.4f}, 평가 {len(test_df):,}행 / {test_df.scenario_id.nunique()} 시나리오")
    for src in ["all"] + sorted(test_df.source.unique()):
        m = np.ones(len(test_df), bool) if src == "all" else (test_df.source == src).to_numpy()
        sub = test_df[m].reset_index(drop=True)
        n_danger = int((sub.groupby("scenario_id").y.max() > 0).sum())
        tr = timely_alarm_rate(sub, rule_alarm[m]); tm = timely_alarm_rate(sub, model_alarm[m])
        far_r = float(rule_alarm[m][y[m] == 0].mean()); far_m = float(model_alarm[m][y[m] == 0].mean())
        print(f"[{src:12s}] 위험 시나리오 {n_danger:4d}  적시경보 규칙 {tr*100:5.1f}% → 모델 {tm*100:5.1f}%"
              f"   오경보 규칙 {far_r*100:.2f}% / 모델 {far_m*100:.2f}%")


if __name__ == "__main__":
    main()
