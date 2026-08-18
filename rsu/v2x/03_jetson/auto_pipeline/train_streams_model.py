#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""build_dataset_from_streams 가 만든 CSV로 학습 단계(train_and_export)만 다시 실행.

왜 따로 있나: --streams(SUMO, 문자 scenario_id)와 --from-scenario-sim(정수 scenario_id)을
섞으면 dataset.split_by_scenario 의 sorted() 가 str/int 비교로 죽는다. 데이터셋 생성(수십 분)은
이미 끝났으니 CSV를 읽어 id 를 문자열로 통일한 뒤 같은 train_and_export 를 호출한다.

사용
    python train_streams_model.py --csv training_dataset_streams.csv --model risk_model_streams.json
"""
import argparse
import sys
from pathlib import Path

import pandas as pd

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.stdout.reconfigure(encoding="utf-8")

from build_dataset_from_streams import train_and_export  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="training_dataset_streams.csv")
    ap.add_argument("--model", default="risk_model_streams.json")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    df = pd.read_csv(args.csv, low_memory=False)
    df["scenario_id"] = df["scenario_id"].astype(str)
    print(f"[load] {len(df):,} rows, {df.scenario_id.nunique()} scenarios, "
          f"sources {sorted(df.source.unique().tolist())}")
    for src in sorted(df.source.unique()):
        d = df[df.source == src]
        print(f"  [{src}] {len(d):,} rows, {d.scenario_id.nunique()} scenarios, "
              f"y=1 {d.y.mean()*100:.1f}%, 위험 시나리오 {int((d.groupby('scenario_id').y.max() > 0).sum())}")
    train_and_export(df, args.model, seed=args.seed)


if __name__ == "__main__":
    main()
