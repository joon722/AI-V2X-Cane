# -*- coding: utf-8 -*-
"""배치 생성한 데이터셋 CSV 여러 개를 합쳐 한 번에 학습한다.

build_dataset_from_streams.py 는 모든 시나리오를 RAM 에 모아 학습하므로
큰 N 은 배치(서로 다른 --seed)로 나눠 생성하고, 여기서 필요한 컬럼만
(usecols, float32) 읽어 합친 뒤 프로젝트의 train_and_export 를 그대로 부른다.
시드가 겹치지 않으면 scenario_id(=seed+i, str)도 겹치지 않는다.

사용:  PYTHONUTF8=1 python combine_train.py out_model.json ds1.csv ds2.csv ...
"""
import sys
from pathlib import Path

JETSON = r"C:\Users\user\OneDrive\바탕 화면\v2x(lux)\03_jetson"
sys.path.insert(0, str(Path(JETSON) / "auto_pipeline"))
sys.path.insert(0, str(Path(JETSON) / "ttc_study"))
sys.path.insert(0, JETSON)

import pandas as pd  # noqa: E402

from features import FEATURE_COLUMNS  # noqa: E402  (ttc_study 정본 15개)
from build_dataset_from_streams import train_and_export  # noqa: E402

# 학습(train)·임계값(team_table_levels)·적시경보(timely_alarm_rate: t, t_hit)에 필요한 것만.
NEEDED = list(FEATURE_COLUMNS) + ["t", "y", "y_train", "scenario_id", "source", "t_hit"]


def load(path):
    df = pd.read_csv(path, usecols=lambda c: c in NEEDED,
                     dtype={c: "float32" for c in FEATURE_COLUMNS})
    df["scenario_id"] = df["scenario_id"].astype(str)
    return df


def main():
    out_model = sys.argv[1]
    paths = sys.argv[2:]
    parts = []
    for p in paths:
        d = load(p)
        print(f"[load] {Path(p).name}: {len(d)} rows, {d.scenario_id.nunique()} scenarios, "
              f"y=1 {d.y.mean()*100:.2f}%")
        parts.append(d)
    df = pd.concat(parts, ignore_index=True)
    n_scen = df.scenario_id.nunique()
    assert n_scen == sum(p.scenario_id.nunique() for p in parts), "scenario_id 충돌(시드 겹침)"
    del parts
    print(f"[total] {len(df)} rows, {n_scen} scenarios, y=1 {df.y.mean()*100:.2f}%")
    train_and_export(df, out_model, seed=0)


if __name__ == "__main__":
    main()
