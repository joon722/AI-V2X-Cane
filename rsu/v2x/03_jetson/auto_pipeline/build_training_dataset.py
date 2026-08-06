#!/usr/bin/env python3
"""
재학습(v2)용 학습 데이터셋 생성기 — Jetson에서 실행

incoming_data/의 자동 생성 시나리오에서 v2 특징 + 라벨을 만든다.

v1 대비 바뀌는 점:
  1. 보행자 좌표 = 실제 이동 좌표 (고정 좌표 우회 제거)
  2. 벡터 특징 추가: dx, dy(상대 위치), dvx, dvy(상대 속도), dcpa(최근접 예상 거리)
     -> 모델이 "다가오는 차"와 "스쳐 갈 차"를 스스로 구분하도록
  3. 라벨 = 팀 채점표 점수 x DCPA 게이트 -> classify_risk_level
     (위험 정의 일원화: 스쳐 가는 차는 라벨부터 안전으로)

클래스 불균형 완화: 위험(L1+) 차량은 전부, 안전-only 차량은 15%만 표본.
시퀀스 경계 보존을 위해 scenario_id, vehicle_id 컬럼 포함.

사용: python3 build_training_dataset.py --max-scenarios 400
출력: ~/training_dataset_v2.csv
"""
import argparse
import random
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path.home() / "v2x" / "03_jetson"))
from step7_risk import (  # noqa: E402
    calculate_risk_score,
    calculate_ttc,
    classify_risk_level,
    dcpa_gate,
)

INCOMING = Path.home() / "incoming_data"
OUT_FILE = Path.home() / "training_dataset_v2.csv"
SAFE_KEEP_RATE = 0.15
ZONE_BASE_RISK = 0.0

FEATURE_ORDER = [
    "ped_x", "ped_y", "veh_x", "veh_y",
    "ped_speed_mps", "veh_speed_mps",
    "distance_m", "rel_speed_mps", "ttc",
    "risk_score", "zone_base_risk",
    "dx", "dy", "dvx", "dvy", "dcpa_m",
]


def build_scenario(scenario_dir: Path) -> pd.DataFrame | None:
    veh_csv = scenario_dir / "feature.csv"
    ped_csv = scenario_dir / "pedestrian.csv"
    if not veh_csv.exists() or not ped_csv.exists():
        return None

    veh = pd.read_csv(veh_csv, sep=";")
    ped = pd.read_csv(ped_csv, sep=";")
    # 각 (차량, 시점)마다 가장 가까운 보행자 기준 (v2 시나리오는 보행자 다수)
    ped = ped[["timestep_time", "person_x", "person_y", "person_speed"]]
    ped.columns = ["timestep_time", "ped_x", "ped_y", "ped_speed_mps"]

    df = veh.merge(ped, on="timestep_time", how="inner")
    if df.empty:
        return None
    d2 = ((df["vehicle_x"] - df["ped_x"]) ** 2 +
          (df["vehicle_y"] - df["ped_y"]) ** 2)
    df = df.loc[d2.groupby([df["vehicle_id"], df["timestep_time"]]).idxmin()]
    df = df.sort_values(["vehicle_id", "timestep_time"]).reset_index(drop=True)
    df = df.rename(columns={"vehicle_x": "veh_x", "vehicle_y": "veh_y",
                            "vehicle_speed": "veh_speed_mps"})

    # 스칼라 특징 (v1과 동일하되 보행자 좌표는 실제 값)
    df["distance_m"] = np.sqrt((df["veh_x"] - df["ped_x"]) ** 2 +
                               (df["veh_y"] - df["ped_y"]) ** 2)
    g = df.groupby("vehicle_id", sort=False)
    dt = g["timestep_time"].diff()
    df["rel_speed_mps"] = (-g["distance_m"].diff() / dt).fillna(0.0)
    df["ttc"] = [calculate_ttc(d, r)
                 for d, r in zip(df["distance_m"], df["rel_speed_mps"])]
    df["risk_score"] = [
        calculate_risk_score(d, r, v, t, ZONE_BASE_RISK)
        for d, r, v, t in zip(df["distance_m"], df["rel_speed_mps"],
                              df["veh_speed_mps"], df["ttc"])]
    df["zone_base_risk"] = ZONE_BASE_RISK

    # 벡터 특징
    rx = df["veh_x"] - df["ped_x"]
    ry = df["veh_y"] - df["ped_y"]
    vid = df["vehicle_id"]
    dvx = rx.groupby(vid, sort=False).diff() / dt
    dvy = ry.groupby(vid, sort=False).diff() / dt
    df["dx"], df["dy"] = rx, ry
    df["dvx"] = dvx.fillna(0.0)
    df["dvy"] = dvy.fillna(0.0)
    v2 = dvx ** 2 + dvy ** 2
    t_cpa = -(rx * dvx + ry * dvy) / v2.where(v2 > 1e-6)
    dcpa = np.sqrt((rx + dvx * t_cpa) ** 2 + (ry + dvy * t_cpa) ** 2)
    df["dcpa_m"] = dcpa.where((t_cpa > 0) & v2.notna(),
                              df["distance_m"]).fillna(df["distance_m"])

    # 라벨: 채점표 점수 x DCPA 게이트 -> 레벨 (위험 정의 일원화)
    gate = np.array([dcpa_gate(d) for d in df["dcpa_m"]])
    df["risk_level"] = [classify_risk_level(s * f)
                        for s, f in zip(df["risk_score"], gate)]

    # 표본 추출: 위험 경험 차량 전부 + 안전-only 차량 15%
    keep = []
    for v, grp in df.groupby("vehicle_id", sort=False):
        if len(grp) < 10:
            continue
        if grp["risk_level"].max() >= 1 or random.random() < SAFE_KEEP_RATE:
            keep.append(grp)
    if not keep:
        return None
    out = pd.concat(keep, ignore_index=True)
    out["scenario_id"] = scenario_dir.name
    cols = (["scenario_id", "vehicle_id", "timestep_time"] +
            FEATURE_ORDER + ["risk_level"])
    return out[cols]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--max-scenarios", type=int, default=400)
    args = p.parse_args()
    random.seed(42)

    dirs = sorted(d for d in INCOMING.glob("scenario_*")
                  if (d / "DONE").exists())[:args.max_scenarios]
    frames = []
    for i, d in enumerate(dirs, 1):
        try:
            df = build_scenario(d)
            if df is not None:
                frames.append(df)
        except Exception as e:
            print(f"{d.name} 실패: {e}")
        if i % 50 == 0:
            print(f"{i}/{len(dirs)} 처리...", flush=True)

    full = pd.concat(frames, ignore_index=True)
    full.to_csv(OUT_FILE, index=False)
    print(f"\n완료: {OUT_FILE}")
    print(f"시나리오 {len(frames)}개, 총 {len(full):,}행, "
          f"차량 {full.groupby(['scenario_id','vehicle_id']).ngroups:,}대")
    print("라벨 분포:")
    print(full["risk_level"].value_counts().sort_index().to_string())


if __name__ == "__main__":
    main()
