# -*- coding: utf-8 -*-
"""
v4 학습 데이터셋 생성 (PC 로컬판) — 현실성 v2 시나리오 전용

Jetson의 build_training_dataset.py와 동일 로직:
  - 차량별 '최근접 보행자' 기준 특징 (v2 시나리오는 보행자 다수)
  - 라벨 = 팀 채점표 x DCPA 게이트 (위험 정의 일원화)
채점표/게이트는 step7_risk.py의 동결 사본 (팀 수치 그대로).

입력: scratchpad/v2data/scenario_*/  (서버에서 rsync로 수신)
출력: training_dataset_v4base.csv
"""
import random
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.stdout.reconfigure(encoding="utf-8")
HERE = Path(__file__).parent
DATA_DIR = HERE / "v2data"
OUT_FILE = HERE / "training_dataset_v4base.csv"
SAFE_KEEP_RATE = 0.15
ZONE_BASE_RISK = 0.0

FEATURE_ORDER = [
    "ped_x", "ped_y", "veh_x", "veh_y",
    "ped_speed_mps", "veh_speed_mps",
    "distance_m", "rel_speed_mps", "ttc",
    "risk_score", "zone_base_risk",
    "dx", "dy", "dvx", "dvy", "dcpa_m",
]


# ── 팀 공식 동결 사본 (step7_risk.py와 동일 수치) ──
def calculate_ttc(distance_m, relative_speed_mps):
    if relative_speed_mps <= 0:
        return 9999.0
    return distance_m / relative_speed_mps


def calculate_risk_score(distance_m, relative_speed_mps, vehicle_speed_mps,
                         ttc, zone_base_risk=0):
    score = 0.0
    if distance_m <= 10: score += 30
    elif distance_m <= 20: score += 25
    elif distance_m <= 40: score += 18
    elif distance_m <= 60: score += 10
    elif distance_m <= 100: score += 5
    if ttc <= 1: score += 35
    elif ttc <= 2: score += 30
    elif ttc <= 3: score += 25
    elif ttc <= 5: score += 20
    elif ttc <= 8: score += 15
    elif ttc <= 12: score += 8
    if relative_speed_mps >= 20: score += 20
    elif relative_speed_mps >= 15: score += 17
    elif relative_speed_mps >= 10: score += 13
    elif relative_speed_mps >= 5: score += 8
    elif relative_speed_mps > 0: score += 3
    if vehicle_speed_mps >= 20: score += 10
    elif vehicle_speed_mps >= 15: score += 8
    elif vehicle_speed_mps >= 10: score += 6
    elif vehicle_speed_mps >= 5: score += 3
    score += min(max(zone_base_risk, 0), 5)
    return round(min(score, 100), 2)


def classify_risk_level(risk_score):
    if risk_score >= 70: return 3
    if risk_score >= 45: return 2
    if risk_score >= 20: return 1
    return 0


def dcpa_gate(dcpa, near_m=2.5, far_m=7.5, floor=0.2):
    if dcpa is None or dcpa <= near_m:
        return 1.0
    if dcpa >= far_m:
        return floor
    frac = (dcpa - near_m) / (far_m - near_m)
    return 1.0 + frac * (floor - 1.0)


def build_scenario(scenario_dir: Path):
    veh_csv = scenario_dir / "feature.csv"
    ped_csv = scenario_dir / "pedestrian.csv"
    if not veh_csv.exists() or not ped_csv.exists():
        return None
    veh = pd.read_csv(veh_csv, sep=";")
    ped = pd.read_csv(ped_csv, sep=";")
    ped = ped[["timestep_time", "person_x", "person_y", "person_speed"]]
    ped.columns = ["timestep_time", "ped_x", "ped_y", "ped_speed_mps"]

    df = veh.merge(ped, on="timestep_time", how="inner")
    if df.empty:
        return None
    # 최근접 보행자 선택
    d2 = ((df["vehicle_x"] - df["ped_x"]) ** 2 +
          (df["vehicle_y"] - df["ped_y"]) ** 2)
    df = df.loc[d2.groupby([df["vehicle_id"], df["timestep_time"]]).idxmin()]

    df = df.sort_values(["vehicle_id", "timestep_time"]).reset_index(drop=True)
    df = df.rename(columns={"vehicle_x": "veh_x", "vehicle_y": "veh_y",
                            "vehicle_speed": "veh_speed_mps"})
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

    gate = np.array([dcpa_gate(d) for d in df["dcpa_m"]])
    df["risk_level"] = [classify_risk_level(s * f)
                        for s, f in zip(df["risk_score"], gate)]

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
    random.seed(42)
    dirs = sorted(d for d in DATA_DIR.glob("scenario_*")
                  if (d / "DONE").exists())
    print(f"시나리오 {len(dirs)}개 처리 시작")
    frames = []
    for i, d in enumerate(dirs, 1):
        try:
            df = build_scenario(d)
            if df is not None:
                frames.append(df)
        except Exception as e:
            print(f"{d.name} 실패: {e}")
        if i % 50 == 0:
            print(f"{i}/{len(dirs)}...", flush=True)
    full = pd.concat(frames, ignore_index=True)
    full.to_csv(OUT_FILE, index=False)
    print(f"완료: {OUT_FILE} — {len(full):,}행, 시나리오 {len(frames)}개")
    print("라벨 분포:", full["risk_level"].value_counts().sort_index().tolist())


if __name__ == "__main__":
    main()
