# -*- coding: utf-8 -*-
"""
v3 (3초 선행 예측) 데이터셋 생성 + 물리 외삽 기준선 평가

1. v2 데이터셋에서:
   - 물리 외삽 특징 3개 추가 (현재 상대 위치/속도 벡터로 3초 앞을 계산)
       phys_min_dist_3s : 3초 안에 도달할 최소 거리 (등속 가정)
       phys_t_cpa       : 최소 거리 도달까지 시간 (0~3초로 제한)
       phys_score_3s    : 그 최소 거리·시간에 팀 채점표를 적용한 점수
   - 미래 라벨: risk_level_future3 = 향후 1~3초의 최대 위험 레벨
2. 물리 단독 예측(기준선)의 성능을 미래 라벨과 비교해 측정
"""
import random
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.stdout.reconfigure(encoding="utf-8")
HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))

SEED = 42
HORIZON = 3  # 예측 범위(초)


# 팀 채점표 (step7_risk.py 동결 사본 — Jetson과 동일 수치)
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


def main():
    print("1. v2 데이터셋 로드...")
    df = pd.read_csv(HERE / "training_dataset_v2.csv")
    print(f"   {len(df):,}행")

    g = df.groupby(["scenario_id", "vehicle_id"], sort=False)

    # ── 물리 외삽: r(τ) = r + v·τ 의 최소 거리 (τ ∈ [0, 3]) ──
    rx, ry = df["dx"].to_numpy(), df["dy"].to_numpy()
    vx, vy = df["dvx"].to_numpy(), df["dvy"].to_numpy()
    v2 = vx ** 2 + vy ** 2
    with np.errstate(divide="ignore", invalid="ignore"):
        t_cpa = np.where(v2 > 1e-6, -(rx * vx + ry * vy) / v2, -1.0)
    t_hit = np.clip(t_cpa, 0.0, float(HORIZON))  # 구간 안에서 가장 가까운 시점
    px, py = rx + vx * t_hit, ry + vy * t_hit
    df["phys_min_dist_3s"] = np.sqrt(px ** 2 + py ** 2)
    df["phys_t_cpa"] = t_hit
    df["phys_score_3s"] = [
        calculate_risk_score(d, r, v, t if r > 0.1 else 9999.0)
        for d, r, v, t in zip(df["phys_min_dist_3s"], df["rel_speed_mps"],
                              df["veh_speed_mps"],
                              np.where(t_hit > 0.05, t_hit, 0.05))]
    df["phys_pred_level"] = [classify_risk_level(s)
                             for s in df["phys_score_3s"]]

    # ── 미래 라벨: 향후 1~3초의 최대 위험 (차량 경계 안에서만) ──
    fut = pd.concat([g["risk_level"].shift(-k) for k in range(1, HORIZON + 1)],
                    axis=1)
    df["risk_level_future3"] = fut.max(axis=1)
    valid = fut.notna().all(axis=1)  # 3초 미래가 온전히 있는 행만
    df = df[valid].copy()
    df["risk_level_future3"] = df["risk_level_future3"].astype(int)
    print(f"2. 미래 라벨 생성 완료 — 유효 {len(df):,}행")
    print("   현재 라벨 분포:", df["risk_level"].value_counts().sort_index().tolist())
    print("   미래(3초) 라벨 분포:",
          df["risk_level_future3"].value_counts().sort_index().tolist())

    df.to_csv(HERE / "training_dataset_v3.csv", index=False)
    print("   저장: training_dataset_v3.csv")

    # ── 물리 기준선 평가 (v2와 같은 평가 시나리오로) ──
    random.seed(SEED)
    scenarios = sorted(df["scenario_id"].unique())
    random.shuffle(scenarios)
    n_test = max(1, int(len(scenarios) * 0.15))
    test = df[df["scenario_id"].isin(set(scenarios[:n_test]))]

    from sklearn.metrics import classification_report, confusion_matrix
    y, p = test["risk_level_future3"], test["phys_pred_level"]
    print(f"\n3. 물리 외삽 기준선 성능 (평가 시나리오 {n_test}개, {len(test):,}행)")
    print(classification_report(y, p, labels=[0, 1, 2, 3],
                                target_names=["안전0", "주의1", "경고2", "위험3"],
                                digits=4))
    print("혼동 행렬 (행=실제 미래, 열=물리 예측):")
    print(confusion_matrix(y, p, labels=[0, 1, 2, 3]))


if __name__ == "__main__":
    main()
