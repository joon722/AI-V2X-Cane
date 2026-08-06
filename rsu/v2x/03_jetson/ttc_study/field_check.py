#!/usr/bin/env python3
"""실외 실측 로그로 시뮬레이터를 교차검증한다.

여기서 답하려는 질문은 "모델이 현장에서 몇 점이냐"가 아니다. 실외 실험은 안전상
차를 사람에게 2m 안까지 붙이지 않으므로(8/5 로그 최소거리 약 5m) 위험 라벨이
0건이고, 정답 기반 채점 자체가 불가능하다.

대신 답할 수 있는 것은 이것이다.

  분포 정합   시뮬레이터가 만든 피처와 실측 피처가 같은 자리에 있는가. 어긋나면
              시뮬레이션에서만 잘 도는 모델을 만든 것이다(sim-to-real gap).
  거동 타당   거리가 줄면 모델 출력이 오르고 멀어지면 내려가는가. 정답이 없어도
              물리적으로 말이 되는지는 확인할 수 있다.

입력은 step8의 raw 로그(`시각 RX/TX 원문`)다. 노드가 10Hz로 올리는 lat/lng를
그대로 담고 있어 실측 궤적으로는 가장 밀도가 높다.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from features import build_features
from scenario_sim import Scenario, Track

METERS_PER_DEGREE_LAT = 111320.0  # step6_kinematics와 같은 상수


def read_raw_log(path, max_lines=None):
    """raw 로그에서 cane/vehicle의 유효 GPS 레코드를 뽑는다."""
    rows = {"cane": [], "vehicle": []}
    with open(path, encoding="utf-8", errors="replace") as handle:
        for n, line in enumerate(handle):
            if max_lines and n >= max_lines:
                break
            parts = line.split(" ", 2)
            if len(parts) < 3 or parts[1] != "RX":
                continue
            try:
                t = float(parts[0])
                payload = json.loads(parts[2])
            except (ValueError, json.JSONDecodeError):
                continue
            kind = payload.get("type")
            if kind not in rows or not payload.get("gps_valid"):
                continue
            rows[kind].append((t, float(payload["lat"]), float(payload["lng"])))
    return rows


def to_enu(records, lat0, lng0):
    """위경도를 평면 미터로. step6_kinematics.LocalFrame과 같은 근사."""
    scale = METERS_PER_DEGREE_LAT * math.cos(math.radians(lat0))
    t = np.array([r[0] for r in records])
    x = np.array([(r[2] - lng0) * scale for r in records])
    y = np.array([(r[1] - lat0) * METERS_PER_DEGREE_LAT for r in records])
    return t, x, y


def resample_pair(cane, vehicle, dt=0.1):
    """두 노드를 공통 시간축에 올려 Scenario 하나로 만든다.

    실측은 두 노드의 도착 시각이 어긋나 있다. 피처 계산이 같은 시각의 두 위치를
    요구하므로 선형 보간으로 맞춘다. 겹치는 구간만 쓴다.
    """
    lat0, lng0 = cane[0][1], cane[0][2]
    ct, cx, cy = to_enu(cane, lat0, lng0)
    vt, vx, vy = to_enu(vehicle, lat0, lng0)

    start, end = max(ct.min(), vt.min()), min(ct.max(), vt.max())
    if end - start < 5.0:
        return None
    grid = np.arange(0.0, end - start, dt)
    abs_grid = grid + start

    def interp(t, v):
        return np.interp(abs_grid, t, v)

    # 속도는 features가 노이즈 모드에서 다시 추정하므로 여기서는 자리만 채운다.
    zeros = np.zeros(len(grid))
    ped = Track(x=interp(ct, cx), y=interp(ct, cy), vx=zeros, vy=zeros)
    veh = Track(x=interp(vt, vx), y=interp(vt, vy), vx=zeros, vy=zeros)
    return Scenario(t=grid, ped=ped, veh=veh, params=None)


def field_features(scenario):
    """실측 궤적의 피처. 좌표에 이미 실제 오차가 들어 있으므로 노이즈를 더하지 않되,
    속도는 현장과 같이 관측에서 추정해야 하므로 칼만 경로를 태운다."""
    from features import _kalman_track

    px, py, pvx, pvy = _kalman_track(scenario.ped.x, scenario.ped.y, scenario.t, 2.5)
    vx_, vy_, vvx, vvy = _kalman_track(scenario.veh.x, scenario.veh.y, scenario.t, 2.5)
    filtered = Scenario(
        t=scenario.t,
        ped=Track(x=px, y=py, vx=pvx, vy=pvy),
        veh=Track(x=vx_, y=vy_, vx=vvx, vy=vvy),
        params=None,
    )
    return build_features(filtered, gps_sigma_m=0.0)


COMPARE_COLUMNS = ("distance_m", "closing_los", "ttc", "veh_speed_mps",
                   "ped_speed_mps", "dcpa_m", "d_closing_dt", "d_heading_dt")

# 이 아래는 정지로 본다. 8/5 분석에서 정지 구간이 68%였고, 그것을 섞어 평균을 내면
# 실제로 움직일 때의 편향이 0으로 씻겨 나간다.
MOVING_MPS = 1.0


def main():
    parser = argparse.ArgumentParser(description="실측 로그로 시뮬레이터를 교차검증한다.")
    parser.add_argument("--log", default="../logs/raw_20260805_190051.log")
    parser.add_argument("--n-scenarios", type=int, default=300)
    parser.add_argument("--max-lines", type=int, default=None)
    args = parser.parse_args()

    path = Path(args.log)
    if not path.exists():
        raise SystemExit(f"로그가 없다: {path}")

    rows = read_raw_log(path, max_lines=args.max_lines)
    print(f"실측 레코드: cane {len(rows['cane']):,}  vehicle {len(rows['vehicle']):,}")
    if not rows["cane"] or not rows["vehicle"]:
        raise SystemExit("두 노드의 유효 GPS가 모두 있어야 한다")

    scenario = resample_pair(rows["cane"], rows["vehicle"])
    if scenario is None:
        raise SystemExit("두 노드가 겹치는 구간이 너무 짧다")
    field = field_features(scenario)
    d = field["distance_m"]
    print(f"공통 구간 {scenario.t[-1]:.0f}초 / {len(scenario.t):,}스텝")
    print(f"실측 거리: 최소 {d.min():.2f}m  중앙 {np.median(d):.2f}m  최대 {d.max():.2f}m")
    print()

    # --- 시뮬레이터 분포와 대조 ---
    from dataset import build_dataset
    sim = build_dataset(args.n_scenarios, gps_sigma_m=2.5, lead_s=2.0)

    # 정지 구간을 섞으면 통계가 희석된다. 8/5 실측 분석에서 접근속도 편향이
    # 정지 68% 때문에 0으로 씻겨 나갔던 것과 같은 함정이라, 움직인 구간만 본다.
    moving = field["veh_speed_mps"] >= MOVING_MPS
    sim_moving = sim.veh_speed_mps.to_numpy() >= MOVING_MPS
    print(f"차량이 움직인 구간: 실측 {int(moving.sum()):,}/{len(moving):,} "
          f"({moving.mean()*100:.1f}%)  시뮬 {sim_moving.mean()*100:.1f}%\n")

    print("피처 분포 비교 — 차량이 움직인 구간만 (중앙값 / 사분위폭)")
    print(f"{'피처':16s}{'실측':>22s}{'시뮬레이터':>24s}")
    for name in COMPARE_COLUMNS:
        f, s = field[name][moving], sim[name].to_numpy()[sim_moving]
        if len(f) == 0 or len(s) == 0:
            continue
        fq = np.percentile(f, [25, 50, 75])
        sq = np.percentile(s, [25, 50, 75])
        print(f"{name:16s}{fq[1]:9.2f} [{fq[0]:7.2f},{fq[2]:7.2f}]"
              f"{sq[1]:11.2f} [{sq[0]:7.2f},{sq[2]:7.2f}]")

    # --- 모델 거동이 물리적으로 말이 되는가 ---
    from dataset import split_by_scenario
    from model import predict_proba, train

    train_df, _ = split_by_scenario(sim, test_ratio=0.3, seed=0)
    clf = train(train_df, seed=0, label_col="y_train")

    import pandas as pd
    proba = predict_proba(clf, pd.DataFrame({c: field[c] for c in field}))

    print(f"\n모델 출력 (실측 궤적): 중앙 {np.median(proba):.3f}  최대 {proba.max():.3f}")
    print(f"{'거리 구간':14s}{'전체 스텝':>10}{'중앙':>8}{'움직인 구간':>12}{'중앙':>8}")
    for lo, hi in ((0, 10), (10, 20), (20, 40), (40, 1e9)):
        m = (d >= lo) & (d < hi)
        if m.sum() == 0:
            continue
        mm = m & moving
        label = f"{lo}~{hi}m" if hi < 1e9 else f"{lo}m+"
        moving_med = f"{np.median(proba[mm]):.3f}" if mm.sum() else "-"
        print(f"{label:14s}{int(m.sum()):10d}{np.median(proba[m]):8.3f}"
              f"{int(mm.sum()):12d}{moving_med:>8s}")

    for label, mask in (("전체", np.ones(len(d), dtype=bool)), ("움직인 구간", moving)):
        if mask.sum() < 10:
            continue
        corr = float(np.corrcoef(d[mask], proba[mask])[0, 1])
        verdict = "타당 (가까울수록 위험)" if corr < -0.2 else "약함"
        print(f"거리 대 위험확률 상관 [{label}]: {corr:+.3f} ({verdict})")

    # 모델이 실제로 경보한 구간이 어떤 상황이었는지
    hot = proba >= 0.5
    if hot.sum():
        print(f"\n위험확률 0.5 이상 구간 {int(hot.sum())}스텝:")
        print(f"  거리 중앙 {np.median(d[hot]):.2f}m  "
              f"차속 중앙 {np.median(field['veh_speed_mps'][hot]):.2f} m/s  "
              f"접근속도 중앙 {np.median(field['closing_los'][hot]):.2f} m/s")
    else:
        print("\n위험확률 0.5 이상 구간 없음 - 이 실험에는 위험 상황이 없었다는 뜻")


if __name__ == "__main__":
    main()
