#!/usr/bin/env python3
"""안전 하한을 DCPA 조건부로 바꿨을 때의 손익을 오라클로 잰다.

    현재:   경보 = (점수표레벨 >= 2)  OR  (TTC <= T_floor)
    조건화: 경보 = (점수표레벨 >= 2)  OR  (TTC <= T_floor AND dcpa < R)

하한이 게이트를 무시하도록 설계된 이유는 step7_risk.py에 적혀 있다 - 스쳐 갈
차라는 판단 자체가 추정이고, 2초도 남지 않았을 때 그 추정이 틀리면 대가가 크다.
이 스크립트는 그 대가가 실제로 얼마인지를 오라클 기준으로 잰다.

R과 GPS sigma를 함께 훑는 이유는 둘이 엮여 있기 때문이다. dcpa는 관측에서
추정한 값이라 위치 오차가 크면 판정도 흔들린다. 결과는 ROADSIDE_ALARM.md 4~6절.

채점은 evaluate.py와 같은 오라클을 쓴다. 규칙과 독립인 정답이라 규칙을 고친
결과를 재도 순환 논리가 되지 않는다.
"""

import argparse

import numpy as np

from baselines import ALARM_LEVEL, score_alarms, team_table_levels
from dataset import build_dataset
from safety_floor import T_FLOOR_S, timely_alarm_rate

TEAM_TABLE_INPUTS = ("distance_m", "closing_los", "ttc", "veh_speed_mps", "dcpa_m")

# 확실히 안전한 시점으로 볼 기준. 3초 안에 이 거리보다 가까워지지 않으면
# 접촉할 수 없다(oracle.D_CRIT_M이 2.0m이므로 두 배로 잡았다). 인도 옆을 스쳐
# 가는 차가 여기 해당하므로, 이 구간의 경보율이 팀이 걱정한 오경보에 가깝다.
CLEARLY_SAFE_M = 4.0

R_VALUES = (2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, float("inf"))


def run_once(n_scenarios, seed, gps_sigma_m=2.5):
    df = build_dataset(
        n_scenarios, gps_sigma_m=gps_sigma_m, seed_offset=seed * n_scenarios
    )

    table_alarm = team_table_levels(
        {c: df[c].to_numpy() for c in TEAM_TABLE_INPUTS}
    ) >= ALARM_LEVEL
    floor_hit = df.ttc.to_numpy() <= T_FLOOR_S
    dcpa = df.dcpa_m.to_numpy()
    clearly_safe = (
        ~df.y.to_numpy().astype(bool)
    ) & (df.d_min_future.to_numpy() >= CLEARLY_SAFE_M)

    rows = {}
    for r in R_VALUES:
        alarm = table_alarm | (floor_hit & (dcpa < r))
        s = score_alarms(df, alarm)
        rows[r] = {
            "timely": timely_alarm_rate(df, alarm),
            "far": s.false_alarm_rate,
            "far_clear": float(alarm[clearly_safe].mean()),
            "detected": s.detected_scenarios,
            "total": s.total_dangerous,
        }
    return rows


def main():
    parser = argparse.ArgumentParser(
        description="안전 하한의 DCPA 조건화를 R과 GPS sigma에 대해 훑는다."
    )
    parser.add_argument("--n-scenarios", type=int, default=1200)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--gps-sigma-m", type=float, default=2.5)
    args = parser.parse_args()

    print(f"시나리오 {args.n_scenarios}개 x {args.repeats}회 | "
          f"T_floor={T_FLOOR_S}s | GPS sigma={args.gps_sigma_m}m", flush=True)

    runs = []
    for seed in range(args.repeats):
        runs.append(run_once(args.n_scenarios, seed, gps_sigma_m=args.gps_sigma_m))
        print(f"  시드 {seed} 완료", flush=True)

    def avg(r, key):
        return float(np.mean([x[r][key] for x in runs]))

    def sd(r, key):
        return float(np.std([x[r][key] for x in runs]))

    inf = float("inf")
    print()
    print(f"{'R (m)':>10}{'적시경보':>15}{'오경보율':>15}"
          f"{'확실안전 경보율':>18}{'검출':>12}")
    for r in R_VALUES:
        label = "무한(현재)" if r == inf else f"{r:.1f}"
        print(f"{label:>10}{avg(r,'timely')*100:8.1f}% ±{sd(r,'timely')*100:4.1f}"
              f"{avg(r,'far')*100:8.2f}% ±{sd(r,'far')*100:4.2f}"
              f"{avg(r,'far_clear')*100:11.2f}% ±{sd(r,'far_clear')*100:4.2f}"
              f"{avg(r,'detected'):8.1f}/{avg(r,'total'):.1f}")

    print()
    print("현재 대비 변화")
    for r in R_VALUES:
        if r == inf:
            continue
        print(f"  R={r:.1f} m : 적시경보 {(avg(r,'timely')-avg(inf,'timely'))*100:+5.1f}%p"
              f" | 오경보 {(avg(r,'far')-avg(inf,'far'))*100:+5.2f}%p"
              f" | 확실안전 경보 {(avg(r,'far_clear')-avg(inf,'far_clear'))*100:+6.2f}%p")


if __name__ == "__main__":
    main()
