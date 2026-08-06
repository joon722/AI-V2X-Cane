#!/usr/bin/env python3
"""세 방법을 같은 정답·같은 오경보 예산으로 비교한다.

공정성 장치가 셋이다.

  같은 평가 세트   모델이 학습한 시나리오는 평가에서 제외한다. 규칙은 학습이
                   없으므로 아무 데서나 재도 되지만, 같은 표본에서 재야 비교가 된다.
  같은 오경보 예산 모델의 임계값을 현 점수표의 오경보율에 맞춘다. 각자 편한
                   지점에서 비교하면 어느 쪽이든 이겼다고 말할 수 있다.
  여러 시드 반복   위험 시나리오는 전체의 6% 남짓이라 한 번 돌린 결과는 시드에
                   휘둘린다. 실제로 150개 평가 세트에서는 위험 시나리오가 11개뿐이라
                   방법 간 차이가 시나리오 한 개였다. 반복해서 분포로 봐야 한다.
"""

import argparse

import numpy as np
from sklearn.metrics import roc_auc_score

from baselines import ALARM_LEVEL, score_alarms, team_table_levels
from dataset import build_dataset, split_by_scenario
from model import predict_proba, threshold_at_false_alarm_rate, train
from safety_floor import T_FLOOR_S, timely_alarm_rate

TEAM_TABLE_INPUTS = ("distance_m", "closing_los", "ttc", "veh_speed_mps", "dcpa_m")


def run_once(n_scenarios, seed, gps_sigma_m=2.5, test_ratio=0.3, lead_s=0.0):
    """한 시드에서 A/B/C를 재고 결과 딕셔너리를 준다.

    lead_s는 학습 라벨에만 적용된다(dataset이 y_train을 따로 만든다). 평가는 항상
    같은 y와 실제 접촉 시각으로 하므로, 라벨을 바꿔도 채점 기준은 움직이지 않는다.
    """
    ds = build_dataset(
        n_scenarios, gps_sigma_m=gps_sigma_m, lead_s=lead_s,
        seed_offset=seed * n_scenarios,
    )
    train_df, test_df = split_by_scenario(ds, test_ratio=test_ratio, seed=seed)

    levels = team_table_levels({c: test_df[c].to_numpy() for c in TEAM_TABLE_INPUTS})
    a_alarm = levels >= ALARM_LEVEL
    a = score_alarms(test_df, a_alarm)

    clf = train(train_df, seed=seed, label_col="y_train" if lead_s > 0 else "y")
    proba = predict_proba(clf, test_df)
    # 모델은 점수표와 같은 오경보만 쓰도록 묶는다
    threshold = threshold_at_false_alarm_rate(
        proba, test_df.y.to_numpy(), target=a.false_alarm_rate
    )
    c_alarm = proba >= threshold
    c = score_alarms(test_df, c_alarm)

    out = {
        "seed": seed,
        "dangerous_scenarios": a.total_dangerous,
        "auc": float(roc_auc_score(test_df.y.to_numpy(), proba)),
        "A_timely": timely_alarm_rate(test_df, a_alarm),
        "A_far": a.false_alarm_rate,
        "C_timely": timely_alarm_rate(test_df, c_alarm),
        "C_far": c.false_alarm_rate,
    }
    for theta in (2.0, 2.5, 3.0):
        b_alarm = test_df.ttc.to_numpy() <= theta
        out[f"B{theta}_timely"] = timely_alarm_rate(test_df, b_alarm)
        out[f"B{theta}_far"] = score_alarms(test_df, b_alarm).false_alarm_rate
    return out


def summarize(runs):
    """반복 결과를 평균 ± 표준편차로 접는다."""
    keys = [k for k in runs[0] if k not in ("seed",)]
    return {k: (float(np.mean([r[k] for r in runs])), float(np.std([r[k] for r in runs])))
            for k in keys}


def main():
    parser = argparse.ArgumentParser(description="A/B/C를 여러 시드로 비교한다.")
    parser.add_argument("--n-scenarios", type=int, default=1200)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--gps-sigma-m", type=float, default=2.5)
    parser.add_argument(
        "--lead-s", type=float, default=0.0,
        help="학습 라벨 창을 이만큼 미룬다(반응 불가능한 코앞 위험을 정답에서 제외)",
    )
    args = parser.parse_args()

    print(f"시나리오 {args.n_scenarios}개 x {args.repeats}회 | "
          f"T_floor={T_FLOOR_S}s | lead={args.lead_s}s\n", flush=True)

    runs = []
    for seed in range(args.repeats):
        r = run_once(args.n_scenarios, seed, gps_sigma_m=args.gps_sigma_m, lead_s=args.lead_s)
        runs.append(r)
        print(f"  시드 {seed}: 위험 시나리오 {r['dangerous_scenarios']:3d}  AUC {r['auc']:.3f}  "
              f"A {r['A_timely']*100:5.1f}%  C {r['C_timely']*100:5.1f}%", flush=True)

    s = summarize(runs)
    print(f"\n{'방법':22s}{'적시경보':>16}{'오경보율':>14}")
    print(f"{'A. 현 점수표':20s}{s['A_timely'][0]*100:9.1f}% ±{s['A_timely'][1]*100:4.1f}"
          f"{s['A_far'][0]*100:9.2f}% ±{s['A_far'][1]*100:4.2f}")
    for theta in (2.0, 2.5, 3.0):
        print(f"{'B. TTC<=' + str(theta) + 's':20s}"
              f"{s[f'B{theta}_timely'][0]*100:9.1f}% ±{s[f'B{theta}_timely'][1]*100:4.1f}"
              f"{s[f'B{theta}_far'][0]*100:9.2f}% ±{s[f'B{theta}_far'][1]*100:4.2f}")
    print(f"{'C. 모델':20s}{s['C_timely'][0]*100:9.1f}% ±{s['C_timely'][1]*100:4.1f}"
          f"{s['C_far'][0]*100:9.2f}% ±{s['C_far'][1]*100:4.2f}")
    print(f"\nAUC {s['auc'][0]:.4f} ±{s['auc'][1]:.4f}")

    gains = [r["C_timely"] - r["A_timely"] for r in runs]
    print(f"C - A 적시경보 차이: 평균 {np.mean(gains)*100:+.1f}%p, "
          f"시드별 {[f'{g*100:+.1f}' for g in gains]}")
    print(f"C가 A를 이긴 시드: {sum(1 for g in gains if g > 0)}/{len(gains)}")


if __name__ == "__main__":
    main()
