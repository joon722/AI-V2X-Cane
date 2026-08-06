#!/usr/bin/env python3
"""모델이 상황마다 다른 TTC에서 경보한다는 것을 보이는 그림.

발표에서 답해야 할 질문이 "그래서 TTC를 몇 초로 정했냐"인데, 이 연구의 답은
"정하지 않았다, 상황이 정한다"이다. 그것을 말로 하면 변명처럼 들리고 그림으로
보이면 증거가 된다.

시나리오를 접근 기하학과 속도로 나누고, 각 유형에서 모델이 처음 경보한 시점의
TTC를 모아 분포로 그린다. 유형마다 분포가 다르면 조건부 판단이 실제로 일어나고
있다는 뜻이다. T_floor 선을 같이 그려 "그래도 최소한은 지킨다"까지 한 장에 담는다.
"""

import argparse
from pathlib import Path

import numpy as np

from baselines import ALARM_LEVEL, team_table_levels
from dataset import build_dataset, split_by_scenario
from model import predict_proba, threshold_at_false_alarm_rate, train
from safety_floor import T_FLOOR_S
from scenario_sim import sample_params

TEAM_TABLE_INPUTS = ("distance_m", "closing_los", "ttc", "veh_speed_mps", "dcpa_m")


def classify(params):
    """시나리오를 사람이 읽을 수 있는 유형으로 나눈다.

    접근 각도는 차가 어디서 오는지, 속도는 얼마나 급한지를 가른다. 두 축 모두
    위험의 성격을 바꾸므로, 조건부 판단이 있다면 여기서 갈려야 한다.
    """
    if params.veh_speed_mps < 0.5:
        return "정지 차량"
    fast = params.veh_speed_mps >= 11.0
    a = params.approach_deg % 360
    frontal = a <= 45 or a >= 315
    side = 45 < a <= 135 or 225 <= a < 315
    where = "정면" if frontal else ("측면" if side else "후방")
    return f"{where} · {'고속' if fast else '저속'}"


def first_alarm_ttc(df, alarm, scenario_ids):
    """시나리오별 (첫 경보 시점의 TTC, 그때의 경보 여유시간).

    여유시간을 같이 보는 이유는 TTC만으로는 그림이 읽히지 않기 때문이다. 옆으로
    지나가는 차는 시선방향 접근속도가 0에 가까워 TTC가 상한(30초)에 붙는데, 그것은
    "위험이 없다"가 아니라 "TTC라는 자로는 잴 수 없다"는 뜻이다.
    """
    out = {}
    for sid in scenario_ids:
        rows = df[df.scenario_id == sid]
        mask = alarm[rows.index.to_numpy()]
        if not mask.any():
            continue
        t_alarm = float(rows.t.to_numpy()[mask][0])
        hits = rows.t_hit.to_numpy()
        hits = hits[~np.isnan(hits)]
        lead = float(hits.min()) - t_alarm if len(hits) else float("nan")
        out[sid] = (float(rows.ttc.to_numpy()[mask][0]), lead)
    return out


def collect(n_scenarios, seed=0):
    ds = build_dataset(n_scenarios, gps_sigma_m=2.5, lead_s=2.0)
    train_df, test_df = split_by_scenario(ds, test_ratio=0.3, seed=seed)

    levels = team_table_levels({c: test_df[c].to_numpy() for c in TEAM_TABLE_INPUTS})
    a_alarm = levels >= ALARM_LEVEL
    target_far = float(a_alarm[test_df.y.to_numpy() == 0].mean())

    clf = train(train_df, seed=seed, label_col="y_train")
    proba = predict_proba(clf, test_df)
    c_alarm = proba >= threshold_at_false_alarm_rate(
        proba, test_df.y.to_numpy(), target=target_far
    )

    # 위험이 실제로 있었던 시나리오만 본다. 안전한 시나리오의 경보는 오경보다.
    dangerous = test_df.loc[test_df.y.to_numpy() == 1, "scenario_id"].unique()
    model_ttc = first_alarm_ttc(test_df, c_alarm, dangerous)
    table_ttc = first_alarm_ttc(test_df, a_alarm, dangerous)

    groups = {}
    for sid, pair in model_ttc.items():
        groups.setdefault(classify(sample_params(sid)), []).append(pair)
    return groups, model_ttc, table_ttc


TTC_UNUSABLE_S = 29.9  # 상한(30s)에 붙었다 = TTC라는 자로는 잴 수 없는 상황


def render(groups, model_ttc, table_ttc, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    for name in ("Malgun Gothic", "NanumGothic", "AppleGothic"):
        if any(f.name == name for f in font_manager.fontManager.ttflist):
            plt.rcParams["font.family"] = name
            break
    plt.rcParams["axes.unicode_minus"] = False

    labels = [k for k in groups if len(groups[k]) >= 2]
    labels.sort(key=lambda k: np.mean([t >= TTC_UNUSABLE_S for t, _ in groups[k]]))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13.5, 5.4),
                                   gridspec_kw={"width_ratios": [1.15, 1]})
    positions = np.arange(len(labels))

    # --- 왼쪽: TTC가 통하는 상황과 통하지 않는 상황 ---
    usable_frac, counts = [], []
    for k in labels:
        ttcs = [t for t, _ in groups[k]]
        usable_frac.append(float(np.mean([t < TTC_UNUSABLE_S for t in ttcs])))
        counts.append(len(ttcs))

    ax1.barh(positions, usable_frac, height=0.6, color="#4a7fb5", label="TTC로 잡히는 위험")
    ax1.barh(positions, [1 - f for f in usable_frac], left=usable_frac, height=0.6,
             color="#c0392b", alpha=0.85, label="TTC가 무한대인 위험")
    for i, (f, n) in enumerate(zip(usable_frac, counts)):
        if 1 - f > 0.08:
            ax1.text(f + (1 - f) / 2, i, f"{(1-f)*100:.0f}%", ha="center", va="center",
                     color="white", fontsize=10, fontweight="bold")
        ax1.text(1.02, i, f"n={n}", va="center", fontsize=8.5, color="#666")

    ax1.set_yticks(positions)
    ax1.set_yticklabels(labels)
    ax1.set_xlim(0, 1.1)
    ax1.set_xticks([0, 0.25, 0.5, 0.75, 1.0])
    ax1.set_xticklabels(["0%", "25%", "50%", "75%", "100%"])
    ax1.set_xlabel("모델이 경보한 위험 중 비율")
    ax1.set_title("TTC 임계값으로는 원리적으로 못 잡는 위험이 있다", fontsize=12, pad=10)
    ax1.legend(loc="lower right", fontsize=9, framealpha=0.95)
    ax1.grid(axis="x", alpha=0.25)

    # --- 오른쪽: 경보 여유시간은 상황마다 다르다 ---
    leads = [[l for _, l in groups[k] if np.isfinite(l)] for k in labels]
    ax2.boxplot(leads, positions=positions, vert=False, widths=0.55, patch_artist=True,
                boxprops=dict(facecolor="#cfe3f7", edgecolor="#4a7fb5"),
                medianprops=dict(color="#1f4e79", linewidth=2),
                whiskerprops=dict(color="#4a7fb5"), capprops=dict(color="#4a7fb5"),
                flierprops=dict(marker=".", markersize=4, markerfacecolor="#7f7f7f",
                                markeredgecolor="none"))
    for i, v in enumerate(leads):
        ax2.scatter(v, np.full(len(v), i) + 0.28, s=14, color="#2f6ea5", alpha=0.55, zorder=3)

    ax2.axvline(T_FLOOR_S, color="#c0392b", linestyle="--", linewidth=2)
    ax2.text(T_FLOOR_S + 0.15, -0.75,
             f"안전 하한 {T_FLOOR_S:.1f}s (AI와 무관하게 발동)",
             color="#c0392b", va="bottom", fontsize=9)
    ax2.set_yticks(positions)
    ax2.set_yticklabels(labels, fontsize=9)
    ax2.set_xlabel("경보 여유시간 = 접촉 시각 빼기 경보 시각 (초)")
    ax2.set_title("상황마다 다른 시점에 경보한다", fontsize=12, pad=10)
    ax2.grid(axis="x", alpha=0.25)
    ax2.set_xlim(left=0)
    ax2.set_ylim(-1.1, len(labels) - 0.4)
    ax1.set_ylim(-1.1, len(labels) - 0.4)

    all_ttc = [t for t, _ in model_ttc.values()]
    unusable = float(np.mean([t >= TTC_UNUSABLE_S for t in all_ttc]))
    fig.suptitle("우리는 TTC 임계값을 정하지 않았다 - 상황이 정한다",
                 fontsize=14, y=0.99)
    fig.text(0.5, 0.015,
             f"위험 시나리오 {len(model_ttc)}개 · 전체의 {unusable*100:.0f}%는 "
             f"TTC가 무한대(접근속도 거의 0)인 상태에서 경보됨 · "
             f"오경보 예산은 현 점수표와 동일하게 고정",
             ha="center", fontsize=9, color="#555")

    fig.tight_layout(rect=[0, 0.04, 1, 0.96])
    fig.savefig(out_path, dpi=150)
    return out_path


def main():
    parser = argparse.ArgumentParser(description="상황별 경보 TTC 분포를 그린다.")
    parser.add_argument("--n-scenarios", type=int, default=1500)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", default="alarm_ttc_by_situation.png")
    args = parser.parse_args()

    groups, model_ttc, table_ttc = collect(args.n_scenarios, seed=args.seed)

    print(f"위험 시나리오 {len(model_ttc)}개에서 모델의 첫 경보\n")
    print(f"{'상황':16s}{'개수':>5}{'TTC무한':>9}{'여유(중앙)':>11}")
    for k in sorted(groups, key=lambda k: -len(groups[k])):
        v = groups[k]
        unusable = np.mean([t >= TTC_UNUSABLE_S for t, _ in v])
        lead = [l for _, l in v if np.isfinite(l)]
        lead_med = np.median(lead) if lead else float("nan")
        print(f"{k:16s}{len(v):5d}{unusable*100:8.0f}%{lead_med:10.2f}s")

    all_ttc = [t for t, _ in model_ttc.values()]
    print(f"\n전체 {np.mean([t >= TTC_UNUSABLE_S for t in all_ttc])*100:.0f}%는 "
          f"TTC가 무한대인 상태에서 경보됨 (TTC 임계값으로는 원리적으로 불가)")
    leads = [np.median([l for _, l in v if np.isfinite(l)])
             for v in groups.values() if len(v) >= 2 and any(np.isfinite(l) for _, l in v)]
    if len(leads) >= 2:
        print(f"유형별 여유시간 중앙값 폭: {min(leads):.2f}s ~ {max(leads):.2f}s")

    path = render(groups, model_ttc, table_ttc, Path(args.out))
    print(f"\n저장: {path}")


if __name__ == "__main__":
    main()
