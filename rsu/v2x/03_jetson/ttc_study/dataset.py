#!/usr/bin/env python3
"""시나리오 여러 개를 하나의 채점용 표로 묶는다.

베이스라인(현 점수표, 고정 임계값)과 모델이 같은 표를 봐야 비교가 성립하므로
조립을 한 곳에 모은다.

두 가지를 구조적으로 지킨다.

  정답은 참값에서, 피처는 관측에서
      라벨까지 노이즈 낀 좌표로 만들면 "GPS가 흔들려서 위험해 보인 순간"을 진짜
      위험으로 가르치게 된다. 젯슨이 풀어야 할 문제는 정확히 그 반대다.

  분할은 시나리오 단위로
      같은 시나리오의 앞이 학습에, 뒤가 평가에 들어가면 모델이 정답을 미리 본
      셈이 된다. 시계열에서 가장 흔한 누수다.
"""

import numpy as np
import pandas as pd

from features import FEATURE_COLUMNS, build_features
from oracle import D_CRIT_M, HORIZON_S, first_contact_time, label_scenario
from scenario_sim import sample_params, simulate

GPS_SIGMA_M = 2.5  # gps_noise.py의 기본값(8/1 야외 실측 근거)과 같다


def build_dataset(
    n_scenarios,
    gps_sigma_m=GPS_SIGMA_M,
    d_crit=D_CRIT_M,
    horizon_s=HORIZON_S,
    lead_s=0.0,
    seed_offset=0,
    velocity_mode="kalman",
):
    """시나리오 n개를 시점 단위 표로 편다.

    시드는 시나리오마다 다르게 주되 seed_offset으로 통째로 옮길 수 있다. 학습용과
    평가용을 아예 다른 시드 대역에서 뽑고 싶을 때 쓴다.

    lead_s는 라벨 창을 그만큼 미뤄, 반응할 수 없는 코앞의 위험을 정답에서 뺀다.
    t_hit은 그 창과 무관하게 실제 접촉 시각을 담는다 - 창을 밀면 창 안의 t_hit도
    같이 밀려 여유 시간이 자동으로 커지므로, 평가 기준은 따로 두어야 한다.
    """
    frames = []
    for i in range(n_scenarios):
        seed = seed_offset + i
        scenario = simulate(sample_params(seed), duration_s=None)

        # 라벨은 노이즈를 타지 않은 참값 궤적에서.
        # y는 평가 기준이라 창을 밀지 않는다. 밀어버리면 코앞의 위험에서 울린 경보가
        # "안전한데 울렸다"로 집계되어 오경보율이 부풀려진다. 늦은 경보는 늦은
        # 것이지 헛경보가 아니다.
        labels = label_scenario(scenario, d_crit=d_crit, horizon_s=horizon_s)
        # y_train은 학습용이다. 창을 T_floor만큼 밀어 "지금 울려야 피할 수 있는
        # 위험"만 정답으로 남긴다.
        train_labels = (
            labels if lead_s <= 0
            else label_scenario(scenario, d_crit=d_crit, horizon_s=horizon_s, lead_s=lead_s)
        )
        # 피처는 현장과 같은 관측에서
        feats = build_features(
            scenario, gps_sigma_m=gps_sigma_m, seed=seed, velocity_mode=velocity_mode
        )

        frame = pd.DataFrame({name: feats[name] for name in FEATURE_COLUMNS})
        frame["y"] = labels.y
        frame["y_train"] = train_labels.y
        frame["t"] = scenario.t
        frame["t_hit"] = first_contact_time(scenario, d_crit=d_crit)
        frame["d_min_future"] = labels.d_min_future
        frame["scenario_id"] = seed
        frames.append(frame)

    return pd.concat(frames, ignore_index=True)


def split_by_scenario(df, test_ratio=0.3, seed=0):
    """시나리오 단위로 학습/평가를 가르되, 위험 시나리오를 양쪽에 비례 배분한다.

    시나리오 단위인 이유는 누수 방지다. 같은 시나리오의 앞이 학습에, 뒤가 평가에
    들어가면 모델이 정답을 미리 본 셈이 된다.

    층화하는 이유는 쏠림 방지다. 위험 시나리오는 전체의 6% 남짓이라 그냥 섞으면
    한쪽으로 몰린다. 실제로 60개를 나눴을 때 위험 5개 중 4개가 평가로 가서 학습에
    1개만 남았고, 모델이 아무것도 배우지 못해 AUC가 0.49(무작위)로 나왔다.
    """
    per_scenario = df.groupby("scenario_id").y.max() > 0
    rng = np.random.default_rng(np.random.SeedSequence(seed))

    test_ids = []
    for ids in (per_scenario[per_scenario].index.to_numpy(),
                per_scenario[~per_scenario].index.to_numpy()):
        ids = np.array(sorted(ids))
        if len(ids) == 0:
            continue
        rng.shuffle(ids)
        n_test = max(int(round(len(ids) * test_ratio)), 1)
        test_ids.extend(ids[:n_test].tolist())

    is_test = df.scenario_id.isin(set(test_ids))
    return df[~is_test].reset_index(drop=True), df[is_test].reset_index(drop=True)


def main():
    import argparse

    parser = argparse.ArgumentParser(description="데이터셋을 만들어 요약을 찍는다.")
    parser.add_argument("--n-scenarios", type=int, default=200)
    parser.add_argument("--gps-sigma-m", type=float, default=GPS_SIGMA_M)
    parser.add_argument("--out", default=None, help="CSV로 저장할 경로")
    args = parser.parse_args()

    ds = build_dataset(args.n_scenarios, gps_sigma_m=args.gps_sigma_m)
    print(f"시나리오 {ds.scenario_id.nunique()}개 / {len(ds):,}행")
    print(f"위험 시점 {int(ds.y.sum()):,} ({ds.y.mean()*100:.2f}%)")
    print(f"위험이 있는 시나리오 {int((ds.groupby('scenario_id').y.max() > 0).sum())}개")
    if args.out:
        ds.to_csv(args.out, index=False)
        print(f"저장: {args.out}")


if __name__ == "__main__":
    main()
