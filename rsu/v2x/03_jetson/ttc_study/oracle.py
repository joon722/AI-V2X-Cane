#!/usr/bin/env python3
"""정답 라벨: "그 시점에 실제로 위험했는가".

이 모듈이 연구 전체의 정답지다. 그래서 규칙 쪽(step7_risk의 점수표, TTC 임계값,
DCPA 게이트)을 절대 참조하지 않는다. 규칙으로 만든 정답으로 규칙을 검증하면
순환 논리가 되고, 모델을 학습시켜도 규칙을 복사할 뿐이다. 현재 파이프라인의
build_training_dataset.py가 정확히 그 상태이며, 이 모듈은 거기서 벗어나기 위해 있다.

판정은 미래 정보와 좌표 기하학만 쓴다.

    d_min_future(t) = min{ 거리(tau) : tau in (t, t+H] }
    y(t)            = 1 if d_min_future(t) <= d_crit else 0
    t_hit(t)        = 그 최소가 일어나는 시각

현실에서는 미래를 볼 수 없지만 시뮬레이션에서는 볼 수 있다. 그 비대칭이 이
방법의 전부다 - 블랙박스 영상을 되감아 "이 순간 경보했어야 했나"를 사후에
판정하는 것과 같다.

교차검증용으로 PET(Post-Encroachment Time)도 계산한다. 한쪽이 어떤 지점을 떠난 뒤
다른 쪽이 그 지점에 도달하기까지의 시간으로, 교통공학에서 상충(conflict)을 재는
표준 지표다. 정의가 전혀 다른 두 방법이 같은 시나리오를 같게 판정하면 라벨
정의가 특정 수식에 기대고 있지 않다는 증거가 된다.
"""

import math
from dataclasses import dataclass

import numpy as np

from scenario_sim import distances

# 접촉으로 볼 거리. 차폭 절반(약 0.9m) + 보행자 신체폭 + 지팡이가 뻗는 궤적.
# 이 값에 결론이 얼마나 민감한지는 sensitivity()로 확인한다.
D_CRIT_M = 2.0

# 미래를 내다보는 창. 3초인 근거는 셋이 겹친다.
#
#   반응 시간   safety_floor.T_FLOOR_S가 2.0초다. 위험을 알아채고 사람이 멈추기까지
#               그만큼 걸리므로, 창이 2초면 경보할 시간이 정확히 0이 된다. 여유 1초를
#               더한 3초가 하한이다.
#   예측 가능성 창을 늘릴수록 맞히기 어려워진다. 실측 AUC가 2초 0.96, 3초 0.71,
#               10초 0.54로 떨어졌다. 10초 뒤에 차가 무엇을 할지는 현재 정보에
#               담겨 있지 않으므로, 그것을 라벨로 삼으면 맞힐 수 없는 문제를 낸다.
#   비교 대상   물리 외삽(features.PHYS_HORIZON_S)과 팀 v3가 모두 3초를 쓴다. 창이
#               다르면 규칙과 모델이 서로 다른 문제를 푸는 셈이라 비교가 성립하지 않는다.
HORIZON_S = 3.0


@dataclass(frozen=True)
class OracleLabels:
    y: np.ndarray             # 0/1 정답
    d_min_future: np.ndarray  # 앞으로 horizon 안의 실제 최소거리
    t_hit: np.ndarray         # 그 최소가 일어나는 시각 (없으면 nan)


def label_scenario(scenario, d_crit=D_CRIT_M, horizon_s=HORIZON_S, lead_s=0.0):
    """시나리오 하나를 시점별 정답으로 바꾼다.

    창은 [t+lead_s, t+horizon_s]다. 현재를 제외하는 이유는 이미 벌어진 접촉이
    계속 위험으로 남아 늦게 울리는 판정기가 좋아 보이는 것을 막기 위해서다.

    lead_s는 그보다 강한 요구다. 0으로 두면 0.5초 뒤에 부딪히는 시점도 정답이
    되는데, 그 순간에 울려봐야 사람은 반응할 수 없다. 모델에게 쓸모없는 경보를
    정답으로 가르치는 셈이다. safety_floor.T_FLOOR_S만큼 밀면 "지금 울려야 피할
    수 있는 위험"만 정답으로 남는다.
    """
    if lead_s >= horizon_s:
        raise ValueError(f"lead_s({lead_s})가 horizon_s({horizon_s}) 이상이면 창이 빈다")

    t = scenario.t
    d = distances(scenario)
    n = len(t)
    dt = t[1] - t[0] if n > 1 else 0.0
    span = max(int(round(horizon_s / dt)), 1) if dt > 0 else 1
    lead_steps = max(int(round(lead_s / dt)), 1) if dt > 0 else 1

    d_min = np.empty(n)
    t_hit = np.empty(n)
    for i in range(n):
        lo, hi = i + lead_steps, min(i + 1 + span, n)
        if lo >= hi:
            d_min[i] = math.inf
            t_hit[i] = math.nan
            continue
        window = d[lo:hi]
        k = int(window.argmin())
        d_min[i] = window[k]
        t_hit[i] = t[lo + k]

    y = (d_min <= d_crit).astype(np.int8)
    t_hit = np.where(y == 1, t_hit, math.nan)
    return OracleLabels(y=y, d_min_future=d_min, t_hit=t_hit)


def first_contact_time(scenario, d_crit=D_CRIT_M):
    """각 시점에서 볼 때 실제로 접촉이 일어나는 첫 시각. 없으면 nan.

    평가 전용이다. 학습 라벨의 t_hit은 창(horizon, lead) 안쪽만 보므로, 창을 밀면
    그 값도 같이 밀려 "여유 시간"이 자동으로 커진다. 경보가 얼마나 일찍 울렸는지는
    창과 무관하게 실제 접촉 시각을 기준으로 재야 한다.
    """
    t = scenario.t
    d = distances(scenario)
    out = np.full(len(t), math.nan)
    upcoming = math.nan
    for i in range(len(t) - 1, -1, -1):
        if d[i] <= d_crit:
            upcoming = float(t[i])
        out[i] = upcoming
    return out


def pet_seconds(scenario, d_crit=D_CRIT_M):
    """Post-Encroachment Time: 두 궤적이 같은 자리를 지난 시간차의 최소값.

    보행자가 시각 t_p에 있던 자리를 차가 시각 t_v에 지났다면 |t_v - t_p|가 그
    지점의 PET다. 여기서는 "같은 자리"를 d_crit 이내로 본다. 경로가 아예 겹치지
    않으면 정의되지 않으므로 None을 준다.

    O(N^2)이지만 시나리오 하나가 수백 스텝이므로 문제되지 않는다. 이 값은 학습에
    쓰지 않고 라벨 정의를 교차검증하는 데만 쓴다.
    """
    t = scenario.t
    dx = scenario.veh.x[:, None] - scenario.ped.x[None, :]
    dy = scenario.veh.y[:, None] - scenario.ped.y[None, :]
    close = np.hypot(dx, dy) <= d_crit          # [차 시각, 보행자 시각]
    if not close.any():
        return None

    lag = np.abs(t[:, None] - t[None, :])
    return float(lag[close].min())


def sensitivity(scenario, d_crits=(1.5, 2.0, 2.5), horizon_s=HORIZON_S):
    """d_crit을 흔들었을 때 위험 시점 수가 얼마나 달라지는지.

    "1.5m로 해도 2.5m로 해도 결론은 같았다"를 말할 수 있어야 이 라벨을 방어할 수
    있으므로, 민감도를 따로 재는 자리를 둔다.
    """
    return {
        d: int(label_scenario(scenario, d_crit=d, horizon_s=horizon_s).y.sum())
        for d in d_crits
    }


def main():
    import argparse

    from scenario_sim import sample_params, simulate

    parser = argparse.ArgumentParser(description="시나리오 하나의 오라클 라벨 요약.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--d-crit", type=float, default=D_CRIT_M)
    parser.add_argument("--horizon-s", type=float, default=HORIZON_S)
    args = parser.parse_args()

    sc = simulate(sample_params(args.seed), duration_s=None)
    lab = label_scenario(sc, d_crit=args.d_crit, horizon_s=args.horizon_s)
    pet = pet_seconds(sc, d_crit=args.d_crit)

    print(f"steps={len(sc.t)}  위험시점={int(lab.y.sum())}  "
          f"PET={'없음' if pet is None else f'{pet:.2f}s'}")
    print(f"민감도(d_crit별 위험시점): {sensitivity(sc, horizon_s=args.horizon_s)}")


if __name__ == "__main__":
    main()
