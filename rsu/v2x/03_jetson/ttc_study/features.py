#!/usr/bin/env python3
"""관측 궤적을 모델 입력 피처로.

무엇을 넣지 않았는지가 무엇을 넣었는지만큼 중요하다.

  risk_score 없음   현재 scaler.json은 입력에 risk_score를 넣고, 그 risk_score로
                    만든 risk_level을 맞히게 했다. 답을 문제지에 적어둔 것이라
                    모델의 정확도 수치가 의미를 잃는다.
  절대좌표 없음     "이 지도의 이 지점"은 다른 장소로 일반화되지 않는다. 위험을
                    정하는 것은 두 물체의 상대 기하학이다.
  TTC 상한          비접근 쌍에 9999를 넣던 관행을 버린다. 그 값이 정규화에 섞여
                    scaler.json의 ttc 평균이 6198이 되었고, 그러면 모델이 TTC를
                    사실상 쓰지 못한다. 30초로 자른다.

가져다 쓴 것:
  dx, dy, dvx, dvy, dcpa   팀의 v2 벡터 특징. "다가오는 차"와 "스쳐 갈 차"는
                           거리와 속력만으로 구분되지 않고 상대 벡터가 있어야
                           갈린다는 판단이 맞다.
  phys_min_dist_3s,        팀의 v3 물리 외삽. 등속을 가정해 3초 앞의 최소 거리를
  phys_t_cpa_3s            미리 계산한다. 현재 정보만 쓰므로 현장에서도 계산된다.

phys_* 와 oracle.d_min_future 의 차이가 이 연구의 핵심이다. 전자는 등속으로
'예측'한 값이고 후자는 미래를 봐서 '실측'한 값이다. 차가 꺾거나 GPS가 흔들리면
둘이 벌어지고, 그 격차가 곧 모델이 배워야 할 몫이다.

좌표는 ENU 미터 평면이고 계산은 step6_kinematics.relative_kinematics를 그대로
쓴다. 파이프라인과 다른 식을 쓰면 여기서 만든 라벨이 현장 데이터에 맞지 않는다.
"""

import sys
from pathlib import Path

import numpy as np

_JETSON = Path(__file__).resolve().parent.parent
for _p in (_JETSON, _JETSON / "auto_pipeline"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from gps_noise import _gauss_markov  # noqa: E402
from step6_kinematics import KalmanCV1D, relative_kinematics  # noqa: E402


# 30초 너머의 TTC는 위험 판단에 아무 정보도 주지 않는다. 자르지 않으면 비접근
# 구간의 거대한 값이 정규화를 지배한다(scaler.json의 실패 사례).
TTC_MAX_S = 30.0

# 물리 외삽이 내다보는 시간. 팀 v3와 같은 3초를 쓴다.
PHYS_HORIZON_S = 3.0

# GPS 오차 상관시간. gps_noise.py의 기본값과 같다.
GPS_TAU_S = 30.0

# 상대 가속·방위각 변화율의 물리적 상한. 차의 제동이 -7 m/s^2, 가속이 3.5 m/s^2
# 수준이고 보행자 몫을 더해도 이 범위를 넘지 않는다. 넘는 값은 통과 순간의
# 기하학적 특이점이지 실제 운동이 아니다.
MAX_REL_ACCEL_MPS2 = 20.0
MAX_BEARING_RATE_DPS = 180.0

FEATURE_COLUMNS = (
    "distance_m",
    "closing_los",
    "ttc",
    "veh_speed_mps",
    "ped_speed_mps",
    "dx",
    "dy",
    "dvx",
    "dvy",
    "dcpa_m",
    "tcpa_s",
    "phys_min_dist_3s",
    "phys_t_cpa_3s",
    # 아래 둘이 없으면 모델이 보는 정보가 물리 외삽과 똑같아진다. 한 시점의
    # 위치·속도만으로는 "지금 등속이 아니다"를 알 길이 없고, 그러면 등속 가정을
    # 깨는 근거를 모델이 가질 수 없다.
    "d_closing_dt",   # 접근속도의 변화율 = 상대 가속. 제동·가속을 드러낸다.
    "d_heading_dt",   # 상대 방위각의 변화율(도/초) = 선회를 드러낸다.
)


def _noisy_xy(x, y, t, sigma_m, rng):
    """좌표에 Gauss-Markov GPS 오차를 입힌다.

    백색잡음이 아니라 상관 오차를 쓰는 이유는 gps_noise.py에 적혀 있다 - 실제
    GPS 오차는 매 순간 새로 튀지 않고 수십 초 동안 한쪽으로 흘러간다.
    """
    return x + _gauss_markov(t, sigma_m, GPS_TAU_S, rng), \
           y + _gauss_markov(t, sigma_m, GPS_TAU_S, rng)


def _velocity_from_positions(x, y, dt):
    """관측 좌표를 단순 미분해 속도를 얻는다. 비교용 하한선.

    노이즈가 낀 위치를 그대로 미분하면 속도 오차가 크게 증폭된다. 실제
    파이프라인은 이렇게 하지 않으므로, 이 모드로 잰 숫자를 베이스라인이라고
    부르면 규칙 쪽을 부당하게 약하게 만드는 것이 된다.
    """
    return np.gradient(x, dt), np.gradient(y, dt)


def _kalman_track(x, y, t, sigma_pos):
    """step6와 같은 등속 칼만으로 위치·속도를 추정한다.

    실제 파이프라인(step6_kinematics -> step7_risk)이 채점하는 것이 이 필터를
    거친 값이므로, 규칙과 모델을 같은 조건에서 비교하려면 여기서도 같은 필터를
    써야 한다. 축이 독립이라 x, y에 하나씩 둔다 - step6가 그렇게 하는 이유와 같다.

    첫 관측에서 속도를 0으로 잡고 수렴해 가는 것도 현장 거동 그대로다. 참값을
    넣어 출발시키면 시나리오 앞부분이 실제보다 정확해진다.
    """
    fx, fy = KalmanCV1D(sigma_pos=sigma_pos), KalmanCV1D(sigma_pos=sigma_pos)
    n = len(t)
    px, py, vx, vy = (np.zeros(n) for _ in range(4))
    for i in range(n):
        dt = float(t[i] - t[i - 1]) if i else 0.0
        fx.observe(float(x[i]), dt)
        fy.observe(float(y[i]), dt)
        px[i], vx[i] = fx.pos, fx.vel
        py[i], vy[i] = fy.pos, fy.vel
    return px, py, vx, vy


def _phys_extrapolation(rx, ry, vx, vy, horizon_s):
    """등속 가정으로 horizon 안의 최소 거리와 그 시각을 구한다.

    r(tau) = r + v*tau 의 최소값. tau를 [0, horizon]으로 자르므로, 최근접이 이미
    지났거나 창 밖이면 경계값이 답이 된다.
    """
    v2 = vx * vx + vy * vy
    with np.errstate(divide="ignore", invalid="ignore"):
        t_star = np.where(v2 > 0, -(rx * vx + ry * vy) / np.where(v2 > 0, v2, 1.0), 0.0)
    t_star = np.clip(t_star, 0.0, horizon_s)
    return np.hypot(rx + vx * t_star, ry + vy * t_star), t_star


def build_features(scenario, gps_sigma_m=0.0, seed=0, velocity_mode="kalman"):
    """시나리오 하나를 시점별 피처 딕셔너리로.

    gps_sigma_m=0 이면 참값 궤적을 그대로 쓴다(검증용). 학습·평가에는 현장과 같은
    노이즈를 넣는다.

    velocity_mode는 노이즈가 있을 때만 의미가 있다.
      kalman  step6와 같은 등속 칼만. 실제 파이프라인과 같은 조건이므로 기본값이다.
      diff    단순 미분. 필터 없이 얼마나 나빠지는지 보여주는 비교용 하한선.
    """
    t = scenario.t
    dt = float(t[1] - t[0]) if len(t) > 1 else 0.1

    ped_x, ped_y = scenario.ped.x, scenario.ped.y
    veh_x, veh_y = scenario.veh.x, scenario.veh.y
    ped_vx, ped_vy = scenario.ped.vx, scenario.ped.vy
    veh_vx, veh_vy = scenario.veh.vx, scenario.veh.vy

    if gps_sigma_m > 0:
        rng = np.random.default_rng(np.random.SeedSequence(seed))
        ped_x, ped_y = _noisy_xy(ped_x, ped_y, t, gps_sigma_m, rng)
        veh_x, veh_y = _noisy_xy(veh_x, veh_y, t, gps_sigma_m, rng)
        if velocity_mode == "kalman":
            ped_x, ped_y, ped_vx, ped_vy = _kalman_track(ped_x, ped_y, t, gps_sigma_m)
            veh_x, veh_y, veh_vx, veh_vy = _kalman_track(veh_x, veh_y, t, gps_sigma_m)
        elif velocity_mode == "diff":
            ped_vx, ped_vy = _velocity_from_positions(ped_x, ped_y, dt)
            veh_vx, veh_vy = _velocity_from_positions(veh_x, veh_y, dt)
        else:
            raise ValueError(f"velocity_mode는 kalman 또는 diff여야 한다: {velocity_mode!r}")

    n = len(t)
    out = {name: np.zeros(n) for name in FEATURE_COLUMNS}

    dx = veh_x - ped_x
    dy = veh_y - ped_y
    dvx = veh_vx - ped_vx
    dvy = veh_vy - ped_vy

    for i in range(n):
        k = relative_kinematics(
            (ped_x[i], ped_y[i]), (ped_vx[i], ped_vy[i]),
            (veh_x[i], veh_y[i]), (veh_vx[i], veh_vy[i]),
        )
        out["distance_m"][i] = k.distance_m
        out["closing_los"][i] = k.closing_los
        out["ttc"][i] = TTC_MAX_S if k.ttc_simple is None else min(k.ttc_simple, TTC_MAX_S)
        # dcpa/tcpa가 None인 경우는 멀어지는 중이거나 상대속도가 0일 때다. 각각
        # "최근접이 앞에 없다"는 뜻이므로 현재 거리와 0으로 채운다.
        out["dcpa_m"][i] = k.distance_m if k.dcpa is None else k.dcpa
        out["tcpa_s"][i] = 0.0 if k.tcpa is None else min(k.tcpa, TTC_MAX_S)

    out["veh_speed_mps"] = np.hypot(veh_vx, veh_vy)
    out["ped_speed_mps"] = np.hypot(ped_vx, ped_vy)
    out["dx"], out["dy"] = dx, dy
    out["dvx"], out["dvy"] = dvx, dvy
    out["phys_min_dist_3s"], out["phys_t_cpa_3s"] = _phys_extrapolation(
        dx, dy, dvx, dvy, PHYS_HORIZON_S
    )

    # 차가 보행자를 스치듯 통과하는 순간 시선 방향이 뒤집혀 접근속도의 부호가
    # 한 스텝에 반전한다. 등속 주행인데도 변화율이 100 m/s^2 같은 값이 되는데,
    # 이는 기하학이 만든 특이점이지 실제 가속이 아니다. 물리적으로 가능한 범위로
    # 자르지 않으면 이 이상치가 학습을 지배한다.
    out["d_closing_dt"] = np.clip(
        np.gradient(out["closing_los"], dt), -MAX_REL_ACCEL_MPS2, MAX_REL_ACCEL_MPS2
    )
    # 상대 방위각은 ±180도에서 감기므로, 감김을 푼 뒤 미분한다. 같은 통과 순간에
    # 방위각도 180도 뒤집히므로 여기에도 상한을 둔다.
    bearing = np.degrees(np.arctan2(dx, dy))
    out["d_heading_dt"] = np.clip(
        np.gradient(np.unwrap(bearing, period=360.0), dt),
        -MAX_BEARING_RATE_DPS, MAX_BEARING_RATE_DPS,
    )
    return out


def main():
    import argparse

    from scenario_sim import sample_params, simulate

    parser = argparse.ArgumentParser(description="시나리오 하나의 피처 요약.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--gps-sigma-m", type=float, default=2.5)
    args = parser.parse_args()

    sc = simulate(sample_params(args.seed), duration_s=None)
    f = build_features(sc, gps_sigma_m=args.gps_sigma_m, seed=args.seed)
    for name in FEATURE_COLUMNS:
        col = f[name]
        print(f"{name:18s} min={col.min():8.2f}  med={np.median(col):8.2f}  max={col.max():8.2f}")


if __name__ == "__main__":
    main()
