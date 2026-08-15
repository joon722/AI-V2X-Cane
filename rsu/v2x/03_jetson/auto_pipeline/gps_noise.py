"""SUMO의 완벽한 좌표에 실제 GPS 오차를 입힌다.

SUMO는 오차 없는 참값을 준다. 반면 현장의 지팡이/차량은 GPS로 위치를 얻으므로
몇 미터씩 흔들린 좌표를 받는다. 그 격차(sim-to-real gap)를 그대로 두면
완벽한 좌표에서만 잘 도는 판정 로직이 만들어진다. 특히 DCPA는 위치를 미분해
상대속도를 구하므로 위치 노이즈에 민감하다.

오차 모델은 1차 Gauss-Markov 과정을 쓴다. 실제 GPS 오차는 매 순간 새로 튀는
백색잡음이 아니라 수십 초 동안 한쪽으로 치우쳐 흘러가기 때문이다(전리층 지연,
다중경로, 위성 배치). 축(x, y)마다 독립이고, 객체마다 독립이다.

    e[k] = phi * e[k-1] + w[k],  phi = exp(-dt / tau)

정상상태 분산을 sigma^2으로 유지하려고 w의 표준편차를 sigma*sqrt(1-phi^2)로 두고,
첫 표본도 N(0, sigma)에서 뽑는다. 0에서 출발시키면 시나리오 앞부분이 실제보다
정확해져서 판정이 낙관적으로 나온다.

sigma 기본값 2.5m의 근거: 2026-08-01 야외 정지 세션 실측에서 지팡이 GPS의
위치 흩어짐이 sigma 약 2m(최악 4m)로 측정되었고, step7_risk.py가 DCPA 게이트
임계값을 유도할 때 쓰는 GPS_SIGMA_M 과 같은 값이다.

이상치(순간적으로 크게 튀는 값)는 넣지 않는다. 위 실측값은 펌웨어의 이상치
제거를 거친 뒤의 좌표에서 측정한 것이고, 파이프라인이 받는 것도 같은 단계의
값이기 때문이다.

process_scenarios.py의 build_features() 안에서, 좌표를 읽은 직후이자
distance_m 을 계산하기 전에 부른다. 그 위치여야 거리/TTC/risk_score/DCPA가
전부 오차 있는 좌표를 쓰게 되어 현장과 같아진다.

    from gps_noise import add_gps_noise
    df = add_gps_noise(df, seed=scenario_seed(scenario_dir.name))

위치 정확도가 달라지면 판정이 얼마나 흔들리는지 보려면 sigma_m 만 바꿔
같은 원본을 다시 돌려 비교한다.
"""
import math
import zlib

import numpy as np

# 실측 기반 기본값. step7_risk.GPS_SIGMA_M 과 같은 값을 쓴다.
DEFAULT_SIGMA_M = 2.5
# 오차가 얼마나 오래 같은 방향으로 머무는지(초). GPS 표준적인 범위.
DEFAULT_TAU_S = 60.0

TIME_COL = "timestep_time"
VEHICLE_COL = "vehicle_id"
VEHICLE_XY = ("veh_x", "veh_y")
PED_XY = ("ped_x", "ped_y")


def scenario_seed(name):
    """시나리오 이름 -> 안정적인 정수 seed.

    같은 시나리오를 다시 처리하면 같은 오차가 재현되도록 한다. 파이썬 hash()는
    실행마다 달라지므로 쓰지 않는다.
    """
    return zlib.crc32(str(name).encode("utf-8"))


def _gauss_markov(times, sigma_m, tau_s, rng):
    """시간 상관을 갖는 1축 위치 오차를 times 길이만큼 생성."""
    n = len(times)
    error = np.empty(n, dtype=float)
    error[0] = rng.normal(0.0, sigma_m)
    for i in range(1, n):
        dt = times[i] - times[i - 1]
        phi = math.exp(-dt / tau_s) if dt > 0 else 1.0
        drive = sigma_m * math.sqrt(max(0.0, 1.0 - phi * phi))
        error[i] = phi * error[i - 1] + rng.normal(0.0, drive)
    return error


def _offset_track(times, sigma_m, tau_s, seed_parts):
    """한 객체의 (x오차, y오차). 축끼리도 독립이다."""
    rng = np.random.default_rng(list(seed_parts))
    return (_gauss_markov(times, sigma_m, tau_s, rng),
            _gauss_markov(times, sigma_m, tau_s, rng))


def add_gps_noise(df, sigma_m=DEFAULT_SIGMA_M, tau_s=DEFAULT_TAU_S, seed=0):
    """차량·보행자 좌표에 GPS 오차를 입힌 df를 반환 (제자리 수정).

    차량은 vehicle_id 마다 독립적인 오차 궤적을 갖는다. 보행자는 한 명이므로
    시각별로 하나의 궤적을 만들어 모든 차량 행에 같은 값을 적용한다 - 같은 순간의
    보행자가 차량마다 다른 곳에 있으면 안 되기 때문이다.
    """
    if sigma_m <= 0:
        return df

    times = df[TIME_COL].to_numpy(dtype=float)

    ped_times = np.unique(times)
    ped_ex, ped_ey = _offset_track(ped_times, sigma_m, tau_s, (seed, 0))
    at = np.searchsorted(ped_times, times)
    df[PED_XY[0]] = df[PED_XY[0]].to_numpy(dtype=float) + ped_ex[at]
    df[PED_XY[1]] = df[PED_XY[1]].to_numpy(dtype=float) + ped_ey[at]

    vehicle_ids = df[VEHICLE_COL].to_numpy()
    veh_x = df[VEHICLE_XY[0]].to_numpy(dtype=float, copy=True)
    veh_y = df[VEHICLE_XY[1]].to_numpy(dtype=float, copy=True)
    for vid in np.unique(vehicle_ids):
        rows = np.flatnonzero(vehicle_ids == vid)
        order = rows[np.argsort(times[rows], kind="stable")]
        ex, ey = _offset_track(times[order], sigma_m, tau_s,
                               (seed, 1, scenario_seed(vid)))
        veh_x[order] += ex
        veh_y[order] += ey
    df[VEHICLE_XY[0]] = veh_x
    df[VEHICLE_XY[1]] = veh_y
    return df
