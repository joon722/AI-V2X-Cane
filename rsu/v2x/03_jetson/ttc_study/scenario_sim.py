#!/usr/bin/env python3
"""보행자-차량 2체 운동학 시나리오 생성기.

SUMO를 쓰지 않는 이유는 측정 결과에 있다. 기존 SUMO 데이터 12,621행에서 차-보행자
최소거리가 9.59m로, 위험 사례가 한 건도 없었다. SUMO 차량이 교차로에서 알아서
양보하기 때문이고, 임계값을 검증하려면 위험이 실제로 일어난 궤적이 필요하다.

여기서 필요한 것은 도로망도 신호도 차량추종 모델도 아니고 "차가 보행자 쪽으로
다양한 각도·속도로 접근하는 궤적"뿐이므로 직접 만든다. 대신 기하학이 의도대로
만들어졌는지는 해석해와 대조해 증명한다(test_scenario_sim.py).

좌표계는 ENU 미터 평면이고 보행자가 원점에서 출발한다. 방위각 규약은
step6_kinematics.velocity_from_heading과 같다 - 북이 0도, 시계방향, 즉
(east, north) = (speed*sin, speed*cos). 파이프라인 나머지와 규약이 어긋나면
여기서 만든 라벨이 현장 데이터에 맞지 않는다.

miss_offset_m 이 이 모듈의 핵심 손잡이다. 차의 진행 직선을 보행자로부터 그만큼
옆으로 밀어놓으므로, 등속 직진에 보행자가 정지해 있는 한 실제 최소거리가 정확히
그 값이 된다. "스쳐 갈 차"와 "덮치는 차"를 연속적으로 만들 수 있고, 만든 값이
맞는지도 바로 확인된다.
"""

import math
from dataclasses import dataclass, fields

import numpy as np


# 샘플링 범위. 근거:
#   veh_speed_mps   2~20 m/s = 7~72 km/h. 도심 생활도로부터 간선도로까지.
#   miss_offset_m   0~15 m. 0은 접촉, 15는 명백히 스쳐 감(GPS sigma 2.5m의 6배).
#   ped_speed_mps   0~1.6 m/s. 정지부터 빠른 보행까지. 시각장애인 평균 보행속도는
#                   1.0~1.2 m/s 대로 알려져 있고 기존 로그의 실측도 그 부근이다.
#   veh_accel_mps2  -3~1. 음수는 제동. 위험 상황에서 운전자가 늦게 밟는 경우를 포함.
#   turn_rate_dps   -20~20. DCPA의 등속직진 가정이 깨지는 경우를 만들기 위한 것으로,
#                   이 구간이 규칙과 모델의 차이가 드러나는 지점이다.
PARAM_RANGES = {
    "approach_deg": (0.0, 360.0),
    "start_distance_m": (30.0, 100.0),
    "veh_speed_mps": (0.0, 20.0),
    "miss_offset_m": (0.0, 15.0),
    "ped_speed_mps": (0.0, 1.6),
    "ped_heading_deg": (0.0, 360.0),
    "veh_accel_mps2": (-3.0, 1.0),
    "turn_rate_dps": (-20.0, 20.0),
    # 보행자도 등속 직진만 하지 않는다. 차량에만 가감속·선회를 주면 모델이
    # "보행자는 예측 가능하다"고 잘못 배우는데, 실제로는 지팡이로 탐색하며 걷는
    # 보행자가 더 자주 멈추고 방향을 바꾼다. 감속(-1.5)이 가속(1.0)보다 빠른 것은
    # 사람이 멈추는 쪽이 더 빠르기 때문이고, 선회는 제자리 방향 전환까지 고려해
    # 차량보다 넓게 잡는다.
    "ped_accel_mps2": (-1.5, 1.0),
    "ped_turn_rate_dps": (-30.0, 30.0),
}


@dataclass(frozen=True)
class ScenarioParams:
    approach_deg: float       # 차가 출발하는 방위각 (보행자 기준)
    start_distance_m: float
    veh_speed_mps: float
    miss_offset_m: float      # 차의 진행 직선이 보행자를 비껴가는 거리
    ped_speed_mps: float
    ped_heading_deg: float
    veh_accel_mps2: float
    turn_rate_dps: float
    ped_accel_mps2: float = 0.0
    ped_turn_rate_dps: float = 0.0


@dataclass(frozen=True)
class Track:
    """한 물체의 시간별 상태. 모든 배열은 같은 길이다."""
    x: np.ndarray
    y: np.ndarray
    vx: np.ndarray
    vy: np.ndarray


@dataclass(frozen=True)
class Scenario:
    t: np.ndarray
    ped: Track
    veh: Track
    params: ScenarioParams


def _unit_from_bearing(deg):
    """방위각을 (east, north) 단위벡터로. step6와 같은 규약."""
    rad = math.radians(deg)
    return math.sin(rad), math.cos(rad)


def _integrate(x0, y0, heading_rad, speed, accel, turn_dps, t, dt):
    """등가속 + 등선회율 운동을 궤적으로 편다. 보행자와 차량이 같은 식을 쓴다.

    속도는 0 아래로 내려가지 않는다. 감속하는 보행자는 멈출 뿐 뒤로 걷지 않고,
    제동하는 차도 정지가 실제 거동이다(후진은 이 연구 범위 밖).
    """
    heading_t = heading_rad + math.radians(turn_dps) * t
    speed_t = np.maximum(speed + accel * t, 0.0)
    vx = speed_t * np.sin(heading_t)
    vy = speed_t * np.cos(heading_t)

    # 사다리꼴 적분: 등속·등가속 직선에서 해석해와 일치한다
    x = x0 + np.concatenate([[0.0], np.cumsum((vx[1:] + vx[:-1]) / 2 * dt)])
    y = y0 + np.concatenate([[0.0], np.cumsum((vy[1:] + vy[:-1]) / 2 * dt)])
    return Track(x=x, y=y, vx=vx, vy=vy)


# 선회와 가속은 균등분포로 뽑으면 안 된다. -20~20 dps에서 균등하게 뽑으면 평균
# |10| dps가 되어 5초에 50도를 도는데, 그러면 miss_offset으로 설정한 접근 기하학이
# 무너져 차가 보행자 근처에 오지도 못한다(실측: 위험 사례 2.4%에 그침).
# 실제 차량은 대부분 직진하고 일부만 방향을 바꾼다. 그 구조를 명시적으로 만든다.
TURN_PROB = 0.20            # 이 확률로만 실제 방향 전환
BRAKE_PROB = 0.25           # 이 확률로만 제동
CRUISE_TURN_DPS = 2.0       # 직진 주행이 갖는 미세한 흔들림
CRUISE_ACCEL_MPS2 = 0.5

# 정지 차량. SUMO 실측에서 차량 속도 중앙값이 0.0 m/s였다 - 주차와 신호대기가
# 그만큼 흔하다. 이 상황이 없으면 "접근하지 않는 차를 위험으로 오판하는지"를
# 평가할 수 없는데, 그것이 현장에서 가장 흔한 오경보 원인이다.
STOPPED_PROB = 0.15
STOPPED_MAX_MPS = 0.5       # 이 아래는 사실상 정지(GPS 지터 수준)
MOVING_MIN_MPS = 2.0        # 움직이는 차의 하한(7 km/h)

# 보행자의 속도·방향 변화. 대부분은 고르게 걷고 일부만 머뭇거린다.
PED_CHANGE_PROB = 0.30
PED_STEADY_ACCEL = 0.2
PED_STEADY_TURN_DPS = 5.0


def sample_params(seed):
    """시드 하나로 재현 가능한 파라미터 한 벌.

    시드를 그대로 쓰면 시나리오마다 난수열이 겹칠 수 있어 SeedSequence로 흩는다.
    속도·선회·가속만 아래 확률 구조를 따르고 나머지는 선언 범위에서 균등하게 뽑는다.
    """
    rng = np.random.default_rng(np.random.SeedSequence(seed))
    plain = ("approach_deg", "start_distance_m",
             "miss_offset_m", "ped_speed_mps", "ped_heading_deg")
    values = {
        name: float(rng.uniform(*PARAM_RANGES[name])) for name in plain
    }

    _, speed_hi = PARAM_RANGES["veh_speed_mps"]
    values["veh_speed_mps"] = float(
        rng.uniform(0.0, STOPPED_MAX_MPS) if rng.random() < STOPPED_PROB
        else rng.uniform(MOVING_MIN_MPS, speed_hi)
    )

    turn_lo, turn_hi = PARAM_RANGES["turn_rate_dps"]
    values["turn_rate_dps"] = float(
        rng.uniform(turn_lo, turn_hi) if rng.random() < TURN_PROB
        else rng.uniform(-CRUISE_TURN_DPS, CRUISE_TURN_DPS)
    )

    accel_lo, _ = PARAM_RANGES["veh_accel_mps2"]
    values["veh_accel_mps2"] = float(
        rng.uniform(accel_lo, 0.0) if rng.random() < BRAKE_PROB
        else rng.uniform(-CRUISE_ACCEL_MPS2, CRUISE_ACCEL_MPS2)
    )

    # 보행자는 가감속과 방향전환을 한 분기로 묶는다. 사람이 머뭇거릴 때는 속도와
    # 방향이 같이 흔들리지, 속도만 또는 방향만 따로 바뀌지 않는다.
    if rng.random() < PED_CHANGE_PROB:
        values["ped_accel_mps2"] = float(rng.uniform(*PARAM_RANGES["ped_accel_mps2"]))
        values["ped_turn_rate_dps"] = float(rng.uniform(*PARAM_RANGES["ped_turn_rate_dps"]))
    else:
        values["ped_accel_mps2"] = float(rng.uniform(-PED_STEADY_ACCEL, PED_STEADY_ACCEL))
        values["ped_turn_rate_dps"] = float(rng.uniform(-PED_STEADY_TURN_DPS, PED_STEADY_TURN_DPS))
    return ScenarioParams(**values)


TAIL_S = 5.0        # 최근접점을 지난 뒤 더 지켜보는 시간
MAX_DURATION_S = 90.0  # 아주 느린 차가 무한정 길어지는 것을 막는 상한


def auto_duration_s(params):
    """차가 최근접점을 지나갈 만큼의 관측 시간.

    등속 직진 근사로 도달 시각을 잡고 꼬리를 붙인다. 가속·선회가 있으면 정확하지
    않지만, 목적이 "최근접점을 놓치지 않을 만큼 충분히 길게"이므로 근사로 족하다.
    감속하는 차는 도달이 늦어지므로 평균 속도로 보정한다.
    """
    end_speed = max(params.veh_speed_mps + params.veh_accel_mps2 * 10.0, 0.5)
    mean_speed = max((params.veh_speed_mps + end_speed) / 2, 0.5)
    return min(params.start_distance_m / mean_speed + TAIL_S, MAX_DURATION_S)


def simulate(params, dt=0.1, duration_s=20.0):
    """파라미터 한 벌을 궤적으로 편다.

    보행자는 등속 직진, 차량은 등가속 + 등선회율. 차량 속도는 0 아래로 내려가지
    않는다(후진하는 상황은 이 연구 범위 밖이고, 제동 후 정지가 실제 거동이다).

    duration_s=None 이면 최근접점을 지나갈 만큼 자동으로 잡는다. 고정 길이를 쓰면
    느리고 먼 차는 도착도 못 한 채 끝나 라벨이 전부 '안전'이 되므로, 대량 생성에는
    자동 쪽을 쓴다.
    """
    if duration_s is None:
        duration_s = auto_duration_s(params)
    n = int(round(duration_s / dt)) + 1
    t = np.arange(n) * dt

    # --- 보행자: 원점에서 출발 ---
    ped = _integrate(
        0.0, 0.0,
        math.radians(params.ped_heading_deg),
        params.ped_speed_mps, params.ped_accel_mps2, params.ped_turn_rate_dps,
        t, dt,
    )

    # --- 차량 시작 위치: 방위각 위에 놓고, 진행 직선을 offset만큼 옆으로 민다 ---
    ae, an = _unit_from_bearing(params.approach_deg)
    start_e = params.start_distance_m * ae
    start_n = params.start_distance_m * an

    # 보행자를 향하는 방향이 초기 진행 방향
    heading = math.atan2(-ae, -an)  # atan2(east, north): 방위각 규약 유지
    # 진행 방향에 수직으로 밀면 직선 전체가 offset만큼 비껴간다
    start_e += params.miss_offset_m * an
    start_n += params.miss_offset_m * -ae

    veh = _integrate(
        start_e, start_n, heading,
        params.veh_speed_mps, params.veh_accel_mps2, params.turn_rate_dps,
        t, dt,
    )
    return Scenario(t=t, ped=ped, veh=veh, params=params)


def distances(scenario):
    """매 시점의 차-보행자 거리."""
    return np.hypot(scenario.veh.x - scenario.ped.x, scenario.veh.y - scenario.ped.y)


def min_distance(scenario):
    """시나리오 전체에서 실제로 가장 가까웠던 거리."""
    return float(distances(scenario).min())


def _param_summary(p):
    return " ".join(
        f"{f.name}={getattr(p, f.name):.2f}" for f in fields(p)
    )


def main():
    import argparse

    parser = argparse.ArgumentParser(description="시나리오를 하나 만들어 요약을 찍는다.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--duration-s", type=float, default=20.0)
    args = parser.parse_args()

    sc = simulate(sample_params(args.seed), dt=args.dt, duration_s=args.duration_s)
    print(_param_summary(sc.params))
    print(f"min_distance={min_distance(sc):.2f}m  steps={len(sc.t)}")


if __name__ == "__main__":
    main()
