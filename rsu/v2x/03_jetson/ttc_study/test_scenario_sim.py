"""시나리오 시뮬레이터 검증.

핵심은 "우리가 만들려고 한 기하학이 실제로 만들어졌는가"다. 등속 직선 운동은
해석해가 있으므로 생성 결과를 정확히 대조할 수 있고, 그 대조가 통과해야
이 시뮬레이터로 만든 라벨을 믿을 수 있다.
"""

import math

import numpy as np
import pytest

from scenario_sim import ScenarioParams, min_distance, simulate


def straight_params(**overrides):
    """정면에서 다가오는 차 + 정지한 보행자. 테스트의 기준 시나리오."""
    base = dict(
        approach_deg=0.0,
        start_distance_m=60.0,
        veh_speed_mps=10.0,
        miss_offset_m=0.0,
        ped_speed_mps=0.0,
        ped_heading_deg=0.0,
        veh_accel_mps2=0.0,
        turn_rate_dps=0.0,
    )
    base.update(overrides)
    return ScenarioParams(**base)


def test_miss_offset_becomes_the_actual_minimum_distance():
    """빗나감 거리 3m로 만든 시나리오는 실제로 3m까지만 가까워진다."""
    sc = simulate(straight_params(miss_offset_m=3.0), dt=0.1, duration_s=20.0)

    assert min_distance(sc) == pytest.approx(3.0, abs=0.05)


def test_head_on_scenario_reaches_contact():
    """빗나감 0이면 실제로 접촉 거리(0)까지 도달한다."""
    sc = simulate(straight_params(miss_offset_m=0.0), dt=0.1, duration_s=20.0)

    assert min_distance(sc) == pytest.approx(0.0, abs=0.1)


def test_vehicle_covers_expected_ground():
    """등속 차량은 duration 동안 speed x duration 만큼 이동한다."""
    sc = simulate(straight_params(veh_speed_mps=10.0), dt=0.1, duration_s=5.0)

    travelled = math.hypot(sc.veh.x[-1] - sc.veh.x[0], sc.veh.y[-1] - sc.veh.y[0])
    assert travelled == pytest.approx(50.0, abs=0.2)


def test_timeline_matches_dt_and_duration():
    """시간축이 dt 간격으로 duration까지 채워진다."""
    sc = simulate(straight_params(), dt=0.1, duration_s=10.0)

    assert len(sc.t) == 101
    assert np.allclose(np.diff(sc.t), 0.1)


def test_walking_pedestrian_moves():
    """보행자 속도를 주면 실제로 그만큼 이동한다."""
    sc = simulate(
        straight_params(ped_speed_mps=1.2, ped_heading_deg=90.0),
        dt=0.1,
        duration_s=10.0,
    )

    travelled = math.hypot(sc.ped.x[-1] - sc.ped.x[0], sc.ped.y[-1] - sc.ped.y[0])
    assert travelled == pytest.approx(12.0, abs=0.1)


def test_approach_angle_places_vehicle_on_that_bearing():
    """접근 방위각 90도면 차는 보행자의 정동쪽에서 출발한다."""
    sc = simulate(
        straight_params(approach_deg=90.0, start_distance_m=60.0),
        dt=0.1,
        duration_s=1.0,
    )

    assert sc.veh.x[0] == pytest.approx(60.0, abs=0.05)
    assert sc.veh.y[0] == pytest.approx(0.0, abs=0.05)


def test_turning_vehicle_changes_heading():
    """선회율을 주면 차의 진행 방향이 실제로 돈다 (DCPA 등속직진 가정이 깨지는 경우)."""
    sc = simulate(straight_params(turn_rate_dps=10.0), dt=0.1, duration_s=5.0)

    start = math.atan2(sc.veh.vy[0], sc.veh.vx[0])
    end = math.atan2(sc.veh.vy[-1], sc.veh.vx[-1])
    turned_deg = abs(math.degrees(end - start))
    assert turned_deg == pytest.approx(50.0, abs=1.0)


def test_auto_duration_runs_past_the_closest_approach():
    """duration을 자동으로 두면 차가 최근접점을 지나간 뒤에 끝난다.

    고정 duration은 느리고 먼 차가 도착도 못 한 채 시나리오가 끝나게 만든다.
    그런 표본은 라벨이 전부 '안전'이 되어 학습에도 평가에도 쓸모가 없다.
    """
    from scenario_sim import distances

    slow_and_far = straight_params(veh_speed_mps=2.0, start_distance_m=100.0)
    sc = simulate(slow_and_far, dt=0.1, duration_s=None)

    d = distances(sc)
    assert 0 < int(d.argmin()) < len(d) - 1, "최근접점이 구간 안에 있어야 한다"


def test_same_seed_gives_same_scenario():
    """랜덤 샘플링은 시드로 재현된다."""
    from scenario_sim import sample_params

    a = sample_params(seed=7)
    b = sample_params(seed=7)
    assert a == b


def test_sampling_produces_enough_dangerous_scenarios():
    """샘플링한 시나리오 중 실제로 위험했던 경우가 충분히 나온다.

    이 시뮬레이터의 존재 이유가 이것이다. 기존 SUMO 데이터는 12,621행 전체에서
    최소거리가 9.59m로 위험 사례가 0건이었고, 그래서 임계값을 검증할 수 없었다.
    같은 실패를 반복하면 여기 만든 라벨도 쓸모가 없다.
    """
    from scenario_sim import min_distance, sample_params

    mins = [min_distance(simulate(sample_params(s), duration_s=None)) for s in range(300)]
    dangerous = sum(1 for d in mins if d <= 2.0)

    assert dangerous / len(mins) >= 0.05, (
        f"위험 사례가 {dangerous}/{len(mins)}건뿐 - 학습·평가에 부족하다"
    )


def test_pedestrian_can_accelerate():
    """보행자도 속도가 변한다.

    차량에만 가감속·선회를 넣고 보행자를 등속으로 두면, 모델이 "보행자는 예측
    가능하다"고 잘못 배운다. 실제로는 지팡이로 탐색하며 걷는 보행자가 더 자주
    멈추고 방향을 바꾼다. 위험의 방향을 뒤집는 요인이므로 대칭을 맞춘다.
    """
    sc = simulate(
        straight_params(ped_speed_mps=1.2, ped_accel_mps2=-0.4),
        dt=0.1,
        duration_s=3.0,
    )

    start = math.hypot(sc.ped.vx[0], sc.ped.vy[0])
    end = math.hypot(sc.ped.vx[-1], sc.ped.vy[-1])
    assert end == pytest.approx(start - 1.2, abs=0.05)


def test_pedestrian_can_turn():
    """보행자도 방향을 바꾼다."""
    sc = simulate(
        straight_params(ped_speed_mps=1.2, ped_turn_rate_dps=20.0),
        dt=0.1,
        duration_s=3.0,
    )

    start = math.atan2(sc.ped.vy[0], sc.ped.vx[0])
    end = math.atan2(sc.ped.vy[-1], sc.ped.vx[-1])
    assert abs(math.degrees(end - start)) == pytest.approx(60.0, abs=1.0)


def test_pedestrian_speed_never_goes_negative():
    """감속하는 보행자는 멈출 뿐 뒤로 걷지 않는다."""
    sc = simulate(
        straight_params(ped_speed_mps=1.0, ped_accel_mps2=-1.5),
        dt=0.1,
        duration_s=10.0,
    )

    speeds = np.hypot(sc.ped.vx, sc.ped.vy)
    assert speeds.min() >= 0.0
    assert speeds[-1] == pytest.approx(0.0, abs=1e-9)


def test_most_pedestrians_walk_roughly_steadily():
    """대부분의 보행자는 거의 등속으로 걷는다. 차량과 같은 확률 구조."""
    from scenario_sim import sample_params

    steady = sum(
        1 for s in range(300)
        if abs(sample_params(s).ped_accel_mps2) <= 0.2
        and abs(sample_params(s).ped_turn_rate_dps) <= 5.0
    )

    assert steady / 300 >= 0.5


def test_stationary_vehicles_are_generated():
    """정지한 차도 생성된다.

    SUMO 실측에서 차량 속도 중앙값이 0.0 m/s였다 - 주차와 신호대기가 그만큼 흔하다.
    이 상황이 데이터에 없으면 "접근하지 않는 차를 위험으로 오판하는지"를 평가할 수
    없고, 실제 현장에서 가장 흔한 오경보 원인을 놓치게 된다.
    """
    from scenario_sim import sample_params

    speeds = [sample_params(s).veh_speed_mps for s in range(300)]
    near_stopped = sum(1 for v in speeds if v < 0.5)

    assert near_stopped >= 15, f"정지 차량이 {near_stopped}/300건뿐"


def test_most_vehicles_drive_roughly_straight():
    """대부분의 차는 거의 직진한다. 균등분포 선회율은 기하학을 무너뜨린다.

    선회율을 -20~20에서 균등하게 뽑으면 평균 |10| dps가 되어 5초에 50도를 돈다.
    그러면 miss_offset으로 설정한 접근 기하학이 의미를 잃고, 차가 보행자 근처에
    오지도 못한 채 시나리오가 끝난다.
    """
    from scenario_sim import sample_params

    turns = [abs(sample_params(s).turn_rate_dps) for s in range(300)]
    near_straight = sum(1 for t in turns if t <= 2.0)

    assert near_straight / len(turns) >= 0.6


def test_sampled_params_are_within_declared_ranges():
    """샘플링된 파라미터가 선언한 범위를 벗어나지 않는다."""
    from scenario_sim import PARAM_RANGES, sample_params

    for seed in range(50):
        p = sample_params(seed=seed)
        for name, (lo, hi) in PARAM_RANGES.items():
            assert lo <= getattr(p, name) <= hi, f"{name} out of range at seed {seed}"
