"""피처 계산 검증.

두 가지를 지킨다.
  1. 규칙 점수(risk_score)가 피처에 섞이지 않는다 - 현재 scaler.json이 저지른
     라벨 누수를 반복하지 않기 위해서다.
  2. 물리 외삽(phys_*)이 등속에서는 정확하고 선회에서는 빗나간다 - 그 격차가
     모델이 배워야 할 몫이므로, 격차가 실제로 존재하는지 확인해 둔다.
"""

import numpy as np
import pytest

from features import FEATURE_COLUMNS, build_features
from scenario_sim import ScenarioParams, simulate


def params(**overrides):
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


def test_closing_speed_is_positive_while_approaching():
    """다가오는 동안 시선방향 접근속도가 양수다."""
    sc = simulate(params(), dt=0.1, duration_s=20.0)

    f = build_features(sc)

    assert f["closing_los"][0] == pytest.approx(10.0, abs=0.1)


def test_ttc_matches_distance_over_closing_speed():
    """정면 등속 접근의 TTC는 거리/접근속도와 같다."""
    sc = simulate(params(), dt=0.1, duration_s=20.0)

    f = build_features(sc)

    assert f["ttc"][0] == pytest.approx(6.0, abs=0.1)


def test_ttc_is_clamped_not_sentinelled():
    """접근하지 않는 쌍의 TTC는 큰 상수가 아니라 상한으로 잘린다.

    팀의 기존 파이프라인은 9999를 넣었고, 그 값이 그대로 정규화에 섞여
    scaler.json의 ttc 평균이 6198이 되었다. 스케일이 망가지면 모델이 TTC를
    사실상 못 쓴다.
    """
    from features import TTC_MAX_S

    sc = simulate(params(veh_speed_mps=0.0), dt=0.1, duration_s=10.0)

    f = build_features(sc)

    assert f["ttc"].max() <= TTC_MAX_S


def test_dcpa_equals_the_miss_offset_for_straight_motion():
    """등속 직진에서 예상 최근접거리는 설정한 빗나감과 같다."""
    sc = simulate(params(miss_offset_m=4.0), dt=0.1, duration_s=20.0)

    f = build_features(sc)

    assert f["dcpa_m"][0] == pytest.approx(4.0, abs=0.1)


def test_relative_vectors_point_from_pedestrian_to_vehicle():
    """dx, dy는 보행자 기준 차량의 상대 위치다."""
    sc = simulate(params(approach_deg=90.0, start_distance_m=60.0), dt=0.1, duration_s=1.0)

    f = build_features(sc)

    assert f["dx"][0] == pytest.approx(60.0, abs=0.1)
    assert f["dy"][0] == pytest.approx(0.0, abs=0.1)


def test_physics_extrapolation_is_exact_for_constant_velocity():
    """등속 직진이면 3초 외삽 최소거리가 실제 3초간 최소거리와 일치한다."""
    sc = simulate(params(miss_offset_m=3.0), dt=0.1, duration_s=20.0)

    f = build_features(sc)
    from scenario_sim import distances

    d = distances(sc)
    actual_min_3s = d[0:31].min()  # 0~3초 실제 최소거리
    assert f["phys_min_dist_3s"][0] == pytest.approx(actual_min_3s, abs=0.2)


def test_physics_extrapolation_misses_when_the_vehicle_turns():
    """차가 선회하면 등속 외삽이 빗나간다.

    이 격차가 모델의 존재 이유다. 격차가 없다면 물리 계산만으로 충분하고
    AI를 쓸 이유가 없으므로, 실제로 벌어지는지 확인해 둔다.
    """
    from scenario_sim import distances

    turning = simulate(params(turn_rate_dps=15.0), dt=0.1, duration_s=20.0)

    f = build_features(turning)
    d = distances(turning)
    actual_min_3s = d[0:31].min()

    error = abs(f["phys_min_dist_3s"][0] - actual_min_3s)
    assert error > 1.0, "선회에서도 외삽이 정확하면 모델이 배울 것이 없다"


def test_gps_noise_perturbs_the_features():
    """GPS 노이즈를 주면 피처가 흔들린다.

    현장의 젯슨은 노이즈 낀 좌표만 본다. 노이즈 없는 피처로 학습하면 시뮬레이션
    에서만 잘 도는 모델이 된다.
    """
    sc = simulate(params(), dt=0.1, duration_s=20.0)

    clean = build_features(sc)
    noisy = build_features(sc, gps_sigma_m=2.5, seed=1)

    assert not np.allclose(clean["distance_m"], noisy["distance_m"])


def test_noise_is_reproducible_by_seed():
    """같은 시드면 같은 노이즈가 나온다."""
    sc = simulate(params(), dt=0.1, duration_s=20.0)

    a = build_features(sc, gps_sigma_m=2.5, seed=42)
    b = build_features(sc, gps_sigma_m=2.5, seed=42)

    assert np.allclose(a["distance_m"], b["distance_m"])


def test_turning_shows_up_in_the_heading_rate():
    """차가 꺾으면 상대 방위각 변화율이 잡힌다.

    한 시점의 위치·속도만 주면 모델이 보는 정보가 물리 외삽과 똑같아지므로,
    등속 가정을 깨는 근거를 모델이 가질 수 없다. 선회와 가속은 "지금 등속이
    아니다"를 알려주는 유일한 신호라서 반드시 피처에 있어야 한다.
    """
    straight = build_features(simulate(params(turn_rate_dps=0.0), dt=0.1, duration_s=10.0))
    turning = build_features(simulate(params(turn_rate_dps=15.0), dt=0.1, duration_s=10.0))

    assert np.median(np.abs(turning["d_heading_dt"])) > \
           np.median(np.abs(straight["d_heading_dt"]))


def test_braking_shows_up_in_the_closing_rate():
    """차가 감속하면 접근속도의 변화율이 잡힌다.

    접근 구간(차가 도달하기 전)만 본다. 통과한 뒤에는 두 시나리오 모두 변화가
    없어져 비교가 무의미해진다.
    """
    approach = slice(0, 50)  # 0~5초. 60m를 10 m/s로 오므로 도달은 6초.
    steady = build_features(simulate(params(veh_accel_mps2=0.0), dt=0.1, duration_s=10.0))
    braking = build_features(simulate(params(veh_accel_mps2=-2.5), dt=0.1, duration_s=10.0))

    assert np.median(np.abs(braking["d_closing_dt"][approach])) > \
           np.median(np.abs(steady["d_closing_dt"][approach]))


def test_constant_velocity_leaves_both_rates_near_zero():
    """등속 직진으로 접근하는 동안 두 변화율이 모두 0에 가깝다."""
    approach = slice(0, 50)
    sc = simulate(params(turn_rate_dps=0.0, veh_accel_mps2=0.0), dt=0.1, duration_s=10.0)

    f = build_features(sc)

    assert np.median(np.abs(f["d_heading_dt"][approach])) < 0.5
    assert np.median(np.abs(f["d_closing_dt"][approach])) < 0.1


def test_rates_never_look_into_the_future():
    """변화율이 과거 값만으로 계산된다.

    np.gradient는 중앙차분이라 다음 시점을 함께 본다. 시뮬레이션에서는 전체
    궤적이 있으니 계산되지만, 젯슨은 미래를 볼 수 없다. 학습 때 미래를 쓴
    피처로 배우고 추론 때 못 쓰면 두 계산이 달라져, 시뮬레이션에서 측정한
    성능이 현장에서 재현되지 않는다.

    검사 방법은 뒤를 늘려 보는 것이다. 실시간에서는 언제나 "지금이 마지막
    시점"이므로, 나중에 데이터가 더 들어왔다고 해서 이미 계산한 값이 바뀌면
    안 된다. 중앙차분은 마지막 점에서만 후방차분을 쓰므로, 뒤가 늘어나면 그
    점의 값이 달라진다.
    """
    from scenario_sim import Scenario, Track

    sc = simulate(params(turn_rate_dps=10.0, veh_accel_mps2=-1.0), dt=0.1, duration_s=10.0)

    def prefix(n):
        return Scenario(
            t=sc.t[:n],
            ped=Track(x=sc.ped.x[:n], y=sc.ped.y[:n],
                      vx=sc.ped.vx[:n], vy=sc.ped.vy[:n]),
            veh=Track(x=sc.veh.x[:n], y=sc.veh.y[:n],
                      vx=sc.veh.vx[:n], vy=sc.veh.vy[:n]),
            params=sc.params,
        )

    n = 60
    short = build_features(prefix(n))
    longer = build_features(prefix(n + 5))

    for name in ("d_closing_dt", "d_heading_dt"):
        assert np.allclose(short[name], longer[name][:n], atol=1e-9), (
            f"{name}: 뒤에 데이터가 들어오자 이미 계산한 값이 바뀐다 = 미래를 본다"
        )


def test_geometric_singularity_is_clamped():
    """통과 순간의 특이점이 물리적 상한으로 잘린다.

    차가 보행자를 스치는 순간 시선 방향이 뒤집혀 접근속도 부호가 한 스텝에
    반전한다. 등속 주행인데도 변화율이 100 m/s^2가 되는데, 이는 기하학이 만든
    값이지 실제 가속이 아니다. 자르지 않으면 이 이상치가 학습을 지배한다.
    """
    from features import MAX_BEARING_RATE_DPS, MAX_REL_ACCEL_MPS2

    sc = simulate(params(miss_offset_m=0.0), dt=0.1, duration_s=20.0)

    f = build_features(sc)

    assert np.abs(f["d_closing_dt"]).max() <= MAX_REL_ACCEL_MPS2
    assert np.abs(f["d_heading_dt"]).max() <= MAX_BEARING_RATE_DPS


def test_kalman_beats_plain_differencing_on_noisy_input():
    """노이즈 낀 좌표에서 칼만 속도 추정이 단순 미분보다 정확하다.

    실제 파이프라인(step6_kinematics)은 칼만을 거친 속도를 step7에 넘긴다.
    베이스라인을 단순 미분으로 두면 규칙 쪽을 실제보다 약하게 만들어놓고
    "AI가 이겼다"고 말하는 셈이 된다. 공정한 비교를 위해 같은 필터를 쓴다.
    """
    sc = simulate(params(veh_speed_mps=10.0), dt=0.1, duration_s=20.0)

    kf = build_features(sc, gps_sigma_m=2.5, seed=3, velocity_mode="kalman")
    diff = build_features(sc, gps_sigma_m=2.5, seed=3, velocity_mode="diff")

    truth = 10.0  # 등속 직진 차량의 참 속도
    kf_err = np.median(np.abs(kf["veh_speed_mps"] - truth))
    diff_err = np.median(np.abs(diff["veh_speed_mps"] - truth))

    assert kf_err < diff_err


def test_kalman_is_the_default_velocity_mode():
    """기본값이 실제 파이프라인과 같은 칼만이다."""
    sc = simulate(params(), dt=0.1, duration_s=20.0)

    default = build_features(sc, gps_sigma_m=2.5, seed=5)
    kalman = build_features(sc, gps_sigma_m=2.5, seed=5, velocity_mode="kalman")

    assert np.allclose(default["veh_speed_mps"], kalman["veh_speed_mps"])


def test_risk_score_is_not_a_feature():
    """규칙 점수는 피처가 아니다.

    현재 scaler.json은 입력에 risk_score를 넣고 그 값으로 만든 라벨을 맞히게
    했다. 답을 문제지에 적어둔 것이라 정확도 수치가 의미를 잃는다.
    """
    banned = {"risk_score", "risk_level", "base_score", "final_score"}

    assert banned.isdisjoint(FEATURE_COLUMNS)


def test_absolute_coordinates_are_not_features():
    """절대 좌표는 피처가 아니다.

    "이 지도의 이 지점"은 다른 장소로 일반화되지 않는다. 위험을 결정하는 것은
    두 물체의 상대 기하학이므로 상대량만 쓴다.
    """
    banned = {"ped_x", "ped_y", "veh_x", "veh_y"}

    assert banned.isdisjoint(FEATURE_COLUMNS)


def test_every_declared_column_is_produced():
    """선언한 피처가 빠짐없이 생성되고 길이가 시간축과 같다."""
    sc = simulate(params(), dt=0.1, duration_s=20.0)

    f = build_features(sc)

    assert set(f) == set(FEATURE_COLUMNS)
    for name in FEATURE_COLUMNS:
        assert len(f[name]) == len(sc.t), name


def test_features_are_finite():
    """정지 차량처럼 경계에 놓인 경우에도 NaN/inf가 나오지 않는다."""
    sc = simulate(params(veh_speed_mps=0.0, ped_speed_mps=0.0), dt=0.1, duration_s=10.0)

    f = build_features(sc)

    for name, col in f.items():
        assert np.all(np.isfinite(col)), name
