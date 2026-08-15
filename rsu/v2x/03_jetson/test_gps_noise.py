"""gps_noise.py 단위 테스트.

SUMO의 완벽한 좌표에 실제 GPS 오차를 입히는 모듈을 검증한다.
실측(2026-08-01 야외 정지 세션)에서 측정한 값: sigma 약 2m, 최악 4m.
팀 코드의 가정값 GPS_SIGMA_M = 2.5m 와 일치하므로 그 값을 기본으로 쓴다.
"""
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent / "auto_pipeline"))

import gps_noise  # noqa: E402


def make_df(n_steps=50, vehicles=("car_1",), dt=1.0):
    """파이프라인이 build_features 중간에 갖는 모양의 데이터프레임."""
    rows = []
    for vid in vehicles:
        for i in range(n_steps):
            rows.append({
                "timestep_time": i * dt,
                "vehicle_id": vid,
                "veh_x": 3600.0 + i,
                "veh_y": 1400.0,
                "ped_x": 3650.0,
                "ped_y": 1420.0,
            })
    return pd.DataFrame(rows)


def offsets(before, after, col):
    return (after[col] - before[col]).to_numpy()


def test_sigma_zero_leaves_coordinates_unchanged():
    df = make_df()
    out = gps_noise.add_gps_noise(df.copy(), sigma_m=0.0, seed=1)
    for col in ("veh_x", "veh_y", "ped_x", "ped_y"):
        assert np.allclose(out[col], df[col])


def test_same_seed_gives_same_result():
    df = make_df()
    a = gps_noise.add_gps_noise(df.copy(), seed=7)
    b = gps_noise.add_gps_noise(df.copy(), seed=7)
    assert np.allclose(a["veh_x"], b["veh_x"])
    assert np.allclose(a["ped_y"], b["ped_y"])


def test_different_seed_gives_different_result():
    df = make_df()
    a = gps_noise.add_gps_noise(df.copy(), seed=1)
    b = gps_noise.add_gps_noise(df.copy(), seed=2)
    assert not np.allclose(a["veh_x"], b["veh_x"])


def test_error_magnitude_matches_sigma():
    df = make_df(n_steps=4000)
    out = gps_noise.add_gps_noise(df.copy(), sigma_m=2.5, seed=3)
    err = offsets(df, out, "veh_x")
    assert 2.0 < err.std() < 3.0, f"측정된 sigma={err.std():.2f}"


def test_error_is_time_correlated_not_white_noise():
    """실제 GPS 오차는 매 순간 튀지 않고 천천히 흘러간다."""
    df = make_df(n_steps=4000)
    out = gps_noise.add_gps_noise(df.copy(), sigma_m=2.5, tau_s=60.0, seed=4)
    err = offsets(df, out, "veh_x")
    lag1 = np.corrcoef(err[:-1], err[1:])[0, 1]
    assert lag1 > 0.9, f"lag-1 상관 {lag1:.3f} - 백색잡음에 가깝다"


def test_vehicles_get_independent_errors():
    df = make_df(vehicles=("car_1", "car_2"))
    out = gps_noise.add_gps_noise(df.copy(), seed=5)
    err = offsets(df, out, "veh_x")
    a = err[out["vehicle_id"].to_numpy() == "car_1"]
    b = err[out["vehicle_id"].to_numpy() == "car_2"]
    assert not np.allclose(a, b)


def test_pedestrian_error_is_shared_across_vehicles_at_same_time():
    """보행자는 한 명이므로 같은 시각이면 어느 차량 행에서 보든 오차가 같아야 한다."""
    df = make_df(vehicles=("car_1", "car_2"))
    out = gps_noise.add_gps_noise(df.copy(), seed=6)
    err = offsets(df, out, "ped_x")
    times = out["timestep_time"].to_numpy()
    ids = out["vehicle_id"].to_numpy()
    for t in np.unique(times):
        vals = err[times == t]
        assert np.allclose(vals, vals[0]), f"t={t}에서 보행자 오차가 갈렸다"
    # 그리고 시각이 다르면 값도 달라야 한다 (상수로 고정된 게 아님)
    assert err[ids == "car_1"].std() > 0


def test_pedestrian_and_vehicle_errors_are_independent():
    df = make_df(n_steps=200)
    out = gps_noise.add_gps_noise(df.copy(), seed=8)
    assert not np.allclose(offsets(df, out, "veh_x"), offsets(df, out, "ped_x"))


def test_first_sample_already_has_error():
    """정상상태에서 시작해야 한다 - 0에서 출발하면 초반 구간이 실제보다 정확해진다."""
    firsts = []
    for seed in range(60):
        df = make_df(n_steps=3)
        out = gps_noise.add_gps_noise(df.copy(), sigma_m=2.5, seed=seed)
        firsts.append(offsets(df, out, "veh_x")[0])
    assert np.std(firsts) > 1.5, f"첫 표본 sigma={np.std(firsts):.2f} - 0에서 시작한 듯"


def test_uwb_sigma_gives_much_smaller_error():
    df = make_df(n_steps=2000)
    gps = gps_noise.add_gps_noise(df.copy(), sigma_m=2.5, seed=9)
    uwb = gps_noise.add_gps_noise(df.copy(), sigma_m=0.1, seed=9)
    assert offsets(df, uwb, "veh_x").std() < offsets(df, gps, "veh_x").std() / 10


def test_rows_and_columns_are_preserved():
    df = make_df(vehicles=("car_1", "car_2"))
    out = gps_noise.add_gps_noise(df.copy(), seed=10)
    assert len(out) == len(df)
    assert list(out.columns) == list(df.columns)


def test_uneven_timesteps_are_handled():
    """시간 간격이 일정하지 않아도 동작해야 한다."""
    df = make_df(n_steps=100)
    df.loc[50:, "timestep_time"] += 30.0  # 중간에 30초 공백
    out = gps_noise.add_gps_noise(df.copy(), seed=11)
    err = offsets(df, out, "veh_x")
    assert np.isfinite(err).all()
