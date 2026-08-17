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


# ---------------------------------------------------------------------------
# 스트림 수준 현장 노이즈 모델 (2026-08-17 실측 기반)
#
# 위 add_gps_noise 는 좌표에 Gauss-Markov 오차만 입힌다. 8/17 오후 실측에서
# 판정을 흔든 잡음은 그 외에 네 가지가 더 있었다: 패킷 lat/lng 의 float32
# 양자화(위도 0.42 m/경도 0.53 m 격자), 3~5 초 GPS 무효(lat=lng=0 송신) 뒤
# 수 m~19 m 떨어진 곳에서 재획득, heading 은 speed>=0.4 m/s 일 때만 유효
# (아니면 0 또는 stale), 5 Hz fix 를 10 Hz 패킷으로 반복 송신. 아래는 깨끗한
# 궤적을 "RSU 가 받는 패킷" 으로 바꾸는 모델의 검증이다.
# ---------------------------------------------------------------------------
import math
import struct

ORIGIN = (37.4977, 126.9528)  # 캠퍼스 중심 (생성기와 같은 값)


def clean_track(seconds=20.0, hz=5.0, speed=0.0, heading=90.0, x0=0.0, y0=10.0):
    """(t, east, north, speed, heading) 표본. 등속 직선."""
    n = int(seconds * hz)
    out = []
    for i in range(n):
        t = i / hz
        out.append((t, x0 + speed * t * math.sin(math.radians(heading)),
                    y0 + speed * t * math.cos(math.radians(heading)), speed, heading))
    return out


def quiet_params(**overrides):
    """잡음 전부 끄고 하나씩 켜서 보기 위한 기본."""
    p = gps_noise.FieldNoiseParams(
        sigma_m=0.0, quantize_float32=False, packet_loss=0.0,
        dropout_per_min=0.0, speed_sigma_mps=0.0, heading_sigma_deg=0.0,
        stall_period_s=None, reboot_per_min=0.0,
    )
    for k, v in overrides.items():
        setattr(p, k, v)
    return p


def to_local(lat, lng):
    east = (lng - ORIGIN[1]) * 111320.0 * math.cos(math.radians(ORIGIN[0]))
    north = (lat - ORIGIN[0]) * 111320.0
    return east, north


def valid_positions(packets):
    return [(p["pc_time"],) + to_local(p["lat"], p["lng"]) for p in packets if p["gps_valid"]]


def test_packets_repeat_each_fix_at_packet_rate():
    """5 Hz fix 를 10 Hz 로 반복 송신 - 패킷 수는 fix 의 2배, 같은 fix 는 같은 좌표."""
    rng = np.random.default_rng(1)
    pk = gps_noise.noisify_track(clean_track(seconds=10.0), quiet_params(), rng, ORIGIN, "vehicle")
    assert len(pk) == 100
    lat = [p["lat"] for p in pk]
    assert lat[0] == lat[1] and lat[2] == lat[3]
    assert all(b["pc_time"] > a["pc_time"] for a, b in zip(pk, pk[1:]))
    assert [p["seq"] for p in pk] == list(range(100))


def test_float32_quantization_puts_positions_on_the_field_grid():
    """패킷 lat/lng 는 float32. 위도 3.8e-6도(0.42 m), 경도 7.6e-6도(0.53 m) 격자."""
    rng = np.random.default_rng(2)
    pk = gps_noise.noisify_track(clean_track(seconds=20.0, speed=1.0, heading=45.0),
                                 quiet_params(quantize_float32=True), rng, ORIGIN, "vehicle")
    # 브리지가 %.6f 로 찍으므로 값 자체는 6자리지만, 격자 간격은 float32 것이
    # 남는다: 위도 3~4 µdeg(0.42 m), 경도 7~8 µdeg(0.53 m). 8/17 raw 실측과 동일.
    lats = sorted({p["lat"] for p in pk})
    lat_steps = [round((b - a) * 1e6) for a, b in zip(lats, lats[1:])]
    assert lat_steps and all(s in (3, 4) or s >= 7 for s in lat_steps), lat_steps[:10]
    lngs = sorted({p["lng"] for p in pk})
    lng_steps = [round((b - a) * 1e6) for a, b in zip(lngs, lngs[1:])]
    assert lng_steps and all(s in (7, 8) or s >= 14 for s in lng_steps), lng_steps[:10]
    # 끄면 1 µdeg 단위로 촘촘해진다
    fine = gps_noise.noisify_track(clean_track(seconds=20.0, speed=1.0, heading=45.0),
                                   quiet_params(quantize_float32=False), np.random.default_rng(2), ORIGIN, "vehicle")
    fine_lats = sorted({p["lat"] for p in fine})
    assert min(round((b - a) * 1e6) for a, b in zip(fine_lats, fine_lats[1:])) <= 2


def test_dropout_sends_zero_position_and_zero_motion():
    """무효 구간: gps_valid=0, lat=lng=0, speed=0, heading=0/heading_valid=0 (펌웨어와 동일)."""
    rng = np.random.default_rng(3)
    p = quiet_params(dropout_per_min=30.0, dropout_s=(3.0, 3.0))
    pk = gps_noise.noisify_track(clean_track(seconds=60.0, speed=1.0), p, rng, ORIGIN, "vehicle")
    invalid = [q for q in pk if not q["gps_valid"]]
    assert invalid, "no dropout happened"
    for q in invalid:
        assert q["lat"] == 0.0 and q["lng"] == 0.0
        assert q["speed_mps"] == 0.0 and q["heading_deg"] == 0.0 and q["heading_valid"] == 0
    # 무효 구간은 요청한 길이(3초 = 30패킷)의 배수로 나온다(연달아 시작하면 합쳐짐)
    runs, run = [], 0
    for q in pk:
        if q["gps_valid"]:
            if run: runs.append(run); run = 0
        else:
            run += 1
    if run: runs.append(run)
    assert all(r >= 28 and (r % 30 <= 2 or r % 30 >= 28) for r in runs), runs
    assert 28 <= sorted(runs)[len(runs) // 2] <= 32, runs


def test_reacquisition_lands_the_configured_distance_away():
    """재획득 첫 fix 는 참값에서 점프 크기만큼 떨어진 곳에서 시작한다(8/17: 최대 19 m)."""
    rng = np.random.default_rng(4)
    p = quiet_params(dropout_per_min=6.0, dropout_s=(3.0, 3.0), reacq_jump_m=(8.0, 8.0))
    track = clean_track(seconds=60.0)  # 정지: 참값 (0, 10)
    pk = gps_noise.noisify_track(track, p, rng, ORIGIN, "vehicle")
    firsts = []
    for a, b in zip(pk, pk[1:]):
        if not a["gps_valid"] and b["gps_valid"]:
            e, n = to_local(b["lat"], b["lng"])
            firsts.append(math.hypot(e - 0.0, n - 10.0))
    assert firsts, "no re-acquisition"
    for d in firsts:
        assert 7.0 <= d <= 9.0, firsts


def test_heading_is_valid_only_above_the_speed_gate():
    rng = np.random.default_rng(5)
    slow = gps_noise.noisify_track(clean_track(speed=0.2, heading=120.0), quiet_params(), rng, ORIGIN, "vehicle")
    fast = gps_noise.noisify_track(clean_track(speed=1.0, heading=120.0), quiet_params(), rng, ORIGIN, "vehicle")
    assert all(q["heading_valid"] == 0 for q in slow)
    assert all(q["heading_deg"] == 0.0 for q in slow)  # heading_zero_prob=1 기본이 아니면 stale 허용
    assert all(q["heading_valid"] == 1 for q in fast)
    assert all(abs(q["heading_deg"] - 120.0) < 1e-6 for q in fast)


def test_gauss_markov_error_has_the_requested_spread():
    rng = np.random.default_rng(6)
    p = quiet_params(sigma_m=2.5, tau_s=5.0)
    pk = gps_noise.noisify_track(clean_track(seconds=600.0), p, rng, ORIGIN, "cane")
    errs = [math.hypot(e - 0.0, n - 10.0) for _, e, n in valid_positions(pk)]
    rms = math.sqrt(sum(x * x for x in errs) / len(errs))
    assert 2.5 < rms < 4.5, rms  # 2축 각 sigma 2.5 -> 반경 rms ≈ 3.5


def test_same_seed_same_stream():
    a = gps_noise.noisify_track(clean_track(), gps_noise.FieldNoiseParams(), np.random.default_rng(7), ORIGIN, "vehicle")
    b = gps_noise.noisify_track(clean_track(), gps_noise.FieldNoiseParams(), np.random.default_rng(7), ORIGIN, "vehicle")
    assert a == b


def test_randomized_params_stay_inside_the_documented_ranges():
    for seed in range(20):
        p = gps_noise.FieldNoiseParams.randomized(np.random.default_rng(seed))
        assert 1.5 <= p.sigma_m <= 6.0
        assert 0.0 <= p.dropout_per_min <= 4.0
        assert 0.0 <= p.reacq_jump_m[0] <= p.reacq_jump_m[1] <= 20.0
        assert 0.0 <= p.packet_loss <= 0.15
        assert p.heading_min_speed_mps == 0.4      # 펌웨어 상수: 랜덤화 대상 아님
        assert p.quantize_float32 is True          # 프로토콜 상수: 랜덤화 대상 아님
