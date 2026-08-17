"""build_dataset_from_streams.py 단위 테스트.

잡음 낀 패킷 스트림을 젯슨 실행 코드(step6 KF → model_features.StreamingFeatures)로
흘려 실행 시와 같은 15개 특징을 만들고, 참값 라벨을 시각으로 붙인다.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent / "auto_pipeline"))
sys.path.insert(0, str(Path(__file__).parent / "ttc_study"))

import build_dataset_from_streams as bds  # noqa: E402
import gps_noise  # noqa: E402
import sim_to_rsu_stream as s2r  # noqa: E402
from features import FEATURE_COLUMNS  # noqa: E402


def _head_on_frames(seconds=20, veh_speed=5.0, start_north=100.0):
    return [{"t": float(t), "ped_x": 0.0, "ped_y": 0.0, "ped_speed": 0.0, "ped_angle": 0.0,
             "veh_x": 0.0, "veh_y": start_north - veh_speed * t, "veh_speed": veh_speed, "veh_angle": 180.0}
            for t in range(seconds + 1)]


def _quiet():
    return gps_noise.FieldNoiseParams(sigma_m=0.0, dropout_per_min=0.0, packet_loss=0.0,
                                      speed_sigma_mps=0.0, heading_sigma_deg=0.0)


def test_features_are_the_runtime_15_and_track_the_approach():
    ped, veh = s2r.interpolate(_head_on_frames(), hz=5.0)
    packets = s2r.make_packets(ped, veh, _quiet(), _quiet(), np.random.default_rng(0), base_epoch=1000.0)
    rows = bds.stream_to_features(packets)
    assert rows, "no judgement produced"
    for r in rows:
        assert set(FEATURE_COLUMNS) <= set(r), set(FEATURE_COLUMNS) - set(r)
        assert all(math.isfinite(r[k]) for k in FEATURE_COLUMNS)
    # 거리는 줄어들고, 자리를 잡은 뒤에는 접근속도가 참값 5 m/s 근처
    assert rows[-1]["distance_m"] < rows[0]["distance_m"]
    late = [r for r in rows if r["t"] - 1000.0 > 5.0]
    assert late and abs(np.median([r["closing_los"] for r in late]) - 5.0) < 1.0


def test_labels_join_by_time_with_and_without_lead():
    ped, veh = s2r.interpolate(_head_on_frames(seconds=20, veh_speed=5.0, start_north=100.0), hz=5.0)
    truth = s2r.truth_rows(ped, veh)
    packets = s2r.make_packets(ped, veh, _quiet(), _quiet(), np.random.default_rng(0), base_epoch=1000.0)
    rows = bds.stream_to_features(packets)
    joined = bds.attach_labels(rows, truth, base_epoch=1000.0, d_crit_m=2.0, horizon_s=3.0, lead_s=2.0)
    assert len(joined) == len(rows)
    # 접촉 t=20 부근: y(창 [t, t+3]) 는 t>=17 부터, y_train(창 [t+2, t+5]) 는 t>=15 부터 1
    def at(t):
        return min(joined, key=lambda r: abs((r["t"] - 1000.0) - t))
    assert at(18.0)["y"] == 1 and at(18.0)["y_train"] == 1
    assert at(16.0)["y"] == 0 and at(16.0)["y_train"] == 1
    assert at(10.0)["y"] == 0 and at(10.0)["y_train"] == 0
    assert all(math.isfinite(r["d_min_future"]) for r in joined)
    # t_hit = 시나리오에서 처음 2 m 이내가 되는 참값 시각(base_epoch 상대) — 적시경보율 계산용
    assert abs(joined[0]["t_hit"] - 1019.6) < 0.3
    far = bds.attach_labels(rows[:5], [{"t": r["t"], "distance_m": 50.0} for r in truth], base_epoch=1000.0)
    assert all(math.isnan(r["t_hit"]) for r in far)


def test_scenario_families_cover_the_field_false_alarm_cases():
    """8/17 실기 오경보의 무대: 3~15 m 정지 쌍, 주차 차 옆을 걷는 보행자, RC 속도 평행 통과.
    기본 시뮬(2~20 m/s 접근)에는 없어서 모델이 '안 위험한데 잡음 때문에 위험해 보이는'
    상황을 배울 재료가 없었다. 각 가족이 뜻하는 파라미터를 내야 한다."""
    p, dur = bds.family_params("still_close", 11)
    assert p.veh_speed_mps == 0.0 and p.ped_speed_mps == 0.0
    assert 3.0 <= p.start_distance_m <= 15.0 and dur >= 40.0
    p, dur = bds.family_params("walk_by_parked", 12)
    assert p.veh_speed_mps == 0.0 and 0.8 <= p.ped_speed_mps <= 1.4
    assert 1.0 <= p.miss_offset_m <= 5.0 and p.approach_deg == p.ped_heading_deg
    p, dur = bds.family_params("parallel_rc", 13)
    assert 0.5 <= p.veh_speed_mps <= 2.5 and 2.0 <= p.miss_offset_m <= 12.0
    p, dur = bds.family_params("rc_approach", 14)
    assert 0.5 <= p.veh_speed_mps <= 2.5 and p.miss_offset_m <= 4.0
    base, dur = bds.family_params("base", 15)
    assert dur is None  # 기본 가족은 scenario_sim 자동 길이


def test_family_mix_is_recorded_per_scenario():
    df = bds.build_from_scenario_sim(n_scenarios=6, seed=0, params=_quiet(), families=True)
    assert "family" in df.columns
    assert df.groupby("scenario_id")["family"].nunique().max() == 1
    assert set(df["family"].unique()) <= set(bds.FAMILY_WEIGHTS)


def test_scenario_sim_source_gives_labelled_rows_per_scenario():
    df = bds.build_from_scenario_sim(n_scenarios=2, seed=0, params=_quiet())
    assert set(FEATURE_COLUMNS) <= set(df.columns)
    assert {"y", "y_train", "t", "scenario_id", "d_min_future"} <= set(df.columns)
    assert sorted(df["scenario_id"].unique().tolist()) == [0, 1]
    assert len(df) > 50
    assert np.isfinite(df[list(FEATURE_COLUMNS)].to_numpy()).all()
