"""sim_to_rsu_stream.py 단위 테스트.

생성기 v3 시나리오(SUMO 1 Hz 참값)를 "RSU 가 받는 패킷 스트림"으로 바꾸는 층.
학습 특징을 실행 파이프라인(step6 KF → model_features)으로 만들 수 있게 하고,
라벨은 참값 궤적에서 뽑는다.
"""
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent / "auto_pipeline"))

import gps_noise  # noqa: E402
import sim_to_rsu_stream as s2r  # noqa: E402
from step3_parse_v2x import normalize_record  # noqa: E402


def frames_head_on(n_seconds=20, veh_speed=5.0, start_north=100.0):
    """보행자 정지(원점), 차량이 북쪽에서 정남으로 veh_speed 로 접근. 1 Hz."""
    out = []
    for t in range(n_seconds + 1):
        out.append({
            "t": float(t), "ped_x": 0.0, "ped_y": 0.0, "ped_speed": 0.0, "ped_angle": 0.0,
            "veh_x": 0.0, "veh_y": start_north - veh_speed * t, "veh_speed": veh_speed, "veh_angle": 180.0,
        })
    return out


def test_interpolation_fills_five_fixes_per_second_and_derives_heading():
    ped, veh = s2r.interpolate(frames_head_on(n_seconds=4), hz=5.0)
    assert len(veh) == 4 * 5 + 1
    t, e, n, spd, hd = veh[3]           # t = 0.6 s
    assert abs(t - 0.6) < 1e-9
    assert abs(e - 0.0) < 1e-9 and abs(n - (100.0 - 3.0)) < 1e-6
    assert abs(spd - 5.0) < 1e-9
    assert abs(hd - 180.0) < 1e-6      # 남쪽으로 이동 = 180°
    assert all(abs(p[3]) < 1e-9 for p in ped)  # 보행자 정지


def test_truth_rows_carry_clean_ttc_and_future_min_distance_label():
    ped, veh = s2r.interpolate(frames_head_on(n_seconds=20, veh_speed=5.0, start_north=100.0), hz=5.0)
    truth = s2r.truth_rows(ped, veh, horizon_s=3.0, hit_m=2.0)
    assert len(truth) == len(veh)
    row0 = truth[0]
    assert abs(row0["distance_m"] - 100.0) < 1e-6
    assert abs(row0["closing_mps"] - 5.0) < 0.05
    assert abs(row0["ttc_s"] - 20.0) < 0.2
    assert row0["label_hit_3s"] == 0
    # 충돌 3초 전(t=17, d=15 m)부터는 3초 안에 2 m 이내 → 1
    at_17 = next(r for r in truth if abs(r["t"] - 17.0) < 1e-9)
    assert at_17["label_hit_3s"] == 1
    at_10 = next(r for r in truth if abs(r["t"] - 10.0) < 1e-9)
    assert at_10["label_hit_3s"] == 0
    assert 0 <= at_17["rule_level_clean"] <= 3


def test_stream_lines_look_like_the_rsu_raw_log_and_parse_with_step3(tmp_path):
    ped, veh = s2r.interpolate(frames_head_on(n_seconds=5), hz=5.0)
    rng = np.random.default_rng(0)
    quiet = gps_noise.FieldNoiseParams(sigma_m=0.0, dropout_per_min=0.0, packet_loss=0.0)
    packets = s2r.make_packets(ped, veh, quiet, quiet, rng, base_epoch=1_700_000_000.0)
    out = tmp_path / "raw_sim.log"
    s2r.write_stream(packets, out)
    lines = out.read_text(encoding="utf-8").splitlines()
    assert lines and all(re.match(r"^\d+\.\d+ RX \{", ln) for ln in lines)
    stamps = [float(ln.split(" ", 1)[0]) for ln in lines]
    assert stamps == sorted(stamps)
    types = set()
    for ln in lines:
        payload = json.loads(ln.split(" ", 2)[2])
        row = normalize_record(payload, "real", now=float(ln.split(" ", 1)[0]))
        types.add(row["type"])
        assert row["lat"] != "" and row["heading_valid"] in (0, 1, "0", "1")
    assert types == {"cane", "vehicle"}


def test_v3_pair_loader_reads_semicolon_csvs(tmp_path):
    (tmp_path / "feature.csv").write_text(
        "timestep_time;vehicle_id;vehicle_x;vehicle_y;vehicle_speed;vehicle_angle\n"
        "0.0;car_a;10.0;20.0;3.0;90.0\n1.0;car_a;13.0;20.0;3.0;90.0\n"
        "0.0;car_b;50.0;50.0;0.0;0.0\n1.0;car_b;50.0;50.0;0.0;0.0\n", encoding="utf-8")
    (tmp_path / "pedestrian.csv").write_text(
        "timestep_time;person_id;person_x;person_y;person_speed;person_angle\n"
        "0.0;p1;0.0;0.0;1.2;0.0\n1.0;p1;0.0;1.2;1.2;0.0\n", encoding="utf-8")
    frames = s2r.load_v3_pair(tmp_path, vehicle_id="car_a", person_id="p1")
    assert [f["t"] for f in frames] == [0.0, 1.0]
    assert frames[1]["veh_x"] == 13.0 and frames[1]["ped_y"] == 1.2
    # id 를 안 주면 첫 차량·첫 보행자
    frames = s2r.load_v3_pair(tmp_path)
    assert frames[0]["veh_x"] == 10.0
