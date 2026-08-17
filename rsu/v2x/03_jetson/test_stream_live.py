"""stream_live.py 단위 테스트.

네트워크 없이 CSV tail·스냅샷 변환·재전송 정책을 검증한다.
헤더는 step8의 CSV_FIELDS에서 직접 만들어서, step8이 열을 바꾸면
이 테스트가 같이 따라간다.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "deploy"))

import stream_live  # noqa: E402
from step8_send_risk import CSV_FIELDS  # noqa: E402

HEADER = ",".join(CSV_FIELDS) + "\n"

DEFAULTS = {
    "pc_time": 100.0,
    "cane_seq": 10,
    "computed_level": 1,
    "effective_level": 1,
    "trusted": 1,
    "reason": "change",
    "target_id": 0,
    "distance_m": 8.5,
    "closing_mps": 1.2,
    "ttc_s": 3.5,
    "risk_score": 55,
    "level_source": "model",
    "cane_gps_valid": 1,
    "cane_lat": 37.496216,
    "cane_lng": 126.955551,
    "cane_speed_mps": 0.15,
    "cane_heading_deg": 90.0,
    "veh_gps_valid": 1,
    "veh_lat": 37.497078,
    "veh_lng": 126.957779,
    "veh_speed_mps": 5.0,
    "veh_heading_deg": 180.0,
    "rel_east_m": 6.3,
    "rel_north_m": 2.1,
}


def make_row(**over):
    values = {field: "" for field in CSV_FIELDS}
    values.update(DEFAULTS)
    values.update(over)
    return ",".join(str(values[field]) for field in CSV_FIELDS) + "\n"


def write_log(tmp_path, lines, name="risk_tx_test.csv", header=HEADER):
    path = tmp_path / name
    path.write_text(header + "".join(lines), encoding="utf-8")
    return path


# ---------- CsvTail ----------

def test_tail_reads_only_rows_appended_since_last_call(tmp_path):
    path = write_log(tmp_path, [make_row(pc_time=100.0)])
    tail = stream_live.CsvTail(path)
    assert len(tail.new_rows()) == 1
    assert tail.new_rows() == []

    with path.open("a", encoding="utf-8") as handle:
        handle.write(make_row(pc_time=101.0))
    rows = tail.new_rows()
    assert [row["pc_time"] for row in rows] == ["101.0"]


def test_tail_ignores_a_half_written_row(tmp_path):
    """step8이 쓰는 도중에 읽으면 잘린 행이 보인다. 완성될 때까지 기다린다."""
    path = write_log(tmp_path, [make_row(pc_time=100.0)])
    tail = stream_live.CsvTail(path)
    tail.new_rows()

    full = make_row(pc_time=101.0)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(full[:20])  # 쓰다 만 행 (개행 없음)
    assert tail.new_rows() == []

    with path.open("a", encoding="utf-8") as handle:
        handle.write(full[20:])
    rows = tail.new_rows()
    assert [row["pc_time"] for row in rows] == ["101.0"]


def test_tail_on_header_only_file_returns_nothing(tmp_path):
    path = write_log(tmp_path, [])
    assert stream_live.CsvTail(path).new_rows() == []


# ---------- 스냅샷 변환 ----------

def test_snapshot_carries_both_nodes_and_the_risk_verdict(tmp_path):
    snapshot = stream_live.snapshot_from_row(
        _rows(tmp_path, [make_row()])[0], "jetson-test"
    )
    assert snapshot["device_id"] == "jetson-test"
    assert snapshot["ts"].startswith("1970-01-01T00:01:40")
    assert snapshot["cane"] == {
        "lat": 37.496216, "lng": 126.955551,
        "speed_mps": 0.15, "heading_deg": 90.0, "gps_valid": True,
    }
    assert snapshot["vehicle"] == {
        "lat": 37.497078, "lng": 126.957779,
        "speed_mps": 5.0, "heading_deg": 180.0, "gps_valid": True,
        "rel_east_m": 6.3, "rel_north_m": 2.1,
    }
    assert snapshot["risk"] == {
        "level": 1, "distance_m": 8.5, "ttc_s": 3.5,
        "closing_mps": 1.2, "source": "model",
    }


def test_vehicle_without_a_fix_reports_null_coordinates(tmp_path):
    """실내 실험에서 차량은 gps_valid=0, lat=0으로 온다.

    (0, 0)을 그대로 실어보내면 화면이 아프리카 앞바다에 차를 그린다.
    """
    row = _rows(tmp_path, [make_row(veh_gps_valid=0, veh_lat=0.0, veh_lng=0.0)])[0]
    vehicle = stream_live.snapshot_from_row(row, "jetson-test")["vehicle"]
    assert vehicle["lat"] is None
    assert vehicle["lng"] is None
    assert vehicle["gps_valid"] is False


def test_old_csv_without_vehicle_columns_still_streams(tmp_path):
    """veh_lat/veh_lng가 없던 시절의 CSV로도 지팡이 위치는 계속 나가야 한다."""
    old_fields = [f for f in CSV_FIELDS if f not in ("veh_lat", "veh_lng")]
    header = ",".join(old_fields) + "\n"
    values = {field: "" for field in old_fields}
    values.update({k: v for k, v in DEFAULTS.items() if k in values})
    line = ",".join(str(values[f]) for f in old_fields) + "\n"

    path = write_log(tmp_path, [line], name="risk_tx_old.csv", header=header)
    row = stream_live.CsvTail(path).new_rows()[0]
    snapshot = stream_live.snapshot_from_row(row, "jetson-test")
    assert snapshot["vehicle"]["lat"] is None
    assert snapshot["cane"]["lat"] == 37.496216


def test_row_without_a_timestamp_is_not_a_snapshot(tmp_path):
    row = _rows(tmp_path, [make_row(pc_time="")])[0]
    assert stream_live.snapshot_from_row(row, "jetson-test") is None


def test_state_snapshot_carries_the_vehicle_heading_flag():
    """8080 상태의 veh.heading_deg는 마지막 유효 방향(없으면 None)이고 heading_valid가
    붙어 온다. 화면이 회전에 쓸지 판단하도록 둘 다 그대로 넘긴다."""
    state = {
        "t": 100.0, "level": 1, "distance_m": 8.5, "ttc_s": 3.5, "closing_mps": 1.2,
        "level_source": "table", "rel_east_m": 6.3, "rel_north_m": 2.1,
        "cane": {"lat": 37.496216, "lng": 126.955551, "speed_mps": 0.15,
                 "heading_deg": 90.0, "gps_valid": 1},
        "veh": {"speed_mps": 0.05, "heading_deg": 180.0, "heading_valid": 0, "gps_valid": 1},
    }
    vehicle = stream_live.snapshot_from_state(state, "jetson-test")["vehicle"]
    assert vehicle["heading_deg"] == 180.0
    assert vehicle["heading_valid"] is False
    state["veh"]["heading_deg"] = None
    assert stream_live.snapshot_from_state(state, "jetson-test")["vehicle"]["heading_deg"] is None


# ---------- 전송 정책 ----------

def _rows(tmp_path, lines):
    return stream_live.CsvTail(write_log(tmp_path, lines)).new_rows()


def test_nothing_is_sent_before_any_data_arrives():
    streamer = stream_live.LiveStreamer("jetson-test")
    assert streamer.payload(now=100.0) is None


def test_only_the_newest_row_is_streamed(tmp_path):
    """실시간 화면에 지나간 위치는 의미가 없다. 밀린 행은 건너뛴다."""
    streamer = stream_live.LiveStreamer("jetson-test")
    streamer.observe(_rows(tmp_path, [
        make_row(pc_time=100.0, effective_level=0),
        make_row(pc_time=101.0, effective_level=2),
    ]))
    payload = streamer.payload(now=101.0)
    assert payload["risk"]["level"] == 2


def test_a_quiet_moment_resends_the_last_snapshot_with_its_age(tmp_path):
    """step8은 1초 heartbeat라 0.5초 주기의 절반은 새 행이 없다.

    그때 아무것도 안 보내면 화면이 멈춘 건지 신호가 끊긴 건지 알 수 없다.
    """
    streamer = stream_live.LiveStreamer("jetson-test")
    streamer.observe(_rows(tmp_path, [make_row(pc_time=100.0)]))

    assert streamer.payload(now=100.0)["age_s"] == 0.0
    streamer.observe([])
    assert streamer.payload(now=100.5)["age_s"] == 0.5


def test_a_stale_snapshot_stops_going_out(tmp_path):
    """엔진이 죽어도 마지막 위치를 계속 보내면 화면은 살아있는 것처럼 보인다."""
    streamer = stream_live.LiveStreamer("jetson-test", max_age_s=10.0)
    streamer.observe(_rows(tmp_path, [make_row(pc_time=100.0)]))

    assert streamer.payload(now=109.9) is not None
    assert streamer.payload(now=110.1) is None


def test_a_new_row_refreshes_the_age(tmp_path):
    streamer = stream_live.LiveStreamer("jetson-test", max_age_s=10.0)
    streamer.observe(_rows(tmp_path, [make_row(pc_time=100.0)]))
    assert streamer.payload(now=109.0)["age_s"] == 9.0

    streamer.observe(_rows(tmp_path, [make_row(pc_time=109.5)], ))
    assert streamer.payload(now=110.0)["age_s"] == 0.5
