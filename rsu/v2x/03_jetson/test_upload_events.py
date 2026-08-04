"""upload_events.py 단위 테스트.

네트워크 없이 가짜 sender로 선별·스로틀·오프셋 재개 로직을 검증한다.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "deploy"))

import upload_events  # noqa: E402

HEADER = (
    "pc_time,cane_seq,computed_level,effective_level,trusted,reason,target_id,"
    "distance_m,closing_mps,ttc_s,risk_score,cane_gps_valid,cane_lat,cane_lng,"
    "cane_speed_mps,cane_heading_deg,veh_gps_valid,veh_speed_mps,veh_heading_deg\n"
)


def make_row(t, level, lat="37.4963", lng="126.9575", gps="1", ttc="3.5"):
    return (
        f"{t},10,{level},{level},1,change,0,8.5,1.2,{ttc},55,{gps},{lat},{lng},"
        f"1.1,90.0,1,5.0,180.0\n"
    )


def write_log(tmp_path, lines, name="risk_tx_test.csv"):
    path = tmp_path / name
    path.write_text(HEADER + "".join(lines), encoding="utf-8")
    return path


class FakeSender:
    def __init__(self, fail_after=None):
        self.sent = []
        self.fail_after = fail_after

    def __call__(self, event):
        if self.fail_after is not None and len(self.sent) >= self.fail_after:
            return False
        self.sent.append(event)
        return True


def process(path, state, sender, min_interval=10.0):
    return upload_events.process_file(
        path, state, sender, min_interval=min_interval, device_id="jetson-test"
    )


# ---------- read_new_rows ----------

def test_read_new_rows_returns_all_then_nothing(tmp_path):
    path = write_log(tmp_path, [make_row(100.0, 1), make_row(101.0, 2)])
    rows, offset = upload_events.read_new_rows(path, 0)
    assert len(rows) == 2
    assert rows[0][0]["effective_level"] == "1"
    rows2, offset2 = upload_events.read_new_rows(path, offset)
    assert rows2 == []
    assert offset2 == offset


def test_read_new_rows_picks_up_appended(tmp_path):
    path = write_log(tmp_path, [make_row(100.0, 1)])
    _, offset = upload_events.read_new_rows(path, 0)
    with path.open("a", encoding="utf-8") as f:
        f.write(make_row(105.0, 2))
    rows, _ = upload_events.read_new_rows(path, offset)
    assert len(rows) == 1
    assert rows[0][0]["pc_time"] == "105.0"


def test_read_new_rows_ignores_partial_line(tmp_path):
    path = write_log(tmp_path, [make_row(100.0, 1)])
    with path.open("a", encoding="utf-8") as f:
        f.write("101.0,11,2,2,1,change")  # 쓰다 만 행 (개행 없음)
    rows, offset = upload_events.read_new_rows(path, 0)
    assert len(rows) == 1
    # 나중에 행이 완성되면 이어서 읽힌다
    with path.open("a", encoding="utf-8") as f:
        f.write(",0,8.5,1.2,3.5,55,1,37.5,126.9,1.1,90.0,1,5.0,180.0\n")
    rows2, _ = upload_events.read_new_rows(path, offset)
    assert len(rows2) == 1
    assert rows2[0][0]["pc_time"] == "101.0"


# ---------- 이벤트 선별/스로틀 ----------

def test_level_zero_not_sent(tmp_path):
    path = write_log(tmp_path, [make_row(100.0, 0), make_row(101.0, 0)])
    sender = FakeSender()
    assert process(path, state := {}, sender)
    assert sender.sent == []


def test_first_risk_row_sent_with_payload(tmp_path):
    path = write_log(tmp_path, [make_row(100.0, 2)])
    sender = FakeSender()
    assert process(path, {}, sender)
    assert len(sender.sent) == 1
    ev = sender.sent[0]
    assert ev["event_uid"] == "jetson-test-100.0"
    assert ev["source"] == "live"
    assert ev["lat"] == 37.4963
    assert ev["lng"] == 126.9575
    assert ev["risk"] == 2
    assert ev["ttc"] == 3.5
    assert ev["distance_m"] == 8.5
    assert ev["device_id"] == "jetson-test"
    assert ev["occurred_at"].startswith("1970-01-01T00:01:40")


def test_sustained_level_throttled_to_interval(tmp_path):
    rows = [make_row(100.0 + i, 1) for i in range(15)]  # 1초 간격 15행
    path = write_log(tmp_path, rows)
    sender = FakeSender()
    assert process(path, {}, sender)
    # 100.0 전송 -> 110.0 전송 (10초 스로틀)
    sent_times = [e["event_uid"].split("-")[-1] for e in sender.sent]
    assert sent_times == ["100.0", "110.0"]


def test_level_rise_sent_immediately(tmp_path):
    path = write_log(
        tmp_path, [make_row(100.0, 1), make_row(101.0, 2), make_row(102.0, 2)]
    )
    sender = FakeSender()
    assert process(path, {}, sender)
    assert [e["risk"] for e in sender.sent] == [1, 2]


def test_drop_to_zero_resets_throttle(tmp_path):
    path = write_log(
        tmp_path, [make_row(100.0, 1), make_row(101.0, 0), make_row(102.0, 1)]
    )
    sender = FakeSender()
    assert process(path, {}, sender)
    assert [e["event_uid"] for e in sender.sent] == [
        "jetson-test-100.0",
        "jetson-test-102.0",
    ]


def test_invalid_gps_or_missing_coords_skipped(tmp_path):
    path = write_log(
        tmp_path,
        [
            make_row(100.0, 2, gps="0"),
            make_row(101.0, 2, lat="", lng=""),
            make_row(102.0, 2),
        ],
    )
    sender = FakeSender()
    assert process(path, {}, sender)
    assert [e["event_uid"] for e in sender.sent] == ["jetson-test-102.0"]


def test_old_header_without_coords_skipped(tmp_path):
    old_header = "pc_time,cane_seq,computed_level,effective_level,trusted,reason,target_id\n"
    path = tmp_path / "risk_tx_old.csv"
    path.write_text(old_header + "100.0,10,2,2,1,change,0\n", encoding="utf-8")
    sender = FakeSender()
    state = {}
    assert process(path, state, sender)
    assert sender.sent == []
    assert state["offset"] > 0  # 오프셋은 전진 (재처리 안 함)


# ---------- 실패/재개 ----------

def test_send_failure_keeps_offset_for_retry(tmp_path):
    path = write_log(
        tmp_path, [make_row(100.0, 1), make_row(115.0, 1), make_row(130.0, 1)]
    )
    sender = FakeSender(fail_after=1)  # 첫 건 성공, 둘째 건부터 실패
    state = {}
    assert not process(path, state, sender)
    assert [e["event_uid"] for e in sender.sent] == ["jetson-test-100.0"]

    # 재시도: 실패했던 행부터 다시 (100.0은 다시 안 보냄)
    sender2 = FakeSender()
    assert process(path, state, sender2)
    assert [e["event_uid"] for e in sender2.sent] == [
        "jetson-test-115.0",
        "jetson-test-130.0",
    ]


def test_state_offset_skips_processed_rows(tmp_path):
    path = write_log(tmp_path, [make_row(100.0, 2)])
    sender = FakeSender()
    state = {}
    assert process(path, state, sender)
    with path.open("a", encoding="utf-8") as f:
        f.write(make_row(200.0, 2))
    sender2 = FakeSender()
    assert process(path, state, sender2)
    assert [e["event_uid"] for e in sender2.sent] == ["jetson-test-200.0"]
