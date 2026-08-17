import unittest

from live_state import TTC_UNBOUNDED_S, LiveState
from step6_kinematics import Kinematics
from step7_risk import RiskAssessment
from step8_send_risk import TxDecision


class FakeStore:
    """step4 StateStore 중 LiveState가 읽는 부분만."""

    def __init__(self, cane=None, vehicle=None):
        self.latest = {
            "cane": {
                "seq": 12,
                "gps_valid": 1,
                "lat": 37.4963,
                "lng": 126.9575,
                "speed_mps": 1.1,
                "heading_deg": 41.0,
                "node_risk": 0,
                **(cane or {}),
            },
            "vehicle": {
                "seq": 8,
                "gps_valid": 1,
                "lat": 37.4966,
                "lng": 126.9579,
                "speed_mps": 8.3,
                "heading_deg": 220.0,
                "heading_valid": 1,
                "node_risk": 0,
                **(vehicle or {}),
            },
        }


def make_kinematics(rel_east=8.2, rel_north=9.1, dcpa=1.4):
    return Kinematics(
        distance_m=12.3,
        closing_los=5.8,
        closing_diff=5.7,
        ttc_simple=2.1,
        tcpa=2.0,
        dcpa=dcpa,
        rel_east=rel_east,
        rel_north=rel_north,
    )


def make_assessment(ttc=2.1, dcpa=1.4, level=2, reason="table"):
    return RiskAssessment(
        distance_m=12.3,
        closing_los=5.8,
        vehicle_speed_mps=8.3,
        ttc=ttc,
        dcpa=dcpa,
        base_score=2.4,
        gate=1.0,
        final_score=2.4,
        risk_level=level,
        reason=reason,
    )


def make_decision(effective=2, computed=2, trusted=True, should_send=True):
    return TxDecision(
        should_send=should_send,
        computed_level=computed,
        effective_level=effective,
        trusted=trusted,
        reason="change",
    )


def updated(state, now=1000.0, **kwargs):
    state.update(
        now=now,
        store=kwargs.pop("store", FakeStore()),
        decision=kwargs.pop("decision", make_decision()),
        assessment=kwargs.pop("assessment", make_assessment()),
        kinematics=kwargs.pop("kinematics", make_kinematics()),
        **kwargs,
    )
    return state


class TestEmptyState(unittest.TestCase):
    def test_snapshot_before_any_update(self):
        """데이터가 오기 전에도 화면이 읽을 수 있는 형태여야 한다."""
        snap = LiveState().snapshot(now=1000.0)
        self.assertIsNone(snap["t"])
        self.assertIsNone(snap["age_s"])
        self.assertEqual(snap["level"], 0)
        self.assertIsNone(snap["distance_m"])

    def test_empty_snapshot_has_same_keys_as_filled(self):
        """빈 스냅샷과 채워진 스냅샷의 모양이 같아야 화면 코드가 갈라지지 않는다."""
        empty = LiveState().snapshot(now=1000.0)
        filled = updated(LiveState()).snapshot(now=1000.0)
        self.assertEqual(set(empty), set(filled))
        self.assertEqual(set(empty["cane"]), set(filled["cane"]))
        self.assertEqual(set(empty["veh"]), set(filled["veh"]))


class TestSnapshotContents(unittest.TestCase):
    def test_carries_measured_values(self):
        snap = updated(LiveState(), now=1000.0).snapshot(now=1000.0)
        self.assertEqual(snap["t"], 1000.0)
        self.assertEqual(snap["distance_m"], 12.3)
        self.assertEqual(snap["ttc_s"], 2.1)
        self.assertEqual(snap["closing_mps"], 5.8)
        self.assertEqual(snap["dcpa_m"], 1.4)
        self.assertEqual(snap["rel_east_m"], 8.2)
        self.assertEqual(snap["rel_north_m"], 9.1)

    def test_level_is_what_the_cane_received(self):
        """화면 등급은 effective_level이다. 지팡이가 받지 않은 등급을 보이면 안 된다."""
        state = updated(
            LiveState(),
            decision=make_decision(effective=0, computed=3, trusted=False),
        )
        snap = state.snapshot(now=1000.0)
        self.assertEqual(snap["level"], 0)
        self.assertFalse(snap["trusted"])

    def test_node_fields_are_floats(self):
        """store 값은 JSON/CSV에서 와 문자열일 수 있다. 화면은 숫자를 받아야 한다."""
        store = FakeStore(cane={"lat": "37.4963", "speed_mps": "1.1"})
        snap = updated(LiveState(), store=store).snapshot(now=1000.0)
        self.assertIsInstance(snap["cane"]["lat"], float)
        self.assertIsInstance(snap["cane"]["speed_mps"], float)

    def test_unparsable_node_field_becomes_none(self):
        store = FakeStore(cane={"heading_deg": ""})
        snap = updated(LiveState(), store=store).snapshot(now=1000.0)
        self.assertIsNone(snap["cane"]["heading_deg"])


class TestVehicleHeading(unittest.TestCase):
    """차량 heading은 GPS 이동방향이라 서 있으면 못 믿는다.

    펌웨어는 speed>=0.4 m/s일 때만 heading_valid=1을 주고, 아니면 이전 값이나
    0(정북)을 heading_deg에 실어 보낸다(2026-08-17 오후: 유효 7~21%). 그 0을
    그대로 화면에 넘기면 차량 1인칭 화면이 정지한 차를 북향으로 돌려 그린다.
    화면에는 마지막으로 믿을 수 있었던 방향을 유지해 내보내고, 유효 여부도
    같이 실어 화면이 스스로 판단할 수 있게 한다.
    """

    def test_valid_heading_is_carried_with_its_flag(self):
        snap = updated(LiveState()).snapshot(now=1000.0)
        self.assertEqual(snap["veh"]["heading_deg"], 220.0)
        self.assertEqual(snap["veh"]["heading_valid"], 1)

    def test_invalid_heading_keeps_the_last_valid_one(self):
        state = updated(LiveState(), now=1000.0)
        store = FakeStore(vehicle={"heading_deg": 0.0, "heading_valid": 0, "speed_mps": 0.05})
        updated(state, now=1000.2, store=store)
        snap = state.snapshot(now=1000.2)
        self.assertEqual(snap["veh"]["heading_deg"], 220.0)
        self.assertEqual(snap["veh"]["heading_valid"], 0)

    def test_no_valid_heading_yet_gives_none_not_north(self):
        store = FakeStore(vehicle={"heading_deg": 0.0, "heading_valid": 0})
        snap = updated(LiveState(), store=store).snapshot(now=1000.0)
        self.assertIsNone(snap["veh"]["heading_deg"])
        self.assertEqual(snap["veh"]["heading_valid"], 0)

    def test_missing_flag_is_not_trusted(self):
        """heading_valid 칸이 없는 옛 레코드의 heading은 화면 회전에 쓰지 않는다."""
        store = FakeStore()
        del store.latest["vehicle"]["heading_valid"]
        snap = updated(LiveState(), store=store).snapshot(now=1000.0)
        self.assertIsNone(snap["veh"]["heading_deg"])
        self.assertEqual(snap["veh"]["heading_valid"], 0)


class TestTtcSentinel(unittest.TestCase):
    def test_non_approaching_ttc_becomes_none(self):
        """step7은 접근하지 않는 쌍에 9999를 준다. 화면에 9999초를 띄우면 안 된다."""
        state = updated(LiveState(), assessment=make_assessment(ttc=9999.0))
        self.assertIsNone(state.snapshot(now=1000.0)["ttc_s"])

    def test_ttc_just_under_the_bound_survives(self):
        state = updated(LiveState(), assessment=make_assessment(ttc=TTC_UNBOUNDED_S - 1))
        self.assertEqual(state.snapshot(now=1000.0)["ttc_s"], TTC_UNBOUNDED_S - 1)

    def test_missing_dcpa_becomes_none(self):
        """dcpa는 멀어지는 쌍에서 None이다."""
        state = updated(
            LiveState(),
            assessment=make_assessment(dcpa=None),
            kinematics=make_kinematics(dcpa=None),
        )
        self.assertIsNone(state.snapshot(now=1000.0)["dcpa_m"])


class TestAge(unittest.TestCase):
    def test_age_grows_with_the_clock(self):
        state = updated(LiveState(), now=1000.0)
        self.assertAlmostEqual(state.snapshot(now=1002.5)["age_s"], 2.5)

    def test_age_is_zero_at_the_moment_of_update(self):
        state = updated(LiveState(), now=1000.0)
        self.assertAlmostEqual(state.snapshot(now=1000.0)["age_s"], 0.0)

    def test_clock_going_backwards_does_not_produce_negative_age(self):
        """단조 시계를 쓰지 않는 호출자가 있어도 화면에 -3초가 뜨면 안 된다."""
        state = updated(LiveState(), now=1000.0)
        self.assertEqual(state.snapshot(now=997.0)["age_s"], 0.0)


class TestAtomicity(unittest.TestCase):
    def test_snapshot_is_not_the_live_dict(self):
        """HTTP 스레드가 받아간 dict를 다음 판정이 덮어쓰면 안 된다."""
        state = updated(LiveState(), now=1000.0)
        first = state.snapshot(now=1000.0)
        updated(state, now=1001.0, assessment=make_assessment(ttc=7.7))
        self.assertEqual(first["ttc_s"], 2.1)

    def test_mutating_a_snapshot_does_not_corrupt_the_state(self):
        state = updated(LiveState(), now=1000.0)
        snap = state.snapshot(now=1000.0)
        snap["level"] = 99
        snap["cane"]["lat"] = 0.0
        fresh = state.snapshot(now=1000.0)
        self.assertEqual(fresh["level"], 2)
        self.assertEqual(fresh["cane"]["lat"], 37.4963)


class TestLevelSource(unittest.TestCase):
    def test_gate_source_is_carried(self):
        class FakeGate:
            level = 2
            source = "model"
            proba = 0.81

        state = updated(LiveState(), gate=FakeGate())
        snap = state.snapshot(now=1000.0)
        self.assertEqual(snap["level_source"], "model")
        self.assertEqual(snap["model_proba"], 0.81)

    def test_without_a_gate_the_source_is_the_rule_table(self):
        snap = updated(LiveState()).snapshot(now=1000.0)
        self.assertEqual(snap["level_source"], "table")
        self.assertIsNone(snap["model_proba"])

    def test_safety_floor_reason_is_visible(self):
        """하한이 발동한 경보는 화면에서도 구분돼야 한다."""
        state = updated(LiveState(), assessment=make_assessment(reason="safety_floor"))
        self.assertEqual(state.snapshot(now=1000.0)["reason"], "safety_floor")


if __name__ == "__main__":
    unittest.main()
