#!/usr/bin/env python3
"""Tests for step 8 risk downlink policy: trust gating + on-change/heartbeat."""

import unittest
from pathlib import Path
from types import SimpleNamespace

from step8_send_risk import (
    CSV_FIELDS,
    RawLog,
    RiskTransmitter,
    TxDecision,
    csv_row,
    format_tx,
)


class OnChangeAndHeartbeatTest(unittest.TestCase):
    def setUp(self):
        # gps_valid=1 so trust never suppresses; this class is about timing.
        self.tx = RiskTransmitter(heartbeat_s=1.0)

    def test_first_value_is_always_sent(self):
        decision = self.tx.consider(2, cane_gps_valid=1, now=0.0)
        self.assertTrue(decision.should_send)
        self.assertEqual(decision.reason, "change")
        self.assertEqual(decision.effective_level, 2)

    def test_same_level_inside_the_heartbeat_window_is_held(self):
        self.tx.consider(2, cane_gps_valid=1, now=0.0)
        decision = self.tx.consider(2, cane_gps_valid=1, now=0.5)
        self.assertFalse(decision.should_send)
        self.assertEqual(decision.reason, "hold")

    def test_same_level_is_resent_once_the_heartbeat_elapses(self):
        self.tx.consider(2, cane_gps_valid=1, now=0.0)
        decision = self.tx.consider(2, cane_gps_valid=1, now=1.0)
        self.assertTrue(decision.should_send)
        self.assertEqual(decision.reason, "heartbeat")

    def test_a_level_change_is_sent_immediately(self):
        self.tx.consider(2, cane_gps_valid=1, now=0.0)
        decision = self.tx.consider(3, cane_gps_valid=1, now=0.1)
        self.assertTrue(decision.should_send)
        self.assertEqual(decision.reason, "change")
        self.assertEqual(decision.effective_level, 3)


class TrustGatingTest(unittest.TestCase):
    def test_untrusted_position_suppresses_a_nonzero_risk_to_zero(self):
        tx = RiskTransmitter(heartbeat_s=1.0)  # allow_untrusted defaults to False
        decision = tx.consider(2, cane_gps_valid=0, now=0.0)
        self.assertEqual(decision.computed_level, 2)
        self.assertEqual(decision.effective_level, 0)
        self.assertFalse(decision.trusted)

    def test_a_trusted_alarm_is_cleared_when_the_fix_is_lost(self):
        """Sends 2 with a valid fix, then loses the fix: a 0 must go out to stop
        the cane vibrating on a now-untrustworthy alarm."""
        tx = RiskTransmitter(heartbeat_s=1.0)
        first = tx.consider(2, cane_gps_valid=1, now=0.0)
        self.assertEqual(first.effective_level, 2)

        cleared = tx.consider(2, cane_gps_valid=0, now=0.1)
        self.assertTrue(cleared.should_send)
        self.assertEqual(cleared.reason, "change")
        self.assertEqual(cleared.effective_level, 0)

    def test_allow_untrusted_passes_the_computed_level_through(self):
        tx = RiskTransmitter(heartbeat_s=1.0, allow_untrusted=True)
        decision = tx.consider(2, cane_gps_valid=0, now=0.0)
        self.assertTrue(decision.trusted)
        self.assertEqual(decision.effective_level, 2)


class CommandFormatTest(unittest.TestCase):
    def test_command_matches_the_verified_downlink_shape(self):
        tx = RiskTransmitter(target_id=0)
        self.assertEqual(tx.command(2), '{"target_id":0,"risk":2}')

    def test_target_id_is_carried_into_the_command(self):
        tx = RiskTransmitter(target_id=4125577512)
        self.assertEqual(tx.command(1), '{"target_id":4125577512,"risk":1}')


def _store(cane_node_risk, veh_node_risk):
    cane = {
        "seq": 7,
        "node_risk": cane_node_risk,
        "gps_valid": 1,
        "lat": 37.496655,
        "lng": 126.957886,
        "speed_mps": 0.5,
        "heading_deg": 180.0,
    }
    vehicle = {
        "node_risk": veh_node_risk,
        "gps_valid": 1,
        "speed_mps": 5.0,
        "heading_deg": 90.0,
    }
    return SimpleNamespace(latest={"cane": cane, "vehicle": vehicle})


class NodeRiskLoggingTest(unittest.TestCase):
    """The downlink is only verifiable if what the nodes report is logged next to it."""

    def setUp(self):
        self.decision = TxDecision(True, 2, 2, True, "change")
        self.assessment = SimpleNamespace(
            distance_m=5.0, closing_los=1.2, ttc=4.1, final_score=0.6
        )

    def test_both_node_risks_land_in_the_row(self):
        row = csv_row(
            1.0, _store(2, 1), RiskTransmitter(), self.decision, self.assessment
        )
        self.assertEqual(row["cane_node_risk"], 2)
        self.assertEqual(row["veh_node_risk"], 1)

    def test_row_keys_match_the_header(self):
        """DictWriter raises on any mismatch, so the CSV would break at runtime."""
        row = csv_row(
            1.0, _store(0, 0), RiskTransmitter(), self.decision, self.assessment
        )
        self.assertEqual(set(row), set(CSV_FIELDS))

    def test_tx_line_shows_what_the_nodes_reported(self):
        line = format_tx(self.decision, 2, 0)
        self.assertIn("cane_node_risk=2", line)
        self.assertIn("veh_node_risk=0", line)


class RawLogTest(unittest.TestCase):
    """통신이 의심스러울 때 볼 수 있는 유일한 기록이라, 방향과 순서가 남아야 한다."""

    def test_rx_and_tx_are_interleaved_in_arrival_order(self):
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "raw.log"
            log = RawLog(path)
            log.write("RX", '{"type":"cane","node_risk":0}')
            log.write("TX", '{"target_id":0,"risk":2}')
            log.write("RX", '{"type":"cane","node_risk":2}')
            log.close()

            lines = path.read_text(encoding="utf-8").splitlines()

        self.assertEqual([line.split(" ", 2)[1] for line in lines], ["RX", "TX", "RX"])
        self.assertIn('"risk":2', lines[1])
        # 앞의 시각이 float로 읽혀야 나중에 지연을 계산할 수 있다.
        self.assertLessEqual(
            float(lines[0].split(" ", 1)[0]), float(lines[2].split(" ", 1)[0])
        )

    def test_unparseable_lines_are_kept(self):
        """깨진 줄이야말로 남겨야 원인을 찾을 수 있다."""
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "raw.log"
            log = RawLog(path)
            log.write("RX", "{broken json")
            log.close()
            self.assertIn("{broken json", path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
