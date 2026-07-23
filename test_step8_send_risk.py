#!/usr/bin/env python3
"""Tests for step 8 risk downlink policy: trust gating + on-change/heartbeat."""

import json
import unittest

from step4_state_store import StateStore
from step6_kinematics import KinematicsPipeline
from step8_send_risk import RiskSender, RiskTransmitter


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


class RsuEchoTest(unittest.TestCase):
    """The RSU echoes risk_tx / risk_broadcast_to_seen after accepting a downlink.

    These are acknowledgments, not pipeline input, so step 8 must consume them
    quietly rather than warn about an unknown type.
    """

    def _sender(self):
        store = StateStore(fresh_window_s=10.0)
        pipeline = KinematicsPipeline(store)
        self.sent = []
        return RiskSender(
            pipeline, RiskTransmitter(), self.sent.append, "unused.csv", {}
        )

    def test_ack_records_are_consumed_without_transmitting(self):
        sender = self._sender()
        for ack in ("risk_tx", "risk_broadcast_to_seen"):
            sender.process_line(json.dumps({"type": ack, "risk": 2}), "fallback")
        self.assertEqual(self.sent, [])
        self.assertEqual(sender.pipeline.store.latest, {})


class CommandFormatTest(unittest.TestCase):
    def test_command_matches_the_verified_downlink_shape(self):
        tx = RiskTransmitter(target_id=0)
        self.assertEqual(tx.command(2), '{"target_id":0,"risk":2}')

    def test_target_id_is_carried_into_the_command(self):
        tx = RiskTransmitter(target_id=4125577512)
        self.assertEqual(tx.command(1), '{"target_id":4125577512,"risk":1}')


if __name__ == "__main__":
    unittest.main()
