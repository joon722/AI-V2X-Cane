#!/usr/bin/env python3
"""Tests for the level hysteresis: rise instantly, fall only after the hold."""

import unittest

from stability import LevelStabilizer


class RiseAndFirstValueTest(unittest.TestCase):
    def test_first_value_is_adopted_immediately(self):
        st = LevelStabilizer(hold_s=2.0)
        self.assertEqual(st.stabilize(1, now=0.0), 1)

    def test_a_rise_is_never_delayed(self):
        st = LevelStabilizer(hold_s=2.0)
        st.stabilize(0, now=0.0)
        self.assertEqual(st.stabilize(2, now=0.1), 2)

    def test_a_rise_during_a_pending_drop_cancels_the_drop(self):
        # 1 -> dip to 0 -> back to 1 before the hold expires: the dip must not
        # survive as a pending drop that later fires. A NEW dip restarts the clock.
        st = LevelStabilizer(hold_s=2.0)
        st.stabilize(1, now=0.0)
        st.stabilize(0, now=0.5)
        st.stabilize(1, now=1.0)                       # cancels the 0.5 dip
        self.assertEqual(st.stabilize(0, now=2.6), 1)  # new dip, clock restarts at 2.6
        self.assertEqual(st.stabilize(0, now=3.1), 1)  # only 0.5s into the new dip
        self.assertEqual(st.stabilize(0, now=4.7), 0)  # 2.1s sustained: adopted


class FallTest(unittest.TestCase):
    def test_a_brief_dip_is_held_at_the_previous_level(self):
        st = LevelStabilizer(hold_s=2.0)
        st.stabilize(1, now=0.0)
        self.assertEqual(st.stabilize(0, now=0.5), 1)
        self.assertEqual(st.stabilize(0, now=1.5), 1)

    def test_a_sustained_drop_is_adopted_after_the_hold(self):
        st = LevelStabilizer(hold_s=2.0)
        st.stabilize(1, now=0.0)
        st.stabilize(0, now=0.5)
        self.assertEqual(st.stabilize(0, now=2.5), 0)

    def test_field_log_flapping_burst_is_flattened(self):
        # The worst 2s burst from risk_tx_20260801_145449.csv: 0/1 toggling on
        # nearly every fix. The output must stay at 1 throughout.
        st = LevelStabilizer(hold_s=2.0)
        burst = [
            (1, 0.00), (0, 0.08), (1, 0.16), (0, 0.24), (1, 0.31),
            (0, 0.55), (1, 0.62), (0, 0.66), (1, 0.73), (0, 0.81),
            (1, 0.87), (0, 0.91), (1, 0.98), (0, 1.10),
        ]
        outputs = {st.stabilize(level, now) for level, now in burst}
        self.assertEqual(outputs, {1})

    def test_zero_hold_disables_the_hysteresis(self):
        st = LevelStabilizer(hold_s=0.0)
        st.stabilize(1, now=0.0)
        self.assertEqual(st.stabilize(0, now=0.001), 0)


if __name__ == "__main__":
    unittest.main()
