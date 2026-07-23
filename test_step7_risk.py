#!/usr/bin/env python3
"""Tests for step 7 risk scoring: vendored team table + DCPA suppression gate."""

import unittest

from step6_kinematics import relative_kinematics
from step7_risk import (
    DCPA_FAR_M,
    DCPA_FLOOR,
    DCPA_NEAR_M,
    assess_risk,
    calculate_risk_score,
    calculate_ttc,
    classify_risk_level,
    dcpa_gate,
)


class VendoredTeamTableTest(unittest.TestCase):
    """The three scoring functions are a verbatim copy of the team table.

    These are frozen regression values computed from that table, so a change to
    the copied numbers is caught here rather than silently diverging.
    """

    def test_close_head_on_scores_level_3(self):
        ttc = calculate_ttc(8.0, 5.0)  # 1.6 s
        score = calculate_risk_score(8.0, 5.0, 5.0, ttc)
        self.assertEqual(score, 71.0)  # 30 + 30 + 8 + 3
        self.assertEqual(classify_risk_level(score), 3)

    def test_distant_slow_approach_scores_level_1(self):
        ttc = calculate_ttc(50.0, 5.0)  # 10 s
        score = calculate_risk_score(50.0, 5.0, 5.0, ttc)
        self.assertEqual(score, 29.0)  # 10 + 8 + 8 + 3
        self.assertEqual(classify_risk_level(score), 1)

    def test_fast_close_approach_saturates_high(self):
        ttc = calculate_ttc(8.0, 25.0)  # 0.32 s
        score = calculate_risk_score(8.0, 25.0, 25.0, ttc)
        self.assertEqual(score, 95.0)  # 30 + 35 + 20 + 10
        self.assertEqual(classify_risk_level(score), 3)

    def test_stationary_vehicle_scores_only_distance(self):
        ttc = calculate_ttc(8.0, 0.0)  # 9999, no approach
        score = calculate_risk_score(8.0, 0.0, 0.0, ttc)
        self.assertEqual(score, 30.0)  # distance only
        self.assertEqual(classify_risk_level(score), 1)

    def test_ttc_is_infinite_when_not_approaching(self):
        self.assertEqual(calculate_ttc(10.0, 0.0), 9999.0)
        self.assertEqual(calculate_ttc(10.0, -3.0), 9999.0)


class DcpaGateTest(unittest.TestCase):
    def test_missing_dcpa_does_not_suppress(self):
        # A receding or stationary pair has no closest approach ahead; there is
        # nothing to gate, so the team table's own low score stands.
        self.assertEqual(dcpa_gate(None), 1.0)

    def test_dead_centre_is_not_suppressed(self):
        self.assertEqual(dcpa_gate(0.0), 1.0)

    def test_inside_the_near_margin_is_not_suppressed(self):
        self.assertEqual(dcpa_gate(DCPA_NEAR_M), 1.0)

    def test_beyond_the_far_margin_is_floored(self):
        self.assertEqual(dcpa_gate(DCPA_FAR_M), DCPA_FLOOR)
        self.assertEqual(dcpa_gate(100.0), DCPA_FLOOR)

    def test_midpoint_interpolates_linearly(self):
        midpoint = (DCPA_NEAR_M + DCPA_FAR_M) / 2
        expected = 1.0 + 0.5 * (DCPA_FLOOR - 1.0)
        self.assertAlmostEqual(dcpa_gate(midpoint), expected, places=9)

    def test_gate_is_monotonically_non_increasing(self):
        values = [dcpa_gate(d) for d in [0.0, 1.0, 2.5, 4.0, 5.0, 6.0, 7.5, 20.0]]
        for earlier, later in zip(values, values[1:]):
            self.assertGreaterEqual(earlier, later)


class AssessRiskTest(unittest.TestCase):
    def test_head_on_collision_course_is_not_gated_and_stays_high(self):
        """Vehicle 8 m due north, driving straight in. dcpa=0 -> gate is a no-op."""
        kin = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(0.0, 8.0),
            veh_vel=(0.0, -5.0),
        )
        result = assess_risk(kin, vehicle_speed_mps=5.0)
        self.assertEqual(result.gate, 1.0)
        self.assertEqual(result.final_score, result.base_score)
        self.assertEqual(result.risk_level, 3)

    def test_wide_glancing_pass_is_suppressed_below_the_head_on_case(self):
        """Same speed and similar range, but the car misses by 9 m.

        Without the gate this scores level 2 on the team table; the gate must
        pull it down because the vehicle never actually reaches the cane.
        """
        kin = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(9.0, 8.0),
            veh_vel=(0.0, -5.0),
        )
        self.assertGreaterEqual(kin.dcpa, DCPA_FAR_M)  # a clear miss

        result = assess_risk(kin, vehicle_speed_mps=5.0)
        self.assertEqual(result.gate, DCPA_FLOOR)
        self.assertEqual(classify_risk_level(result.base_score), 2)  # ungated
        self.assertLess(result.risk_level, 2)  # gated down
        self.assertAlmostEqual(result.final_score, result.base_score * DCPA_FLOOR, places=2)

    def test_receding_vehicle_has_no_gate_and_low_score(self):
        kin = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(0.0, 8.0),
            veh_vel=(0.0, 5.0),
        )
        self.assertIsNone(kin.dcpa)
        result = assess_risk(kin, vehicle_speed_mps=5.0)
        self.assertEqual(result.gate, 1.0)
        # Distance term still fires (8 m), but no TTC and no closing speed.
        self.assertEqual(result.risk_level, 1)


if __name__ == "__main__":
    unittest.main()
