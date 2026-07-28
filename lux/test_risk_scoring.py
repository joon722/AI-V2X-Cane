#!/usr/bin/env python3
"""Tests for step 7 risk scoring: vendored team table + DCPA suppression gate."""

import importlib.util
import unittest
from pathlib import Path

from kinematics import relative_kinematics
from risk_scoring import (
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


TEAM_TABLE_PATH = Path(__file__).resolve().parent.parent / "scripts" / "risk_calculator.py"


def _load_team_table():
    """Import the team's own scoring table straight from its file.

    lux/ is imported flat rather than as a package, so scripts/ is not on
    sys.path; loading by path keeps this independent of how the caller set
    PYTHONPATH. Returns None when the file is absent, which is the case on a
    Jetson that only carries lux/.
    """
    if not TEAM_TABLE_PATH.exists():
        return None
    spec = importlib.util.spec_from_file_location("team_risk_calculator", TEAM_TABLE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TeamTableDriftTest(unittest.TestCase):
    """The vendored copy must stay numerically identical to the team's own file.

    risk_scoring.py freezes the table so lux/ runs on the Jetson without needing
    scripts/ alongside it. The price of that copy is drift: if the team retunes
    scripts/risk_calculator.py, nothing else in lux/ would notice. This is the
    tripwire -- it fails the moment the two disagree, and a failure here means
    "re-sync the three functions", not "change these numbers".
    """

    # distance_m, rel_speed_mps, vehicle_speed_mps, zone_base_risk.
    # Chosen to cross every cutoff in the table, including the not-approaching
    # branch and the zone clamp at both ends.
    CASES = (
        (8.0, 5.0, 5.0, 0),
        (50.0, 5.0, 5.0, 0),
        (8.0, 25.0, 25.0, 0),
        (8.0, 0.0, 0.0, 0),
        (15.0, 12.0, 18.0, 3),
        (95.0, 1.0, 2.0, 5),
        (3.0, -2.0, 0.0, 2),
        (120.0, 30.0, 30.0, 0),
        (35.0, 16.0, 11.0, 7),
    )

    @classmethod
    def setUpClass(cls):
        cls.team = _load_team_table()

    def setUp(self):
        if self.team is None:
            self.skipTest(f"team table not present at {TEAM_TABLE_PATH}")

    def test_ttc_matches_the_team_table(self):
        for distance, rel_speed, _veh, _zone in self.CASES:
            with self.subTest(distance=distance, rel_speed=rel_speed):
                self.assertEqual(
                    calculate_ttc(distance, rel_speed),
                    self.team.calculate_ttc(distance, rel_speed),
                )

    def test_score_matches_the_team_table(self):
        for distance, rel_speed, veh_speed, zone in self.CASES:
            with self.subTest(distance=distance, rel_speed=rel_speed, zone=zone):
                ttc = calculate_ttc(distance, rel_speed)
                self.assertEqual(
                    calculate_risk_score(distance, rel_speed, veh_speed, ttc, zone),
                    self.team.calculate_risk_score(distance, rel_speed, veh_speed, ttc, zone),
                )

    def test_level_cutoffs_match_the_team_table(self):
        for score in (0, 19.99, 20, 44.99, 45, 69.99, 70, 100):
            with self.subTest(score=score):
                self.assertEqual(
                    classify_risk_level(score), self.team.classify_risk_level(score)
                )


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
