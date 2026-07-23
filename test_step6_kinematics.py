#!/usr/bin/env python3
"""Tests for step 6 distance / closing speed / TTC / CPA math."""

import math
import unittest

from step3_parse_v2x import normalize_record
from step4_state_store import StateStore
from step5_test_vehicle import offset_position
from step6_kinematics import (
    KinematicsPipeline,
    LocalFrame,
    NodeTracker,
    relative_kinematics,
    velocity_from_heading,
)


CANE_LAT = 37.0
CANE_LNG = 127.0


class LocalFrameTest(unittest.TestCase):
    def test_recovers_the_offset_distance_used_to_build_the_point(self):
        """A point placed 50 m away must measure 50 m after the ENU transform.

        Step 5 builds the simulated vehicle with offset_position, so this is the
        round trip that lets the computed distance be checked against the
        generator's intended distance_m.
        """
        frame = LocalFrame(CANE_LAT, CANE_LNG)
        for bearing in (0.0, 45.0, 90.0, 200.0):
            lat, lng = offset_position(CANE_LAT, CANE_LNG, bearing, 50.0)
            east, north = frame.to_enu(lat, lng)
            self.assertAlmostEqual(math.hypot(east, north), 50.0, places=6)


class VelocityFromHeadingTest(unittest.TestCase):
    def test_heading_180_points_south(self):
        east, north = velocity_from_heading(5.0, 180.0)
        self.assertAlmostEqual(east, 0.0, places=9)
        self.assertAlmostEqual(north, -5.0, places=9)

    def test_heading_90_points_east(self):
        east, north = velocity_from_heading(5.0, 90.0)
        self.assertAlmostEqual(east, 5.0, places=9)
        self.assertAlmostEqual(north, 0.0, places=9)


class HeadOnApproachTest(unittest.TestCase):
    """Vehicle 50 m due north of a stationary cane, driving straight at it."""

    def setUp(self):
        self.result = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(0.0, 50.0),
            veh_vel=(0.0, -5.0),
        )

    def test_distance_is_the_straight_line_gap(self):
        self.assertAlmostEqual(self.result.distance_m, 50.0, places=9)

    def test_closing_speed_equals_the_vehicle_speed(self):
        self.assertAlmostEqual(self.result.closing_los, 5.0, places=9)

    def test_simple_ttc_is_distance_over_closing_speed(self):
        self.assertAlmostEqual(self.result.ttc_simple, 10.0, places=9)

    def test_cpa_agrees_with_simple_ttc_and_predicts_a_hit(self):
        self.assertAlmostEqual(self.result.tcpa, 10.0, places=9)
        self.assertAlmostEqual(self.result.dcpa, 0.0, places=9)


class GlancingPassTest(unittest.TestCase):
    """Vehicle passes 5 m to the side. Simple TTC cries collision, CPA does not."""

    def setUp(self):
        self.result = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(5.0, 50.0),
            veh_vel=(0.0, -5.0),
        )

    def test_simple_ttc_still_reports_an_imminent_collision(self):
        self.assertLess(self.result.ttc_simple, 11.0)

    def test_dcpa_reveals_the_vehicle_never_gets_closer_than_5m(self):
        self.assertAlmostEqual(self.result.dcpa, 5.0, places=9)
        self.assertAlmostEqual(self.result.tcpa, 10.0, places=9)


class RecedingVehicleTest(unittest.TestCase):
    def setUp(self):
        self.result = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(0.0, 50.0),
            veh_vel=(0.0, 5.0),
        )

    def test_closing_speed_goes_negative(self):
        self.assertAlmostEqual(self.result.closing_los, -5.0, places=9)

    def test_no_ttc_because_the_gap_is_growing(self):
        self.assertIsNone(self.result.ttc_simple)

    def test_no_cpa_because_the_closest_approach_is_in_the_past(self):
        self.assertIsNone(self.result.tcpa)
        self.assertIsNone(self.result.dcpa)


class BothStationaryTest(unittest.TestCase):
    def test_zero_relative_speed_yields_no_ttc_instead_of_dividing_by_zero(self):
        result = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(0.0, 50.0),
            veh_vel=(0.0, 0.0),
        )
        self.assertAlmostEqual(result.closing_los, 0.0, places=9)
        self.assertIsNone(result.ttc_simple)
        self.assertIsNone(result.tcpa)
        self.assertIsNone(result.dcpa)


class ClosingByDifferenceTest(unittest.TestCase):
    def test_distance_derivative_matches_the_projected_closing_speed(self):
        """Head-on approach: both ways of measuring closing speed must agree."""
        result = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(0.0, 50.0),
            veh_vel=(0.0, -5.0),
            prev_distance_m=55.0,
            dt_s=1.0,
        )
        self.assertAlmostEqual(result.closing_diff, 5.0, places=9)
        self.assertAlmostEqual(result.closing_diff, result.closing_los, places=9)

    def test_closing_diff_is_absent_without_a_previous_sample(self):
        result = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(0.0, 50.0),
            veh_vel=(0.0, -5.0),
        )
        self.assertIsNone(result.closing_diff)


class AsynchronousArrivalTest(unittest.TestCase):
    """The cane reports at 10 Hz and the vehicle at 5 Hz, as on real hardware.

    Distance only moves when a vehicle record lands, so a naive sample-to-sample
    derivative alternates between 0 and double the true speed. The reported
    closing speed has to survive that.
    """

    def _run(self, seconds=6.0):
        store = StateStore(fresh_window_s=10.0)
        pipeline = KinematicsPipeline(store)
        results = []
        self.last_vehicle_distance = None
        for tick in range(int(seconds / 0.1)):
            now = tick * 0.1
            # The two nodes never land on the same millisecond in practice, and
            # that few-ms skew is exactly what a naive derivative trips over.
            rows = [(_cane_payload(tick), now)]
            if tick % 2 == 0:
                self.last_vehicle_distance = 50.0 - 5.0 * now
                rows.append((_vehicle_payload(tick, self.last_vehicle_distance), now + 0.003))
            for payload, stamp in rows:
                row = normalize_record(payload, "test", now=stamp)
                store.update(row)
                pipeline.observe(row)
                result = pipeline.compute()
                if result is not None:
                    results.append(result)
        return results

    def test_raw_distance_matches_what_the_generator_intended(self):
        """The whole point of sharing step 5's earth constant: an exact match.

        The comparison is against the newest vehicle coordinate rather than the
        newest record of any kind, because a cane-only update moves the clock
        without moving the vehicle.
        """
        _, raw, _ = self._run()[-1]
        self.assertAlmostEqual(raw.distance_m, self.last_vehicle_distance, places=6)

    def test_closing_diff_stays_near_the_true_closing_speed(self):
        diffs = [raw.closing_diff for _, raw, _ in self._run() if raw.closing_diff is not None]
        self.assertTrue(diffs, "no closing_diff was ever reported")
        for value in diffs:
            self.assertAlmostEqual(value, 5.0, delta=0.5)

    def test_filtered_closing_speed_converges_on_the_true_closing_speed(self):
        filtered = [f.closing_los for _, _, f in self._run()]
        for value in filtered[-6:]:
            self.assertAlmostEqual(value, 5.0, delta=0.5)


def _cane_payload(seq):
    return {
        "type": "cane",
        "node_id": 4125577512,
        "seq": seq,
        "gps_valid": 0,
        "lat": CANE_LAT,
        "lng": CANE_LNG,
        "speed_mps": 0.0,
        "heading_deg": 0.0,
        "node_risk": 0,
        "source_mode": "fallback",
    }


def _vehicle_payload(seq, distance_m):
    lat, lng = offset_position(CANE_LAT, CANE_LNG, 0.0, distance_m)
    return {
        "type": "vehicle",
        "node_id": 900000001,
        "seq": seq,
        "gps_valid": 1,
        "lat": lat,
        "lng": lng,
        "speed_mps": 5.0,
        "heading_deg": 180.0,
        "node_risk": 0,
        "source_mode": "simulation",
    }


class NodeTrackerTest(unittest.TestCase):
    def test_velocity_converges_on_a_clean_constant_velocity_track(self):
        """Fed noiseless 5 m/s northward samples, the filter must learn 5 m/s."""
        tracker = NodeTracker()
        for step in range(50):
            t = step * 0.1
            tracker.observe(0.0, 5.0 * t, t)

        _, _, vel_e, vel_n = tracker.state_at(4.9)
        self.assertAlmostEqual(vel_n, 5.0, places=1)
        self.assertAlmostEqual(vel_e, 0.0, places=1)

    def test_state_at_extrapolates_forward_without_consuming_a_measurement(self):
        tracker = NodeTracker()
        for step in range(50):
            t = step * 0.1
            tracker.observe(0.0, 5.0 * t, t)

        pos_now = tracker.state_at(4.9)[1]
        pos_later = tracker.state_at(5.4)[1]
        self.assertAlmostEqual(pos_later - pos_now, 2.5, places=1)

    def test_first_observation_is_taken_as_the_position_with_unknown_speed(self):
        tracker = NodeTracker()
        tracker.observe(3.0, 4.0, 0.0)
        pos_e, pos_n, vel_e, vel_n = tracker.state_at(0.0)
        self.assertAlmostEqual(pos_e, 3.0, places=6)
        self.assertAlmostEqual(pos_n, 4.0, places=6)
        self.assertAlmostEqual(vel_e, 0.0, places=6)
        self.assertAlmostEqual(vel_n, 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
