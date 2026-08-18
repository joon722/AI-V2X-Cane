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
    measurement_time,
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


class MeasurementTimeTest(unittest.TestCase):
    def test_valid_gps_time_is_preferred_over_arrival_time(self):
        row = {"pc_time": 1000.0, "gps_time_ms": 45_296_500}
        seconds, is_gps = measurement_time(row)
        self.assertTrue(is_gps)
        self.assertAlmostEqual(seconds, 45_296.5)

    def test_missing_empty_and_sentinel_values_fall_back_to_arrival_time(self):
        for raw in ("missing", "", 4294967295, -1, 86_400_000):
            row = {"pc_time": 1000.0}
            if raw != "missing":
                row["gps_time_ms"] = raw
            seconds, is_gps = measurement_time(row)
            self.assertFalse(is_gps, f"gps_time_ms={raw!r}")
            self.assertAlmostEqual(seconds, 1000.0)


class FixDedupAndResetTest(unittest.TestCase):
    def test_the_same_fix_repeated_by_the_send_loop_is_only_counted_once(self):
        """10 Hz packets carry a 1 Hz fix; repeats must not tighten the filter."""
        fed_once = NodeTracker()
        fed_once.observe(0.0, 0.0, 100.0, True)
        fed_once.observe(0.0, 5.0, 101.0, True)

        fed_repeats = NodeTracker()
        for _ in range(10):
            fed_repeats.observe(0.0, 0.0, 100.0, True)
        for _ in range(10):
            fed_repeats.observe(0.0, 5.0, 101.0, True)

        self.assertEqual(fed_once.north.cov, fed_repeats.north.cov)
        self.assertAlmostEqual(fed_once.north.pos, fed_repeats.north.pos, places=9)

    def test_clock_base_change_restarts_the_track(self):
        tracker = NodeTracker()
        tracker.observe(0.0, 0.0, 1000.0, False)  # arrival clock
        tracker.observe(0.0, 5.0, 50.0, True)     # gps clock
        # A restart treats the second sample as a first observation:
        # position taken as-is, speed unknown (zero).
        pos_n = tracker.state_ahead(0.0)[1]
        vel_n = tracker.state_ahead(0.0)[3]
        self.assertAlmostEqual(pos_n, 5.0, places=6)
        self.assertAlmostEqual(vel_n, 0.0, places=6)

    def test_a_long_gap_restarts_the_track_instead_of_extrapolating(self):
        tracker = NodeTracker()
        tracker.observe(0.0, 0.0, 100.0, True)
        tracker.observe(0.0, 5.0, 101.0, True)    # learned ~5 m/s northward
        tracker.observe(0.0, 5.0, 200.0, True)    # 99 s of silence
        vel_n = tracker.state_ahead(0.0)[3]
        self.assertAlmostEqual(vel_n, 0.0, places=6)

    def test_midnight_wrap_keeps_time_monotonic(self):
        tracker = NodeTracker()
        tracker.observe(0.0, 0.0, 86_399.0, True)  # 23:59:59
        tracker.observe(0.0, 5.0, 0.0, True)       # 00:00:00 next day
        self.assertAlmostEqual(tracker.last_time, 86_400.0, places=6)

    def test_a_gap_over_two_seconds_restarts_the_track(self):
        """2026-08-17 field: a vehicle silent for 3-5 s (GPS outage, node
        stall) came back metres away; extrapolating the old velocity across
        that gap read the jump as 5-11 m/s of approach and tripped level 3.
        Ten missing 5 Hz fixes is an outage, so the track starts over."""
        tracker = NodeTracker()
        tracker.observe(0.0, 0.0, 100.0, True)
        tracker.observe(0.0, 5.0, 101.0, True)    # learned ~5 m/s northward
        tracker.observe(0.0, 5.0, 103.5, True)    # 2.5 s of silence
        vel_n = tracker.state_ahead(0.0)[3]
        self.assertAlmostEqual(vel_n, 0.0, places=6)

    def test_a_position_jump_far_beyond_the_prediction_restarts_the_track(self):
        """2026-08-17 17:48:19: the vehicle repeated one stale fix for 3 s at
        10 Hz (so the track never went quiet), then the next fix landed 13 m
        away while the node reported 0.04 m/s. Fed as motion, that read as
        10 m/s of approach and level 3. A residual that far outside what the
        filter predicted is a GPS jump, and the track starts over there."""
        tracker = NodeTracker()
        for k in range(30):
            tracker.observe(0.0, 0.0, 100.0 + 0.1 * k, False, speed_hint=0.04)
        tracker.observe(0.0, 13.0, 103.1, False, speed_hint=0.04)
        pos_n, vel_n = tracker.state_ahead(0.0)[1], tracker.state_ahead(0.0)[3]
        self.assertAlmostEqual(pos_n, 13.0, places=6)
        self.assertAlmostEqual(vel_n, 0.0, places=6)

    def test_a_fast_car_is_not_mistaken_for_a_jump(self):
        """20 m/s at 1 Hz moves 20 m per fix. The node says 20 m/s, so that
        displacement is expected and the track must keep learning it."""
        tracker = NodeTracker()
        for k in range(6):
            tracker.observe(0.0, 20.0 * k, 100.0 + 1.0 * k, False, speed_hint=20.0)
        self.assertGreater(tracker.state_ahead(0.0)[3], 15.0)

    def test_a_single_grid_hop_is_not_read_as_half_a_metre_per_second(self):
        """Packet lat/lng are float32, so a still node hops between grid cells
        0.42-0.53 m apart. Starting the filter with a 10 m/s speed prior let
        one such hop at 5 Hz read as ~0.55 m/s (a phantom above the closing
        deadband). The prior must be tight enough that a hop reads well below."""
        tracker = NodeTracker()
        tracker.observe(0.0, 0.0, 100.0, False)
        tracker.observe(0.0, 0.45, 100.2, False)
        self.assertLess(abs(tracker.state_ahead(0.0)[3]), 0.2)


class FrozenFixTest(unittest.TestCase):
    """8/18 실측: 차량 노드가 움직이면서 같은 좌표를 1~17 s 얼려 보냄(≥1 s 65회·212 s/38분,
    1.13 m/s로 달리며 6.9 s = 7.8 m 이동이 안 보임). 패킷에 fix 시각이 없어 그 반복 좌표가
    매 패킷 새 측정으로 KF에 들어갔고, 도플러 융합이 끊기는 <0.25 m/s 순간 KF 속도가
    뒤집혀 1.3~1.4 m/s 유령(16:31:44·16:37:47, 도플러 기반 raw closing은 ≈0)을 만들었다.

    같은 좌표의 반복은 새 위치 정보가 아니다 — 위치 갱신은 건너뛰되 트랙은 살아 있고
    (무음이 아니라 도플러는 계속 온다), 그 사이 도플러로 속도만 갱신해 추측항법한다.
    """

    def test_an_identical_repeat_does_not_advance_the_position_filter(self):
        tr = NodeTracker()
        tr.observe(0.0, 0.0, 100.0, False)
        tr.observe(0.0, 1.0, 100.2, False)
        cov_before = tr.north.cov
        self.assertFalse(tr.observe(0.0, 1.0, 100.4, False))   # 같은 좌표 반복
        self.assertEqual(tr.last_time, 100.2)                   # 마지막 '새' fix
        self.assertEqual(tr.last_seen, 100.4)                   # 마지막 패킷
        self.assertEqual(tr.north.cov, cov_before)

    def test_repeats_keep_the_track_alive_past_the_gap_limit(self):
        # 3 s 동안 좌표가 얼어 있어도 패킷은 계속 왔다: 무음이 아니므로 리셋하지 않는다.
        tr = NodeTracker()
        tr.observe(0.0, 0.0, 100.0, False)
        tr.observe(0.0, 1.0, 100.2, False)
        for k in range(1, 16):
            tr.observe(0.0, 1.0, 100.2 + 0.2 * k, False)
        tr.observe(0.0, 16.0, 103.4, False, speed_hint=5.0)
        self.assertEqual(tr.track_start, 100.0)

    def test_settled_counts_the_track_age_in_packets_not_distinct_fixes(self):
        # 서 있는 노드는 같은 좌표를 몇 초씩 보낸다 — 그래도 트랙은 익는다.
        tr = NodeTracker()
        tr.observe(0.0, 0.0, 100.0, False)
        for k in range(1, 8):
            tr.observe(0.0, 0.0, 100.0 + 0.2 * k, False)
        self.assertTrue(tr.settled())


class VehicleDopplerDuringFreezeTest(unittest.TestCase):
    """좌표가 얼어 있는 동안 도플러가 하는 일: 살아 있으면 추측항법, 0이면 정지(ZUPT)."""

    def _row(self, pipeline, type_, seq, now, north_m, speed, heading, heading_valid):
        lat, lng = offset_position(CANE_LAT, CANE_LNG, 0.0, north_m)
        payload = {
            "type": type_, "node_id": 1 if type_ == "cane" else 2, "seq": seq,
            "gps_valid": 1,
            "lat": lat if type_ == "vehicle" else CANE_LAT,
            "lng": lng if type_ == "vehicle" else CANE_LNG,
            "speed_mps": speed, "heading_deg": heading, "heading_valid": heading_valid,
        }
        row = normalize_record(payload, "test", now=now)
        pipeline.store.update(row)
        pipeline.observe(row)
        return pipeline.compute()

    def _run(self, freeze_speed, freeze_heading_valid=1, freeze_s=3.0):
        """차량 20 m 북쪽에서 남향 1 m/s(heading_valid) 3 s → 좌표 동결 freeze_s 동안
        도플러 freeze_speed → 마지막 판정 반환 (t, filtered, veh_state)."""
        pipeline = KinematicsPipeline(StateStore(fresh_window_s=10.0))
        seq = 0
        out = []
        for step in range(15):                       # 3 s 정상 접근
            t = step * 0.2
            self._row(pipeline, "cane", seq, t, 0.0, 0.02, 0.0, 0)
            north = 20.0 - 1.0 * t
            r = self._row(pipeline, "vehicle", seq, t + 0.01, north, 1.0, 180.0, 1)
            out.append((t, r)); seq += 1
        frozen_north = 20.0 - 1.0 * (14 * 0.2)     # 마지막 좌표 17.2 m
        for step in range(15, 15 + int(freeze_s / 0.2)):
            t = step * 0.2
            self._row(pipeline, "cane", seq, t, 0.0, 0.02, 0.0, 0)
            r = self._row(pipeline, "vehicle", seq, t + 0.01, frozen_north, freeze_speed, 180.0,
                          freeze_heading_valid)
            out.append((t, r)); seq += 1
        return out, pipeline, frozen_north

    def test_live_doppler_dead_reckons_through_a_frozen_position(self):
        out, pipeline, frozen_north = self._run(freeze_speed=1.0)
        t, (_, _, filtered) = out[-1]
        veh_north = pipeline.last_states[1][1]
        # 3 s 동결 동안 1 m/s 남하 → 마지막 좌표(17.2)보다 ≈3 m 남쪽에 있어야 한다.
        # (동결 좌표를 계속 먹이던 이전 코드는 2 m 뒤에서 멈춘 채 평형을 이뤘다.)
        self.assertAlmostEqual(veh_north, frozen_north - 3.0, delta=0.6)
        self.assertAlmostEqual(filtered.closing_los, 1.0, delta=0.4)

    def test_snap_back_after_a_frozen_drift_makes_no_phantom_approach(self):
        # 현장 16:37:44~50 재현: 멀어지던 차(0.9 m/s)의 좌표가 얼고, 도플러는 0.9→0.4(헤딩 유효)
        # →0.2→0.05(무효)로 감속. 이전 코드: 융합 중 KF 위치가 얼린 좌표에서 1~2 m 흘러가고,
        # 융합이 끊기자 그 좌표로 되튀며 closing +0.6~1.4 유령. 이제: 얼린 좌표는 위치를 못
        # 당기고, 도플러 <0.25면 ZUPT → 유령 없음.
        pipeline = KinematicsPipeline(StateStore(fresh_window_s=10.0))
        seq = 0
        for step in range(15):                       # 8 m 북쪽에서 북향(멀어짐) 0.9 m/s 3 s
            t = step * 0.2
            self._row(pipeline, "cane", seq, t, 0.0, 0.02, 0.0, 0)
            self._row(pipeline, "vehicle", seq, t + 0.01, 8.0 + 0.9 * t, 0.9, 0.0, 1)
            seq += 1
        frozen_north = 8.0 + 0.9 * (14 * 0.2)
        profile = [(0.9, 1)] * 5 + [(0.7, 1)] * 4 + [(0.4, 1)] * 4 + [(0.2, 0)] * 3 + [(0.1, 0)] * 3 + [(0.05, 0)] * 10
        worst = -9.0
        for i, (speed, hv) in enumerate(profile):
            t = (15 + i) * 0.2
            self._row(pipeline, "cane", seq, t, 0.0, 0.02, 0.0, 0)
            r = self._row(pipeline, "vehicle", seq, t + 0.01, frozen_north, speed, 0.0, hv)
            seq += 1
            if r is not None and hv == 0:
                worst = max(worst, r[2].closing_los)
        self.assertLess(worst, 0.25)

    def test_a_still_vehicle_hopping_between_grid_cells_reports_no_speed(self):
        # 정지 차량: 좌표가 0.42 m 격자 사이를 5 Hz로 오가고 도플러 0.05 → 속도 <0.2 (ZUPT).
        pipeline = KinematicsPipeline(StateStore(fresh_window_s=10.0))
        speeds = []
        for step in range(30):
            t = step * 0.2
            self._row(pipeline, "cane", step, t, 0.0, 0.02, 0.0, 0)
            north = 10.0 + (0.42 if step % 2 else 0.0)
            r = self._row(pipeline, "vehicle", step, t + 0.01, north, 0.05, 0.0, 0)
            if r is not None and t > 2.0:
                ve, vn = pipeline.last_states[1][2], pipeline.last_states[1][3]
                speeds.append(math.hypot(ve, vn))
        self.assertTrue(speeds)
        self.assertLess(max(speeds), 0.2)


class TrackSettleGateTest(unittest.TestCase):
    """After a track (re)starts, the first second of velocity is a guess made
    from one or two noisy fixes. Reporting a closing speed from it fabricated
    approaches (2026-08-17 18:09:24 / 18:27:24: the vehicle came back after a
    10.8 s stall and level 3 fired on nothing). Until the track has settled the
    pair reports "no closing speed"; distance and relative position stay live."""

    def _observe(self, pipeline, type_, seq, now, north_m, speed=0.0):
        # speed = the node's own Doppler. A moving vehicle reports it (heading
        # stays invalid here so velocity is still learned from positions); a
        # vehicle reporting 0.0 is treated as standing still (ZUPT, 8/18).
        lat, lng = offset_position(CANE_LAT, CANE_LNG, 0.0, north_m)
        payload = {
            "type": type_,
            "node_id": 1 if type_ == "cane" else 2,
            "seq": seq,
            "gps_valid": 1,
            "lat": lat if type_ == "vehicle" else CANE_LAT,
            "lng": lng if type_ == "vehicle" else CANE_LNG,
            "speed_mps": speed,
            "heading_deg": 0.0,
            "heading_valid": 0,
        }
        row = normalize_record(payload, "test", now=now)
        pipeline.store.update(row)
        pipeline.observe(row)
        return pipeline.compute()

    def _run(self):
        pipeline = KinematicsPipeline(StateStore())
        results = []
        seq = 0
        # 3 s of both nodes reporting at 5 Hz: tracks settle.
        for step in range(15):
            t = step * 0.2
            self._observe(pipeline, "cane", seq, t, 0.0)
            results.append((t, self._observe(pipeline, "vehicle", seq, t + 0.01, 15.0)))
            seq += 1
        # 3 s with only the cane: vehicle GPS outage.
        for step in range(15, 30):
            t = step * 0.2
            results.append((t, self._observe(pipeline, "cane", seq, t, 0.0)))
            seq += 1
        # Vehicle re-acquired 5 m closer and now approaching at 1 m/s, 5 Hz, 2 s.
        for step in range(30, 40):
            t = step * 0.2
            self._observe(pipeline, "cane", seq, t, 0.0)
            north = 10.0 - 1.0 * (t - 6.0)
            results.append((t, self._observe(pipeline, "vehicle", seq, t + 0.01, north, speed=1.0)))
            seq += 1
        return results

    def test_first_second_after_restart_reports_no_closing_speed(self):
        restart_t = 30 * 0.2
        early = [f for t, r in self._run() if r is not None
                 and restart_t <= t < restart_t + 1.0 for _, _, f in [r]]
        self.assertTrue(early, "no results right after the restart")
        for filtered in early:
            self.assertEqual(filtered.closing_los, 0.0)
            self.assertIsNone(filtered.ttc_simple)
            self.assertIsNone(filtered.tcpa)
            self.assertIsNone(filtered.dcpa)
            # The pair is still tracked: distance and bearing stay live.
            self.assertAlmostEqual(filtered.distance_m, 10.0, delta=1.5)
            self.assertAlmostEqual(filtered.rel_north, 10.0, delta=1.5)

    def test_settled_track_reports_the_approach_again(self):
        restart_t = 30 * 0.2
        late = [f for t, r in self._run() if r is not None
                and t >= restart_t + 1.2 for _, _, f in [r]]
        self.assertTrue(late, "no results after the settle window")
        for filtered in late:
            self.assertGreater(filtered.closing_los, 0.5)
            self.assertIsNotNone(filtered.ttc_simple)


class GpsTimeAlignmentTest(unittest.TestCase):
    """Packets repeat each 1 Hz fix ten times; gps_time must deduplicate them."""

    def test_filtered_closing_speed_converges_despite_repeated_fixes(self):
        store = StateStore(fresh_window_s=10.0)
        pipeline = KinematicsPipeline(store)
        results = []
        for tick in range(200):
            now = 1000.0 + tick * 0.1
            fix_second = tick // 10
            gps_ms = 43_200_000 + fix_second * 1000

            cane = _cane_payload(tick)
            cane["gps_time_ms"] = gps_ms
            vehicle = _vehicle_payload(tick, 150.0 - 5.0 * fix_second)
            vehicle["gps_time_ms"] = gps_ms

            for payload, stamp in ((cane, now), (vehicle, now + 0.003)):
                row = normalize_record(payload, "test", now=stamp)
                store.update(row)
                pipeline.observe(row)
                result = pipeline.compute()
                if result is not None:
                    results.append(result)

        filtered = [f.closing_los for _, _, f in results]
        self.assertTrue(filtered, "pipeline never produced a result")
        for value in filtered[-10:]:
            self.assertAlmostEqual(value, 5.0, delta=0.5)


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


class DopplerVelocityObservationTest(unittest.TestCase):
    """Feeding the receiver's own Doppler velocity into the filter."""

    def test_velocity_observation_pulls_the_filter_velocity(self):
        tracker = NodeTracker()
        tracker.observe(0.0, 0.0, 0.0)
        tracker.observe_velocity(3.0, 0.0, 0.5)
        _, _, vel_e, vel_n = tracker.state_ahead(0.0)
        self.assertGreater(vel_e, 2.0)
        self.assertAlmostEqual(vel_n, 0.0, places=6)

    def test_zero_velocity_observation_suppresses_phantom_speed(self):
        """A stationary node's position noise must not fabricate velocity.

        2026-08-12 field data: a cane lying still was rendered at up to
        6.85 m/s by position noise alone, which fabricated approach speed and
        therefore TTC. The zero-velocity observation is the fix.
        """
        import random

        rng = random.Random(42)
        with_zupt = NodeTracker()
        without_zupt = NodeTracker()
        max_with = 0.0
        max_without = 0.0
        for step in range(50):
            t = step * 0.2
            east = rng.gauss(0.0, 2.0)
            north = rng.gauss(0.0, 2.0)
            with_zupt.observe(east, north, t)
            with_zupt.observe_velocity(0.0, 0.0, 0.2)
            without_zupt.observe(east, north, t)
            if step > 5:
                _, _, ve, vn = with_zupt.state_ahead(0.0)
                max_with = max(max_with, math.hypot(ve, vn))
                _, _, ve, vn = without_zupt.state_ahead(0.0)
                max_without = max(max_without, math.hypot(ve, vn))
        self.assertLess(max_with, max_without * 0.5)
        self.assertLess(max_with, 0.6)

    def test_velocity_observation_converges_faster_than_positions_alone(self):
        """Two fixes plus Doppler must know the speed positions need many for."""
        doppler = NodeTracker()
        position_only = NodeTracker()
        for step in range(3):
            t = step * 0.2
            doppler.observe(0.0, -2.0 * t, t)
            doppler.observe_velocity(0.0, -2.0, 0.5)
            position_only.observe(0.0, -2.0 * t, t)
        _, _, _, vel_n_doppler = doppler.state_ahead(0.0)
        _, _, _, vel_n_position = position_only.state_ahead(0.0)
        self.assertLess(abs(vel_n_doppler - (-2.0)), abs(vel_n_position - (-2.0)))
        self.assertLess(abs(vel_n_doppler - (-2.0)), 0.5)


class PipelineDopplerPolicyTest(unittest.TestCase):
    """Which node gets which Doppler observation, decided from the row itself."""

    def _observe(self, pipeline, type_, seq, now, lat, lng, speed, heading,
                 heading_valid):
        payload = {
            "type": type_,
            "node_id": 1 if type_ == "cane" else 2,
            "seq": seq,
            "gps_valid": 1,
            "lat": lat,
            "lng": lng,
            "speed_mps": speed,
            "heading_deg": heading,
            "heading_valid": heading_valid,
        }
        row = normalize_record(payload, "test", now=now)
        pipeline.store.update(row)
        pipeline.observe(row)

    def _pipeline(self):
        return KinematicsPipeline(StateStore())

    def test_valid_vehicle_heading_feeds_the_velocity_vector(self):
        pipeline = self._pipeline()
        self._observe(pipeline, "cane", 0, 0.0, CANE_LAT, CANE_LNG, 0.0, 0.0, 0)
        for step in range(3):
            t = step * 0.2
            lat, lng = offset_position(CANE_LAT, CANE_LNG, 0.0, 30.0 - 2.0 * t)
            self._observe(pipeline, "vehicle", step, t, lat, lng, 2.0, 180.0, 1)
        _, _, _, vel_n = pipeline.trackers["vehicle"].state_ahead(0.0)
        self.assertLess(abs(vel_n - (-2.0)), 0.5)

    def test_slow_but_valid_vehicle_heading_is_still_fused(self):
        """RC 차량은 대부분 0.3~1 m/s 로 움직인다(8/17: 이동 중 중앙 0.53). 펌웨어 heading
        문턱을 0.4→0.25 로 내리면(런타임 headspeed) 그 속도에서도 heading_valid=1 이 오는데,
        젯슨 쪽 문턱이 0.4 로 남아 있으면 그 벡터를 버린다. 노드가 보증한 값은 쓴다."""
        pipeline = self._pipeline()
        self._observe(pipeline, "cane", 0, 0.0, CANE_LAT, CANE_LNG, 0.0, 0.0, 0)
        for step in range(3):
            t = step * 0.2
            lat, lng = offset_position(CANE_LAT, CANE_LNG, 0.0, 30.0 - 0.3 * t)
            self._observe(pipeline, "vehicle", step, t, lat, lng, 0.3, 180.0, 1)
        _, _, _, vel_n = pipeline.trackers["vehicle"].state_ahead(0.0)
        self.assertLess(abs(vel_n - (-0.3)), 0.15)

    def test_invalid_vehicle_heading_is_never_trusted(self):
        """heading_valid=0 means the heading field may be stale garbage."""
        pipeline = self._pipeline()
        self._observe(pipeline, "cane", 0, 0.0, CANE_LAT, CANE_LNG, 0.0, 0.0, 0)
        for step in range(3):
            t = step * 0.2
            lat, lng = offset_position(CANE_LAT, CANE_LNG, 0.0, 30.0 - 2.0 * t)
            # Stale heading says east while the vehicle actually moves south.
            self._observe(pipeline, "vehicle", step, t, lat, lng, 2.0, 90.0, 0)
        _, _, vel_e, _ = pipeline.trackers["vehicle"].state_ahead(0.0)
        # Had the stale heading been folded in, east velocity would be near 2.
        self.assertLess(abs(vel_e), 1.0)

    def test_still_cane_gets_the_zero_velocity_observation(self):
        pipeline = self._pipeline()
        import random

        rng = random.Random(7)
        for step in range(30):
            t = step * 0.2
            lat = CANE_LAT + rng.gauss(0.0, 2.0) / 111320.0
            lng = CANE_LNG + rng.gauss(0.0, 2.0) / 111320.0
            self._observe(pipeline, "cane", step, t, lat, lng, 0.2, 0.0, 0)
        _, _, vel_e, vel_n = pipeline.trackers["cane"].state_ahead(0.0)
        self.assertLess(math.hypot(vel_e, vel_n), 0.6)


class VehicleBiasTest(unittest.TestCase):
    """Two receivers on the same spot read metres apart (2026-08-17: 6.98 m E,
    -1.35 m N). The pipeline must subtract that systematic offset from the
    vehicle before any relative geometry, in raw and filtered paths alike."""

    BIAS_E = 6.98
    BIAS_N = -1.35

    def _run(self, vehicle_bias):
        pipeline = KinematicsPipeline(StateStore(), vehicle_bias=vehicle_bias)
        # Both antennas physically at the cane; the vehicle receiver reports
        # coordinates displaced by the bias.
        bearing = math.degrees(math.atan2(self.BIAS_E, self.BIAS_N)) % 360.0
        veh_lat, veh_lng = offset_position(
            CANE_LAT, CANE_LNG, bearing, math.hypot(self.BIAS_E, self.BIAS_N)
        )
        result = None
        for step in range(5):
            t = step * 0.2
            for type_, lat, lng in (("cane", CANE_LAT, CANE_LNG),
                                    ("vehicle", veh_lat, veh_lng)):
                payload = {
                    "type": type_,
                    "node_id": 1 if type_ == "cane" else 2,
                    "seq": step,
                    "gps_valid": 1,
                    "lat": lat,
                    "lng": lng,
                    "speed_mps": 0.0,
                    "heading_deg": 0.0,
                    "heading_valid": 0,
                }
                row = normalize_record(payload, "test", now=t)
                pipeline.store.update(row)
                pipeline.observe(row)
            result = pipeline.compute()
        self.assertIsNotNone(result)
        return result

    def test_without_bias_the_pair_reads_the_raw_offset(self):
        _, raw, _ = self._run((0.0, 0.0))
        self.assertAlmostEqual(
            raw.distance_m, math.hypot(self.BIAS_E, self.BIAS_N), delta=0.05
        )

    def test_with_bias_the_pair_reads_co_located(self):
        _, raw, filtered = self._run((self.BIAS_E, self.BIAS_N))
        self.assertLess(raw.distance_m, 0.05)
        self.assertLess(filtered.distance_m, 0.1)
        self.assertLess(abs(filtered.rel_east), 0.1)
        self.assertLess(abs(filtered.rel_north), 0.1)


if __name__ == "__main__":
    unittest.main()
