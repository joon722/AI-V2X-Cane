#!/usr/bin/env python3
"""Step 9 integration: a real vehicle stream through the step 7/8 stack.

Step 9 is not new production code; it is running the pipeline with the vehicle
arriving on the input stream (source_mode=real) instead of the --test-vehicle
generator. This locks in two things the head-on test vehicle could never show,
because it always ran dead-centre (dcpa=0):

  head-on real vehicle   -> full risk, a real alarm is transmitted
  glancing real vehicle  -> DCPA gate suppresses it, nothing is transmitted

Timing is driven with controlled pc_time, the same style as
test_kinematics.AsynchronousArrivalTest, so the run is deterministic.
"""

import unittest

from parse_v2x import normalize_record
from state_store import StateStore
from sim_vehicle import offset_position
from kinematics import KinematicsPipeline, to_float
from risk_scoring import DCPA_FLOOR, assess_risk
from send_risk import RiskTransmitter


CANE_LAT, CANE_LNG = 37.0, 127.0


def _cane_payload(seq):
    # Indoor fallback: no fix, fixed coordinate. gps_valid=0 like the real cane.
    return {
        "type": "cane", "node_id": 4125577512, "seq": seq, "gps_valid": 0,
        "lat": CANE_LAT, "lng": CANE_LNG, "speed_mps": 0.0, "heading_deg": 0.0,
        "node_risk": 0, "source_mode": "fallback",
    }


def _real_vehicle_payload(seq, north_m, east_m):
    """A vehicle north_m ahead and east_m to the side, driving due south.

    With east_m held constant the closest approach distance (dcpa) is ~east_m.
    source_mode=real marks it as a live vehicle rather than the simulator.
    """
    lat, lng = offset_position(CANE_LAT, CANE_LNG, 0.0, north_m)
    lat, lng = offset_position(lat, lng, 90.0, east_m)
    return {
        "type": "vehicle", "node_id": 900000002, "seq": seq, "gps_valid": 1,
        "lat": lat, "lng": lng, "speed_mps": 5.0, "heading_deg": 180.0,
        "node_risk": 0, "source_mode": "real",
    }


class _Sample:
    __slots__ = ("distance", "level", "gate", "dcpa", "effective", "sent")

    def __init__(self, distance, level, gate, dcpa, effective, sent):
        self.distance = distance
        self.level = level
        self.gate = gate
        self.dcpa = dcpa
        self.effective = effective
        self.sent = sent


def _run_approach(east_offset_m, seconds=8.0):
    """Feed a cane+vehicle approach and return one _Sample per vehicle update."""
    store = StateStore(fresh_window_s=10.0)
    pipeline = KinematicsPipeline(store)
    # cane is indoors (gps_valid=0), so open the trust gate to observe transmit
    # decisions; outdoors a real fix would open it on its own.
    transmitter = RiskTransmitter(heartbeat_s=1.0, allow_untrusted=True)

    samples = []
    for tick in range(int(seconds / 0.1)):
        now = tick * 0.1
        rows = [(_cane_payload(tick), now)]
        if tick % 2 == 0:
            north = max(0.0, 50.0 - 5.0 * now)
            rows.append((_real_vehicle_payload(tick, north, east_offset_m), now + 0.003))
        for payload, stamp in rows:
            row = normalize_record(payload, "real", now=stamp)
            store.update(row)
            pipeline.observe(row)
            result = pipeline.compute()
            if result is None:
                continue
            now_t, _raw, filtered = result
            vehicle_speed = to_float(store.latest["vehicle"]["speed_mps"])
            assessment = assess_risk(filtered, vehicle_speed)
            decision = transmitter.consider(
                assessment.risk_level, store.latest["cane"]["gps_valid"], now_t
            )
            if row["type"] == "vehicle":
                samples.append(
                    _Sample(
                        assessment.distance_m,
                        assessment.risk_level,
                        assessment.gate,
                        assessment.dcpa,
                        decision.effective_level,
                        decision.should_send,
                    )
                )
    return store, samples


class RealVehicleModeTest(unittest.TestCase):
    def test_stream_vehicle_is_labelled_real_not_simulation(self):
        store, _ = _run_approach(0.0)
        self.assertEqual(store.latest["vehicle"]["source_mode"], "real")


class HeadOnRealVehicleTest(unittest.TestCase):
    """dcpa~0: the gate is a no-op and a genuine alarm goes out."""

    def setUp(self):
        _, self.samples = _run_approach(0.0)

    def test_gate_never_suppresses_a_head_on_course(self):
        for s in self.samples:
            self.assertEqual(s.gate, 1.0)

    def test_risk_rises_to_a_warning_as_it_closes(self):
        self.assertGreaterEqual(max(s.level for s in self.samples), 2)

    def test_a_nonzero_level_is_actually_transmitted(self):
        self.assertGreaterEqual(max(s.effective for s in self.samples), 2)


class GlancingRealVehicleTest(unittest.TestCase):
    """dcpa~8 m: the car will miss, so the gate must keep it silent."""

    def setUp(self):
        _, self.samples = _run_approach(8.0)

    def test_dcpa_is_recovered_as_the_miss_distance(self):
        settled = [s for s in self.samples if s.dcpa is not None]
        self.assertTrue(settled)
        for s in settled:
            self.assertAlmostEqual(s.dcpa, 8.0, delta=0.5)

    def test_far_miss_is_floored_by_the_gate(self):
        for s in self.samples:
            if s.dcpa is not None and s.dcpa >= 7.5:
                self.assertEqual(s.gate, DCPA_FLOOR)

    def test_risk_level_never_leaves_zero(self):
        self.assertEqual(max(s.level for s in self.samples), 0)

    def test_nothing_nonzero_is_ever_transmitted(self):
        self.assertEqual(max(s.effective for s in self.samples), 0)


class SuppressionContrastTest(unittest.TestCase):
    def test_a_glancing_pass_scores_below_the_same_speed_head_on(self):
        _, head_on = _run_approach(0.0)
        _, glancing = _run_approach(8.0)
        self.assertGreater(
            max(s.level for s in head_on), max(s.level for s in glancing)
        )


if __name__ == "__main__":
    unittest.main()
