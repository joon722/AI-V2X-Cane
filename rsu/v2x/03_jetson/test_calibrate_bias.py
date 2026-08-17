#!/usr/bin/env python3
"""Tests for calibrate_bias: mean(vehicle − cane) offset from the raw log tail."""

import json
import math
import os
import tempfile
import unittest
from pathlib import Path

from calibrate_bias import (
    CalibrationError,
    estimate_bias,
    newest_raw_log,
    read_fixes,
)
from step5_test_vehicle import offset_position
from step6_kinematics import LocalFrame


CANE_LAT = 37.0
CANE_LNG = 127.0
BIAS_E = 6.98
BIAS_N = -1.35


def _shift(lat, lng, east_m, north_m):
    bearing = math.degrees(math.atan2(east_m, north_m)) % 360.0
    return offset_position(lat, lng, bearing, math.hypot(east_m, north_m))


def _rx(t, type_, lat, lng, gps_valid=1):
    payload = {"type": type_, "node_id": 1, "seq": 0, "gps_valid": gps_valid,
               "lat": lat, "lng": lng, "speed_mps": 0.1, "heading_deg": 0.0}
    return f"{t:.3f} RX {json.dumps(payload)}"


def _side_by_side_log(seconds=10.0, hz=5.0, t0=1000.0):
    """Both nodes still, vehicle receiver displaced by the planted bias."""
    veh_lat, veh_lng = _shift(CANE_LAT, CANE_LNG, BIAS_E, BIAS_N)
    lines = []
    for i in range(int(seconds * hz)):
        t = t0 + i / hz
        lines.append(_rx(t, "cane", CANE_LAT, CANE_LNG))
        lines.append(_rx(t + 0.01, "vehicle", veh_lat, veh_lng))
    return lines


class ReadFixesTest(unittest.TestCase):
    def test_keeps_only_valid_cane_and_vehicle_rx_fixes(self):
        lines = [
            _rx(1.0, "cane", CANE_LAT, CANE_LNG),
            _rx(1.1, "vehicle", CANE_LAT, CANE_LNG),
            "1.2 TX RISK2 0 0",
            _rx(1.3, "vehicle", CANE_LAT, CANE_LNG, gps_valid=0),
            _rx(1.4, "cane", 0.0, 0.0),
            '1.5 RX {"type":"risk_tx","ok":1}',
            "1.6 RX not json at all",
        ]
        fixes = read_fixes(lines)
        self.assertEqual([(t, k) for t, k, _, _ in fixes],
                         [(1.0, "cane"), (1.1, "vehicle")])


class EstimateBiasTest(unittest.TestCase):
    def test_recovers_the_planted_offset(self):
        result = estimate_bias(read_fixes(_side_by_side_log()))
        veh_lat, veh_lng = _shift(CANE_LAT, CANE_LNG, BIAS_E, BIAS_N)
        exp_e, exp_n = LocalFrame(CANE_LAT, CANE_LNG).to_enu(veh_lat, veh_lng)
        self.assertAlmostEqual(result["bias_east_m"], exp_e, delta=0.02)
        self.assertAlmostEqual(result["bias_north_m"], exp_n, delta=0.02)
        # 8 s window at 5 Hz per node; the exact edge sample may or may not land.
        self.assertGreaterEqual(result["n_cane"], 39)
        self.assertGreaterEqual(result["n_vehicle"], 39)
        self.assertLess(result["spread_cane_m"], 0.01)

    def test_samples_before_the_window_are_ignored(self):
        """A vehicle fix from 20 s ago and 100 m away must not pull the mean."""
        far_lat, far_lng = _shift(CANE_LAT, CANE_LNG, 100.0, 0.0)
        lines = [_rx(980.0, "vehicle", far_lat, far_lng)] + _side_by_side_log()
        result = estimate_bias(read_fixes(lines))
        self.assertAlmostEqual(result["bias_east_m"], BIAS_E, delta=0.05)

    def test_too_few_samples_is_rejected(self):
        lines = _side_by_side_log(seconds=1.0)  # 5 per node
        with self.assertRaises(CalibrationError):
            estimate_bias(read_fixes(lines))

    def test_a_moving_node_is_rejected(self):
        """The vehicle drifting 16 m across the window is not a calibration."""
        lines = []
        for i in range(40):
            t = 1000.0 + i / 5.0
            lat, lng = _shift(CANE_LAT, CANE_LNG, 0.4 * i, 0.0)
            lines.append(_rx(t, "cane", CANE_LAT, CANE_LNG))
            lines.append(_rx(t + 0.01, "vehicle", lat, lng))
        with self.assertRaises(CalibrationError):
            estimate_bias(read_fixes(lines))

    def test_empty_input_is_rejected(self):
        with self.assertRaises(CalibrationError):
            estimate_bias([])


class NewestRawLogTest(unittest.TestCase):
    def test_picks_the_most_recently_written_not_the_highest_name(self):
        """The engine's session file is named from a clock that may jump at
        boot; what matters is which file is being written now."""
        with tempfile.TemporaryDirectory() as tmp:
            old = Path(tmp) / "raw_20260817_100000.log"
            new_name = Path(tmp) / "raw_20260817_120000.log"
            old.write_text("x\n")
            new_name.write_text("")
            os.utime(new_name, (1000.0, 1000.0))
            os.utime(old, (2000.0, 2000.0))
            self.assertEqual(newest_raw_log(tmp), old)

    def test_no_log_is_an_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(CalibrationError):
                newest_raw_log(tmp)


if __name__ == "__main__":
    unittest.main()
