import io
import unittest
from contextlib import redirect_stderr, redirect_stdout

from state_store import StateStore, inject_vehicle, process_line
from sim_vehicle import TestVehicle


CANE = (
    '{"type":"cane","node_id":4125577512,"seq":3360,"gps_valid":1,'
    '"lat":37.0,"lng":127.0,"node_risk":0}'
)
VEHICLE = (
    '{"type":"vehicle","node_id":111,"seq":10,"gps_valid":1,'
    '"lat":37.001,"lng":127.001,"node_risk":0}'
)


def row(node_type, pc_time, gps_valid=1, lat=37.0, lng=127.0):
    return {
        "type": node_type,
        "pc_time": pc_time,
        "gps_valid": gps_valid,
        "lat": lat,
        "lng": lng,
    }


class StateStoreTests(unittest.TestCase):
    def test_missing_until_first_record(self):
        store = StateStore()
        statuses, pair_valid, risk_valid = store.snapshot(now=100.0)
        self.assertEqual(statuses, {"cane": "MISSING", "vehicle": "MISSING"})
        self.assertFalse(pair_valid)
        self.assertFalse(risk_valid)

    def test_cane_only_is_ready_but_not_paired(self):
        store = StateStore()
        store.update(row("cane", 100.0))
        statuses, pair_valid, risk_valid = store.snapshot(now=100.1)
        self.assertEqual(statuses, {"cane": "READY", "vehicle": "MISSING"})
        self.assertFalse(pair_valid)
        self.assertFalse(risk_valid)

    def test_both_fresh_pairs_and_allows_risk(self):
        store = StateStore()
        store.update(row("cane", 100.0))
        store.update(row("vehicle", 100.2))
        statuses, pair_valid, risk_valid = store.snapshot(now=100.4)
        self.assertEqual(statuses, {"cane": "READY", "vehicle": "READY"})
        self.assertTrue(pair_valid)
        self.assertTrue(risk_valid)

    def test_record_older_than_window_is_stale(self):
        store = StateStore()
        store.update(row("cane", 100.0))
        store.update(row("vehicle", 100.9))
        statuses, pair_valid, _ = store.snapshot(now=100.9)
        self.assertEqual(statuses, {"cane": "STALE", "vehicle": "READY"})
        self.assertFalse(pair_valid)

    def test_fallback_position_still_allows_risk(self):
        # The cane reports gps_valid=0 indoors but keeps its fallback coordinates.
        store = StateStore()
        store.update(row("cane", 100.0, gps_valid=0))
        store.update(row("vehicle", 100.0, gps_valid=0))
        _, pair_valid, risk_valid = store.snapshot(now=100.1)
        self.assertTrue(pair_valid)
        self.assertTrue(risk_valid)

    def test_null_island_position_blocks_risk(self):
        store = StateStore()
        store.update(row("cane", 100.0, lat=0.0, lng=0.0))
        store.update(row("vehicle", 100.0))
        _, pair_valid, risk_valid = store.snapshot(now=100.1)
        self.assertTrue(pair_valid)
        self.assertFalse(risk_valid)

    def test_missing_position_blocks_risk(self):
        store = StateStore()
        store.update(row("cane", 100.0, lat="", lng=""))
        store.update(row("vehicle", 100.0))
        _, pair_valid, risk_valid = store.snapshot(now=100.1)
        self.assertTrue(pair_valid)
        self.assertFalse(risk_valid)

    def test_latest_record_replaces_previous(self):
        store = StateStore()
        store.update(row("cane", 100.0))
        store.update(row("cane", 100.9))
        self.assertEqual(store.status("cane", now=101.0), "READY")


class ProcessLineTests(unittest.TestCase):
    def test_cane_line_reports_missing_vehicle(self):
        store = StateStore()
        output = io.StringIO()
        with redirect_stdout(output):
            self.assertTrue(process_line(CANE, "test", store))
        text = output.getvalue()
        self.assertIn("[STATE] type=cane seq=3360 gps_valid=1 source=test", text)
        self.assertIn("cane=READY", text)
        self.assertIn("vehicle=MISSING", text)
        self.assertIn("pair_valid=False", text)
        self.assertIn("risk_valid=False", text)

    def test_vehicle_line_completes_the_pair(self):
        store = StateStore()
        output = io.StringIO()
        with redirect_stdout(output):
            process_line(CANE, "test", store)
            process_line(VEHICLE, "test", store)
        text = output.getvalue()
        self.assertIn("pair_valid=True", text)
        self.assertIn("risk_valid=True", text)

    def test_unknown_type_is_ignored(self):
        store = StateStore()
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            self.assertFalse(process_line('{"type":"rsu","seq":1}', "test", store))
        self.assertEqual(store.latest, {})

    def test_bad_json_is_ignored(self):
        store = StateStore()
        with redirect_stderr(io.StringIO()):
            self.assertFalse(process_line("not-json", "test", store))
        self.assertEqual(store.latest, {})


class InjectVehicleTests(unittest.TestCase):
    def feed_cane(self, store):
        with redirect_stdout(io.StringIO()):
            process_line(CANE, "test", store)

    def test_no_injection_before_the_cane_is_known(self):
        store = StateStore()
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            self.assertFalse(
                inject_vehicle(TestVehicle(), store, "test", now=100.0)
            )
        self.assertEqual(store.latest, {})

    def test_injection_completes_the_pair(self):
        store = StateStore()
        self.feed_cane(store)
        output = io.StringIO()
        with redirect_stdout(output):
            self.assertTrue(inject_vehicle(TestVehicle(), store, "test", now=100.0))
        text = output.getvalue()
        self.assertIn("[TESTVEH] seq=1 distance_m=50.0", text)
        self.assertIn("source=simulation", text)
        self.assertIn("pair_valid=True", text)
        self.assertIn("risk_valid=True", text)

    def test_injection_respects_the_tick_period(self):
        store = StateStore()
        self.feed_cane(store)
        vehicle = TestVehicle(period_s=0.2)
        with redirect_stdout(io.StringIO()):
            self.assertTrue(inject_vehicle(vehicle, store, "test", now=100.0))
            self.assertFalse(inject_vehicle(vehicle, store, "test", now=100.1))
            self.assertTrue(inject_vehicle(vehicle, store, "test", now=100.2))

    def test_injected_vehicle_closes_in_on_the_cane(self):
        store = StateStore()
        self.feed_cane(store)
        vehicle = TestVehicle(start_distance_m=50.0, speed_mps=5.0, period_s=0.0)
        with redirect_stdout(io.StringIO()):
            inject_vehicle(vehicle, store, "test", now=100.0)
            first = store.latest["vehicle"]["lat"]
            inject_vehicle(vehicle, store, "test", now=101.0)
            second = store.latest["vehicle"]["lat"]
        cane_lat = store.latest["cane"]["lat"]
        self.assertLess(abs(second - cane_lat), abs(first - cane_lat))


if __name__ == "__main__":
    unittest.main()
