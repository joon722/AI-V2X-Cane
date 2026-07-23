import unittest

from sim_vehicle import (
    METERS_PER_DEGREE_LAT,
    TestVehicle,
    offset_position,
)


class OffsetPositionTests(unittest.TestCase):
    def test_north_bearing_moves_latitude_only(self):
        lat, lng = offset_position(37.0, 127.0, 0.0, 111.32)
        self.assertAlmostEqual(lat, 37.001, places=6)
        self.assertAlmostEqual(lng, 127.0, places=9)

    def test_east_bearing_moves_longitude_only(self):
        lat, lng = offset_position(37.0, 127.0, 90.0, 100.0)
        self.assertAlmostEqual(lat, 37.0, places=9)
        self.assertGreater(lng, 127.0)

    def test_zero_distance_returns_target(self):
        lat, lng = offset_position(37.0, 127.0, 45.0, 0.0)
        self.assertAlmostEqual(lat, 37.0, places=9)
        self.assertAlmostEqual(lng, 127.0, places=9)

    def test_longitude_degree_shrinks_with_latitude(self):
        # 100 m east spans more degrees of longitude further from the equator.
        _, near_equator = offset_position(0.0, 127.0, 90.0, 100.0)
        _, far_north = offset_position(60.0, 127.0, 90.0, 100.0)
        self.assertGreater(far_north - 127.0, near_equator - 127.0)


class DistanceTests(unittest.TestCase):
    def test_distance_shrinks_with_elapsed_time(self):
        vehicle = TestVehicle(start_distance_m=50.0, speed_mps=5.0)
        self.assertEqual(vehicle.distance_at(0.0), 50.0)
        self.assertEqual(vehicle.distance_at(1.0), 45.0)
        self.assertEqual(vehicle.distance_at(5.0), 25.0)

    def test_distance_stops_at_the_target(self):
        vehicle = TestVehicle(start_distance_m=50.0, speed_mps=5.0)
        self.assertEqual(vehicle.distance_at(10.0), 0.0)
        self.assertEqual(vehicle.distance_at(60.0), 0.0)


class DueTests(unittest.TestCase):
    def test_first_tick_is_due(self):
        self.assertTrue(TestVehicle().is_due(100.0))

    def test_not_due_before_the_period_elapses(self):
        vehicle = TestVehicle(period_s=0.2)
        vehicle.record(37.0, 127.0, now=100.0)
        self.assertFalse(vehicle.is_due(100.1))
        self.assertTrue(vehicle.is_due(100.2))


class RecordTests(unittest.TestCase):
    def test_payload_shape(self):
        vehicle = TestVehicle(start_distance_m=50.0, speed_mps=5.0)
        payload, distance_m = vehicle.record(37.0, 127.0, now=100.0)
        self.assertEqual(payload["type"], "vehicle")
        self.assertEqual(payload["source_mode"], "simulation")
        self.assertEqual(payload["gps_valid"], 1)
        self.assertEqual(payload["speed_mps"], 5.0)
        self.assertEqual(distance_m, 50.0)

    def test_seq_increments(self):
        vehicle = TestVehicle(period_s=0.0)
        first, _ = vehicle.record(37.0, 127.0, now=100.0)
        second, _ = vehicle.record(37.0, 127.0, now=100.2)
        self.assertEqual(first["seq"], 1)
        self.assertEqual(second["seq"], 2)

    def test_successive_records_close_in(self):
        vehicle = TestVehicle(start_distance_m=50.0, speed_mps=5.0, period_s=0.0)
        first, first_distance = vehicle.record(37.0, 127.0, now=100.0)
        second, second_distance = vehicle.record(37.0, 127.0, now=101.0)
        self.assertLess(second_distance, first_distance)
        # Starting due north, closing in means the latitude drops toward the target.
        self.assertLess(second["lat"], first["lat"])
        self.assertGreater(second["lat"], 37.0)

    def test_first_record_sits_at_the_start_distance(self):
        vehicle = TestVehicle(start_distance_m=50.0, bearing_deg=0.0)
        payload, _ = vehicle.record(37.0, 127.0, now=100.0)
        expected_lat = 37.0 + 50.0 / METERS_PER_DEGREE_LAT
        self.assertAlmostEqual(payload["lat"], expected_lat, places=9)

    def test_heading_is_the_reverse_of_the_start_bearing(self):
        payload, _ = TestVehicle(bearing_deg=0.0).record(37.0, 127.0, now=100.0)
        self.assertEqual(payload["heading_deg"], 180.0)


if __name__ == "__main__":
    unittest.main()
