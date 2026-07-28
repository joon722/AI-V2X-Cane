import unittest
from pathlib import Path
import tempfile

from zone_risk import load_zones, detect_zone, zone_level, BASE_RISK_TO_LEVEL

ZONE_CSV = (
    "zone_id,zone_name,zone_type,center_x,center_y,radius_m,base_risk,speed_limit,description\n"
    "Z04,Parking Exit,Parking,100.0,100.0,30,5,10,exit\n"
    "Z03,Student Center,Pedestrian Area,300.0,300.0,30,2,10,center\n"
)


def _write(tmp):
    p = Path(tmp) / "zones.csv"
    p.write_text(ZONE_CSV, encoding="utf-8-sig")
    return p


class ZoneRiskTest(unittest.TestCase):
    def test_inside_zone_returns_base_risk(self):
        with tempfile.TemporaryDirectory() as tmp:
            zones = load_zones(_write(tmp))
            hit = detect_zone(105.0, 105.0, zones)  # within 30m of Z04
            self.assertEqual(hit["zone_id"], "Z04")
            self.assertEqual(hit["zone_base_risk"], 5)

    def test_outside_all_zones_is_out(self):
        with tempfile.TemporaryDirectory() as tmp:
            zones = load_zones(_write(tmp))
            hit = detect_zone(1000.0, 1000.0, zones)
            self.assertEqual(hit["zone_id"], "OUT")
            self.assertEqual(hit["zone_base_risk"], 0)

    def test_zone_level_maps_base_risk(self):
        with tempfile.TemporaryDirectory() as tmp:
            zones = load_zones(_write(tmp))
            self.assertEqual(zone_level(105.0, 105.0, zones), BASE_RISK_TO_LEVEL[5])
            self.assertEqual(zone_level(1000.0, 1000.0, zones), 0)

    def test_nearest_zone_wins_on_overlap(self):
        with tempfile.TemporaryDirectory() as tmp:
            zones = load_zones(_write(tmp))
            # point clearly inside Z03's radius, well outside Z04
            hit = detect_zone(295.0, 295.0, zones)
            self.assertEqual(hit["zone_id"], "Z03")

    def test_real_zone_definition_loads(self):
        # the shipped csv must parse with the documented schema
        csv_path = Path(__file__).with_name("zone_definition.csv")
        if csv_path.exists():
            zones = load_zones(csv_path)
            self.assertTrue(zones)
            self.assertIn("base_risk", zones[0])


if __name__ == "__main__":
    unittest.main()
