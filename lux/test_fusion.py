import unittest

from fusion import fuse_risk, FusedRisk


class FusionTest(unittest.TestCase):
    def test_max_wins(self):
        fused = fuse_risk(rule_level=1, zone_level=3, predicted_level=2)
        self.assertEqual(fused.level, 3)
        self.assertEqual(fused.reason, "zone")

    def test_none_prediction_is_ignored(self):
        fused = fuse_risk(rule_level=2, zone_level=0, predicted_level=None)
        self.assertEqual(fused.level, 2)
        self.assertEqual(fused.reason, "rule")

    def test_all_zero_is_zero_and_reason_none(self):
        fused = fuse_risk(rule_level=0, zone_level=0, predicted_level=0)
        self.assertEqual(fused.level, 0)
        self.assertEqual(fused.reason, "none")

    def test_tie_lists_all_winners_in_priority_order(self):
        fused = fuse_risk(rule_level=2, zone_level=2, predicted_level=0)
        self.assertEqual(fused.level, 2)
        # rule listed before zone before predicted on a tie
        self.assertEqual(fused.reason, "rule+zone")

    def test_sources_recorded(self):
        fused = fuse_risk(rule_level=1, zone_level=2, predicted_level=3)
        self.assertEqual(fused.sources, {"rule": 1, "zone": 2, "predicted": 3})

    def test_none_prediction_absent_from_sources(self):
        fused = fuse_risk(rule_level=1, zone_level=0, predicted_level=None)
        self.assertNotIn("predicted", fused.sources)


if __name__ == "__main__":
    unittest.main()
