import unittest

from probe_risk_downlink import build_command, verdict


class BuildCommandTests(unittest.TestCase):
    def test_matches_the_documented_shape(self):
        self.assertEqual(build_command(0, 2), '{"target_id":0,"risk":2}')


class VerdictTests(unittest.TestCase):
    def test_no_bytes_at_all(self):
        code, _ = verdict([0, 2], [], raw_lines=0)
        self.assertEqual(code, "NO_DATA")

    def test_bytes_arrived_but_nothing_parsed(self):
        code, message = verdict([0, 2], [], raw_lines=137)
        self.assertEqual(code, "NO_PARSE")
        self.assertIn("137", message)

    def test_node_risk_never_moves(self):
        code, message = verdict([0, 2], [0, 0, 0, 0])
        self.assertEqual(code, "NO_CHANGE")
        self.assertIn("0", message)

    def test_sent_value_echoes_back(self):
        code, _ = verdict([0, 2, 0], [0, 0, 2, 2, 0])
        self.assertEqual(code, "RESPONDS")

    def test_changed_to_an_unrelated_value(self):
        code, _ = verdict([0, 2], [0, 0, 1, 3])
        self.assertEqual(code, "CHANGED_OTHER")

    def test_zero_alone_does_not_count_as_a_response(self):
        # Sending risk=0 and seeing 0 proves nothing; it is the resting value.
        code, _ = verdict([0], [0, 0])
        self.assertEqual(code, "NO_CHANGE")


if __name__ == "__main__":
    unittest.main()
