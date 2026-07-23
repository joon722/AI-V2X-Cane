import csv
import io
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

from parse_v2x import normalize_record, process_line


SAMPLE = (
    '{"type":"cane","node_id":4125577512,"seq":3360,"gps_valid":1,'
    '"lat":37.0,"lng":127.0,"speed_mps":0.0,"heading_deg":0.0,'
    '"node_risk":0,"rssi":-42}'
)


class Step3ParserTests(unittest.TestCase):
    def test_cli_mode_is_retained_in_csv_and_log(self):
        with tempfile.TemporaryDirectory() as directory:
            csv_path = Path(directory) / "log.csv"
            output = io.StringIO()
            with redirect_stdout(output):
                self.assertTrue(process_line(SAMPLE, "test", csv_path))

            with csv_path.open(newline="", encoding="utf-8") as handle:
                row = next(csv.DictReader(handle))
            self.assertEqual(row["source_mode"], "test")
            self.assertIn(
                "[STATE] type=cane seq=3360 node_risk=0 gps_valid=1 source=test",
                output.getvalue(),
            )

    def test_embedded_mode_overrides_stream_default(self):
        row = normalize_record(
            {"type": "vehicle", "source_mode": "simulation"}, "test", now=1.0
        )
        self.assertEqual(row["source_mode"], "simulation")

    def test_unknown_mode_is_rejected(self):
        with self.assertRaises(ValueError):
            normalize_record({"source_mode": "unknown"}, "test")

    def test_bad_json_is_not_written(self):
        with tempfile.TemporaryDirectory() as directory:
            csv_path = Path(directory) / "log.csv"
            with redirect_stderr(io.StringIO()):
                self.assertFalse(process_line("not-json", "test", csv_path))
            self.assertFalse(csv_path.exists())


if __name__ == "__main__":
    unittest.main()
