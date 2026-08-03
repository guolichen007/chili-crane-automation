import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONTROL_PYTHON = ROOT / "src" / "chili_crane_control" / "src"
sys.path.insert(0, str(CONTROL_PYTHON))

from chili_crane_control.servo_csv import (  # noqa: E402
    NOT_CONFIGURED,
    ServoCsvSample,
    classify_sample,
    evidence_age_seconds,
    load_samples,
    mapped_sample_time,
    scheduled_monotonic_time,
)


HEADER = (
    "stamp_sec,raw_position,position_m,velocity_mps,valid,homed,fault\n"
)


class ServoCsvContractTest(unittest.TestCase):
    def _write_csv(self, content):
        directory = tempfile.TemporaryDirectory()
        path = Path(directory.name) / "servo.csv"
        path.write_text(content, encoding="utf-8")
        self.addCleanup(directory.cleanup)
        return path

    def test_timestamps_must_be_strictly_increasing(self):
        path = self._write_csv(
            HEADER
            + "1,0,0,0,true,true,false\n"
            + "1,1,1,0,true,true,false\n"
        )
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            load_samples(path)

    def test_missing_column_is_rejected(self):
        path = self._write_csv("stamp_sec,raw_position\n1,0\n")
        with self.assertRaisesRegex(ValueError, "missing required CSV columns"):
            load_samples(path)

    def test_non_finite_numeric_value_is_rejected(self):
        path = self._write_csv(
            HEADER + "1,nan,0,0,true,true,false\n"
        )
        with self.assertRaisesRegex(ValueError, "must be finite"):
            load_samples(path)

    def test_unconfigured_calibration_is_not_valid(self):
        sample = ServoCsvSample(1.0, 0.0, 0.0, 0.0, True, True, False)
        self.assertEqual(NOT_CONFIGURED, classify_sample("", sample))
        self.assertEqual(
            NOT_CONFIGURED,
            classify_sample("NOT_CONFIGURED", sample),
        )

    def test_wall_schedule_is_independent_from_ros_time(self):
        self.assertEqual(102.5, scheduled_monotonic_time(100.0, 10.0, 12.5))
        self.assertEqual(52.5, mapped_sample_time(50.0, 10.0, 12.5))

    def test_old_evidence_age_is_not_refreshed(self):
        self.assertEqual(2.0, evidence_age_seconds(12.0, 10.0))
        self.assertEqual(5.0, evidence_age_seconds(15.0, 10.0))

    def test_publisher_uses_monotonic_wall_clock(self):
        publisher = (
            ROOT
            / "src"
            / "chili_crane_control"
            / "scripts"
            / "mock_servo_csv_publisher.py"
        ).read_text(encoding="utf-8")
        self.assertIn("time.monotonic()", publisher)
        self.assertNotIn("rospy.Time.now() - start_wall", publisher)


if __name__ == "__main__":
    unittest.main()
