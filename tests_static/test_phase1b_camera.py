"""Camera counter/time semantics without installed proprietary SDK or live camera."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/chili_crane_hardware/src"))
from chili_crane_hardware.camera_time import CameraTimeModel


def info(number=1, device=10000, host=10000):
    return {"nFrameNum": number, "nDevTimeStampHigh": device >> 32, "nDevTimeStampLow": device & 0xFFFFFFFF,
            "nHostTimeStamp": host, "fExposureTime": 100, "nLostPacket": 0}


def config(**changes):
    cfg = {"camera_control_authority": False, "host_seconds_per_tick": .001,
        "device_tick_frequency_hz": 1000, "host_epoch_offset_sec": 0, "device_epoch_offset_sec": 0,
        "host_time_evidence_id": "synthetic-only", "device_time_evidence_id": "synthetic-only",
        "sdk_document_reference": "synthetic-fixture-not-field-proof", "maximum_time_offset_sec": .1,
        "maximum_timestamp_jump_sec": .5}
    cfg.update(changes)
    return cfg


class CameraTimingTests(unittest.TestCase):
    def test_unknown_units_do_not_infer_valid(self):
        m = CameraTimeModel({"camera_control_authority": False})
        result = m.observe(info(), 10.01, "Slave", 1)
        self.assertEqual("CAMERA_RECEIVE_ESTIMATE", result["timestamp_mode"])
        self.assertIsNone(result["device_timestamp_sec"])
        self.assertFalse(result["high_precision_time_valid"])

    def test_device_slave_preferred(self):
        r = CameraTimeModel(config()).observe(info(), 10.01, "Slave", 1)
        self.assertEqual("CAMERA_PTP_DEVICE", r["timestamp_mode"])
        self.assertEqual(10, r["stamp"])
        self.assertAlmostEqual(.01, r["host_receive_latency_sec"])

    def test_not_slave_host_fallback_is_not_ptp(self):
        for status in ("Master", "Listening", "NOT_CONFIGURED", "3"):
            r = CameraTimeModel(config()).observe(info(), 10.01, status, 1)
            self.assertEqual("CAMERA_HOST_TIMESTAMP", r["timestamp_mode"])
            self.assertFalse(r["high_precision_time_valid"])

    def test_regression_reverts_to_estimate(self):
        m = CameraTimeModel(config())
        m.observe(info(), 10.01, "Slave", 1)
        r = m.observe(info(2, 9999, 10001), 10.02, "Slave", 2)
        self.assertEqual("CAMERA_RECEIVE_ESTIMATE", r["timestamp_mode"])
        self.assertEqual(1, r["timestamp_regression_count"])

    def test_frame_gap_and_counter_reset(self):
        m = CameraTimeModel(config())
        m.observe(info(), 10.01, "Slave", 1)
        r = m.observe(info(4, 10010, 10010), 10.02, "Slave", 2)
        self.assertEqual(2, r["frame_gaps"])
        r = m.observe(info(1, 10020, 10020), 10.03, "Slave", 3)
        self.assertEqual("DEGRADED", r["validity"])

    def test_timestamp_jump_and_future_reject_precision(self):
        m = CameraTimeModel(config())
        m.observe(info(), 10.01, "Slave", 1)
        r = m.observe(info(2, 11000, 11000), 10.02, "Slave", 2)
        self.assertEqual(1, r["timestamp_jump_count"])
        self.assertFalse(r["high_precision_time_valid"])

    def test_no_document_reference_no_validity(self):
        r = CameraTimeModel(config(sdk_document_reference="NOT_CONFIGURED")).observe(info(), 10, "Slave", 1)
        self.assertIsNone(r["device_timestamp_sec"])
        self.assertEqual("DEGRADED", r["validity"])

    def test_camera_cannot_authorize_control(self):
        with self.assertRaises(ValueError):
            CameraTimeModel(config(camera_control_authority=True))
        r = CameraTimeModel(config()).observe(info(), 10.01, "Slave", 1)
        self.assertFalse(r["camera_control_authority"])
        self.assertFalse(r["rolling_shutter_compensated"])
