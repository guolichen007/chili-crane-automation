"""Synthetic time evidence, never physical clock acceptance."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/chili_crane_slam/src"))
from chili_crane_slam.timebase import ClockContract, FrameTime
from chili_crane_slam.dual_lidar import Cloud, DualLidarSynchronizer
from chili_crane_slam.pointcloud import CANONICAL


def cloud(header, start, end):
    pts = np.zeros(2, dtype=CANONICAL)
    pts["timestamp"] = [start, end]
    return Cloud(header, 1.0, "sensor", pts, 4, end)


class TimebaseTests(unittest.TestCase):
    def test_first_point_is_not_whole_frame_error(self):
        f = cloud(10, 10, 10.1).timing
        f.check(10.11, .01, .5, .001)
        self.assertAlmostEqual(.1, f.span)
        self.assertAlmostEqual(10.05, f.mid)

    def test_bad_first_header(self):
        with self.assertRaisesRegex(ValueError, "FIRST_POINT"):
            cloud(10.02, 10, 10.1).timing.check(10.11, .01, .5, .001)

    def test_future_bounded_and_unconfigured(self):
        cloud(10, 10, 10.005).timing.check(10, .01, .5)
        for tolerance in (None, float("inf"), -1, 2):
            with self.assertRaises(ValueError):
                cloud(10, 10, 10).timing.check(10, tolerance, .5)
        with self.assertRaisesRegex(ValueError, "FUTURE"):
            cloud(10, 10, 10.02).timing.check(10, .01, .5)

    def test_midpoint_pair_not_header(self):
        s = DualLidarSynchronizer(.005, .5, maximum_future_skew=.01)
        s.push("204", cloud(10, 10, 10.1), 10.2, 1)
        pair = s.push("205", cloud(10.04, 10.04, 10.06), 10.2, 1)
        self.assertEqual(1, len(pair))
        self.assertAlmostEqual(0, s.pair_delta)
        self.assertEqual([], s.push("204", cloud(10, 10, 10.1), 10.2, 1))

    def test_same_headers_different_midpoints_do_not_pair(self):
        s = DualLidarSynchronizer(.005, .5)
        s.push("204", cloud(10, 10, 10.1), 10.2, 1)
        self.assertEqual([], s.push("205", cloud(10, 10, 10.02), 10.2, 1))

    def test_interval_overlap_union_ratio(self):
        f = FrameTime.from_points(10, [10, 10.1])
        overlap, ratio = f.overlap(FrameTime.from_points(10.05, [10.05, 10.15]))
        self.assertAlmostEqual(.05, overlap)
        self.assertAlmostEqual(1 / 3, ratio)
        self.assertEqual((0, 0), f.overlap(FrameTime.from_points(11, [11, 11.1])))
        self.assertEqual((0, 1), FrameTime.from_points(10, [10]).overlap(FrameTime.from_points(10, [10])))

    def test_duplicate_and_regression_counts(self):
        s = DualLidarSynchronizer(.01, .5)
        for f in (cloud(10.1, 10.1, 10.2), cloud(10.1, 10.1, 10.2), cloud(10, 10, 10.1)):
            s.push("204", f, 10.2, 1)
        self.assertEqual(1, s.duplicates["204"])
        self.assertEqual(1, s.regressions["204"])
        self.assertEqual(2, s.dropped["204"])

    def test_small_future_sync_acceptance(self):
        s = DualLidarSynchronizer(.01, .5, maximum_future_skew=.01)
        s.push("204", cloud(10, 10, 10.005), 10, 1)
        self.assertEqual(1, len(s.push("205", cloud(10, 10, 10.005), 10, 1)))
        s.push("204", cloud(10.1, 10.1, 10.2), 10, 1)
        self.assertEqual(1, s.future_count["204"])

    def test_point_time_invalid(self):
        for ts in ([], [0], [-1], [float("nan")], [float("inf")]):
            with self.assertRaises(ValueError):
                FrameTime.from_points(10, ts)

    def test_host_semantics_and_common_domain(self):
        cfg = {"clock_mode": "HOST_DERIVED", "clock_sync_state": "PROVISIONAL", "clock_domain": "host-A"}
        a = ClockContract.from_config(cfg)
        self.assertFalse(a.ptp_verified)
        self.assertTrue(a.compatible(ClockContract.from_config(cfg)))
        self.assertFalse(a.compatible(ClockContract.from_config(dict(cfg, clock_domain="host-B"))))
        with self.assertRaises(ValueError):
            ClockContract.from_config(dict(cfg, clock_sync_state="VALID"))

    def test_ptp_service_is_not_evidence(self):
        cfg = {"clock_mode": "SENSOR_PTP", "clock_sync_state": "VALID", "clock_domain": "ptp-0"}
        with self.assertRaises(ValueError):
            ClockContract.from_config(cfg)
        ptp = ClockContract.from_config(dict(cfg, time_evidence_id="synthetic-only"))
        self.assertTrue(ptp.ptp_verified)
        self.assertFalse(ptp.compatible(ClockContract("HOST_DERIVED", "PROVISIONAL", "ptp-0")))
