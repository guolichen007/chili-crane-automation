"""Synthetic clouds only; does not assert physical calibration or sensor operation."""
import unittest
from dataclasses import replace
import numpy as np
from chili_crane_slam.dual_lidar import Cloud, DualLidarHealth, DualLidarSynchronizer, Extrinsic, DualLidarMerger
from chili_crane_slam.pointcloud import CANONICAL, normalize_cloud, crop


def points(stamp=10):
    result = np.zeros(2, dtype=CANONICAL)
    result["x"] = [1, 2]
    result["timestamp"] = stamp
    return result


def cloud(stamp=10, sensor="204", received=1):
    return Cloud(stamp, received, "er1_" + sensor, points(stamp), 4)


def extrinsic(sensor="204", **changes):
    cfg = {"config_state": "VALID", "calibration_id": "synthetic-only",
           "source_frame": "er1_" + sensor, "target_frame": "base",
           "rotation_matrix": np.eye(3).tolist(), "translation_m": [0, 0, 0]}
    cfg.update(changes)
    return Extrinsic.from_config(cfg)


class SynchronizerTests(unittest.TestCase):
    def test_exact_pair_consumed_once(self):
        s = DualLidarSynchronizer(.01, .5, 2)
        self.assertEqual([], s.push("204", cloud(), 10, 1))
        self.assertEqual(1, len(s.push("205", cloud(sensor="205"), 10, 1)))
        self.assertEqual(0, sum(map(len, s.queues.values())))
        self.assertEqual([], s.push("205", cloud(10.005, "205"), 10.005, 1))
        self.assertEqual([], s.push("204", cloud(), 10.005, 1))
        self.assertEqual(1, s.paired_count)

    def test_edge_delta_and_mismatch_drop(self):
        s = DualLidarSynchronizer(.01, .5, 2)
        s.push("204", cloud(), 10, 1)
        self.assertEqual(1, len(s.push("205", cloud(10.01, "205"), 10.01, 1)))
        s.push("204", cloud(11), 11, 1)
        self.assertEqual([], s.push("205", cloud(11.02, "205"), 11.02, 1))
        self.assertEqual(1, s.dropped["204"])

    def test_queue_overflow_bounded(self):
        s = DualLidarSynchronizer(.01, 1, 2)
        for stamp in (10, 10.1, 10.2):
            s.push("204", cloud(stamp), stamp, 1)
        self.assertEqual(2, len(s.queues["204"]))
        self.assertEqual(1, s.dropped["204"])

    def test_stale_both_source_and_receive(self):
        for c, now, mono in [(cloud(), 11, 1), (cloud(), 10, 2), (cloud(11), 10, 1)]:
            s = DualLidarSynchronizer(.01, .5)
            self.assertEqual([], s.push("204", c, now, mono))
            self.assertEqual(1, s.dropped["204"])

    def test_single_lidar_loss_expires_without_fallback(self):
        s = DualLidarSynchronizer(.01, .5)
        s.push("204", cloud(), 10, 1)
        s.expire(11, 2)
        self.assertEqual(0, s.paired_count)
        self.assertFalse(s.queues["204"])

    def test_bad_stamp_empty_and_nonfinite(self):
        for stamp in (0, -1, float("nan")):
            s = DualLidarSynchronizer(.01, .5)
            self.assertEqual([], s.push("204", replace(cloud(), stamp=stamp), 10, 1))
        for p in (np.zeros(0, dtype=CANONICAL), points()):
            if len(p): p["x"] = float("nan")
            with self.assertRaises(ValueError):
                DualLidarSynchronizer(.01, .5).push("204", replace(cloud(), points=p), 10, 1)

    def test_out_of_order_no_old_frame_reuse(self):
        s = DualLidarSynchronizer(.01, .5)
        s.push("204", cloud(10.1), 10.1, 1)
        s.push("204", cloud(10), 10.1, 1)
        self.assertEqual(1, s.dropped["204"])
        s.push("205", cloud(10.1, "205"), 10.1, 1)
        self.assertEqual(1, s.paired_count)
        s.push("205", cloud(10.1, "205"), 10.1, 1)
        self.assertEqual(1, s.paired_count)

    def test_invalid_configuration(self):
        for delta, stale, queue in [(None, .5, 2), (.5, .1, 2), (.01, .5, 0), (.01, .5, 129)]:
            with self.assertRaises(ValueError):
                DualLidarSynchronizer(delta, stale, queue)


class MergerTests(unittest.TestCase):
    def test_receive_time_is_original_fact_not_publication_time(self):
        pair = (replace(cloud(), received_source_time=10.02),
                replace(cloud(sensor="205"), received_source_time=10.03))
        merged = DualLidarMerger(extrinsic(), extrinsic("205")).merge(pair)
        self.assertEqual(10.03, merged.received_source_time)
        self.assertEqual(10, merged.stamp)

    def test_merged_buffer_preserves_declared_point_step_and_offsets(self):
        merged = DualLidarMerger(extrinsic(), extrinsic("205")).merge((cloud(), cloud(sensor="205")))
        self.assertEqual(CANONICAL, merged.points.dtype)
        self.assertEqual(32 * 4, len(merged.points.tobytes()))
        self.assertEqual(24, merged.points.dtype.fields["timestamp"][1])
        fields = [("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1),
                  ("intensity", 12, 7, 1), ("ring", 16, 4, 1), ("sensor_id", 18, 2, 1), ("timestamp", 24, 8, 1)]
        decoded = normalize_cloud(merged.points.tobytes(), fields, 32, 128, 4, 1)
        np.testing.assert_array_equal(merged.points["timestamp"], decoded["timestamp"])

    def test_preserves_intensity_ring_and_timestamp(self):
        a = cloud()
        a.points["intensity"] = [3, 4]
        a.points["ring"] = [1, 2]
        merged = DualLidarMerger(extrinsic(translation_m=[1, 0, 0]), extrinsic("205")).merge(
            (a, cloud(sensor="205")))
        self.assertEqual("base", merged.frame_id)
        np.testing.assert_array_equal([2, 3, 1, 2], merged.points["x"])
        np.testing.assert_array_equal([1, 2, 0, 0], merged.points["ring"])
        np.testing.assert_array_equal([10, 10, 10, 10], merged.points["timestamp"])

    def test_invalid_extrinsic_never_identity_fallback(self):
        for cfg in ({"config_state": "NOT_CONFIGURED"}, {"rotation_matrix": [[0]*3]*3},
                    {"translation_m": [float("nan"), 0, 0]}, {"calibration_id": "NOT_CONFIGURED"}):
            with self.assertRaises(ValueError): extrinsic(**cfg)

    def test_frame_and_provenance_mismatch_rejected(self):
        merger = DualLidarMerger(extrinsic(), extrinsic("205"))
        for bad in (replace(cloud(sensor="205"), frame_id="wrong"),
                    replace(cloud(sensor="205"), source_type=1)):
            with self.assertRaises(ValueError): merger.merge((cloud(), bad))

    def test_roi_invalid_or_explicit(self):
        self.assertEqual(1, len(crop(points(), [0, 1.5, -1, 1, -1, 1])))
        with self.assertRaises(ValueError): crop(points(), None)


class NormalizationTests(unittest.TestCase):
    def test_vendor_unaligned_timestamp_and_padded_rows(self):
        dt = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "timestamp"],
            "formats": [">f4", ">f4", ">f4", ">f4", ">u2", ">f8"],
            "offsets": [0,4,8,12,16,18], "itemsize": 26})
        p = np.zeros(1, dtype=dt)
        p["x"], p["timestamp"] = 2, 10
        fields = [(n, o, t, 1) for n,o,t in (("x",0,7),("y",4,7),("z",8,7),
                                           ("intensity",12,7),("ring",16,4),("timestamp",18,8))]
        result = normalize_cloud(p.tobytes() + b"pad", fields, 26, 29, 1, 1, True)
        self.assertEqual(2, result["x"][0])
        self.assertEqual(10, result["timestamp"][0])

    def test_missing_timestamp_no_fabrication(self):
        with self.assertRaisesRegex(ValueError, "missing"):
            normalize_cloud(bytes(16), [(n,i*4,7,1) for i,n in enumerate(("x","y","z","intensity"))],
                            16, 16, 1, 1)

    def test_bad_buffer_and_nan(self):
        fields = [(n, CANONICAL.fields[n][1], 8 if n == "timestamp" else 7, 1)
                  for n in ("x", "y", "z", "intensity", "timestamp")]
        p = points()
        for data in (p.tobytes()[:-1],):
            with self.assertRaises(ValueError): normalize_cloud(data, fields, 32, 64, 2, 1)
        p["x"] = float("nan")
        with self.assertRaises(ValueError): normalize_cloud(p.tobytes(), fields, 32, 64, 2, 1)


class HealthTests(unittest.TestCase):
    def test_age_and_loss(self):
        h = DualLidarHealth(.5)
        h.accepted("204", cloud())
        self.assertTrue(h.status("204", 10.1, 1.1)["fresh"])
        self.assertFalse(h.status("204", 11, 1.1)["fresh"])
        self.assertFalse(h.status("204", 10.1, 2)["online"])
        self.assertFalse(h.status("205", 10.1, 1.1)["online"])
