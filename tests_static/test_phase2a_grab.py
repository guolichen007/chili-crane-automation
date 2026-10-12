"""Native synthetic model registration, ESKF and conservative-geometry regressions."""
import sys
import unittest
from pathlib import Path
from dataclasses import replace
import numpy as np
from scipy.spatial.transform import Rotation
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/chili_crane_perception/src"))
sys.path.insert(0, str(ROOT / "src/chili_crane_slam/src"))
from chili_crane_perception.geometry import GrabGeometryModel, PitBoundaryModel, envelope
from chili_crane_perception.registration import GrabModelRegistration, GrabObservationValidator, equivalent_pose
from chili_crane_perception.pipeline import GrabPerceptionPipeline, LatestFrameSlot, policy_from_config
from chili_crane_perception.tracker import GrabStateEstimator, SuspensionPrior, SwingStability
from chili_crane_perception.synthetic import fixture, scene, pit


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.config = fixture()
        self.model = GrabGeometryModel(self.config["model"])
        self.pose = np.eye(4)
        self.pose[2, 3] = 2

    def env(self, pose=None, opening="CLOSED", cov=None, span=.1, velocity=(0, 0, 0)):
        return envelope(self.model, self.pose if pose is None else pose,
            np.eye(6)*1e-5 if cov is None else cov, opening, np.eye(6)*1e-6,
            velocity, [0, 0, 0], span, 3)

    def test_three_layers_and_transition_union(self):
        self.assertGreater(len(self.model.shapes["OPEN"].sampled_surface), 8)
        union = self.env(opening="TRANSITION")
        from shapely.geometry import Polygon
        self.assertTrue(Polygon(union.conservative_footprint).covers(Polygon(self.env(opening="OPEN").nominal_footprint)))
        self.assertTrue(Polygon(union.conservative_footprint).covers(Polygon(self.env().nominal_footprint)))

    def test_unknown_geometry_and_policy_fail_closed(self):
        with self.assertRaises(ValueError):
            GrabGeometryModel({})
        with self.assertRaises(ValueError):
            policy_from_config({})

    def test_model_bottom_reference_not_raw_minimum(self):
        self.assertAlmostEqual(1.4, self.env().bottom_nominal)
        self.assertLess(self.env().bottom_lower_bound, 1.4)

    def test_tilted_bottom_bound_contains_lowest_corner(self):
        from chili_crane_slam.calibration_geometry import transform
        self.pose[:3, :3] = Rotation.from_euler("xyz", [.2, .1, .4]).as_matrix()
        lowest = transform(self.model.safety("CLOSED"), self.pose)[:, 2].min()
        self.assertLessEqual(self.env().bottom_lower_bound, lowest)

    def test_anisotropic_circumscribed_envelope(self):
        cov = np.diag([.04, .0001, .0001, 0, 0, 0])
        expanded = self.env(cov=cov)
        self.assertGreater(np.ptp(expanded.conservative_footprint[:, 0]) - np.ptp(expanded.nominal_footprint[:, 0]),
                           np.ptp(expanded.conservative_footprint[:, 1]) - np.ptp(expanded.nominal_footprint[:, 1]))

    def test_smear_increases_bound_without_discarding_timestamps(self):
        slow, fast = self.env(), self.env(span=.2, velocity=(0, 0, 1))
        self.assertGreater(fast.smear_margin, slow.smear_margin)
        self.assertLess(fast.bottom_lower_bound, slow.bottom_lower_bound)

    def test_inside_touch_cross_outside_wall_and_vertical_prism(self):
        boundary = PitBoundaryModel(pit(), "map")
        for x in (0, 3.2, 4.0, 6.0):
            pose = self.pose.copy()
            pose[0, 3] = x
            result = boundary.clearance(self.env(pose))
            self.assertLessEqual(result["conservative_clearance_m"], result["raw_horizontal_clearance_m"])
            self.assertEqual(x >= 3.2, result["wall_collision_risk"])
        pose[2, 3] = 10
        result = boundary.clearance(self.env(pose))
        self.assertFalse(result["wall_collision_relevant"])
        self.assertTrue(np.isfinite(result["conservative_clearance_m"]))

    def test_concave_pit_does_not_use_vertex_only_containment(self):
        data = pit()
        data["polygon_map_m"] = [[-4,-4],[4,-4],[4,4],[.2,4],[.2,0],[-.2,0],[-.2,4],[-4,4]]
        boundary = PitBoundaryModel(data, "map")
        result = boundary.clearance(self.env())
        self.assertTrue(result["wall_collision_risk"])

    def test_latest_wins_no_backlog(self):
        slot = LatestFrameSlot()
        for i in range(10):
            slot.put(i)
        self.assertEqual(9, slot.take())
        self.assertEqual(9, slot.dropped)
        self.assertIsNone(slot.take())

    def test_symmetry_innovation_no_euler_wrap(self):
        measured = self.pose.copy()
        measured[:3,:3] = Rotation.from_euler("z", np.pi).as_matrix()
        np.testing.assert_allclose(equivalent_pose(measured, self.pose, np.pi), self.pose, atol=1e-10)

    def test_negative_or_non_psd_covariance_rejected(self):
        with self.assertRaises(ValueError):
            self.env(cov=-np.eye(6))


class NativeGrabTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = fixture()
        cls.model = GrabGeometryModel(cls.config["model"])
        cls.registration = GrabModelRegistration(cls.model, cls.config["policy"])

    def points(self, **kwargs):
        pose = np.eye(4)
        pose[:3,3] = [.15, -.1, 2]
        return pose, scene(self.config["model"], pose, 10, **kwargs)

    def test_native_coarse_fine_and_robust_registration(self):
        pose, points = self.points(noise=.002, outliers=30)
        seed = pose.copy()
        seed[:3,3] += [.05, -.03, .02]
        observed = self.registration.register(points, seed, "CLOSED")
        status, reasons, actual = GrabObservationValidator().evaluate(
            observed, pose, .1, "CLOSED", self.model, self.config["policy"])
        self.assertEqual("ACCEPTED", status, reasons)
        np.testing.assert_allclose(actual[:3,3], pose[:3,3], atol=.02)
        self.assertTrue(np.linalg.eigvalsh(observed.covariance).min() >= -1e-12)

    def test_partial_occlusion_and_single_sensor_inflate_uncertainty(self):
        pose, points = self.points()
        full = self.registration.register(points, pose, "CLOSED")
        pose, partial = self.points(occlusion=.35, one_sensor=True)
        observed = self.registration.register(partial, pose, "CLOSED")
        self.assertEqual((0,), observed.sensors)
        self.assertGreater(np.trace(observed.covariance), np.trace(full.covariance))

    def test_opening_conflict_not_accepted(self):
        pose, points = self.points()
        observed = self.registration.register(points, pose, "CLOSED")
        status, reasons, _ = GrabObservationValidator().evaluate(
            observed, pose, .1, "OPEN", self.model, self.config["policy"])
        self.assertEqual("REJECTED", status)
        self.assertIn("OPENING_CONFLICT_OR_TRANSITION", reasons)

    def test_convergence_alone_is_not_valid(self):
        pose, points = self.points()
        observed = self.registration.register(points, pose, "CLOSED")
        for bad in (replace(observed, sectors=1), replace(observed, inlier_ratio=.1),
                    replace(observed, eigenvalues=np.zeros(6)), replace(observed, converged=False)):
            self.assertEqual("REJECTED", GrabObservationValidator().evaluate(
                bad, pose, .1, "CLOSED", self.model, self.config["policy"])[0])

    def test_tracker_confirmation_loss_and_reacquisition(self):
        tracker = GrabStateEstimator(self.config["policy"])
        pose, points = self.points()
        observed = self.registration.register(points, pose, "CLOSED")
        for t in (10, 10.1, 10.2):
            tracker.predict(t)
            tracker.update(observed.pose, observed.covariance, self.model)
        self.assertEqual("TRACKED", tracker.tracking)
        tracker.predict(10.3)
        tracker.missing(10.3)
        self.assertEqual(("LOST_HOLD", "PREDICTED"), (tracker.tracking, tracker.source))
        tracker.predict(11)
        tracker.missing(11)
        self.assertEqual("LOST", tracker.tracking)
        tracker.predict(11.1)
        tracker.update(pose, observed.covariance, self.model)
        self.assertEqual("CANDIDATE", tracker.tracking)
        with self.assertRaises(ValueError):
            tracker.predict(11.1)

    def test_eskf_rotation_covariance_and_no_command_time_z(self):
        tracker = GrabStateEstimator(self.config["policy"])
        pose, points = self.points()
        observed = self.registration.register(points, pose, "CLOSED")
        for i in range(10):
            tracker.predict(10+i*.1)
            shifted = pose.copy()
            shifted[:3,:3] = Rotation.from_rotvec([.005*i, 0, .01*i]).as_matrix()
            tracker.update(shifted, observed.covariance, self.model)
        self.assertGreater(np.linalg.norm(tracker.omega), 0)
        np.testing.assert_allclose(tracker.pose[:3,:3].T @ tracker.pose[:3,:3], np.eye(3), atol=1e-10)
        self.assertGreaterEqual(np.linalg.eigvalsh(tracker.covariance).min(), -1e-10)
        self.assertAlmostEqual(2, tracker.pose[2,3])

    def test_pipeline_repeated_measurements_and_source_isolation(self):
        pipeline = GrabPerceptionPipeline(self.config)
        pose, _ = self.points()
        for i in range(3):
            points = scene(self.config["model"], pose, 10+i*.1)
            result = pipeline.process(points, 10+i*.1, "CLOSED", "SYNTHETIC", np.eye(6)*1e-6)
        self.assertEqual("TRACKED", result["tracking"], result["reasons"])
        self.assertFalse(result["valid_for_control"])
        with self.assertRaisesRegex(ValueError, "SYNTHETIC_MODEL_ON_PHYSICAL"):
            pipeline.process(points, 11, "CLOSED", "PHYSICAL", np.eye(6)*1e-6)
