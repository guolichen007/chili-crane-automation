"""Synthetic installation and calibration contracts. Never field validation."""
import importlib.util
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
import yaml
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/chili_crane_slam/src"))
from chili_crane_slam.calibration_geometry import mechanical_seed, relative_seed, transform, coverage, lifecycle, mask_points
from chili_crane_slam.calibration_capture import capture_gate, SceneAccumulator
from chili_crane_slam.pointcloud import CANONICAL
HAS_SOLVER = importlib.util.find_spec("small_gicp") is not None and importlib.util.find_spec("scipy") is not None
if HAS_SOLVER:
    from chili_crane_slam.extrinsic_solver import parameters, preprocess, register, quality, consensus, solve_scenes, se3_exp, se3_log, classify_information


def experiment():
    return {"parameter_scope": "BENCH_EXPERIMENT", "voxel_size_m": .08, "vgicp_voxel_size_m": .03,
        "max_correspondence_distance_m": .5, "range_min_m": 0, "range_max_m": 20,
        "outlier_neighbors": 8, "outlier_std_ratio": 4, "normal_neighbors": 12,
        "maximum_planarity_ratio": .01,
        "max_iterations": 100, "rotation_epsilon_rad": .001, "translation_epsilon_m": .001,
        "observability_eigen_ratio": .001, "consensus_huber_delta": .03, "rotation_to_translation_scale_m": 1,
        "maximum_points": 20000, "minimum_accumulated_frames": 20, "minimum_calibration_scenes": 5,
        "minimum_validation_scenes": 1, "acceptance": {
            "min_overlap_ratio": .6, "max_nn_p95_m": .1, "max_plane_p95_m": .08,
            "max_normal_angle_p95_deg": 15, "max_plane_offset_p95_m": .1,
            "max_translation_dispersion_m": .1, "max_rotation_dispersion_rad": .1,
            "max_crosscheck_translation_m": .1, "max_crosscheck_rotation_rad": .1}}


def geometry():
    from chili_crane_slam.calibration_geometry import DYNAMIC_CLASSES
    return {"coordinate_frame": "crane_01/er1_204", "roi_bounds_m": [[-5, 5]] * 3,
        "coverage_cell_size_m": .5, "maximum_coverage_cells": 100000,
        "dynamic_masks": [{"class": c, "state": "ABSENT_VERIFIED", "evidence_id": "SYNTHETIC"} for c in DYNAMIC_CLASSES]}


def seed(source="crane_01/er1_204", xyz=None, rpy=None):
    return {"status": "MECHANICAL_SEED", "calibration_state": "NOT_CALIBRATED",
        "source_frame": source, "target_frame": "crane_01/sensor_rig", "translation_m": xyz or [.4, .2, .3],
        "roll_pitch_yaw_rad": rpy or [.02, -.03, .04], "measurement_method": "synthetic fixture",
        "measurement_uncertainty": {"translation_m": .01, "rotation_rad": .01}, "operator": "synthetic",
        "date": "2026-10-12", "mount_revision": "fixture-r1", "extrinsic_valid": False, "dual_lidar_ready": False}


def cloud(kind="corner", count=400, rng_seed=1):
    rng = np.random.default_rng(rng_seed)
    floor = np.c_[rng.uniform(-2, 2, (count, 2)), np.zeros(count)]
    wall = np.c_[np.zeros(count), rng.uniform(-2, 2, (count, 2))]
    corner = np.c_[rng.uniform(-2, 2, count), np.zeros(count), rng.uniform(-2, 2, count)]
    return floor if kind == "plane" else np.vstack([floor, wall]) if kind == "wall" else np.vstack([floor, wall, corner])


class InstallationContracts(unittest.TestCase):
    def test_s15_timechain_frozen(self):
        cfg = yaml.safe_load((ROOT / "config/contracts/phase1b_timechain_frozen.yaml").read_text())
        for file, digest in cfg["files"].items():
            self.assertEqual(digest, hashlib.sha256((ROOT / file).read_bytes()).hexdigest(), file)

    def test_seed_transform_convention(self):
        a, b = seed(), seed("crane_01/er1_205", [.8, .1, .5], [.02, .05, .12])
        t = relative_seed(a, b, "fixture-r1")
        pts = np.array([[1., 2., 3.]])
        np.testing.assert_allclose(transform(transform(pts, t), mechanical_seed(a, "fixture-r1")),
                                   transform(pts, mechanical_seed(b, "fixture-r1")), atol=1e-12)

    def test_unknown_mechanical_seed_never_identity(self):
        path = ROOT / "config/sites/crane_01/calibration/mechanical_seed_er1_204.yaml"
        cfg = yaml.safe_load(path.read_text())
        self.assertIsNone(cfg["translation_m"])
        with self.assertRaises(ValueError):
            mechanical_seed(cfg, "new")

    def test_mechanical_seed_never_valid(self):
        cfg = seed()
        cfg["extrinsic_valid"] = True
        with self.assertRaisesRegex(ValueError, "AUTHORIZE"):
            mechanical_seed(cfg, "fixture-r1")
        with self.assertRaises(ValueError):
            lifecycle({"status": "MECHANICAL_SEED", "mount_revision": "r"}, "FIELD_VALIDATED", "owner", "r", True, True)

    def test_support_movement_invalidates(self):
        result = lifecycle({"status": "FIELD_VALIDATED", "mount_revision": "old"}, "FIELD_VALIDATED", "owner", "new", True, True)
        self.assertEqual("INVALIDATED", result["status"])

    def test_synthetic_no_field_validation(self):
        with self.assertRaisesRegex(ValueError, "SYNTHETIC"):
            lifecycle({"status": "REVIEWED", "mount_revision": "r", "observability": "GOOD", "source_type": "SYNTHETIC"},
                      "FIELD_VALIDATED", "owner", "r", True, True)

    def test_capture_without_fresh_ptp_or_rig_refused(self):
        evidence = {s: {"received_at": 10, "measurement_at": 10, "source_type": 4, "valid": True,
            "normalization_valid": True, "clock_valid": True, "phase_valid": True, "timing_ready": True,
            "stationary": True, "mount_revision": "r", "evidence_id": "synthetic", "verification_method": "SYNTHETIC_FIXTURE"}
            for s in ("204", "205", "dual", "rig")}
        self.assertTrue(capture_gate(10.1, evidence, "r", "STATIC_PARENT", .2, 4))
        self.assertTrue(capture_gate(10.1, evidence, "r", "DYNAMIC_PARENT", .2, 4))
        self.assertFalse(capture_gate(10.1, evidence, "r", "NOT_CONFIGURED", .2, 4))
        self.assertFalse(capture_gate(11, evidence, "r", "STATIC_PARENT", .2, 4))
        evidence["204"]["clock_valid"] = False
        self.assertFalse(capture_gate(10.1, evidence, "r", "STATIC_PARENT", .2, 4))

    def test_capture_multiframe_hashes_and_abort(self):
        acc = SceneAccumulator(20, .001, 2)
        for i in range(22):
            frames = {}
            for s in ("204", "205"):
                raw = np.zeros(2, dtype=CANONICAL)
                raw["timestamp"] = 137 + i
                can = raw.copy()
                can["timestamp"] -= 37
                frames[s] = {"raw_header_ns": (137 + i) * 1000000000, "canonical_header_ns": (100 + i) * 1000000000,
                    "raw_points": raw, "canonical_points": can, "applied_offset_sec": 37}
            acc.add(frames, True)
        self.assertEqual(20, len(acc.rows))
        with tempfile.TemporaryDirectory() as tmp:
            m = acc.save(Path(tmp) / "scene", {"split": "CALIBRATION", "source_type": "SYNTHETIC"}, ROOT)
            self.assertEqual(4, len(m["artifacts"]))
            self.assertEqual(20, m["frame_count"])
        acc.add({}, False)
        self.assertEqual([], acc.rows)

    def test_capture_physical_cannot_use_synthetic_stationarity(self):
        e = {s: {"received_at": 10, "measurement_at": 10, "source_type": 1, "valid": True,
            "normalization_valid": True, "clock_valid": True, "phase_valid": True, "timing_ready": True,
            "stationary": True, "mount_revision": "r", "evidence_id": "fixture", "verification_method": "SYNTHETIC_FIXTURE"}
            for s in ("204", "205", "dual", "rig")}
        self.assertFalse(capture_gate(10.1, e, "r", "STATIC_PARENT", .2, 1))
        e["rig"]["verification_method"] = "MECHANICALLY_LOCKED_VERIFIED"
        self.assertTrue(capture_gate(10.1, e, "r", "STATIC_PARENT", .2, 1))
        for timestamp in (0, 20, float("nan")):
            e["204"]["measurement_at"] = timestamp
            self.assertFalse(capture_gate(10.1, e, "r", "STATIC_PARENT", .2, 1))

    def test_missing_dynamic_mask_blocks_solver_roi(self):
        g = geometry()
        g["dynamic_masks"][0]["state"] = "NOT_CONFIGURED"
        with self.assertRaisesRegex(ValueError, "NOT_VERIFIED"):
            mask_points(np.array([[0, 0, 0]]), g)

    def test_failed_report_cannot_be_reviewed(self):
        with self.assertRaises(ValueError):
            lifecycle({"status": "CANDIDATE", "mount_revision": "r", "observability": "GOOD"},
                "REVIEWED", "owner", "r", False, True)

    def test_roi_coverage_configured_only(self):
        g = geometry()
        stats = coverage(np.array([[0, 0, 0], [1, 0, 0]]), np.array([[0, 0, 0], [2, 0, 0]]), g)
        self.assertEqual(1, stats["overlap_cells"])
        g["coverage_cell_size_m"] = None
        with self.assertRaises(ValueError):
            coverage(np.zeros((1, 3)), np.zeros((1, 3)), g)

    def test_moving_object_mask(self):
        g = geometry()
        person = next(m for m in g["dynamic_masks"] if m["class"] == "person")
        person.update(state="CONFIGURED", bounds_m=[[2, 4]] * 3)
        pts = np.array([[0, 0, 0], [3, 3, 3]])
        np.testing.assert_array_equal([[0, 0, 0]], mask_points(pts, g))

    def test_default_profiles_all_outputs_false_and_no_control_nodes(self):
        for path in (ROOT / "config/runtime").glob("*.yaml"):
            cfg = yaml.safe_load(path.read_text())
            self.assertFalse(cfg.get("camera_control_authority", False))
            self.assertFalse(cfg.get("physical_output_enabled", False))
            self.assertFalse(cfg.get("automatic_control_enabled", False))
        text = (ROOT / "src/chili_crane_bringup/launch/lidar_installation_preview.launch.py").read_text()
        self.assertNotIn('package="chili_crane_control"', text)
        self.assertNotIn("StaticTransformBroadcaster", text)


@unittest.skipUnless(HAS_SOLVER, "optional pinned calibration dependencies unavailable")
class SolverSynthetic(unittest.TestCase):
    def setUp(self):
        self.cfg, self.geo = parameters(experiment()), geometry()
        self.true = se3_exp(np.array([.35, -.15, .08, .025, -.02, .04]))
        self.initial = self.true @ se3_exp(np.array([.03, -.02, .01, .004, -.003, .003]))
        self.target = cloud()
        self.source = transform(self.target, np.linalg.inv(self.true))

    def prepared(self, target=None, source=None):
        return preprocess(self.target if target is None else target, np.eye(4), self.geo, self.cfg), preprocess(
            self.source if source is None else source, self.initial, self.geo, self.cfg)

    def test_known_se3_recovery_and_initial_perturbation(self):
        a, b = self.prepared()
        result = register(a, b, self.initial, self.cfg)
        self.assertTrue(result["converged"])
        self.assertLess(np.linalg.norm(se3_log(np.linalg.inv(self.true) @ result["T_target_source"])), .01)

    def test_vgicp_crosscheck_real_binding(self):
        a, b = self.prepared()
        result = register(a, b, self.initial, self.cfg, "VGICP")
        self.assertTrue(result["converged"])
        self.assertLess(np.linalg.norm(se3_log(np.linalg.inv(self.true) @ result["T_target_source"])), .1)

    def test_coarse_vgicp_convergence_is_not_acceptance(self):
        from chili_crane_slam.extrinsic_solver import accepted
        a, b = self.prepared()
        result = register(a, b, self.initial, dict(self.cfg, vgicp_voxel_size_m=.5), "VGICP")
        self.assertFalse(accepted(result["quality"], self.cfg))

    def test_planar_scene_cannot_pass_quantitative_gate(self):
        from chili_crane_slam.extrinsic_solver import accepted
        a = cloud("plane")
        self.assertFalse(accepted(quality(a, transform(a, np.linalg.inv(self.true)), self.true, self.cfg), self.cfg))

    def test_plane_degenerate_wall_weak_corner_good(self):
        expected = {"plane": "DEGENERATE", "wall": "WEAK_XY", "corner": "GOOD"}
        for kind, status in expected.items():
            a = cloud(kind)
            b = transform(a, np.linalg.inv(self.true))
            self.assertEqual(status, quality(a, b, self.true, self.cfg)["observability"]["state"])

    def test_weak_yaw_classification(self):
        self.assertEqual("WEAK_YAW", classify_information(np.diag([1, 1, 1, 1, 1, 0]), .001)["state"])

    def test_partial_overlap_and_outliers(self):
        extra = np.random.default_rng(2).uniform(3, 4, (30, 3))
        a, b = self.prepared(source=np.vstack([self.source[:900], extra]))
        result = register(a, b, self.initial, self.cfg)
        self.assertLess(np.linalg.norm(se3_log(np.linalg.inv(self.true) @ result["T_target_source"])), .03)

    def test_no_threshold_defaults(self):
        cfg = dict(self.cfg, voxel_size_m=None)
        with self.assertRaises(ValueError):
            parameters(cfg)

    def test_se3_consensus_not_euler_average(self):
        matrices = [self.true @ se3_exp(np.array([.001 * i, 0, 0, 0, 0, .001 * i])) for i in (-2, -1, 0, 1, 2)]
        matrices.append(self.true @ se3_exp(np.array([.2, .1, .2, 0, 0, .2])))
        t, dispersion = consensus(matrices, self.cfg)
        self.assertLess(np.linalg.norm(se3_log(np.linalg.inv(self.true) @ t)), .03)
        self.assertIn("XYZ_std_m", dispersion)

    def test_multiscene_independent_holdout(self):
        scenes = []
        for i in range(6):
            a = cloud(count=1200, rng_seed=i + 30)
            scenes.append({"scene_id": "scene" + str(i), "split": "CALIBRATION" if i < 5 else "VALIDATION",
                "target": a, "source": transform(a, np.linalg.inv(self.true)), "frame_count": 20,
                "capture_gate_passed": True, "mount_revision": "r", "artifact_sha256": {"fixture": "a" * 64},
                "source_type": "SYNTHETIC", "source_frame": "crane_01/er1_205", "target_frame": "crane_01/er1_204"})
        result = solve_scenes(scenes, self.initial, self.cfg, self.geo, "r", "a" * 40)
        self.assertEqual("CANDIDATE", result["status"])
        self.assertEqual(5, len(result["calibration_scenes"]))
        self.assertEqual(1, len(result["validation_scenes"]))
        self.assertTrue(result["quantitative_pass"])
        self.assertFalse(result["extrinsic_valid"])
        scenes[-1]["split"] = "CALIBRATION"
        with self.assertRaisesRegex(ValueError, "SCENE_SPLIT"):
            solve_scenes(scenes, self.initial, self.cfg, self.geo, "r", "a" * 40)
