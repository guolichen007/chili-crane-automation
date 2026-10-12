"""Bounded model-based predict-observe-validate-filter shadow pipeline."""
from threading import Lock
import time
import numpy as np
from scipy.spatial.transform import Rotation
from .geometry import GrabGeometryModel, positive, envelope
from .registration import GeometricGrabCandidateProvider, GrabModelRegistration, GrabObservationValidator
from .tracker import GrabStateEstimator, SwingStability

NUMERIC_PARAMETERS = (
    "candidate_voxel_m", "cluster_radius_m", "tracked_roi_m", "lost_hold_roi_m",
    "maximum_swing_radius_m", "coarse_voxel_m", "fine_voxel_m", "correspondence_m",
    "registration_translation_epsilon_m", "registration_rotation_epsilon_rad",
    "huber_m", "sensor_noise_m", "information_eigen_ratio", "single_sensor_covariance_factor",
    "residual_p50_m", "residual_p95_m", "minimum_inlier_ratio", "minimum_extent_ratio",
    "maximum_condition", "innovation_translation_m", "innovation_rotation_rad",
    "maximum_linear_speed_mps", "maximum_vertical_speed_mps", "maximum_angular_speed_radps",
    "maximum_prediction_dt_sec", "lost_timeout_sec", "acceleration_noise",
    "angular_acceleration_noise", "initial_twist_sigma", "minimum_stable_duration_sec",
    "uncertainty_sigma", "evidence_timeout_sec")
INTEGER_PARAMETERS = {"maximum_input_points": (32, 200000), "maximum_candidate_points": (32, 20000),
    "minimum_candidate_points": (20, 20000), "maximum_candidates": (1, 8), "normal_neighbors": (5, 50),
    "registration_iterations": (1, 100), "robust_iterations": (1, 20),
    "minimum_support_sectors": (1, 8), "confirmation_frames": (2, 20)}


def policy_from_config(config):
    p = dict(config)
    for key in NUMERIC_PARAMETERS:
        positive(p.get(key), key)
    for key, (low, high) in INTEGER_PARAMETERS.items():
        if type(p.get(key)) is not int or not low <= p[key] <= high:
            raise ValueError("NOT_CONFIGURED:" + key)
    if not 0 < p["information_eigen_ratio"] < 1 or p["single_sensor_covariance_factor"] < 1:
        raise ValueError("INVALID_INFORMATION_POLICY")
    if any(not 0 < p[key] <= 1 for key in ("minimum_inlier_ratio", "minimum_extent_ratio")):
        raise ValueError("INVALID_SUPPORT_POLICY")
    from chili_crane_slam.pointcloud import crop, CANONICAL
    crop(np.zeros(0, dtype=CANONICAL), p.get("search_bounds_m"))
    for key in ("stable_enter", "stable_exit"):
        if not isinstance(p.get(key), list) or len(p[key]) != 4:
            raise ValueError("STABILITY_POLICY_NOT_CONFIGURED")
        for value in p[key]:
            positive(value, key)
    if np.any(np.asarray(p["stable_enter"]) >= np.asarray(p["stable_exit"])):
        raise ValueError("STABILITY_HYSTERESIS_INVALID")
    return p


class LatestFrameSlot:
    def __init__(self):
        self.lock, self.item, self.dropped = Lock(), None, 0

    def put(self, item):
        with self.lock:
            self.dropped += int(self.item is not None)
            self.item = item

    def take(self):
        with self.lock:
            item, self.item = self.item, None
            return item

    def restore_if_empty(self, item):
        with self.lock:
            if self.item is None:
                self.item = item


class GrabPerceptionPipeline:
    def __init__(self, config):
        self.policy = policy_from_config(config["policy"])
        self.model = GrabGeometryModel(config["model"])
        self.provider = GeometricGrabCandidateProvider()
        self.registration = GrabModelRegistration(self.model, self.policy)
        self.validator = GrabObservationValidator()
        self.tracker = GrabStateEstimator(self.policy)
        self.stability = SwingStability(self.policy)
        self.config = config

    def smear_bounds(self, span):
        state, policy = self.tracker, self.policy
        if state.pose is None or state.confirmations < 2:
            speed, angular = policy["maximum_linear_speed_mps"], policy["maximum_angular_speed_radps"]
        else:
            speed = np.linalg.norm(state.velocity) + policy["uncertainty_sigma"] * np.sqrt(
                max(np.linalg.eigvalsh(state.covariance[6:9,6:9]).max(),0))
            angular = np.linalg.norm(state.omega) + policy["uncertainty_sigma"] * np.sqrt(
                max(np.linalg.eigvalsh(state.covariance[9:12,9:12]).max(),0))
        return float(speed*span/2), float(angular*span/2)

    def process(self, points, stamp, opening, source_type, extrinsic_covariance, suspension=None):
        if source_type not in {"PHYSICAL", "REPLAY", "SIMULATED", "SYNTHETIC"}:
            raise ValueError("SOURCE_UNKNOWN")
        if source_type == "PHYSICAL" and self.model.source_type != "PHYSICAL":
            raise ValueError("SYNTHETIC_MODEL_ON_PHYSICAL_INPUT")
        if suspension is not None and not suspension.fresh(stamp, self.policy["evidence_timeout_sec"]):
            suspension = None
        total_start, stages = time.perf_counter(), {}
        dt = self.tracker.predict(stamp)
        start = time.perf_counter()
        candidates = self.provider.candidates(points, self.tracker, self.policy, suspension)
        stages["candidate"] = time.perf_counter() - start
        predicted = self.tracker.pose.copy() if self.tracker.pose is not None else None
        observations, rejected = [], []
        registration_seconds, validation_seconds = 0.0, 0.0
        start = time.perf_counter()
        openings = [opening] if opening in self.model.shapes else list(self.model.shapes)
        for candidate in candidates:
            xyz = np.column_stack([candidate[k] for k in ("x", "y", "z")])
            center = (xyz.min(axis=0) + xyz.max(axis=0)) / 2
            base = predicted.copy() if predicted is not None else np.eye(4)
            if predicted is None:
                base[:3, 3] = center
            # Local tracking uses predicted yaw. Lost search tries PCA yaw and its perpendicular.
            angles = [0.0]
            if predicted is None:
                _, vectors = np.linalg.eigh(np.cov(xyz[:, :2].T))
                yaw = float(np.arctan2(vectors[1, -1], vectors[0, -1]))
                angles = [yaw, yaw + np.pi / 2]
            for geometry_opening in openings:
                for yaw in angles:
                    seed = base.copy()
                    seed[:3, :3] = seed[:3, :3] @ Rotation.from_rotvec([0, 0, yaw]).as_matrix()
                    try:
                        registration_start = time.perf_counter()
                        measured = self.registration.register(candidate, seed, geometry_opening)
                        registration_seconds += time.perf_counter()-registration_start
                        validation_start = time.perf_counter()
                        status, reasons, canonical_pose = self.validator.evaluate(
                            measured, predicted, dt if dt > 0 else None, opening, self.model, self.policy, suspension)
                        validation_seconds += time.perf_counter()-validation_start
                        if status == "ACCEPTED":
                            observations.append((measured.p95, measured, canonical_pose))
                        else:
                            rejected.extend(reasons)
                    except (ValueError, RuntimeError) as exc:
                        rejected.append(str(exc))
        stages["registration_validation"] = time.perf_counter() - start
        stages["registration"], stages["validation"] = registration_seconds, validation_seconds
        start = time.perf_counter()
        accepted = min(observations, key=lambda o: o[0]) if observations else None
        if accepted is not None and any(np.linalg.norm(o[2][:3, 3] - accepted[2][:3, 3])
                > self.policy["innovation_translation_m"] for o in observations):
            rejected.append("AMBIGUOUS_MODEL_ASSOCIATION")
            accepted = None
        if accepted is not None:
            measurement_cov = accepted[1].covariance.copy()
            basis = np.eye(6)
            basis[3:,3:] = accepted[2][:3,:3].T @ accepted[1].pose[:3,:3]
            measurement_cov = basis @ measurement_cov @ basis.T
            span = float(np.ptp(points["timestamp"]))
            radius = float(np.linalg.norm(self.model.safety(opening), axis=1).max())
            translation_smear, rotation_smear = self.smear_bounds(span)
            smear = translation_smear + radius*rotation_smear
            measurement_cov[:3, :3] += np.eye(3) * smear**2
            measurement_cov[3:,3:] += np.eye(3) * rotation_smear**2
            self.tracker.update(accepted[2], measurement_cov, self.model)
        else:
            self.tracker.missing(stamp)
        stages["filter"] = time.perf_counter() - start
        result = dict(tracking=self.tracker.tracking, source=self.tracker.source,
                      source_type=source_type, valid_for_control=False,
                      reasons=list(dict.fromkeys(rejected)), candidate_count=len(candidates),
                      accepted=accepted is not None, envelope=None, swing={"valid": False, "motion_stable": False})
        if self.tracker.pose is not None and opening in {"OPEN", "CLOSED", "TRANSITION"}:
            start = time.perf_counter()
            span = float(np.ptp(points["timestamp"]))
            translation_smear, rotation_smear = self.smear_bounds(span)
            result["motion_smear_translation_margin_m"] = translation_smear
            result["motion_smear_rotation_margin_rad"] = rotation_smear
            result["envelope"] = envelope(self.model, self.tracker.pose, self.tracker.covariance[:6, :6],
                opening, extrinsic_covariance, self.tracker.velocity, self.tracker.omega,
                0, self.policy["uncertainty_sigma"])
            # Smear is an independent bounded displacement, not a fake new pose measurement.
            from dataclasses import replace
            radius = float(np.linalg.norm(self.model.safety(opening),axis=1).max())
            smear = translation_smear + radius*rotation_smear
            state_env = result["envelope"]
            from .geometry import convex_xy
            angles = np.arange(32)*2*np.pi/32
            disk = np.column_stack([np.cos(angles),np.sin(angles)])*smear/np.cos(np.pi/32)
            expanded = (state_env.conservative_footprint[:,None,:]+disk[None,:,:]).reshape(-1,2)
            result["envelope"] = replace(state_env, conservative_footprint=convex_xy(np.c_[expanded,np.zeros(len(expanded))]),
                bottom_lower_bound=state_env.bottom_lower_bound-smear,
                bottom_upper_bound=state_env.bottom_upper_bound+smear,
                top_upper_bound=state_env.top_upper_bound+smear, smear_margin=smear)
            attachment = self.model.shapes[opening if opening in self.model.shapes else "OPEN"].suspension_reference
            result["swing"] = self.stability.update(self.tracker, suspension, attachment)
            stages["geometry"] = time.perf_counter() - start
        result["observation"] = accepted[1] if accepted else None
        result["stage_seconds"] = stages
        result["total_seconds"] = time.perf_counter() - total_start
        return result
