"""Deterministic synthetic fixtures. Every numeric value here is NON-PRODUCTION."""
import itertools
import numpy as np
from scipy.spatial.transform import Rotation
from chili_crane_slam.pointcloud import CANONICAL
from chili_crane_slam.calibration_geometry import transform
from .pipeline import NUMERIC_PARAMETERS


def box_shape(dimensions):
    half = np.asarray(dimensions) / 2
    vertices = np.array(list(itertools.product(*[(-h, h) for h in half])))
    faces = []
    for axis in range(3):
        other = [n for n in range(3) if n != axis]
        for sign in (-1, 1):
            for a, b in itertools.product(np.linspace(-1, 1, 13), repeat=2):
                point = np.zeros(3)
                point[axis] = sign * half[axis]
                point[other] = [a * half[other[0]], b * half[other[1]]]
                faces.append(point)
    return dict(sampled_surface=np.unique(faces, axis=0).tolist(), safety_vertices=vertices.tolist(),
                bottom_reference_xyz=[0, 0, -half[2]], top_reference_xyz=[0, 0, half[2]],
                suspension_reference_xyz=[0, 0, half[2]])


def fixture():
    model = dict(model_id="SYNTHETIC_BOX_GRAB", geometry_version="synthetic-v1", model_frame="grab",
        source_type="SYNTHETIC", source_cad_version="GENERATED_NON_PRODUCTION",
        measurement_provenance="SYNTHETIC_FIXTURE", symmetry_yaw_period_rad=float(np.pi),
        geometry_translation_uncertainty_m=.002, geometry_rotation_uncertainty_rad=.002,
        OPEN=box_shape([2.0, 1.8, .9]), CLOSED=box_shape([1.6, 1, 1.2]))
    policy = dict(candidate_voxel_m=.04, cluster_radius_m=.22, tracked_roi_m=2.5, lost_hold_roi_m=3.5,
        maximum_swing_radius_m=4, coarse_voxel_m=.12, fine_voxel_m=.06, correspondence_m=.3,
        registration_translation_epsilon_m=1e-5, registration_rotation_epsilon_rad=1e-5,
        huber_m=.02, sensor_noise_m=.004, information_eigen_ratio=1e-5,
        single_sensor_covariance_factor=4, residual_p50_m=.03, residual_p95_m=.06,
        minimum_inlier_ratio=.6, minimum_extent_ratio=.4, maximum_condition=1e6,
        innovation_translation_m=.3, innovation_rotation_rad=.4, maximum_linear_speed_mps=3,
        maximum_vertical_speed_mps=2, maximum_angular_speed_radps=2,
        maximum_prediction_dt_sec=.5, lost_timeout_sec=.4, acceleration_noise=.2,
        angular_acceleration_noise=.2, initial_twist_sigma=.1, minimum_stable_duration_sec=.3,
        uncertainty_sigma=3, evidence_timeout_sec=.2,
        maximum_input_points=10000, maximum_candidate_points=8000, minimum_candidate_points=40,
        maximum_candidates=3, normal_neighbors=10, registration_iterations=35, robust_iterations=5,
        minimum_support_sectors=4, confirmation_frames=3,
        search_bounds_m=[-6, 6, -6, 6, 0, 8], self_masks=[],
        stable_enter=[.03, .08, .03, .02], stable_exit=[.06, .15, .06, .04])
    return dict(model=model, policy=policy, common_frame="map", source_type="SYNTHETIC",
                extrinsic_covariance=(np.eye(6) * 1e-6).tolist())


def scene(model, pose, stamp, opening="CLOSED", noise=0, occlusion=0, outliers=0,
          one_sensor=False, span=.1, velocity=(0, 0, 0), seed=1):
    rng = np.random.default_rng(seed)
    shape = model["OPEN" if opening == "TRANSITION" else opening]
    local = np.asarray(shape["sampled_surface"])
    times = np.linspace(-span / 2, span / 2, len(local))
    xyz = transform(local, pose) + times[:, None] * np.asarray(velocity)[None, :]
    xyz += rng.normal(0, noise, xyz.shape)
    p = np.zeros(len(xyz), dtype=CANONICAL)
    for i, name in enumerate(("x", "y", "z")):
        p[name] = xyz[:, i]
    p["timestamp"] = stamp + times
    p["sensor_id"] = 0 if one_sensor else np.arange(len(p)) % 2
    keep = rng.random(len(p)) >= occlusion
    p = p[keep]
    if outliers:
        clutter = np.zeros(outliers, dtype=CANONICAL)
        for i, name in enumerate(("x", "y", "z")):
            clutter[name] = pose[i, 3] + rng.uniform(-2, 2, outliers)
        clutter["timestamp"] = stamp
        combined = np.empty(len(p) + len(clutter), dtype=CANONICAL)
        combined[:len(p)], combined[len(p):] = p, clutter
        p = combined
    return p


def pit():
    return dict(pit_id="SYNTHETIC_PIT", config_state="VALID",
                polygon_map_m=[[-4, -4], [4, -4], [4, 4], [-4, 4]],
                wall_top_z_m=3, pit_bottom_z_m=0, safety_inset_m=.1)
