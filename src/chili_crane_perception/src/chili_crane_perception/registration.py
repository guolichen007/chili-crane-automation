"""Geometric proposals, cached native GICP and explicit Huber point-plane refinement."""
from dataclasses import dataclass
from importlib.metadata import version
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from chili_crane_slam.calibration_geometry import rigid, transform
from chili_crane_slam.extrinsic_solver import local_planes, skew
from chili_crane_slam.pointcloud import CANONICAL, validate_points, crop
from .geometry import positive


class GrabCandidateProvider:
    def candidates(self, points, state, policy, suspension=None):
        raise NotImplementedError


class GeometricGrabCandidateProvider(GrabCandidateProvider):
    def candidates(self, points, state, policy, suspension=None):
        validate_points(points)
        if len(points) > policy["maximum_input_points"]:
            raise ValueError("INPUT_POINT_BUDGET_EXCEEDED")
        region = list(policy["search_bounds_m"])
        if state.pose is not None and state.tracking != "LOST":
            margin = policy["tracked_roi_m"] if state.tracking == "TRACKED" else policy["lost_hold_roi_m"]
            center = state.pose[:3, 3]
            region = [float(v) for c in center for v in (c - margin, c + margin)]
        selected = crop(points, region)
        if suspension is not None:
            delta = np.column_stack([selected[k] for k in ("x", "y")]) - np.asarray(suspension.point)[:2]
            selected = selected[np.linalg.norm(delta, axis=1) <= policy["maximum_swing_radius_m"]]
        for bounds in policy.get("self_masks", []):
            ids = {tuple(p) for p in crop(selected, bounds)}
            selected = selected[[tuple(p) not in ids for p in selected]]
        if not len(selected):
            return []
        xyz = np.column_stack([selected[k] for k in ("x", "y", "z")]).astype(float)
        voxel = policy["candidate_voxel_m"]
        _, ids = np.unique(np.floor(xyz / voxel).astype(np.int64), axis=0, return_index=True)
        selected, xyz = selected[ids], xyz[ids]
        if len(xyz) > policy["maximum_candidate_points"]:
            raise ValueError("CANDIDATE_POINT_BUDGET_EXCEEDED")
        tree, unseen, groups = cKDTree(xyz), set(range(len(xyz))), []
        while unseen:
            seed = min(unseen)
            unseen.remove(seed)
            queue, component = [seed], []
            while queue:
                at = queue.pop()
                component.append(at)
                neighbors = set(tree.query_ball_point(xyz[at], policy["cluster_radius_m"])) & unseen
                unseen.difference_update(neighbors)
                queue.extend(sorted(neighbors))
            if len(component) >= policy["minimum_candidate_points"]:
                groups.append(selected[component].copy())
        groups.sort(key=lambda p: -len(p))
        return groups[:policy["maximum_candidates"]]


@dataclass(frozen=True)
class Observation:
    pose: np.ndarray
    covariance: np.ndarray
    opening: str
    converged: bool
    inlier_ratio: float
    p50: float
    p95: float
    sectors: int
    extent_ratio: float
    sensors: tuple
    eigenvalues: np.ndarray
    condition: float
    residual_variance: float


def equivalent_pose(measured, predicted, period):
    candidates = []
    for index in range(round(2 * math.pi / period)):
        rotation = Rotation.from_rotvec([0, 0, index * period]).as_matrix()
        t = measured.copy()
        t[:3, :3] = measured[:3, :3] @ rotation
        distance = np.linalg.norm(Rotation.from_matrix(predicted[:3, :3].T @ t[:3, :3]).as_rotvec())
        candidates.append((distance, t))
    return min(candidates, key=lambda item: item[0])[1]


class GrabModelRegistration:
    def __init__(self, model, policy):
        import small_gicp
        if version("small-gicp") != "1.0.1":
            raise ValueError("UNPINNED_SMALL_GICP")
        self.native, self.model, self.policy = small_gicp, model, policy
        self.cached = {}
        for opening, shape in model.shapes.items():
            levels = []
            for voxel in (policy["coarse_voxel_m"], policy["fine_voxel_m"]):
                cloud, tree = small_gicp.preprocess_points(
                    shape.sampled_surface, voxel, policy["normal_neighbors"], 1)
                xyz, normals = cloud.points()[:, :3].copy(), cloud.normals()[:, :3].copy()
                levels.append((cloud, tree, xyz, normals, cKDTree(xyz)))
            self.cached[opening] = levels

    def register(self, candidate, seed, opening):
        observed = np.column_stack([candidate[k] for k in ("x", "y", "z")]).astype(float)
        inverse = np.linalg.inv(rigid(seed))
        cfg, converged = self.policy, False
        for level, voxel in zip(self.cached[opening], (cfg["coarse_voxel_m"], cfg["fine_voxel_m"])):
            source, _ = self.native.preprocess_points(observed, voxel, cfg["normal_neighbors"], 1)
            if source.size() < cfg["normal_neighbors"]:
                raise ValueError("INSUFFICIENT_REGISTRATION_POINTS")
            result = self.native.align(level[0], source, level[1],
                init_T_target_source=inverse, registration_type="GICP",
                max_correspondence_distance=cfg["correspondence_m"],
                max_iterations=cfg["registration_iterations"], num_threads=1,
                translation_epsilon=cfg["registration_translation_epsilon_m"],
                rotation_epsilon=cfg["registration_rotation_epsilon_rad"])
            inverse, converged = rigid(result.T_target_source), bool(result.converged)
        pose = np.linalg.inv(inverse)
        _, _, model_points, model_normals, tree = self.cached[opening][-1]
        # Cached MODEL is the target, measured common-frame candidate the source.
        # Refine forward pose using world translation + right/body rotation Jacobian.
        for _ in range(cfg["robust_iterations"]):
            local = transform(observed, np.linalg.inv(pose))
            distance, ids = tree.query(local)
            keep = distance <= cfg["correspondence_m"]
            if keep.sum() < cfg["normal_neighbors"]:
                raise ValueError("INSUFFICIENT_ROBUST_SUPPORT")
            mp = model_points[ids[keep]]
            normals = model_normals[ids[keep]] @ pose[:3, :3].T
            residual = np.einsum("ij,ij->i", normals, transform(mp, pose) - observed[keep])
            weights = np.minimum(1, cfg["huber_m"] / np.maximum(np.abs(residual), 1e-12))
            rotation_jacobian = np.cross(mp, normals @ pose[:3, :3])
            jacobian = np.hstack([normals, rotation_jacobian])
            h = jacobian.T @ (weights[:, None] * jacobian)
            step = -np.linalg.pinv(h, rcond=cfg["information_eigen_ratio"]) @ (jacobian.T @ (weights * residual))
            if np.linalg.norm(step[:3]) > cfg["correspondence_m"] or np.linalg.norm(step[3:]) > .5:
                raise ValueError("ROBUST_STEP_DIVERGED")
            pose[:3, 3] += step[:3]
            pose[:3, :3] = pose[:3, :3] @ Rotation.from_rotvec(step[3:]).as_matrix()
            if np.linalg.norm(step) < cfg["registration_translation_epsilon_m"]:
                break
        # Recompute all statistics at FINAL pose; never reuse pre-step Hessian.
        local = transform(observed, np.linalg.inv(pose))
        distance, ids = tree.query(local)
        keep = distance <= cfg["correspondence_m"]
        if keep.sum() < cfg["normal_neighbors"]:
            raise ValueError("FINAL_SUPPORT_MISSING")
        mp = model_points[ids[keep]]
        normals = model_normals[ids[keep]] @ pose[:3, :3].T
        residual = np.einsum("ij,ij->i", normals, transform(mp, pose) - observed[keep])
        weights = np.minimum(1, cfg["huber_m"] / np.maximum(np.abs(residual), 1e-12))
        # Observability comes from OBSERVED surface normals, not invisible CAD faces.
        # Native fine source already has its own normal/covariance preprocessing.
        source_xyz, source_normals = source.points()[:,:3], source.normals()[:,:3]
        _, source_ids = cKDTree(source_xyz).query(observed[keep])
        observed_normals = source_normals[source_ids]
        j = np.hstack([observed_normals, np.cross(mp, observed_normals @ pose[:3, :3])])
        h = j.T @ (weights[:, None] * j)
        eigenvalues, eigenvectors = np.linalg.eigh((h + h.T) / 2)
        floor = max(eigenvalues.max() * cfg["information_eigen_ratio"], 1e-12)
        sectors = len(np.unique((mp >= 0).astype(int) @ np.array([1, 2, 4])))
        extent = float(np.min(np.ptp(mp, axis=0) / np.maximum(np.ptp(model_points, axis=0), 1e-12)))
        support = max(float(keep.mean()) * max(sectors / 8, 1e-3), 1e-3)
        sensors = tuple(map(int, np.unique(candidate["sensor_id"][keep])))
        variance = max(cfg["sensor_noise_m"]**2, float(np.median(np.abs(residual)))**2) / support
        if len(sensors) == 1:
            variance *= cfg["single_sensor_covariance_factor"]
        uncertainty = (eigenvectors * (variance / np.maximum(eigenvalues, floor))) @ eigenvectors.T
        return Observation(rigid(pose), uncertainty, opening, converged, float(keep.mean()),
            float(np.percentile(np.abs(residual), 50)), float(np.percentile(np.abs(residual), 95)),
            sectors, extent, sensors, eigenvalues,
            float(eigenvalues.max() / max(eigenvalues.min(), 1e-15)), variance)


class GrabObservationValidator:
    def evaluate(self, observation, predicted, dt, opening, model, policy, suspension=None):
        reasons = []
        if not observation.converged:
            reasons.append("NATIVE_NOT_CONVERGED")
        if opening not in {"OPEN", "CLOSED"} or observation.opening != opening:
            reasons.append("OPENING_CONFLICT_OR_TRANSITION")
        if (observation.inlier_ratio < policy["minimum_inlier_ratio"] or observation.p95 > policy["residual_p95_m"]
                or observation.p50 > policy["residual_p50_m"]):
            reasons.append("RESIDUAL_OR_INLIER_GATE")
        if observation.sectors < policy["minimum_support_sectors"] or observation.extent_ratio < policy["minimum_extent_ratio"]:
            reasons.append("INSUFFICIENT_SPATIAL_SUPPORT")
        if (observation.eigenvalues.min() <= observation.eigenvalues.max() * policy["information_eigen_ratio"]
                or observation.condition > policy["maximum_condition"]):
            reasons.append("WEAK_OBSERVABILITY")
        if not observation.sensors:
            reasons.append("SENSOR_SUPPORT_MISSING")
        pose = observation.pose
        if predicted is not None:
            pose = equivalent_pose(pose, predicted, model.period)
            dp = pose[:3, 3] - predicted[:3, 3]
            da = Rotation.from_matrix(predicted[:3, :3].T @ pose[:3, :3]).as_rotvec()
            if np.linalg.norm(dp) > policy["innovation_translation_m"] or np.linalg.norm(da) > policy["innovation_rotation_rad"]:
                reasons.append("INNOVATION_GATE")
            if dt is not None and dt > 0 and (
                    np.linalg.norm(dp) / dt > policy["maximum_linear_speed_mps"]
                    or abs(dp[2]) / dt > policy["maximum_vertical_speed_mps"]
                    or np.linalg.norm(da) / dt > policy["maximum_angular_speed_radps"]):
                reasons.append("IMPOSSIBLE_INNOVATION_SPEED")
        if suspension is not None:
            offset = transform(model.shapes[observation.opening].suspension_reference[None], pose)[0] - suspension.point
            if np.linalg.norm(offset[:2]) > policy["maximum_swing_radius_m"] or offset[2] >= 0:
                reasons.append("SUSPENSION_CORRIDOR_GATE")
        status = "ACCEPTED"
        if reasons:
            status = "DEGRADED" if set(reasons) <= {"WEAK_OBSERVABILITY", "INSUFFICIENT_SPATIAL_SUPPORT"} else "REJECTED"
        return status, reasons, pose
