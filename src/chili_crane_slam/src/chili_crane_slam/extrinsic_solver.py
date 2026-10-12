"""Seeded offline GICP/VGICP, geometry observability and SE(3) holdout validation."""
import hashlib
import json
from importlib.metadata import version
from pathlib import Path
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from .calibration_geometry import rigid, transform, mask_points


class CalibrationRejected(ValueError):
    """Preserve rejected scene diagnostics; never turn convergence into acceptance."""
    def __init__(self, reason, results):
        super().__init__(reason)
        self.results = results


def parameters(cfg):
    ranges = {"voxel_size_m": (1e-4, 5), "vgicp_voxel_size_m": (1e-4, 5),
        "max_correspondence_distance_m": (1e-4, 20), "range_min_m": (0, 1000),
        "range_max_m": (1e-3, 1000), "outlier_std_ratio": (.01, 20),
        "rotation_epsilon_rad": (1e-8, .1), "translation_epsilon_m": (1e-8, .1),
        "observability_eigen_ratio": (1e-8, .5), "consensus_huber_delta": (1e-5, 10),
        "rotation_to_translation_scale_m": (1e-3, 1000)}
    ranges["maximum_planarity_ratio"] = (1e-6, .2)
    for key, (lo, hi) in ranges.items():
        v = cfg.get(key)
        if type(v) not in (int, float) or not math.isfinite(v) or not lo <= v <= hi:
            raise ValueError("SOLVER_PARAMETER_NOT_CONFIGURED:" + key)
    for key, lo, hi in (("normal_neighbors", 5, 64), ("outlier_neighbors", 5, 64),
                       ("max_iterations", 5, 500), ("maximum_points", 100, 2000000),
                       ("minimum_accumulated_frames", 20, 50), ("minimum_calibration_scenes", 5, 50),
                       ("minimum_validation_scenes", 1, 50)):
        if type(cfg.get(key)) is not int or not lo <= cfg[key] <= hi:
            raise ValueError("SOLVER_PARAMETER_NOT_CONFIGURED:" + key)
    if cfg["range_min_m"] >= cfg["range_max_m"]:
        raise ValueError("INVALID_RANGE_FILTER")
    if cfg.get("parameter_scope") not in {"BENCH_EXPERIMENT", "FIELD_FROZEN"}:
        raise ValueError("PARAMETER_SCOPE_NOT_CONFIGURED")
    return cfg


def preprocess(p, seed, geometry, cfg):
    p = np.asarray(p, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) > 2000000 or not np.isfinite(p).all():
        raise ValueError("CLOUD_SHAPE_OR_SIZE_INVALID")
    ranges = np.linalg.norm(p, axis=1)
    selected = (ranges >= cfg["range_min_m"]) & (ranges <= cfg["range_max_m"])
    p = p[selected]
    common = transform(p, seed)
    # Masks are in target coordinates; preserve corresponding original source points.
    from .calibration_geometry import inside, DYNAMIC_CLASSES
    keep = inside(common, geometry.get("roi_bounds_m"))
    masks = geometry.get("dynamic_masks", [])
    if {m.get("class") for m in masks} != DYNAMIC_CLASSES:
        raise ValueError("DYNAMIC_MASK_CLASSES_INCOMPLETE")
    for mask in masks:
        if mask.get("state") == "CONFIGURED":
            keep &= ~inside(common, mask.get("bounds_m"))
        elif mask.get("state") != "ABSENT_VERIFIED" or not mask.get("evidence_id"):
            raise ValueError("DYNAMIC_MASK_NOT_VERIFIED")
    p = p[keep]
    cells = np.floor(p / cfg["voxel_size_m"]).astype(np.int64)
    _, ids, inverse = np.unique(cells, axis=0, return_index=True, return_inverse=True)
    sums = np.zeros((len(ids), 3))
    np.add.at(sums, inverse, p)
    p = sums / np.bincount(inverse)[:, None]
    if not cfg["normal_neighbors"] < len(p) <= cfg["maximum_points"]:
        raise ValueError("INSUFFICIENT_OR_EXCESSIVE_FILTERED_POINTS")
    distance, _ = cKDTree(p).query(p, k=cfg["outlier_neighbors"] + 1, workers=1)
    mean = distance[:, 1:].mean(axis=1)
    p = p[mean <= mean.mean() + cfg["outlier_std_ratio"] * mean.std() + 1e-12]
    if len(p) <= cfg["normal_neighbors"]:
        raise ValueError("INSUFFICIENT_INLIERS")
    return np.ascontiguousarray(p)


def local_planes(points, neighbors):
    _, indices = cKDTree(points).query(points, k=neighbors, workers=1)
    normals, centers, ratios = [], [], []
    for batch in np.array_split(indices, max(1, math.ceil(len(indices) / 4096))):
        samples = points[batch]
        center = samples.mean(axis=1)
        centered = samples - center[:, None, :]
        covariance = np.einsum("nki,nkj->nij", centered, centered) / neighbors
        values, vectors = np.linalg.eigh(covariance)
        normals.append(vectors[:, :, 0])
        centers.append(center)
        ratios.append(values[:, 0] / np.maximum(values[:, 1], 1e-15))
    return np.vstack(normals), np.vstack(centers), np.concatenate(ratios)


def quantiles(values):
    return {key: float(np.percentile(values, q)) if len(values) else None
            for key, q in (("P50", 50), ("P90", 90), ("P95", 95))}


def classify_information(h, ratio):
    h = (h + h.T) / 2
    values, vectors = np.linalg.eigh(h)
    maximum = max(float(values.max()), 1e-15)
    weak = values <= maximum * ratio
    rank = int((~weak).sum())
    null = vectors[:, weak]
    if rank == 6:
        label = "GOOD"
    elif rank <= 3:
        label = "DEGENERATE"
    elif float(np.square(null[:2]).sum()) > .5:
        label = "WEAK_XY"
    elif float(np.square(null[5]).sum()) > .5:
        label = "WEAK_YAW"
    else:
        label = "DEGENERATE"
    return {"state": label, "rank": rank, "eigenvalues": values.tolist(),
            "axis_information": np.diag(h).tolist(), "axis_order": ["X", "Y", "Z", "ROLL", "PITCH", "YAW"],
            "kind": "APPROXIMATE_POINT_TO_PLANE_GEOMETRY_NOT_A_SAFETY_CERTIFICATE"}


def quality(target, source, t, cfg):
    moved = transform(source, t)
    distance, ids = cKDTree(target).query(moved, k=1, workers=1)
    valid = distance <= cfg["max_correspondence_distance_m"]
    if int(valid.sum()) < cfg["normal_neighbors"]:
        raise ValueError("INSUFFICIENT_CORRESPONDENCES")
    normals, centers, target_ratios = local_planes(target, cfg["normal_neighbors"])
    source_normals, source_centers, source_ratios = local_planes(source, cfg["normal_neighbors"])
    nn_statistics = quantiles(distance[valid])
    overlap_ratio, correspondence_count = float(valid.mean()), int(valid.sum())
    valid &= (target_ratios[ids] <= cfg["maximum_planarity_ratio"]) & (source_ratios <= cfg["maximum_planarity_ratio"])
    if valid.sum() < cfg["normal_neighbors"]:
        raise ValueError("INSUFFICIENT_PLANAR_SUPPORT")
    n, p = normals[ids[valid]], moved[valid]
    residual = np.abs(np.einsum("ij,ij->i", n, p - target[ids[valid]]))
    angles = np.degrees(np.arccos(np.clip(np.abs(np.einsum("ij,ij->i",
        source_normals[valid] @ t[:3, :3].T, n)), 0, 1)))
    offsets = np.abs(np.einsum("ij,ij->i", n, transform(source_centers[valid], t) - centers[ids[valid]]))
    centered = p - p.mean(axis=0)
    radius = max(float(np.sqrt(np.square(centered).sum(axis=1).mean())), 1e-12)
    jacobian = np.hstack((n, np.cross(centered / radius, n)))
    info = jacobian.T @ jacobian / len(jacobian)
    return {"overlap_ratio": overlap_ratio, "correspondences": correspondence_count, "planar_support_count": int(valid.sum()),
        "nearest_neighbor_m": nn_statistics, "point_to_plane_m": quantiles(residual),
        "plane_normal_angle_deg": quantiles(angles), "plane_offset_m": quantiles(offsets),
        "observability": classify_information(info, cfg["observability_eigen_ratio"])}


def register(target, source, seed, cfg, method="GICP"):
    import small_gicp
    if version("small-gicp") != "1.0.1" or method not in {"GICP", "VGICP"}:
        raise ValueError("UNPINNED_SOLVER_OR_UNSUPPORTED_METHOD")
    seed = rigid(seed)
    tc, tree = small_gicp.preprocess_points(target, cfg["voxel_size_m"], cfg["normal_neighbors"], 1)
    sc, _ = small_gicp.preprocess_points(source, cfg["voxel_size_m"], cfg["normal_neighbors"], 1)
    options = dict(init_T_target_source=seed, max_correspondence_distance=cfg["max_correspondence_distance_m"],
        num_threads=1, max_iterations=cfg["max_iterations"], rotation_epsilon=cfg["rotation_epsilon_rad"],
        translation_epsilon=cfg["translation_epsilon_m"])
    if method == "GICP":
        result = small_gicp.align(tc, sc, tree, registration_type="GICP", **options)
    else:
        voxelmap = small_gicp.GaussianVoxelMap(cfg["vgicp_voxel_size_m"])
        voxelmap.insert(tc)
        result = small_gicp.align(voxelmap, sc, **options)
    matrix = rigid(result.T_target_source)
    return {"method": method, "T_target_source": matrix.tolist(), "converged": bool(result.converged),
        "iterations": int(result.iterations), "num_inliers": int(result.num_inliers),
        "fitness_mean_cost": float(result.error) / max(1, int(result.num_inliers)),
        "final_hessian": np.asarray(result.H).tolist(), "quality": quality(target, source, matrix, cfg)}


def skew(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def se3_exp(twist):
    rho, w = twist[:3], twist[3:]
    theta = np.linalg.norm(w)
    k = skew(w)
    j = (np.eye(3) + k / 2 + k @ k / 6 if theta < 1e-8 else
         np.eye(3) + (1 - math.cos(theta)) / theta**2 * k + (theta - math.sin(theta)) / theta**3 * k @ k)
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = Rotation.from_rotvec(w).as_matrix(), j @ rho
    return t


def se3_log(t):
    t = rigid(t)
    w = Rotation.from_matrix(t[:3, :3]).as_rotvec()
    theta, k = np.linalg.norm(w), skew(w)
    j = (np.eye(3) + k / 2 + k @ k / 6 if theta < 1e-8 else
         np.eye(3) + (1 - math.cos(theta)) / theta**2 * k + (theta - math.sin(theta)) / theta**3 * k @ k)
    return np.r_[np.linalg.solve(j, t[:3, 3]), w]


def consensus(matrices, cfg):
    matrices = [rigid(t) for t in matrices]
    scale = cfg["rotation_to_translation_scale_m"]
    def residuals(t):
        return np.array([se3_log(np.linalg.inv(t) @ m) for m in matrices])
    def norms(res):
        return np.linalg.norm(res * np.array([1, 1, 1, scale, scale, scale]), axis=1)
    center = matrices[int(np.argmin([np.median(norms(residuals(t))) for t in matrices]))].copy()
    for _ in range(cfg["max_iterations"]):
        residual = residuals(center)
        magnitude = norms(residual)
        weights = np.minimum(1, cfg["consensus_huber_delta"] / np.maximum(magnitude, 1e-15))
        update = np.average(residual, axis=0, weights=weights)
        center = center @ se3_exp(update)
        if np.linalg.norm(update) < 1e-9:
            break
    residual = residuals(center)
    return rigid(center), {"XYZ_std_m": np.std(residual[:, :3], axis=0).tolist(),
        "translation_norm_m": quantiles(np.linalg.norm(residual[:, :3], axis=1)),
        "rotation_rad": quantiles(np.linalg.norm(residual[:, 3:], axis=1)),
        "method": "ROBUST_SE3_LOG_EXP_HUBER_NOT_EULER_AVERAGE"}


def accepted(metrics, cfg):
    acceptance = cfg.get("acceptance", {})
    def check(key, value, lower=False):
        limit = acceptance.get(key)
        if type(limit) not in (int, float) or not math.isfinite(limit) or limit < 0 or value is None:
            return False
        return value >= limit if lower else value <= limit
    q = metrics
    return (q["observability"]["state"] == "GOOD" and
        check("min_overlap_ratio", q["overlap_ratio"], True) and
        check("max_nn_p95_m", q["nearest_neighbor_m"]["P95"]) and
        check("max_plane_p95_m", q["point_to_plane_m"]["P95"]) and
        check("max_normal_angle_p95_deg", q["plane_normal_angle_deg"]["P95"]) and
        check("max_plane_offset_p95_m", q["plane_offset_m"]["P95"]))


def solve_scenes(scenes, seed, cfg, geometry, mount_revision, git_sha):
    parameters(cfg)
    if len(git_sha) != 40 or not mount_revision or mount_revision == "NOT_CONFIGURED":
        raise ValueError("CALIBRATION_PROVENANCE_MISSING")
    calibration = [s for s in scenes if s["split"] == "CALIBRATION"]
    validation = [s for s in scenes if s["split"] == "VALIDATION"]
    ids = [s["scene_id"] for s in scenes]
    if (not scenes or geometry.get("coordinate_frame") != scenes[0].get("target_frame")
            or any(s.get("source_frame") != scenes[0].get("source_frame")
            or s.get("target_frame") != scenes[0].get("target_frame") for s in scenes)):
        raise ValueError("SCENE_OR_ROI_FRAME_MISMATCH")
    if len(ids) != len(set(ids)) or len(calibration) < cfg["minimum_calibration_scenes"] or len(validation) < cfg["minimum_validation_scenes"]:
        raise ValueError("INDEPENDENT_SCENE_SPLIT_REQUIRED")
    if len({s["source_type"] for s in scenes}) != 1:
        raise ValueError("MIXED_EVIDENCE_SOURCE_TYPES")
    for s in scenes:
        if (s["frame_count"] < cfg["minimum_accumulated_frames"] or not s.get("capture_gate_passed")
                or s.get("mount_revision") != mount_revision or not s.get("artifact_sha256")):
            raise ValueError("SCENE_CAPTURE_OR_PROVENANCE_INVALID")
    processed = {}
    for s in scenes:
        processed[s["scene_id"]] = (preprocess(s["target"], np.eye(4), geometry, cfg),
                                    preprocess(s["source"], seed, geometry, cfg))
    results = []
    for s in calibration:
        target, source = processed[s["scene_id"]]
        g, v = register(target, source, seed, cfg), register(target, source, seed, cfg, "VGICP")
        delta = se3_log(np.linalg.inv(rigid(g["T_target_source"])) @ rigid(v["T_target_source"]))
        results.append({"scene_id": s["scene_id"], "GICP": g, "VGICP": v,
            "crosscheck_translation_m": float(np.linalg.norm(delta[:3])),
            "crosscheck_rotation_rad": float(np.linalg.norm(delta[3:]))})
    usable = [r for r in results if r["GICP"]["converged"] and r["VGICP"]["converged"]
              and r["GICP"]["quality"]["observability"]["state"] == "GOOD"]
    if len(usable) != len(calibration):
        raise CalibrationRejected("DEGENERATE_OR_UNCONVERGED_SCENE_NO_CANDIDATE", results)
    final, dispersion = consensus([r["GICP"]["T_target_source"] for r in usable], cfg)
    held_out = [{"scene_id": s["scene_id"], "quality": quality(*processed[s["scene_id"]], final, cfg)} for s in validation]
    limits = cfg.get("acceptance", {})
    def bounded_metric(key, value):
        limit = limits.get(key)
        return type(limit) in (int, float) and math.isfinite(limit) and 0 <= value <= limit
    passed = (all(accepted(r["quality"], cfg) for r in held_out) and
        all(accepted(r["GICP"]["quality"], cfg) and accepted(r["VGICP"]["quality"], cfg) for r in results) and
        all(bounded_metric("max_crosscheck_translation_m", r["crosscheck_translation_m"]) and
            bounded_metric("max_crosscheck_rotation_rad", r["crosscheck_rotation_rad"]) for r in results) and
        bounded_metric("max_translation_dispersion_m", dispersion["translation_norm_m"]["P95"]) and
        bounded_metric("max_rotation_dispersion_rad", dispersion["rotation_rad"]["P95"]))
    from datetime import datetime, timezone
    identity = hashlib.sha256(json.dumps({"ids": ids, "revision": mount_revision, "t": final.tolist()}, sort_keys=True).encode()).hexdigest()
    return {"calibration_id": "CAL-" + identity[:16], "status": "CANDIDATE",
        "source_frame": scenes[0]["source_frame"], "target_frame": scenes[0]["target_frame"],
        "transform_convention": "T_A_B maps B points into A", "T_target_source": final.tolist(),
        "translation_m": final[:3, 3].tolist(), "quaternion_xyzw": Rotation.from_matrix(final[:3, :3]).as_quat().tolist(),
        "mechanical_seed": rigid(seed).tolist(), "solver": "small_gicp", "solver_version": version("small-gicp"),
        "git_sha": git_sha, "mount_revision": mount_revision, "scene_ids": ids,
        "source_type": scenes[0]["source_type"], "scene_artifacts": {s["scene_id"]: s["artifact_sha256"] for s in scenes},
        "scene_config_hashes": {s["scene_id"]: s.get("config_hash", "NOT_REPORTED") for s in scenes},
        "calibration_scenes": results, "validation_scenes": held_out, "dispersion": dispersion,
        "observability": "GOOD", "quantitative_pass": bool(passed), "independent_validation": True,
        "parameters": cfg, "created_at": datetime.now(timezone.utc).isoformat(), "reviewed_at": None,
        "extrinsic_valid": False, "dual_lidar_ready": False}
