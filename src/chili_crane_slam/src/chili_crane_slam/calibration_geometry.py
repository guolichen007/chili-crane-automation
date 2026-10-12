"""T_A_B maps B points into A. Preview geometry never grants extrinsic validity."""
import math
import numpy as np

DYNAMIC_CLASSES = {"grab", "rope", "person", "vehicle", "moving_trolley_structure", "dynamic_material"}
STATUSES = {"NOT_CONFIGURED", "MECHANICAL_SEED", "CANDIDATE", "REVIEWED", "FIELD_VALIDATED", "INVALIDATED"}


def rigid(matrix):
    t = np.asarray(matrix, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all() or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-10)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-6)
            or not math.isclose(np.linalg.det(t[:3, :3]), 1, abs_tol=1e-6)):
        raise ValueError("INVALID_SE3")
    return t


def transform(points, t):
    t = rigid(t)
    p = np.asarray(points)
    return p @ t[:3, :3].T + t[:3, 3]


def mechanical_seed(cfg, revision):
    if cfg.get("status") != "MECHANICAL_SEED" or cfg.get("calibration_state") != "NOT_CALIBRATED":
        raise ValueError("NOT_A_MECHANICAL_SEED")
    if cfg.get("extrinsic_valid") is not False or cfg.get("dual_lidar_ready") is not False:
        raise ValueError("SEED_CANNOT_AUTHORIZE")
    if not revision or revision == "NOT_CONFIGURED" or cfg.get("mount_revision") != revision:
        raise ValueError("MOUNT_REVISION_NOT_CONFIGURED_OR_CHANGED")
    for key in ("measurement_method", "operator", "date", "source_frame", "target_frame"):
        if not cfg.get(key) or cfg[key] == "NOT_CONFIGURED":
            raise ValueError("SEED_PROVENANCE_MISSING:" + key)
    xyz, rpy = np.asarray(cfg.get("translation_m"), dtype=float), np.asarray(cfg.get("roll_pitch_yaw_rad"), dtype=float)
    uncertainty = cfg.get("measurement_uncertainty")
    if (xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite(xyz).all() or not np.isfinite(rpy).all()
            or not isinstance(uncertainty, dict) or any(type(uncertainty.get(k)) not in (int, float)
            or not math.isfinite(uncertainty[k]) or uncertainty[k] < 0 for k in ("translation_m", "rotation_rad"))):
        raise ValueError("MECHANICAL_MEASUREMENT_NOT_CONFIGURED")
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = rz @ ry @ rx, xyz
    return rigid(t)


def relative_seed(a, b, revision):
    if a.get("target_frame") != b.get("target_frame") or a.get("source_frame") == b.get("source_frame"):
        raise ValueError("SEED_FRAME_MISMATCH")
    return np.linalg.inv(mechanical_seed(a, revision)) @ mechanical_seed(b, revision)


def bounds(value):
    a = np.asarray(value, dtype=float)
    if a.shape != (3, 2) or not np.isfinite(a).all() or (a[:, 0] >= a[:, 1]).any():
        raise ValueError("ROI_NOT_CONFIGURED")
    return a


def inside(points, value):
    box = bounds(value)
    return ((points >= box[:, 0]) & (points <= box[:, 1])).all(axis=1)


def mask_points(points, geometry, strict=True):
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or not np.isfinite(p).all():
        raise ValueError("INVALID_POINTS")
    keep = inside(p, geometry.get("roi_bounds_m"))
    masks = geometry.get("dynamic_masks", [])
    if strict and {m.get("class") for m in masks} != DYNAMIC_CLASSES:
        raise ValueError("DYNAMIC_MASK_CLASSES_INCOMPLETE")
    for mask in masks:
        if mask.get("state") == "CONFIGURED":
            keep &= ~inside(p, mask.get("bounds_m"))
        elif strict and (mask.get("state") != "ABSENT_VERIFIED" or not mask.get("evidence_id")):
            raise ValueError("DYNAMIC_MASK_NOT_VERIFIED")
    return p[keep]


def coverage(a, b, geometry):
    box = bounds(geometry.get("roi_bounds_m"))
    cell = geometry.get("coverage_cell_size_m")
    if type(cell) not in (int, float) or not math.isfinite(cell) or cell <= 0:
        raise ValueError("COVERAGE_GRID_NOT_CONFIGURED")
    dimensions = np.ceil((box[:, 1] - box[:, 0]) / cell)
    if not np.isfinite(dimensions).all() or (dimensions > geometry.get("maximum_coverage_cells", 1000000)).any():
        raise ValueError("COVERAGE_GRID_TOO_LARGE")
    shape = dimensions.astype(int)
    count = math.prod(map(int, shape))
    if count > geometry.get("maximum_coverage_cells", 1000000):
        raise ValueError("COVERAGE_GRID_TOO_LARGE")
    def occupied(p):
        p = p[inside(p, box)]
        cells = np.floor((p - box[:, 0]) / cell).astype(int)
        cells = np.minimum(cells, shape - 1)
        return {tuple(row) for row in cells}
    ca, cb = occupied(a), occupied(b)
    return {"204_only_cells": len(ca - cb), "205_only_cells": len(cb - ca),
            "overlap_cells": len(ca & cb), "observed_cells": len(ca | cb), "roi_cells": count,
            "blind_region_estimate": 1 - len(ca | cb) / count,
            "meaning": "SAMPLING_OCCUPANCY_ESTIMATE_NOT_SENSOR_SAFETY_COVERAGE"}


def lifecycle(artifact, state, operator, revision, quantitative_pass=False, independent_validation=False):
    current = artifact.get("status", "NOT_CONFIGURED")
    if state not in STATUSES or not operator:
        raise ValueError("INVALID_REVIEW")
    output = dict(artifact)
    if state == "INVALIDATED" or artifact.get("mount_revision") != revision:
        output.update(status="INVALIDATED", invalidated_by=operator, invalidation_reason="SUPPORT_MOVED_OR_REVISION_CHANGED")
        return output
    allowed = (current, state) in {("CANDIDATE", "REVIEWED"), ("REVIEWED", "FIELD_VALIDATED")}
    if not allowed or not quantitative_pass or not independent_validation or artifact.get("observability") != "GOOD":
        raise ValueError("CALIBRATION_REVIEW_GATE_REJECTED")
    if state == "FIELD_VALIDATED" and artifact.get("source_type") != "PHYSICAL":
        raise ValueError("SYNTHETIC_CANNOT_BE_FIELD_VALIDATED")
    from datetime import datetime, timezone
    output.update(status=state, reviewed_by=operator, reviewed_at=datetime.now(timezone.utc).isoformat())
    return output
