"""Three model layers and conservative common-frame geometry, not control authority."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon, Point, LineString
from chili_crane_slam.calibration_geometry import rigid, transform
from chili_crane_slam.extrinsic_solver import skew

SOURCES = {"PHYSICAL", "REPLAY", "SIMULATED", "SYNTHETIC"}


def positive(value, name, zero=False):
    if type(value) not in (int, float) or not math.isfinite(value) or value < 0 or (not zero and value == 0):
        raise ValueError("NOT_CONFIGURED:" + name)
    return float(value)


def covariance(value, size):
    p = np.asarray(value, dtype=float)
    if (p.shape != (size, size) or not np.isfinite(p).all()
            or not np.allclose(p, p.T, atol=1e-9) or np.linalg.eigvalsh(p).min() < -1e-10):
        raise ValueError("INVALID_COVARIANCE")
    return (p + p.T) / 2


@dataclass(frozen=True)
class ModelShape:
    sampled_surface: np.ndarray
    safety_vertices: np.ndarray
    bottom_reference: np.ndarray
    top_reference: np.ndarray
    suspension_reference: np.ndarray

    @classmethod
    def from_config(cls, data):
        arrays = [np.asarray(data.get(k), dtype=float) for k in (
            "sampled_surface", "safety_vertices", "bottom_reference_xyz",
            "top_reference_xyz", "suspension_reference_xyz")]
        for a in arrays[:2]:
            if a.ndim != 2 or a.shape[1] != 3 or len(a) < 8 or not np.isfinite(a).all():
                raise ValueError("MODEL_CLOUD_NOT_CONFIGURED")
        if any(a.shape != (3,) or not np.isfinite(a).all() for a in arrays[2:]):
            raise ValueError("MODEL_REFERENCE_NOT_CONFIGURED")
        hull = ConvexHull(arrays[1])
        if np.any(arrays[0] @ hull.equations[:, :3].T + hull.equations[:, 3] > 1e-7):
            raise ValueError("SAFETY_HULL_DOES_NOT_CONTAIN_MODEL")
        if (arrays[2][2] > arrays[1][:, 2].min() + 1e-7
                or arrays[3][2] < arrays[1][:, 2].max() - 1e-7):
            raise ValueError("BOTTOM_TOP_REFERENCE_NOT_CONSERVATIVE")
        for a in arrays:
            a.setflags(write=False)
        return cls(*arrays)


class GrabGeometryModel:
    def __init__(self, config):
        for key in ("model_id", "geometry_version", "model_frame", "source_cad_version", "measurement_provenance"):
            if not isinstance(config.get(key), str) or config[key] in ("", "NOT_CONFIGURED"):
                raise ValueError("MODEL_PROVENANCE_NOT_CONFIGURED:" + key)
        if config.get("source_type") not in SOURCES:
            raise ValueError("MODEL_SOURCE_UNKNOWN")
        self.config = dict(config)
        self.source_type = config["source_type"]
        self.shapes = {k: ModelShape.from_config(config[k]) for k in ("OPEN", "CLOSED")}
        self.period = positive(config.get("symmetry_yaw_period_rad"), "symmetry_yaw_period_rad")
        order = 2 * math.pi / self.period
        if not math.isclose(order, round(order), abs_tol=1e-8) or not 1 <= order <= 8:
            raise ValueError("SYMMETRY_NOT_A_FINITE_YAW_GROUP")
        self.translation_uncertainty = positive(config.get("geometry_translation_uncertainty_m"), "geometry_translation_uncertainty_m", True)
        self.rotation_uncertainty = positive(config.get("geometry_rotation_uncertainty_rad"), "geometry_rotation_uncertainty_rad", True)

    def safety(self, opening):
        if opening in self.shapes:
            return self.shapes[opening].safety_vertices
        if opening == "TRANSITION":
            return np.vstack([s.safety_vertices for s in self.shapes.values()])
        raise ValueError("OPENING_UNKNOWN_OR_CONFLICT")

    def references(self, opening):
        if opening in self.shapes:
            s = self.shapes[opening]
            return s.bottom_reference, s.top_reference
        if opening == "TRANSITION":
            # Exact intermediate geometry unknown; union extremal references only.
            return (min(self.shapes.values(), key=lambda s: s.bottom_reference[2]).bottom_reference,
                    max(self.shapes.values(), key=lambda s: s.top_reference[2]).top_reference)
        raise ValueError("OPENING_UNKNOWN")


@dataclass(frozen=True)
class GrabEnvelope:
    nominal_footprint: np.ndarray
    conservative_footprint: np.ndarray
    bottom_nominal: float
    bottom_lower_bound: float
    bottom_upper_bound: float
    top_upper_bound: float
    pose_margin: float
    geometry_margin: float
    extrinsic_margin: float
    smear_margin: float


def convex_xy(vertices):
    p = np.asarray(vertices)[:, :2]
    return p[ConvexHull(p).vertices]


def envelope(model, pose, pose_covariance, opening, extrinsic_covariance, velocity,
             angular_velocity, frame_span, sigma, ellipse_sides=32):
    pose = rigid(pose)
    pc, ec = covariance(pose_covariance, 6), covariance(extrinsic_covariance, 6)
    span, k = positive(frame_span, "frame_span", True), positive(sigma, "sigma")
    if type(ellipse_sides) is not int or not 8 <= ellipse_sides <= 128:
        raise ValueError("INVALID_ELLIPSE_RESOLUTION")
    v, w = np.asarray(velocity), np.asarray(angular_velocity)
    if v.shape != (3,) or w.shape != (3,) or not np.isfinite(np.r_[v, w]).all():
        raise ValueError("TWIST_UNKNOWN")
    vertices = model.safety(opening)
    world = transform(vertices, pose)
    radius = float(np.linalg.norm(vertices, axis=1).max())
    # Use Loewner-safe bound on full pose cross covariance, then include rotation
    # displacement as an enclosing disk. This cannot be smaller than point marginals.
    def uncertainty(p):
        cross = np.linalg.norm(p[:3, 3:], 2)
        return p[:3, :3] + np.eye(3) * cross, p[3:, 3:] + np.eye(3) * cross
    pp, pr = uncertainty(pc)
    ep, er = uncertainty(ec)
    # ec rotation is common-frame; pc is right/body orientation. Eigen upper bound is invariant.
    ellipse_cov = 2 * (pp[:2, :2] + ep[:2, :2])  # Safe even when pose/extrinsic errors correlate.
    values, vectors = np.linalg.eigh(ellipse_cov)
    angles = np.arange(ellipse_sides) * 2 * math.pi / ellipse_sides
    circle = np.column_stack([np.cos(angles), np.sin(angles)])
    # Circumscribed, not inscribed: polygon must CONTAIN the k-sigma ellipse.
    ellipse = circle @ (vectors @ np.diag(k * np.sqrt(np.maximum(values, 0)))).T / math.cos(math.pi / ellipse_sides)
    rotation_margin = radius * k * (math.sqrt(max(np.linalg.eigvalsh(pr).max(), 0))
                                   + math.sqrt(max(np.linalg.eigvalsh(er).max(), 0)))
    geometry_margin = model.translation_uncertainty + radius * model.rotation_uncertainty
    smear = float(np.linalg.norm(v) * span / 2 + radius * np.linalg.norm(w) * span / 2)
    disk_margin = rotation_margin + geometry_margin + smear
    disk = circle * disk_margin / math.cos(math.pi / ellipse_sides)
    nominal = convex_xy(world)
    expanded = (nominal[:, None, None, :] + ellipse[None, :, None, :] + disk[None, None, :, :]).reshape(-1, 2)
    conservative = convex_xy(np.column_stack([expanded, np.zeros(len(expanded))]))
    bottom_ref, top_ref = model.references(opening)
    bottom = float(transform(bottom_ref[None, :], pose)[0, 2])
    # Rotating the reference alone may not produce the lowest tilted corner.
    # Bound ALL safety vertices relative to the reference as well.
    pose_z = k * math.sqrt(max(pp[2, 2], 0)) + radius * k * math.sqrt(max(np.linalg.eigvalsh(pr).max(), 0))
    extrinsic_z = k * math.sqrt(max(ep[2, 2], 0)) + radius * k * math.sqrt(max(np.linalg.eigvalsh(er).max(), 0))
    z_margin = pose_z + extrinsic_z + geometry_margin + smear
    return GrabEnvelope(nominal, conservative, bottom,
        min(bottom, float(world[:, 2].min())) - z_margin,
        max(bottom, float(world[:, 2].min())) + z_margin,
        max(float(transform(top_ref[None, :], pose)[0, 2]), float(world[:, 2].max())) + z_margin,
        pose_z, geometry_margin, extrinsic_z, smear)


class PitBoundaryModel:
    """Python view of existing SemanticMap.PitDefinition fields; no new map schema."""
    def __init__(self, pit, frame_id):
        if pit.get("config_state", pit.get("validity")) not in ("VALID", 4):
            raise ValueError("PIT_NOT_CONFIGURED")
        self.pit_id = pit["pit_id"]
        self.frame_id = frame_id
        self.polygon = Polygon(pit["polygon_map_m"])
        if not self.polygon.is_valid or self.polygon.is_empty or self.polygon.area <= 0:
            raise ValueError("INVALID_PIT_POLYGON")
        self.inset = positive(pit.get("safety_inset_m"), "safety_inset", True)
        self.top, self.bottom = float(pit["wall_top_z_m"]), float(pit["pit_bottom_z_m"])
        if not math.isfinite(self.top + self.bottom) or self.top <= self.bottom:
            raise ValueError("PIT_VERTICAL_RANGE_UNKNOWN")
        self.safe_polygon = self.polygon.buffer(-self.inset, join_style=1)
        if self.safe_polygon.is_empty:
            raise ValueError("EMPTY_SAFE_PIT")

    def signed_clearance(self, footprint):
        poly = Polygon(footprint)
        if not poly.is_valid or poly.is_empty:
            raise ValueError("INVALID_FOOTPRINT")
        if self.polygon.covers(poly):
            return float(poly.distance(self.polygon.boundary))
        # Crossing/outside has no safe positive distance. Conservative penetration
        # bound uses union bounding diameter, rather than claiming a sampled negative is exact.
        bounds = self.polygon.union(poly).bounds
        return -math.hypot(bounds[2] - bounds[0], bounds[3] - bounds[1])

    def clearance(self, geometry):
        nominal, conservative = Polygon(geometry.nominal_footprint), Polygon(geometry.conservative_footprint)
        raw = self.signed_clearance(geometry.nominal_footprint)
        safe = min(raw, self.signed_clearance(geometry.conservative_footprint))
        overlap = geometry.bottom_lower_bound <= self.top and geometry.top_upper_bound >= self.bottom
        safe_inside = self.polygon.covers(conservative) and conservative.distance(self.polygon.boundary) >= self.inset
        segments = [LineString([a, b]) for a, b in zip(self.polygon.exterior.coords,
                    list(self.polygon.exterior.coords)[1:])]
        return dict(pit_id=self.pit_id, raw_horizontal_clearance_m=raw, conservative_clearance_m=safe,
            inside_pit=self.polygon.covers(conservative), inside_safe_polygon=safe_inside,
            vertical_wall_overlap=overlap, wall_collision_relevant=overlap,
            wall_collision_risk=overlap and not safe_inside,
            nearest_boundary_index=int(np.argmin([s.distance(nominal) for s in segments])))
