"""Observed-only bounded 2.5D evidence. No TF lookup, interpolation or control."""
from collections import deque
from dataclasses import dataclass, replace
import hashlib
import json
import math
import numpy as np
from shapely.geometry import Point, Polygon, box


def number(value, name, positive=False):
    if type(value) not in (int, float) or not math.isfinite(value) or value < 0 or positive and value == 0:
        raise ValueError(name + "_NOT_CONFIGURED")
    return float(value)


def identity(value):
    return isinstance(value, str) and value not in ("", "NOT_CONFIGURED", "UNKNOWN")


@dataclass(frozen=True)
class PlanningSpatialContext:
    source_frame: str
    target_frame: str
    map_id: str
    semantic_version: str
    pit_id: str
    transform_id: str
    calibration_id: str
    transform_state: str
    source_type: str
    measurement_epoch: float
    uncertainty_m: float
    transform: tuple
    sensor_calibration_ids: tuple
    platform_motion_stable: bool
    motion_evidence_id: str
    field_extrinsic_id: str = "NOT_CONFIGURED"
    axis_to_map_id: str = "NOT_CONFIGURED"
    crane_pose_evidence_id: str = "NOT_CONFIGURED"

    @property
    def key(self):
        # Measurement time changes, spatial identity/covariance must not drift silently.
        d = self.__dict__.copy()
        d.pop("measurement_epoch")
        return hashlib.sha256(json.dumps(d, sort_keys=True, allow_nan=False).encode()).hexdigest()

    def validate(self, config, semantic, cloud_frame, epoch, now):
        for key in ("source_frame", "target_frame", "map_id", "semantic_version", "pit_id",
                    "transform_id", "calibration_id", "motion_evidence_id"):
            if not identity(getattr(self, key)):
                raise ValueError("SPATIAL_PROVENANCE_NOT_CONFIGURED")
        if (self.source_frame != cloud_frame or self.target_frame != semantic["frame_id"]
                or self.map_id != semantic["map_id"] or self.semantic_version != semantic["semantic_version"]
                or self.pit_id != config["pit_id"] or self.source_type != config["source_type"]):
            raise ValueError("SPATIAL_MAP_FRAME_SOURCE_MISMATCH")
        required = "FIELD_VALIDATED" if self.source_type == "PHYSICAL" else "SYNTHETIC_VALIDATED"
        if self.transform_state != required:
            raise ValueError("PIT_SURFACE_SPATIAL_ELIGIBILITY_FALSE")
        if self.source_type == "PHYSICAL" and any(not identity(v) for v in
                (self.field_extrinsic_id, self.axis_to_map_id, self.crane_pose_evidence_id)):
            raise ValueError("FIELD_SPATIAL_CHAIN_NOT_CONFIGURED")
        if (self.source_type not in ("SYNTHETIC", "REPLAY", "SIMULATED", "PHYSICAL")
                or len(self.sensor_calibration_ids) != 2 or not all(identity(v) for v in self.sensor_calibration_ids)):
            raise ValueError("SENSOR_PROVENANCE_UNKNOWN")
        if self.platform_motion_stable is not True:
            raise ValueError("PLATFORM_MOVING_NO_VERIFIED_COMPENSATION")
        number(self.uncertainty_m, "TRANSFORM_UNCERTAINTY")
        if not all(math.isfinite(v) and v > 0 for v in (epoch, now, self.measurement_epoch)):
            raise ValueError("MEASUREMENT_EPOCH_UNKNOWN")
        if abs(self.measurement_epoch - epoch) > config["epoch_tolerance_sec"]:
            raise ValueError("SPATIAL_MEASUREMENT_EPOCH_MISMATCH")
        if not 0 <= now - epoch <= config["frame_ttl_sec"]:
            raise ValueError("FRAME_STALE_OR_FUTURE")
        t = np.asarray(self.transform, dtype=float).reshape(4, 4)
        r = t[:3, :3]
        if (not np.isfinite(t).all() or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-9)
                or not np.allclose(r.T @ r, np.eye(3), atol=1e-8) or not np.isclose(np.linalg.det(r), 1)):
            raise ValueError("SPATIAL_TRANSFORM_INVALID")
        return t


@dataclass(frozen=True)
class SurfaceCell:
    index: int
    observed: bool = False
    sample_count: int = 0
    frame_count: int = 0
    sensor_support_mask: int = 0
    last_observed_epoch: float = 0.
    z_median: float = math.nan
    z_lower_bound: float = math.nan
    z_upper_bound: float = math.nan
    roughness: float = math.nan
    slope: float = math.nan
    height_uncertainty: float = math.nan
    neighborhood_coverage: float = 0.
    neighborhood_support: int = 0
    validity: str = "UNKNOWN"


@dataclass(frozen=True)
class SurfaceSnapshot:
    snapshot_id: str
    generation: int
    window_start: float
    window_end: float
    context: PlanningSpatialContext | None
    cells: tuple
    origin: tuple
    resolution: float
    width: int
    height: int
    validity: str
    reason: str
    config_sha256: str
    valid_for_control: bool = False
    last_disturbance_id: str = ''
    disturbance_counter: int = 0


class PitSurfaceGrid:
    def __init__(self, config, semantic):
        self.config = json.loads(json.dumps(config, allow_nan=False))
        self.semantic = json.loads(json.dumps(semantic, allow_nan=False))
        c, s = self.config, self.semantic
        if c.get("config_state") != "VALID" or s.get("config_state") != "VALID" or s.get("schema_version") != 1:
            raise ValueError("PIT_SURFACE_NOT_CONFIGURED")
        if not identity(c.get("config_version")) or c.get("source_type") not in ("PHYSICAL", "REPLAY", "SIMULATED", "SYNTHETIC"):
            raise ValueError("SOURCE_CONFIG_NOT_CONFIGURED")
        if any(c.get(k) is not False for k in ("physical_output_enabled", "automatic_control_enabled", "camera_control_authority")):
            raise ValueError("SHADOW_FLAGS_REQUIRED")
        for key in ("resolution_m", "frame_ttl_sec", "surface_ttl_sec", "window_sec", "maximum_frame_span_sec",
                    "sensor_uncertainty_m", "single_sensor_extra_uncertainty_m", "mad_scale", "max_height_dispersion_m"):
            number(c.get(key), key, True)
        for key in ("epoch_tolerance_sec",):
            number(c.get(key), key)
        if not 0 < c.get("minimum_neighborhood_coverage", 0) <= 1:
            raise ValueError("NEIGHBORHOOD_POLICY_NOT_CONFIGURED")
        for key, upper in (("maximum_input_points", 2000000), ("maximum_grid_cells", 100000),
                           ("maximum_window_frames", 64), ("minimum_cell_samples", 10000), ("minimum_cell_frames", 64)):
            if type(c.get(key)) is not int or not 1 <= c[key] <= upper:
                raise ValueError(key + "_NOT_CONFIGURED")
        if c["minimum_cell_frames"] > c["maximum_window_frames"]:
            raise ValueError("FRAME_SUPPORT_POLICY_INVALID")
        pits = [p for p in s["pits"] if p["pit_id"] == c["pit_id"]]
        if len(pits) != 1:
            raise ValueError("PIT_ID_UNKNOWN")
        self.pit = pits[0]
        self.polygon = Polygon(self.pit["polygon_map_m"])
        if not self.polygon.is_valid or self.polygon.is_empty or self.pit["pit_bottom_z_m"] >= self.pit["wall_top_z_m"]:
            raise ValueError("PIT_GEOMETRY_INVALID")
        x0, y0, x1, y1 = self.polygon.bounds
        self.origin = (x0, y0)
        self.resolution = c["resolution_m"]
        self.width, self.height = math.ceil((x1-x0)/self.resolution), math.ceil((y1-y0)/self.resolution)
        if self.width * self.height > c["maximum_grid_cells"]:
            raise ValueError("GRID_CAPACITY_EXCEEDED")
        self.frames = deque(maxlen=c["maximum_window_frames"])
        self.generation, self.last_epoch, self.last_counter = 0, 0., -1
        self.context, self.reason = None, "NO_OBSERVATIONS"
        self.last_disturbance_id,self.disturbance_counter='',0
        self.invalidated_at = np.zeros(self.width*self.height)
        self.config_hash = hashlib.sha256(json.dumps(c, sort_keys=True).encode()).hexdigest()

    def cell_polygon(self, index):
        x, y = self.center(index)
        r = self.resolution / 2
        return box(x-r, y-r, x+r, y+r)

    def center(self, index):
        return (self.origin[0]+(index % self.width+.5)*self.resolution,
                self.origin[1]+(index // self.width+.5)*self.resolution)

    def clear(self, reason):
        self.frames.clear()
        self.context = None
        self.generation += 1
        self.reason = reason

    def ingest(self, xyz, sensor_ids, frame, context, now):
        """Consume once. Caller supplies canonical epoch/source counter; no arrival renewal."""
        try:
            epoch, counter = frame["epoch"], frame["source_counter"]
            t = context.validate(self.config, self.semantic, frame["frame_id"], epoch, now)
            if type(counter) is not int or counter <= self.last_counter or epoch <= self.last_epoch:
                raise ValueError("DUPLICATE_OR_REGRESSING_FRAME")
            if not 0 <= frame["span_sec"] <= self.config["maximum_frame_span_sec"]:
                raise ValueError("FRAME_SPAN_INVALID")
            if frame.get("source_type") != context.source_type or frame.get("calibration_id") != context.calibration_id:
                raise ValueError("SENSOR_CALIBRATION_SOURCE_MISMATCH")
            p, sensors = np.asarray(xyz, dtype=float), np.asarray(sensor_ids)
            if (p.ndim != 2 or p.shape[1] != 3 or not 0 < len(p) <= self.config["maximum_input_points"]
                    or sensors.shape != (len(p),) or not np.isfinite(p).all() or not np.isin(sensors, [0, 1]).all()):
                raise ValueError("POINT_CAPACITY_OR_PROVENANCE_INVALID")
            if self.context is not None and self.context.key != context.key:
                self.clear("SPATIAL_GENERATION_CHANGED")
            p = p @ t[:3, :3].T + t[:3, 3]
            # No hull clipping can create synthetic measurements.
            keep = np.array([self.polygon.covers(Point(v[:2])) for v in p])
            keep &= (p[:, 2] >= self.pit["pit_bottom_z_m"]) & (p[:, 2] <= self.pit["wall_top_z_m"])
            p, sensors = p[keep], sensors[keep]
            ij = np.floor((p[:, :2]-self.origin)/self.resolution).astype(int)
            keep = (ij[:, 0] >= 0) & (ij[:, 0] < self.width) & (ij[:, 1] >= 0) & (ij[:, 1] < self.height)
            p, sensors, ij = p[keep], sensors[keep], ij[keep]
            indices = ij[:, 1]*self.width + ij[:, 0]
            keep = epoch > self.invalidated_at[indices]
            p,sensors,indices = p[keep],sensors[keep],indices[keep]
            observations = {}
            for index in np.unique(indices):
                values, support = p[indices == index, 2], sensors[indices == index]
                median = float(np.median(values))
                mad = float(np.median(np.abs(values-median)))
                mask = sum(1 << int(v) for v in np.unique(support))
                observations[int(index)] = (median, mad, len(values), mask)
            self.frames.append((epoch, observations, context.uncertainty_m))
            self.context, self.last_epoch, self.last_counter = context, epoch, counter
            self.generation += 1
            self.reason = "SHADOW_OBSERVED_ONLY"
            self.expire(now)
            return True
        except (ValueError, TypeError, KeyError, AttributeError) as exc:
            self.clear(str(exc))
            return False

    def expire(self, now):
        changed = False
        while self.frames and (now-self.frames[0][0] > min(self.config["window_sec"], self.config["surface_ttl_sec"])
                               or now < self.frames[0][0]):
            self.frames.popleft()
            changed = True
        if changed:
            self.generation += 1
        if not self.frames:
            self.reason = "SURFACE_STALE"

    def invalidate(self, polygon, event_epoch):
        """Remove old samples in disturbed intersecting cells, never just reduce score."""
        indices = {i for i in range(self.width*self.height) if polygon.intersects(self.cell_polygon(i))}
        for i in indices:
            self.invalidated_at[i] = max(self.invalidated_at[i],event_epoch)
        self.frames = deque(((e, {i: v for i, v in obs.items() if i not in indices or e > event_epoch}, u)
                             for e, obs, u in self.frames), maxlen=self.config["maximum_window_frames"])
        self.generation += 1

    def snapshot(self, now):
        self.expire(now)
        c, cells = self.config, []
        for index in range(self.width*self.height):
            rows = [(e, obs[index], u) for e, obs, u in self.frames if index in obs]
            count = sum(r[1][2] for r in rows)
            if len(rows) < c["minimum_cell_frames"] or count < c["minimum_cell_samples"]:
                cells.append(SurfaceCell(index))
                continue
            heights = np.array([r[1][0] for r in rows])
            z = float(np.median(heights))
            dispersion = max(float(np.median(np.abs(heights-z))), max(r[1][1] for r in rows))
            mask = int(np.bitwise_or.reduce([r[1][3] for r in rows]))
            single_frame_support = any(r[1][3] != 3 for r in rows)
            uncertainty = (c["mad_scale"]*dispersion + c["sensor_uncertainty_m"] + max(r[2] for r in rows)
                           + (c["single_sensor_extra_uncertainty_m"] if single_frame_support else 0))
            # Include temporal range, rather than claiming a median is a maximum surface.
            lower, upper = float(min(heights)-uncertainty), float(max(heights)+uncertainty)
            valid = dispersion <= c["max_height_dispersion_m"]
            cells.append(SurfaceCell(index, True, count, len(rows), mask, max(r[0] for r in rows), z,
                lower, upper, dispersion, math.nan, uncertainty, validity="VALID" if valid and not single_frame_support else "DEGRADED"))
        # Fit local plane only to actual observed neighbors; no fill of holes.
        original = tuple(cells)
        for i, cell in enumerate(original):
            if not cell.observed:
                continue
            row, col = divmod(i, self.width)
            neighbors = [original[r*self.width+k] for r in range(max(0, row-1), min(self.height, row+2))
                         for k in range(max(0, col-1), min(self.width, col+2))]
            observed = [n for n in neighbors if n.observed and n.height_uncertainty <= c["max_height_dispersion_m"]]
            coverage = len(observed)/9  # Boundary absence remains unknown, not an invented neighbor.
            slope, residual = math.nan, cell.roughness
            if len(observed) >= 3 and coverage >= c["minimum_neighborhood_coverage"]:
                a = np.array([[*self.center(n.index), 1] for n in observed])
                z = np.array([n.z_median for n in observed])
                if np.linalg.matrix_rank(a) == 3:
                    weights = np.ones(len(z))
                    for _ in range(3):
                        beta = np.linalg.lstsq(a*weights[:, None]**.5, z*weights**.5, rcond=None)[0]
                        errors = z-a@beta
                        scale = max(c["sensor_uncertainty_m"], 1.4826*float(np.median(np.abs(errors))))
                        weights = np.minimum(1., 1.5*scale/np.maximum(np.abs(errors), 1e-12))
                    slope = float(math.atan(np.linalg.norm(beta[:2])))
                    residual = max(residual, float(np.median(np.abs(errors))))
            cells[i] = replace(cell, slope=slope, roughness=residual, neighborhood_coverage=coverage,
                               neighborhood_support=len(observed),
                               validity=cell.validity if math.isfinite(slope) else "DEGRADED")
        starts = [e for e, _, _ in self.frames]
        end = max(starts, default=self.last_epoch)
        snapshot_id = f"{self.config_hash[:12]}:{self.generation}:{self.context.key[:12] if self.context else 'unknown'}"
        return SurfaceSnapshot(snapshot_id, self.generation, min(starts, default=end), end, self.context,
            tuple(cells), self.origin, self.resolution, self.width, self.height,
            "DEGRADED" if starts else "STALE", self.reason, self.config_hash,False,
            self.last_disturbance_id,self.disturbance_counter)
