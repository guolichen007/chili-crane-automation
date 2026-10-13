"""Independent masks: never modify Grab pose or classify masked points as material."""
from dataclasses import dataclass
import math
from time import perf_counter
import numpy as np
from shapely.geometry import Point, Polygon
from .pit_surface import PitSurfaceGrid, number, identity


@dataclass(frozen=True)
class GrabMask:
    epoch: float
    frame_id: str
    map_id: str
    semantic_version: str
    pit_id: str
    calibration_id: str
    source_type: str
    tracking_state: str
    measurement_accepted: bool
    motion_stable: bool
    footprint_xy: tuple
    bottom_lower_bound_m: float
    top_upper_bound_m: float
    evidence_age_sec: float
    extrinsic_state: str = 'NOT_CONFIGURED'

    def validate(self, context, now, ttl):
        if (self.frame_id != context.target_frame or self.map_id != context.map_id
                or self.semantic_version != context.semantic_version or self.pit_id != context.pit_id
                or self.calibration_id != context.field_extrinsic_id or self.source_type != context.source_type):
            raise ValueError("GRAB_MASK_PROVENANCE_MISMATCH")
        required='FIELD_VALIDATED' if context.source_type=='PHYSICAL' else 'SYNTHETIC_VALIDATED'
        if self.extrinsic_state != required:
            raise ValueError('GRAB_MASK_SPATIAL_ELIGIBILITY_FALSE')
        if not identity(self.calibration_id) or not math.isfinite(self.epoch) or not 0 <= now-self.epoch <= ttl:
            raise ValueError("GRAB_MASK_STALE")
        if not math.isfinite(self.evidence_age_sec) or not 0 <= self.evidence_age_sec <= ttl:
            raise ValueError("GRAB_MASK_STALE")
        if self.tracking_state != "TRACKED" or self.measurement_accepted is not True or self.motion_stable is not True:
            raise ValueError("GRAB_MASK_UNTRACKED_OR_MOVING")
        if (not all(math.isfinite(v) for v in (self.bottom_lower_bound_m, self.top_upper_bound_m))
                or self.bottom_lower_bound_m >= self.top_upper_bound_m):
            raise ValueError("GRAB_MASK_VERTICAL_BOUNDS_UNKNOWN")
        p = Polygon(self.footprint_xy)
        if not p.is_valid or p.is_empty or p.area <= 0:
            raise ValueError("GRAB_MASK_FOOTPRINT_INVALID")
        return p


class SurfaceMasks:
    def __init__(self, config):
        self.dilation = number(config.get("dynamic_mask_dilation_m"), "MASK_DILATION")
        if config.get("static_masks_state") != "VALID" or not isinstance(config.get("static_masks"), list):
            raise ValueError("STATIC_SELF_MASKS_NOT_CONFIGURED")
        if len(config["static_masks"]) > 128:
            raise ValueError("STATIC_MASK_CAPACITY_EXCEEDED")
        self.static = []
        for mask in config["static_masks"]:
            p = Polygon(mask["footprint_xy_m"])
            low, high = mask["bottom_m"], mask["top_m"]
            if (not identity(mask.get("mask_id")) or not p.is_valid or p.is_empty
                    or not all(math.isfinite(v) for v in (low,high)) or low >= high):
                raise ValueError("STATIC_SELF_MASK_INVALID")
            self.static.append((p, low, high))

    def material_indices(self, map_xyz, grab_polygon, grab):
        # Inflate chord-approximated buffer to include the requested Euclidean disk.
        p = grab_polygon.buffer(self.dilation/math.cos(math.pi/64), resolution=16)
        masks = self.static + [(p, grab.bottom_lower_bound_m-self.dilation, grab.top_upper_bound_m+self.dilation)]
        keep = np.ones(len(map_xyz), dtype=bool)
        for polygon, bottom, top in masks:
            candidates = np.flatnonzero(keep & (map_xyz[:,2] >= bottom) & (map_xyz[:,2] <= top))
            for i in candidates:
                if polygon.covers(Point(map_xyz[i,:2])):
                    keep[i] = False
        return keep


class PitSurfaceBuilder:
    """Gate, transform, mask, then accumulate once in the authoritative grid."""
    def __init__(self, config, semantic):
        self.grid = PitSurfaceGrid(config, semantic)
        self.masks = SurfaceMasks(config)
        self.masked_count = 0

    def ingest(self, xyz, sensors, frame, context, grab, now):
        try:
            if grab is None:
                raise ValueError("GRAB_MASK_MISSING")
            t = context.validate(self.grid.config,self.grid.semantic,frame["frame_id"],frame["epoch"],now)
            polygon = grab.validate(context,now,self.grid.config["frame_ttl_sec"])
            if abs(frame["epoch"]-grab.epoch) > self.grid.config["maximum_grab_mask_time_skew_sec"]:
                raise ValueError("GRAB_MASK_TIME_ALIGNMENT_REJECTED")
            p = np.asarray(xyz, dtype=float)
            if p.ndim != 2 or p.shape[1] != 3 or not 0 < len(p) <= self.grid.config["maximum_input_points"]:
                raise ValueError("POINT_CAPACITY_INVALID")
            if (not np.isfinite(p).all() or np.asarray(sensors).shape != (len(p),)
                    or not np.isin(sensors,[0,1]).all()):
                raise ValueError("POINT_INPUT_INVALID")
            tick = perf_counter()
            map_xyz = p @ t[:3,:3].T + t[:3,3]
            transform_ms = (perf_counter()-tick)*1000
            tick = perf_counter()
            keep = self.masks.material_indices(map_xyz,polygon,grab)
            self.masked_count = int((~keep).sum())
            if not keep.any():
                raise ValueError("ALL_POINTS_MASKED_NO_MATERIAL_EVIDENCE")
            # Grid performs its own identity/time/capacity checks on the original frame.
            mask_ms = (perf_counter()-tick)*1000
            result = self.grid.ingest(p[keep],np.asarray(sensors)[keep],frame,context,now,map_xyz[keep])
            self.grid.stage_ms.update(transform=transform_ms,grab_mask=mask_ms)
            self.grid.metrics['input_points'] = len(p)
            return result
        except (ValueError,TypeError,KeyError,AttributeError) as exc:
            self.grid.clear(str(exc))
            return False
