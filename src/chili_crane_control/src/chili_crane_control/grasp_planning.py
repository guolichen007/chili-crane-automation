"""Explainable shadow proposals. Geofence v2 is referenced, never reimplemented."""
from dataclasses import dataclass
import hashlib
import json
import math
import numpy as np
from shapely import affinity
from shapely.geometry import Point, Polygon, box


@dataclass(frozen=True)
class GraspCaptureKernel:
    kernel_id: str
    provenance: str
    footprint_xy: tuple
    capture_depth_m: float
    fixed_yaw_rad: float

    @classmethod
    def configure(cls, config, source_type):
        k = config["kernel"]
        required = "FIELD_VALIDATED" if source_type == "PHYSICAL" else "SYNTHETIC_NON_PRODUCTION"
        if k.get("state") != required or k.get("kernel_id") in (None,"","NOT_CONFIGURED") or k.get("provenance") in (None,"","NOT_CONFIGURED"):
            raise ValueError("GRASP_KERNEL_NOT_CONFIGURED")
        if k.get("yaw_control_available") is not False or k.get("yaw_mode") not in ("FIXED_MECHANICAL", "YAW_CONSERVATIVE"):
            raise ValueError("YAW_EXECUTION_CAPABILITY_NOT_CONFIGURED")
        p = Polygon(k["footprint_xy_m"])
        depth, yaw = k["capture_depth_m"], k["fixed_yaw_rad"]
        if (not p.is_valid or p.is_empty or not p.covers(Point(0,0)) or not math.isfinite(depth)
                or depth <= 0 or not math.isfinite(yaw)):
            raise ValueError("GRASP_KERNEL_GEOMETRY_INVALID")
        if k["yaw_mode"] == "YAW_CONSERVATIVE":
            # Caller must supply a proven all-yaw outer bound; no optimized yaw exists.
            if k.get("all_yaw_bound_verified") is not True:
                raise ValueError("YAW_BOUND_NOT_VERIFIED")
        return cls(k["kernel_id"], k["provenance"], tuple(map(tuple,k["footprint_xy_m"])),depth,yaw)

    def footprint(self, xy):
        p = affinity.rotate(Polygon(self.footprint_xy),self.fixed_yaw_rad,origin=(0,0),use_radians=True)
        return affinity.translate(p,xy[0],xy[1])


@dataclass(frozen=True)
class GraspCandidate:
    candidate_id: str
    target_xy: tuple
    footprint_xy: tuple
    accepted: bool
    rejection_reasons: tuple
    observed_area_fraction: float
    unknown_area_fraction: float
    surface_nominal_m: float
    surface_lower_m: float
    surface_upper_m: float
    roughness_m: float
    slope_rad: float
    material_volume_proxy_m3: float
    material_volume_proxy_lower_bound_m3: float
    wall_clearance_m: float
    recent_grab_penalty: float = 0.
    valid_for_control: bool = False


@dataclass(frozen=True)
class GraspProposal:
    proposal_id: str
    snapshot_id: str
    generation: int
    measurement_epoch: float
    candidates: tuple
    selected: GraspCandidate | None
    reason: str
    context: object
    geofence_provenance: dict
    kernel_id: str
    valid_for_control: bool = False


class GraspPlanner:
    def __init__(self, config, geofence):
        self.config, self.geofence = json.loads(json.dumps(config,allow_nan=False)), geofence
        self.config_hash = hashlib.sha256(json.dumps(self.config,sort_keys=True).encode()).hexdigest()
        self.kernel = GraspCaptureKernel.configure(config,config["source_type"])
        if config["source_type"] != geofence.config["source_type"] or config["pit_id"] != geofence.config["pit_id"]:
            raise ValueError("PLANNER_GEOFENCE_SOURCE_PIT_MISMATCH")
        if any(config.get(k) is not False for k in ("physical_output_enabled","automatic_control_enabled","camera_control_authority")):
            raise ValueError("SHADOW_FLAGS_REQUIRED")
        if type(config.get("maximum_candidates")) is not int or not 1 <= config["maximum_candidates"] <= 4096:
            raise ValueError("CANDIDATE_CAPACITY_NOT_CONFIGURED")
        p = config["planning"]
        for key in ("minimum_observed_fraction","maximum_unknown_fraction","maximum_slope_rad","maximum_roughness_m","wall_margin_m"):
            if type(p.get(key)) not in (int,float) or not math.isfinite(p[key]) or p[key] < 0:
                raise ValueError("PLANNING_POLICY_NOT_CONFIGURED")
        if not 0 < p["minimum_observed_fraction"] <= 1 or not 0 <= p["maximum_unknown_fraction"] < 1:
            raise ValueError("COVERAGE_POLICY_INVALID")

    def _empty(self, snapshot, reason):
        return GraspProposal(f"{snapshot.snapshot_id}:none",snapshot.snapshot_id,snapshot.generation,snapshot.window_end,
            (),None,reason,snapshot.context,self.geofence.provenance(),self.kernel.kernel_id)

    def plan(self, snapshot, now, expected_context, expected_generation, recent=None):
        """A caller MUST bind latest generation and independently current spatial context."""
        g, p = self.geofence, self.config["planning"]
        if (snapshot.context is None or expected_context is None or snapshot.context.key != expected_context.key
                or snapshot.generation != expected_generation or snapshot.context.map_id != g.semantic["map_id"]
                or snapshot.context.semantic_version != g.semantic["semantic_version"] or snapshot.context.pit_id != g.config["pit_id"]
                or snapshot.context.target_frame != g.semantic["frame_id"] or snapshot.context.source_type != self.config["source_type"]):
            return self._empty(snapshot,"PROVENANCE_OR_GENERATION_MISMATCH")
        if (snapshot.validity not in ("VALID","DEGRADED") or not math.isfinite(now)
                or not 0 <= now-snapshot.window_end <= self.config["surface_ttl_sec"]):
            return self._empty(snapshot,"SURFACE_STALE")
        # Context time is checked independently; publication of an old snapshot cannot renew it.
        try:
            expected_context.validate(self.config,g.semantic,expected_context.source_frame,
                                      expected_context.measurement_epoch,now)
        except (ValueError,TypeError,KeyError):
            return self._empty(snapshot,"SPATIAL_CONTEXT_STALE_OR_INVALID")
        if (snapshot.config_sha256 != self.config_hash or snapshot.resolution != self.config["resolution_m"]
                or len(snapshot.cells) != snapshot.width*snapshot.height or len(snapshot.cells) > self.config["maximum_grid_cells"]
                or any(c.index != i for i,c in enumerate(snapshot.cells))
                or tuple(snapshot.origin) != tuple(g.polygon.bounds[:2])
                or snapshot.width != math.ceil((g.polygon.bounds[2]-snapshot.origin[0])/snapshot.resolution)
                or snapshot.height != math.ceil((g.polygon.bounds[3]-snapshot.origin[1])/snapshot.resolution)
                or not 0 < snapshot.window_start <= snapshot.window_end
                or abs(snapshot.context.measurement_epoch-snapshot.window_end) > self.config['epoch_tolerance_sec']):
            return self._empty(snapshot,"GRID_CAPACITY_OR_LAYOUT_INVALID")
        candidates = []
        distance_field = None
        if p.get("distance_acceleration_enabled") is True:
            from .distance_acceleration import RejectionDistanceField
            distance_field = RejectionDistanceField(snapshot,g.polygon)
        radius=max(math.hypot(x,y) for x,y in self.kernel.footprint_xy)
        for cell in snapshot.cells:
            # Generate centers ONLY inside admissible domain; exact checks still follow.
            x = snapshot.origin[0]+(cell.index%snapshot.width+.5)*snapshot.resolution
            y = snapshot.origin[1]+(cell.index//snapshot.width+.5)*snapshot.resolution
            xy = (x,y)
            if not cell.observed or not g.safe_region.covers(Point(xy)) or not g.track_inside(np.array(xy)):
                continue
            if len(candidates) >= self.config["maximum_candidates"]:
                break
            fp = self.kernel.footprint(xy)
            reasons=[]
            if distance_field is not None and distance_field.definitely_reject(xy,radius+p["wall_margin_m"]):
                reasons.append("DISTANCE_FIELD_FAST_REJECT")
            clearance = float(g.polygon.boundary.distance(fp)) if g.polygon.covers(fp) else 0.
            pit = next(v for v in g.semantic["pits"] if v["pit_id"] == self.config["pit_id"])
            if not g.polygon.covers(fp) or clearance < p["wall_margin_m"] + pit["safety_inset_m"]:
                reasons.append("WALL_OR_PIT_GEOMETRY")
            if g.polygon.boundary.distance(Point(xy)) < g.margin:
                reasons.append("GEOFENCE_EXACT_MARGIN")
            # Intersection-area quadrature on real grid cells, not center-point coverage.
            r=snapshot.resolution
            minx,miny,maxx,maxy=fp.bounds
            col0=max(0,math.floor((minx-snapshot.origin[0])/r))
            row0=max(0,math.floor((miny-snapshot.origin[1])/r))
            col1=min(snapshot.width-1,math.floor((maxx-snapshot.origin[0])/r))
            row1=min(snapshot.height-1,math.floor((maxy-snapshot.origin[1])/r))
            supported=[]
            for row in range(row0,row1+1):
                for col in range(col0,col1+1):
                    index=row*snapshot.width+col
                    current=snapshot.cells[index]
                    area=fp.intersection(box(snapshot.origin[0]+col*r,snapshot.origin[1]+row*r,
                        snapshot.origin[0]+(col+1)*r,snapshot.origin[1]+(row+1)*r)).area
                    if area <= 1e-12:
                        continue
                    finite=all(math.isfinite(v) for v in (current.z_median,current.z_lower_bound,current.z_upper_bound,
                                                         current.roughness,current.slope,current.height_uncertainty))
                    fresh=0 <= now-current.last_observed_epoch <= self.config["surface_ttl_sec"]
                    if (current.observed and fresh and finite and current.z_lower_bound <= current.z_median <= current.z_upper_bound
                            and current.height_uncertainty <= self.config["max_height_dispersion_m"]
                            and current.sample_count >= self.config['minimum_cell_samples']
                            and current.frame_count >= self.config['minimum_cell_frames']
                            and current.sensor_support_mask in (1,2,3) and current.validity in ('VALID','DEGRADED')
                            and snapshot.window_start <= current.last_observed_epoch <= snapshot.window_end
                            and current.neighborhood_support >= 3
                            and current.neighborhood_coverage >= self.config['minimum_neighborhood_coverage']):
                        supported.append((current,area))
            observed_area=sum(a for _,a in supported)
            coverage=min(1.,observed_area/fp.area)
            unknown=max(0.,1.-coverage)
            if coverage + 1e-10 < p["minimum_observed_fraction"] or unknown > p["maximum_unknown_fraction"]+1e-10:
                reasons.append("UNKNOWN_OR_INSUFFICIENT_OBSERVED_COVERAGE")
            heights=[c for c,_ in supported]
            slope=max((c.slope for c in heights),default=math.inf)
            roughness=max((c.roughness for c in heights),default=math.inf)
            if slope > p["maximum_slope_rad"]:
                reasons.append("EXCESS_SLOPE")
            if roughness > p["maximum_roughness_m"]:
                reasons.append("EXCESS_ROUGHNESS")
            nominal=sum(c.z_median*a for c,a in supported)/observed_area if observed_area else 0.
            lower=min((c.z_lower_bound for c in heights),default=0.)
            upper=max((c.z_upper_bound for c in heights),default=0.)
            volume=sum(a*min(self.kernel.capture_depth_m,max(0.,c.z_median-pit["pit_bottom_z_m"])) for c,a in supported)
            volume_lower=sum(a*min(self.kernel.capture_depth_m,max(0.,c.z_lower_bound-pit["pit_bottom_z_m"])) for c,a in supported)
            penalty=recent.penalty(fp,now,snapshot.context.key) if recent is not None else 0.
            candidates.append(GraspCandidate(f"{snapshot.snapshot_id}:{cell.index}",xy,tuple(fp.exterior.coords)[:-1],
                not reasons,tuple(sorted(set(reasons))),coverage,unknown,nominal,lower,upper,roughness,slope,
                volume,volume_lower,clearance,penalty))
        accepted=[c for c in candidates if c.accepted]
        accepted.sort(key=lambda c:(-c.material_volume_proxy_lower_bound_m3,-c.observed_area_fraction,
                                    c.roughness_m,c.slope_rad,c.recent_grab_penalty,c.candidate_id))
        selected=accepted[0] if accepted else None
        return GraspProposal(f"{snapshot.snapshot_id}:proposal",snapshot.snapshot_id,snapshot.generation,snapshot.window_end,
            tuple(candidates),selected,"SHADOW_NOT_FOR_CONTROL" if selected else "NO_HARD_GATE_CANDIDATE",
            snapshot.context,g.provenance(),self.kernel.kernel_id)
