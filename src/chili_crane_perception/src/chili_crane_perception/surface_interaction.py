"""Phase2C local lifecycle adapter around the FROZEN Phase2B grid methods."""
import math
from dataclasses import replace
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union
from chili_crane_control.grasp_interaction import RefreshedSurface
from chili_crane_control.interaction_contracts import Evidence


class SurfaceInteractionLifecycle:
    def __init__(self,grid,config):
        self.grid,self.config=grid,dict(config)
        self.binding,self.frozen_snapshot=None,None
        self.affected=set();self.interaction_id='';self.interaction_epoch=0.
        self.observation_after=None;self.ready=False
        self.generation_ready=False
        self.invalidated_generation=0;self.last_event_counter=-1

    def bind(self,binding,snapshot,now):
        if self.binding is not None and self.binding.cycle_id==binding.cycle_id:
            if self.binding.binding_digest_sha256!=binding.binding_digest_sha256:
                raise ValueError('SURFACE_BINDING_SWAP_REJECTED')
            return
        if self.interaction_id and not self.generation_ready:
            raise ValueError('PREVIOUS_INTERACTION_SURFACE_NOT_REFRESHED')
        ctx=snapshot.context
        if (ctx is None or snapshot.surface_session_id!=self.grid.surface_session_id
                or binding.surface_session_id!=snapshot.surface_session_id
                or binding.surface_snapshot_id!=snapshot.snapshot_id or binding.surface_generation!=snapshot.generation
                or binding.map_id!=ctx.map_id or binding.semantic_version!=ctx.semantic_version or binding.pit_id!=ctx.pit_id
                or binding.spatial_calibration_id!=ctx.calibration_id
                or binding.evidence.source_type!=ctx.source_type
                or not binding.evidence.fresh(now,self.config['evidence_ttl_sec'],self.config['source_type'])):
            raise ValueError('FROZEN_SURFACE_BINDING_PROVENANCE_INVALID')
        self.binding,self.frozen_snapshot=binding,snapshot
        self.affected=set();self.interaction_id='';self.interaction_epoch=0.
        self.observation_after=None;self.ready=False;self.last_event_counter=-1
        self.generation_ready=False

    def interact(self,interaction_id,event_epoch,event_counter,footprints,evidence,now):
        b,c=self.binding,self.config
        if b is None or self.grid.context is None:raise ValueError('INTERACTION_BINDING_OR_CONTEXT_MISSING')
        if (not evidence.fresh(now,c['evidence_ttl_sec'],c['source_type'])
                or evidence.calibration_id!=b.spatial_calibration_id or evidence.frame_id!=b.evidence.frame_id
                or not math.isfinite(event_epoch) or not 0 < event_epoch <= evidence.epoch <= now
                or type(event_counter) is not int or event_counter<0 or event_counter>evidence.counter
                or interaction_id!=b.binding_id+':interaction'
                or self.grid.surface_session_id!=b.surface_session_id
                or self.grid.context.calibration_id!=b.spatial_calibration_id):
            raise ValueError('INTERACTION_EXECUTION_EVIDENCE_REJECTED')
        if c['source_type']=='PHYSICAL' and c.get('config_state')!='FIELD_VALIDATED':
            raise ValueError('PHYSICAL_INTERACTION_NOT_CONFIGURED')
        if (self.interaction_id and (interaction_id!=self.interaction_id or event_epoch!=self.interaction_epoch)
                or evidence.counter<self.last_event_counter):
            raise ValueError('INTERACTION_IDENTITY_OR_SEQUENCE_CONFLICT')
        if len(footprints)>c['maximum_swept_footprints']:raise ValueError('SWEPT_CAPACITY_EXCEEDED')
        polygons=[]
        for points in (b.capture_footprint,)+tuple(footprints):
            if not 3 <= len(points) <= 256 or not np.isfinite(np.asarray(points,dtype=float)).all():
                raise ValueError('INTERACTION_FOOTPRINT_INVALID')
            polygon=Polygon(points)
            if not polygon.is_valid or polygon.is_empty:raise ValueError('INTERACTION_FOOTPRINT_INVALID')
            polygons.append(polygon)
        margin=c['collapse_margin_m']
        if type(margin) not in (int,float) or not math.isfinite(margin) or margin<0:
            raise ValueError('COLLAPSE_MARGIN_NOT_CONFIGURED')
        region=unary_union(polygons).buffer(margin/math.cos(math.pi/64),resolution=16).intersection(self.grid.polygon)
        indices={i for i,active in enumerate(self.grid.pit_active_cell_mask) if active and region.intersects(self.grid.cell_polygon(i))}
        if not indices:raise ValueError('INTERACTION_NOT_IN_BOUND_PIT')
        changed=bool(indices-self.affected)
        if changed:
            # Remove ALL already-fused affected samples, including samples fused
            # after event time while its reliable notification was in flight.
            self.grid.invalidate(region,max(event_epoch,self.grid.last_epoch))
            self.affected|=indices;self.ready=False;self.observation_after=None
            self.generation_ready=False
            self.invalidated_generation=self.grid.generation
        self.interaction_id=interaction_id;self.interaction_epoch=event_epoch
        self.last_event_counter=max(self.last_event_counter,evidence.counter)
        return changed

    def allow_observation(self,binding_id,digest,evidence,rise,bound_clearance,now):
        b,c=self.binding,self.config
        if (b is None or not self.interaction_id or binding_id!=b.binding_id or digest!=b.binding_digest_sha256
                or not evidence.fresh(now,c['evidence_ttl_sec'],c['source_type'])
                or evidence.calibration_id!=b.spatial_calibration_id or evidence.frame_id!=b.evidence.frame_id
                or evidence.epoch<=self.interaction_epoch or not all(math.isfinite(v) for v in (rise,bound_clearance))
                or rise<c['minimum_observed_rise_m'] or bound_clearance<c['bound_clearance_margin_m']):
            raise ValueError('BOUND_SURFACE_CLEAR_EVIDENCE_REJECTED')
        if self.observation_after is None:self.observation_after=evidence.epoch
        return self.observation_after

    def filter_points(self,map_xyz,point_epochs):
        p=np.asarray(map_xyz,dtype=float);epochs=np.asarray(point_epochs,dtype=float)
        if p.shape!=(len(epochs),3) or not np.isfinite(p).all() or not np.isfinite(epochs).all():
            raise ValueError('INTERACTION_POINT_LAYOUT_INVALID')
        keep=np.ones(len(p),dtype=bool)
        if not self.affected:return keep
        ij=np.floor((p[:,:2]-self.grid.origin)/self.grid.resolution).astype(int)
        for k,(col,row) in enumerate(ij):
            if 0<=col<self.grid.width and 0<=row<self.grid.height and row*self.grid.width+col in self.affected:
                keep[k]=(self.observation_after is not None and epochs[k]>max(self.observation_after,self.interaction_epoch))
        return keep

    def refresh(self,snapshot,now):
        b=self.binding
        if b is None or not self.affected:return None
        c=self.grid.config;ctx=snapshot.context
        valid=(self.observation_after is not None and ctx is not None and ctx.platform_motion_stable
               and snapshot.surface_session_id==b.surface_session_id and snapshot.generation>b.surface_generation
               and ctx.map_id==b.map_id and ctx.semantic_version==b.semantic_version and ctx.pit_id==b.pit_id
               and ctx.calibration_id==b.spatial_calibration_id and snapshot.window_end>self.observation_after
               and 0<=now-snapshot.window_end<=min(c['surface_ttl_sec'],self.config['evidence_ttl_sec']))
        qualified=[];self.qualified_indices=set()
        if valid:
            for i in self.affected:
                v=snapshot.cells[i]
                if (v.observed and v.frame_count>=c['minimum_cell_frames'] and v.sample_count>=c['minimum_cell_samples']
                        and v.last_observed_epoch>self.observation_after and 0<=now-v.last_observed_epoch<=c['surface_ttl_sec']
                        and all(math.isfinite(x) for x in (v.z_lower_bound,v.z_median,v.z_upper_bound,v.slope,v.height_uncertainty))
                        and v.height_uncertainty<=c['max_height_dispersion_m']
                        and v.neighborhood_coverage>=c['minimum_neighborhood_coverage']
                        and v.roughness<=c['planning']['maximum_roughness_m'] and v.slope<=c['planning']['maximum_slope_rad']):
                    qualified.append(v)
                    self.qualified_indices.add(i)
        # Readiness describes a new observed generation, never global graspability.
        # UNKNOWN remains explicit; the unchanged B planner owns local admission.
        self.ready=bool(valid and qualified)
        self.generation_ready=bool(valid)
        self.qualified_cell_count=len(qualified)
        # Unknown means no usable heights. Never interpolate or fill missing cells.
        upper=max((v.z_upper_bound for v in qualified),default=0.)
        lower=min((v.z_lower_bound for v in qualified),default=0.)
        nominal=float(np.median([v.z_median for v in qualified])) if qualified else 0.
        return RefreshedSurface(Evidence.at(snapshot.window_end,self.grid.last_counter,ctx.source_type if ctx else self.config['source_type'],
            b.spatial_calibration_id,c['config_version'],frame_id=b.evidence.frame_id,validity=4 if self.ready else 3,
            reported_age=max(0.,now-snapshot.window_end)),b.binding_id,b.binding_digest_sha256,snapshot.snapshot_id,
            snapshot.generation,snapshot.surface_session_id,upper,lower,nominal,self.ready,b.lineage,
            len(self.affected),len(qualified),len(self.affected)-len(qualified),
            len(qualified)/len(self.affected),bool(valid))

    def planning_snapshot(self,snapshot,now):
        """Conservative C projection; frozen B statistics and stored samples stay intact."""
        if not self.affected:return snapshot
        from .pit_surface import SurfaceCell
        self.refresh(snapshot,now)
        cells=tuple(SurfaceCell(v.index) if v.index in self.affected and v.index not in self.qualified_indices
                    else v for v in snapshot.cells)
        observed=[v for v in cells if v.observed and self.grid.pit_active_cell_mask[v.index]]
        qualified=[v for v in observed if math.isfinite(v.slope)
                   and v.height_uncertainty<=self.grid.config['max_height_dispersion_m']
                   and v.neighborhood_coverage>=self.grid.config['minimum_neighborhood_coverage']]
        return replace(snapshot,cells=cells,observed_cell_count=len(observed),support_qualified_cell_count=len(qualified),
                       dual_supported_cell_count=sum(v.sensor_support_mask==3 for v in qualified))
