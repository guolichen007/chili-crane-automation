"""Deterministic integrated NON_PRODUCTION fixtures, NEVER loaded by runtime."""
from dataclasses import replace
import numpy as np
from chili_crane_control.interaction_contracts import TargetBinding,Evidence
from chili_crane_control.grasp_interaction import GraspInteractionEngine,GrabObservation,HoistObservation,IoObservation,CartObservation,ZoneObservation
from chili_crane_control.load_features import LoadSample


def post_grab_observations(points,binding,visibility):
    """Shared synthetic/replay observations, not runtime production parameters."""
    cx,cy=binding.target_suspension_map[:2]
    x,y=points[:,0],points[:,1]
    points[:,2]=.9-.10*np.exp(-((x-cx)**2+(y-cy)**2)/.4)+.03*(x-cx)
    if visibility=='occluded':return np.zeros(len(points),dtype=bool)
    partial=(x>=cx-.5)&(x<cx)&(y>=cy-.75)&(y<cy+.5)
    if visibility=='partial':
        points[(y<cy-.3)&partial,2]-=.25
        return partial
    if visibility=='recovery':
        rx=-.875 if cx>0 else .875
        ry=-.875 if cy>0 else .875
        return partial|((x>=rx-.75)&(x<rx+.75)&(y>=ry-.75)&(y<ry+.75))
    return np.ones(len(points),dtype=bool)


class SyntheticInteractionRun:
    """Uses actual B planner/grid/lifecycle, not a hand-written refresh_ready bit.

    This validation fixture lives on perception's existing one-way control
    dependency. No runtime package is permitted to load this fixture.
    """
    def __init__(self,config,geofence_template):
        from chili_crane_perception.phase2b_synthetic import fixture,scene,ingest
        from chili_crane_perception.surface_mask import PitSurfaceBuilder
        from chili_crane_perception.surface_interaction import SurfaceInteractionLifecycle
        from chili_crane_control.geofence import OperationalGeofence
        from chili_crane_control.grasp_planning import GraspPlanner
        bc,s,gc,ctx,grab=fixture(geofence_template)
        bc.update(frame_ttl_sec=2.,surface_ttl_sec=2.5,window_sec=3.,maximum_window_frames=4)
        self.builder=PitSurfaceBuilder(bc,s,'c'*32)
        self.points,self.sensors=scene(self.builder.grid,'flat')
        self.ctx=ctx;self.counter=1;self.t=10.1
        for i in range(2):assert ingest(self.builder,self.points,self.sensors,ctx,grab,10.+.05*i,i+1)
        snapshot=self.builder.grid.snapshot(10.06)
        planner=GraspPlanner(bc,OperationalGeofence(gc,s));proposal=planner.plan(snapshot,10.06,snapshot.context,snapshot.generation)
        candidate=proposal.selected
        assert candidate is not None
        provenance=proposal.geofence_provenance
        self.binding=TargetBinding('synthetic-run','synthetic-task','synthetic-task:1',1,'synthetic-binding',proposal.proposal_id,
            candidate.candidate_id,ctx.map_id,ctx.semantic_version,ctx.pit_id,snapshot.snapshot_id,snapshot.surface_session_id,
            snapshot.generation,ctx.transform_id,ctx.calibration_id,provenance['geofence_id'],provenance['geofence_version'],
            provenance['config_sha256'],proposal.kernel_id,(*candidate.target_xy,0.),candidate.footprint_xy,candidate.surface_nominal_m,
            candidate.surface_lower_m,candidate.surface_upper_m,candidate.material_volume_proxy_lower_bound_m3,
            10_100_000_000,2,Evidence.at(10.05,2,calibration_id=ctx.calibration_id,config_version=bc['config_version']))
        self.engine=GraspInteractionEngine(config,s);self.engine.bind(self.binding,10.1)
        self.life=SurfaceInteractionLifecycle(self.builder.grid,config);self.life.bind(self.binding,snapshot,10.1)
        self.refresh=None;self.last_result=None;self.commit_count=0
        self.visibility='full';self.history=[];self.planner=planner

    def post_grab_observations(self,points):
        """Observed bowl/slope plus selectable real occlusion; never interpolate."""
        return post_grab_observations(points,self.binding,self.visibility)

    def proposal(self):
        snapshot=self.life.planning_snapshot(self.builder.grid.snapshot(self.t),self.t)
        return self.planner.plan(snapshot,self.t,snapshot.context,snapshot.generation)

    def next_candidate(self):
        proposal=self.proposal();candidate=proposal.selected;b=self.binding
        if candidate is None:return None
        return replace(b,cycle_id=b.cycle_id+':next',cycle_index=b.cycle_index+1,binding_id=b.binding_id+':next',
            proposal_id=proposal.proposal_id,candidate_id=candidate.candidate_id,surface_snapshot_id=proposal.snapshot_id,
            surface_session_id=proposal.surface_session_id,
            surface_generation=proposal.generation,target_suspension_map=(*candidate.target_xy,0.),
            capture_footprint=candidate.footprint_xy,surface_nominal_m=candidate.surface_nominal_m,
            surface_lower_m=candidate.surface_lower_m,surface_upper_m=candidate.surface_upper_m,
            material_volume_proxy_lower_bound_m3=candidate.material_volume_proxy_lower_bound_m3,
            evidence=Evidence.at(proposal.measurement_epoch,self.builder.grid.last_counter,
                calibration_id=b.spatial_calibration_id,config_version=self.builder.grid.config['config_version']),
            binding_source_counter=self.builder.grid.last_counter,binding_epoch_ns=int(round(self.t*1e9)),
            binding_digest_sha256='')

    def step(self,phase,z=4.,motion='STOPPED',opening='OPEN',closed=False,opened=True,value=100.,stable=True,empty=False):
        from chili_crane_perception.surface_mask import GrabMask
        self.t+=.05;self.counter+=1
        e=Evidence.at(self.t,self.counter,calibration_id='synthetic-extrinsic')
        b=self.binding
        grab=GrabObservation(e,z-.05,z+.05,z+.8,z,b.capture_footprint,opening,'synthetic-extrinsic',motion_stable=stable)
        hoist=HoistObservation(replace(e,calibration_id='synthetic-hoist'),motion)
        io=IoObservation(replace(e,calibration_id='synthetic-io'),opened,closed)
        load=LoadSample(replace(e,calibration_id='synthetic-load'),value,'raw',suspended_empty_known=empty)
        cart=CartObservation(replace(e,calibration_id='synthetic-cart'),'synthetic-only',True,True,False,False)
        zone=ZoneObservation(replace(e,calibration_id='synthetic-zone'),(-3.,3.,-3.,3.,3.5,5.),'synthetic-zone',True)
        next_candidate=self.next_candidate() if stable and self.refresh is not None and self.refresh.generation_ready else None
        result=self.engine.evaluate(phase,grab,hoist,io,load,self.t,b.lineage,self.refresh,cart,zone,next_candidate)
        derived=replace(e,calibration_id=b.spatial_calibration_id)
        if result.interaction['invalidate']:
            changed=self.life.interact(result.interaction['interaction_id'],result.interaction['epoch'],
                result.interaction['source_counter'],result.interaction['swept'],derived,self.t)
            if changed:self.commit_count+=1
        if result.lift['clear_of_bound_surface']:
            self.life.allow_observation(b.binding_id,b.binding_digest_sha256,derived,result.lift['observed_rise'],
                                        result.lift['bound_clearance'],self.t)
        # Identical opt-in pause policy as the ROS runtime: moving measurements
        # never fuse. TTL expiry remains in the untouched grid.snapshot().
        if stable:
            points=self.points.copy()
            visible=np.ones(len(points),dtype=bool)
            if self.life.interaction_id:
                visible=self.post_grab_observations(points)
            times=np.linspace(self.t-.01,self.t+.01,len(points))
            keep=self.life.filter_points(points,times)&visible
            frame=dict(epoch=self.t-.01,mid_epoch=self.t,source_counter=self.counter+1,frame_id=self.ctx.source_frame,
                span_sec=.02,source_type='SYNTHETIC',calibration_id=self.ctx.calibration_id)
            mask=GrabMask(self.t,'map',b.map_id,b.semantic_version,b.pit_id,self.ctx.field_extrinsic_id,'SYNTHETIC',
                'TRACKED',True,True,b.capture_footprint,grab.bottom_lower,grab.top_upper,0.,'SYNTHETIC_VALIDATED')
            if keep.any():
                assert self.builder.ingest(points[keep],self.sensors[keep],frame,replace(self.ctx,measurement_epoch=frame['epoch']),mask,self.t)
        self.refresh=self.life.refresh(self.builder.grid.snapshot(self.t),self.t)
        self.last_result=result
        self.history.append(result)
        return result

    def warm(self):
        for _ in range(20):self.step('WAIT_STABLE',empty=True)

    def interact(self):
        for i in range(30):self.step('LOWER',z=4.-3.2*(i+1)/30,motion='DOWN',stable=False)
        for i in range(6):self.step('LOWER',z=.8-.005*i,motion='DOWN',stable=False)
        assert self.engine.committed
        for _ in range(8):self.step('CLOSE',z=.775,opening='CLOSED',closed=True,opened=False)
        assert self.last_result.closure['state']=='CLOSURE_CONFIRMED'

    def lift(self,value=101.,acceleration=False):
        for i in range(30):
            z=.775+3.225*(i+1)/30 if not acceleration else .775+.008*(i+1)**2
            result=self.step('RAISE',z=z,motion='UP',opening='CLOSED',closed=True,opened=False,value=value,stable=False)
            if acceleration:assert result.payload['state']!='PAYLOAD_CONFIRMED'
        if not acceleration:
            for _ in range(40):self.step('RAISE',opening='CLOSED',closed=True,opened=False,value=value)
        return self.last_result
