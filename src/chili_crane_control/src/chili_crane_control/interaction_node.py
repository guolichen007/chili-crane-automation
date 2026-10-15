"""Phase2C normalized evidence-only ROS node; no permit/command/output publisher."""
from dataclasses import replace
from collections import deque
from pathlib import Path
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from chili_crane_msgs.msg import (TargetBinding,GraspInteractionState,GrabGeometrySafetyState,GrabState,
    HoistState,GrabIoState,LoadState,CycleStatus,GraspProposal,SurfaceInteractionEvidence,SurfaceRefreshEvidence,
    ContactEvidence,ClosureEvidence,PayloadEvidence,LiftEvidence,UnloadEvidence,UnloadSafeZone,UnloadingCartState,
    PlanningSpatialContext,PitSurfaceStatus)
from .grasp_interaction import (GraspInteractionEngine,GrabObservation,HoistObservation,IoObservation,CartObservation,
    ZoneObservation,check_architecture_mode)
from .load_features import LoadSample
from .interaction_contracts import Evidence,require_observation_successor
from .interaction_transport import (binding_from_message,binding_to_message,observation_evidence,ns,
    result_to_message,refresh_from_message,binding_from_proposal)
from .cycle import CycleState

QOS=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
STATE_QOS=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)
BIND_QOS=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)


def latest_callback(inputs,key):
    def callback(message):
        inputs[key]=(message,time.monotonic())
    return callback


class InteractionShadowNode(Node):
    def __init__(self):
        super().__init__('grasp_interaction_shadow_node')
        self.declare_parameter('config_file','')
        self.declare_parameter('cycle_snapshot_mode',True)
        self.snapshot_mode=bool(self.get_parameter('cycle_snapshot_mode').value)
        self.config,self.engine={},None;self.error='NOT_CONFIGURED';self.inputs={};self.phase='SCAN';self.cycle_input=None
        self.cycle_error='CYCLE_INPUT_MISSING';self.refresh_generation_floor=0
        self.proposal_history=deque(maxlen=32)
        try:
            self.config=yaml.safe_load(Path(self.get_parameter('config_file').value).read_text(encoding='utf-8'))
            check_architecture_mode(self.snapshot_mode,self.config)
            self.config['surface_mode']='SNAPSHOT_PER_CYCLE' if self.snapshot_mode else 'LEGACY_R1_ARCHITECTURE'
            semantic_file=self.config.get('semantic_map_file')
            semantic=yaml.safe_load(Path(semantic_file).read_text(encoding='utf-8')) if semantic_file else None
            self.engine=GraspInteractionEngine(self.config,semantic)
        except (OSError,ValueError,KeyError,TypeError) as exc:self.error=str(exc)
        from .snapshot_cycle import SnapshotCycleGate
        self.cycle_gate=SnapshotCycleGate(self.config.get('source_type'),self.config.get('evidence_ttl_sec',0.))
        self.state_pub=self.create_publisher(GraspInteractionState,'phase2c/interaction_state',STATE_QOS)
        self.binding_pub=self.create_publisher(TargetBinding,'phase2c/target_binding',BIND_QOS)
        kinds=(('contact',ContactEvidence),('interaction',SurfaceInteractionEvidence),('closure',ClosureEvidence),
               ('payload',PayloadEvidence),('lift',LiftEvidence),('unload',UnloadEvidence))
        self.evidence_publishers={name:self.create_publisher(kind,'phase2c/'+name+'_evidence',STATE_QOS) for name,kind in kinds}
        self.create_subscription(TargetBinding,'phase2c/binding_request',self.bind,STATE_QOS)
        self.create_subscription(CycleStatus,'task/cycle_status',self.cycle,QOS)
        self.create_subscription(PitSurfaceStatus,'perception/pit_surface_status',self.surface_status,QOS)
        for key,kind,topic,qos in (
                ('geometry',GrabGeometrySafetyState,'perception/grab_geometry_safety',QOS),
                ('grab',GrabState,'perception/grab_state',QOS),('hoist',HoistState,'hardware/hoist_state',QOS),
                ('io',GrabIoState,'hardware/grab_io_state',QOS),('load',LoadState,'hardware/load_state',QOS),
                ('context',PlanningSpatialContext,'planning/spatial_context',QOS),
                ('refresh',SurfaceRefreshEvidence,'perception/surface_refresh_evidence',STATE_QOS),
                ('cart',UnloadingCartState,'perception/unloading_cart_state',QOS),('zone',UnloadSafeZone,'planning/unload_safe_zone',QOS)):
            if not (self.snapshot_mode and key in ('context','refresh')):
                self.create_subscription(kind,topic,latest_callback(self.inputs,key),qos)
        self.create_subscription(GraspProposal,'planning/grasp_proposal_shadow',self.proposal,STATE_QOS)
        self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def now(self):return self.get_clock().now().nanoseconds/1e9

    def surface_status(self,m):
        if self.engine is None:return
        if self.snapshot_mode:self.inputs['scan_status']=(m,time.monotonic())
        try:
            b=self.engine.binding
            e=observation_evidence(m)
            if b is not None and self.engine.active_cycle and m.surface_session_id!=b.surface_session_id:
                import re
                if re.fullmatch('[0-9a-f]{32}',m.surface_session_id or ''):
                    self.engine.session_conflict=True;self.engine.invalidate_pit_exit()
                    raise ValueError('SURFACE_RESTART_DURING_ACTIVE_CYCLE_ABORT_REQUIRED')
            if not e.fresh(self.now(),self.config['evidence_ttl_sec'],self.config['source_type']):
                raise ValueError('SURFACE_SESSION_STATUS_STALE_OR_INVALID')
            if b is not None and (m.map_id,m.semantic_version,m.pit_id,m.spatial_calibration_id,m.header.frame_id)!=(
                    b.map_id,b.semantic_version,b.pit_id,b.spatial_calibration_id,b.evidence.frame_id):
                raise ValueError('SURFACE_STATUS_LINEAGE_CONFLICT')
            changed=self.engine.observe_surface_session(m.surface_session_id,e,self.now())
            if changed:
                for key in ('refresh','proposal'):self.inputs.pop(key,None)
                self.proposal_history.clear()
                self.refresh_generation_floor=0
                self.error='NEW_SESSION_FULL_RESCAN_REQUIRED'
        except ValueError as exc:
            if 'CONFLICT' in str(exc) or 'ABORT_REQUIRED' in str(exc):self.engine.invalidate_pit_exit()
            self.error=str(exc)

    def proposal(self,m):
        if self.snapshot_mode:
            from .snapshot_cycle_transport import proof_from_proposal
            try:
                if not self.cycle_gate.accepts(proof_from_proposal(m),self.now()):return
            except (ValueError,TypeError,KeyError):return
        if self.engine is not None and self.engine.surface_session_id not in (None,m.surface_session_id):return
        self.inputs['proposal']=(m,time.monotonic())
        self.proposal_history.append((m,time.monotonic()))

    def bind(self,m):
        if self.engine is None:return
        try:
            b=binding_from_message(m)
            matches=[(p,arrival) for p,arrival in self.proposal_history if p.proposal_id==b.proposal_id
                     and p.surface_session_id==b.surface_session_id]
            if not matches:raise ValueError('BINDING_REQUIRES_FRESH_SCAN_PROPOSAL')
            proposal,arrival=matches[-1]
            pe=observation_evidence(proposal,time.monotonic()-arrival)
            if not pe.fresh(self.now(),self.config['evidence_ttl_sec'],self.config['source_type']):
                raise ValueError('BINDING_PROPOSAL_STALE')
            expected=binding_from_proposal(proposal,b.run_id,b.task_id,b.cycle_id,b.cycle_index,b.binding_epoch_ns/1e9)
            if any(getattr(expected.evidence,k)!=getattr(b.evidence,k) for k in
                   ('epoch_ns','counter','source_type','calibration_id','config_version','frame_id','validity')):
                raise ValueError('BINDING_PROPOSAL_SOURCE_IDENTITY_CONFLICT')
            # Publication residence/receive stamps are bookkeeping, not a new
            # candidate. Both evidence ages are checked independently above.
            expected=replace(expected,evidence=b.evidence,binding_digest_sha256='')
            if expected.binding_digest_sha256!=b.binding_digest_sha256:
                raise ValueError('BINDING_REQUIRES_CURRENT_PHASE2B_HARD_GATE_CANDIDATE')
            proof=None
            if self.snapshot_mode:
                from .snapshot_cycle_transport import proof_from_proposal,proof_from_status
                proof=proof_from_proposal(proposal)
                status,_=self.input('scan_status')
                current=proof_from_status(status)
                if (not self.cycle_gate.accepts(proof,self.now()) or self.cycle_gate.cycle.phase!=CycleState.PLAN_PICK
                        or status.observed_cycle_state!=int(CycleState.PLAN_PICK)
                        or not self.cycle_gate.accepts(current,self.now())
                        or (current.session,current.generation,current.snapshot_id)!=(proof.session,proof.generation,proof.snapshot_id)):
                    raise ValueError('BINDING_REQUIRES_CURRENT_FROZEN_SCAN_SNAPSHOT')
            elif not self.engine.active_cycle and self.engine.binding is not None:
                refresh,re=self.input('refresh',True)
                if refresh is not None:
                    if refresh.surface_generation<self.refresh_generation_floor:
                        raise ValueError('REFRESH_GENERATION_REGRESSION_CONFLICT')
                    self.engine.consider_refresh(replace(refresh_from_message(refresh),evidence=re),self.now())
            self.engine.bind(b,self.now(),proof);self.binding_pub.publish(binding_to_message(b))
            if self.snapshot_mode:self.cycle_gate.freeze((b.run_id,b.task_id,b.cycle_id,b.cycle_index))
            self.inputs.pop('refresh',None)
            if not self.snapshot_mode:self.cycle_input=None;self.cycle_error='CYCLE_INPUT_MISSING'
            self.refresh_generation_floor=b.surface_generation
            self.error='SHADOW_ONLY'
        except (ValueError,TypeError,KeyError,AttributeError) as exc:
            self.error=str(exc);self.get_logger().warning(self.error)

    def cycle(self,m):
        if self.engine is None:return
        try:
            if self.snapshot_mode:
                from .snapshot_cycle_transport import cycle_observation
                reset=self.cycle_gate.observe(cycle_observation(m),self.now())
                if reset:
                    self.proposal_history.clear();self.inputs.pop('proposal',None)
                    if self.engine.binding is not None:self.engine.invalidate_pit_exit()
                self.phase=CycleState(m.state).name;self.cycle_input=(m,time.monotonic());self.cycle_error=''
                if self.engine.binding is not None and self.phase in ('COMPLETE','ABORTED'):
                    self.engine.last_phase=self.phase
                return
            if self.engine.binding is None:return
            b=self.engine.binding;e=observation_evidence(m)
            ttl=self.config.get('evidence_ttl_sec') or 0.
            if m.run_id!=b.run_id and self.engine.active_cycle:
                self.engine.session_conflict=True
                raise ValueError('RUN_SWAP_DURING_ACTIVE_CYCLE_ABORT_REQUIRED')
            if (m.run_id!=b.run_id or m.task_id!=b.task_id or m.cycle_id!=b.cycle_id or m.cycle_index!=b.cycle_index
                    or not e.fresh(self.now(),ttl,self.config['source_type'])):
                raise ValueError('CYCLE_BINDING_IDENTITY_OR_FRESHNESS_CONFLICT')
            previous=observation_evidence(self.cycle_input[0]) if self.cycle_input else None
            old_state=self.cycle_input[0].state if self.cycle_input else None
            require_observation_successor(previous,old_state,e,m.state)
            self.phase=CycleState(m.state).name;self.cycle_input=(m,time.monotonic())
            if self.phase in ('COMPLETE','ABORTED'):self.engine.last_phase=self.phase
            self.cycle_error=''
        except (ValueError,TypeError,KeyError,AttributeError) as exc:
            if 'CONFLICT' in str(exc) or 'ABORT_REQUIRED' in str(exc):self.engine.invalidate_pit_exit()
            self.cycle_error='CYCLE_'+str(exc)

    def input(self,key,optional=False):
        if key not in self.inputs:
            if optional:return None,None
            raise ValueError('INPUT_MISSING_'+key)
        m,arrival=self.inputs[key]
        if time.monotonic()-arrival>(self.config.get('evidence_ttl_sec') or 0.):
            if optional:return None,None
            raise ValueError('INPUT_RESIDENCE_STALE_'+key)
        return m,observation_evidence(m,time.monotonic()-arrival)

    def tick(self):
        try:
            if self.engine is None or self.engine.binding is None:raise ValueError(self.error)
            b=self.engine.binding
            if self.snapshot_mode:
                if not self.cycle_gate.fresh(self.now()):raise ValueError(self.cycle_gate.reason)
                if self.cycle_gate.cycle.key!=(b.run_id,b.task_id,b.cycle_id,b.cycle_index):
                    raise ValueError('NEW_CYCLE_SCAN_REQUIRED')
            if self.cycle_error:raise ValueError(self.cycle_error)
            if self.cycle_input is None:raise ValueError('CYCLE_INPUT_MISSING')
            cycle,cycle_arrival=self.cycle_input
            ce=observation_evidence(cycle,time.monotonic()-cycle_arrival)
            if (time.monotonic()-cycle_arrival>(self.config.get('evidence_ttl_sec') or 0.)
                    or not ce.fresh(self.now(),self.config.get('evidence_ttl_sec') or 0.,self.config['source_type'])
                    or (cycle.run_id,cycle.task_id,cycle.cycle_id,cycle.cycle_index)!=(b.run_id,b.task_id,b.cycle_id,b.cycle_index)):
                raise ValueError('CYCLE_STATE_STALE_OR_BINDING_CONFLICT')
            geometry,ge=self.input('geometry');grab,se=self.input('grab')
            # Independent DDS topics can arrive on opposite sides of a timer
            # callback. Wait for a common observation before comparing content.
            if (ge.epoch_ns,ge.counter)!=(se.epoch_ns,se.counter):
                raise ValueError('GRAB_GEOMETRY_OBSERVATION_PAIR_PENDING')
            if (geometry.header!=grab.header
                    or geometry.opening_state!= {0:'UNKNOWN',1:'OPEN',2:'CLOSED',3:'TRANSITION',4:'CONFLICT'}.get(grab.opening_state)):
                raise ValueError('GRAB_GEOMETRY_STATE_IDENTITY_CONFLICT')
            g=GrabObservation(ge,geometry.bottom_lower_bound_m,geometry.bottom_upper_bound_m,geometry.top_upper_bound_m,
                grab.pose.pose.position.z,tuple((p.x,p.y) for p in geometry.conservative_footprint.points),geometry.opening_state,
                geometry.extrinsic_calibration_id,geometry.tracking_state=='TRACKED' and grab.tracking_state==grab.TRACK_TRACKED,
                geometry.measurement_accepted and grab.source!=grab.SOURCE_PREDICTED,geometry.motion_stable and grab.motion_stable,grab.fault)
            hoist,he=self.input('hoist');h=HoistObservation(he,{-1:'DOWN',0:'STOPPED',1:'UP'}.get(hoist.motion,'UNKNOWN'),hoist.fault)
            io,ie=self.input('io',True)
            i=IoObservation(ie,io.open_limit,io.closed_limit,all(s in io.known_signals for s in ('open_limit','closed_limit')),
                            io.fault,io.running) if io is not None else None
            load,le=self.input('load',True)
            l=LoadSample(le,load.value,load.unit,load.load_state==load.LOAD_SENSOR_FAULT,
                         load.load_state==load.LOAD_SUSPENDED_EMPTY) if load is not None else None
            r=None
            if not self.snapshot_mode:
                refresh,re=self.input('refresh',True)
                if (refresh is not None and re.fresh(self.now(),self.config['evidence_ttl_sec'],self.config['source_type'])
                        and (refresh.binding_id,refresh.binding_digest_sha256,refresh.surface_session_id,re.calibration_id,re.frame_id)==
                           (b.binding_id,b.binding_digest_sha256,b.surface_session_id,b.spatial_calibration_id,b.evidence.frame_id)):
                    if refresh.surface_generation<self.refresh_generation_floor:
                        raise ValueError('REFRESH_GENERATION_REGRESSION_CONFLICT')
                    self.refresh_generation_floor=max(self.refresh_generation_floor,refresh.surface_generation)
                try:r=refresh_from_message(refresh) if refresh is not None else None
                except ValueError:r=None
                if r is not None:r=replace(r,evidence=re)
            cart,ce=self.input('cart',True);zone,ze=self.input('zone',True)
            ca=CartObservation(ce,cart.station_id,cart.present,cart.aligned,cart.full,cart.overflow) if cart is not None else None
            zo=ZoneObservation(ze,(zone.x_min,zone.x_max,zone.y_min,zone.y_max,zone.z_min,zone.z_max),
                zone.geometry_version,zone.flow_projection_verified) if zone is not None else None
            lineage=b.lineage
            next_candidate=None
            if not self.snapshot_mode:
                if 'proposal' in self.inputs:
                    proposal,_=self.inputs['proposal']
                    if proposal.validity in (3,4) and proposal.selected_candidate_id:
                        lineage=tuple(getattr(proposal,key) for key in ('map_id','semantic_version','pit_id','surface_session_id',
                            'spatial_calibration_id','geofence_id','geofence_version','geofence_config_sha256','kernel_id'))
                post_exit=(self.engine.pit_exit is not None and self.phase in
                    ('MOVE_TO_UNLOAD','VERIFY_CART','POSITION_UNLOAD','OPEN_GRAB','VERIFY_EMPTY','COMPLETE'))
                context,live=self.input('context',post_exit)
                live_fresh=live is not None and live.fresh(self.now(),self.config['evidence_ttl_sec'],self.config['source_type'])
                if not post_exit and (not live_fresh or not context.platform_motion_stable):
                    raise ValueError('LIVE_SPATIAL_CONTEXT_STALE_OR_UNSTABLE')
                # A historical pit exit does not need a fresh pit planning transform
                # at the unload station. Any fresh conflicting lineage still rejects.
                if live_fresh and (context.target_frame!=b.evidence.frame_id or context.source_type!=self.config['source_type']
                        or context.transform_state!=('FIELD_VALIDATED' if self.config['source_type']=='PHYSICAL' else 'SYNTHETIC_VALIDATED')
                        or (context.map_id,context.semantic_version,context.pit_id,context.calibration_id)!=
                           (b.map_id,b.semantic_version,b.pit_id,b.spatial_calibration_id)):
                    raise ValueError('LIVE_SPATIAL_LINEAGE_STALE_OR_CONFLICT')
                next_candidate=None
                try:
                    proposal,pe=self.input('proposal')
                    if pe.fresh(self.now(),self.config['evidence_ttl_sec'],self.config['source_type']):
                        next_candidate=binding_from_proposal(proposal,b.run_id,b.task_id,b.cycle_id+':next',
                                                             b.cycle_index+1,self.now())
                except ValueError:pass
            result=self.engine.evaluate(self.phase,g,h,i,l,self.now(),lineage,r,ca,zo,next_candidate)
            derived=replace(ge,calibration_id=b.spatial_calibration_id,config_version=self.config['config_version'])
            state=result_to_message(result,b,derived,self.config)
            self.state_pub.publish(state)
            for key,publisher in self.evidence_publishers.items():publisher.publish(getattr(state,key))
        except (ValueError,TypeError,KeyError,AttributeError) as exc:
            if self.engine is not None and ('CONFLICT' in str(exc) or 'ABORT_REQUIRED' in str(exc)):
                self.engine.invalidate_pit_exit()
            state=GraspInteractionState(validity=1,reason=str(exc),valid_for_control=False,
                abort_required='CONFLICT' in str(exc))
            state.evidence.validity=1;state.evidence.reason=str(exc);state.evidence.evidence_age_sec=1e9
            self.state_pub.publish(state)


def main():
    rclpy.init();node=InteractionShadowNode()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
