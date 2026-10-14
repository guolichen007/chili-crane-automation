"""Phase2C normalized evidence-only ROS node; no permit/command/output publisher."""
from dataclasses import replace
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
    PlanningSpatialContext)
from .grasp_interaction import GraspInteractionEngine,GrabObservation,HoistObservation,IoObservation,CartObservation,ZoneObservation
from .load_features import LoadSample
from .interaction_contracts import Evidence
from .interaction_transport import (binding_from_message,binding_to_message,observation_evidence,ns,
    result_to_message,refresh_from_message)
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
        self.config,self.engine={},None;self.error='NOT_CONFIGURED';self.inputs={};self.phase='SCAN';self.cycle_input=None
        try:
            self.config=yaml.safe_load(Path(self.get_parameter('config_file').value).read_text(encoding='utf-8'))
            semantic_file=self.config.get('semantic_map_file')
            semantic=yaml.safe_load(Path(semantic_file).read_text(encoding='utf-8')) if semantic_file else None
            self.engine=GraspInteractionEngine(self.config,semantic)
        except (OSError,ValueError,KeyError,TypeError) as exc:self.error=str(exc)
        self.state_pub=self.create_publisher(GraspInteractionState,'phase2c/interaction_state',STATE_QOS)
        self.binding_pub=self.create_publisher(TargetBinding,'phase2c/target_binding',BIND_QOS)
        kinds=(('contact',ContactEvidence),('interaction',SurfaceInteractionEvidence),('closure',ClosureEvidence),
               ('payload',PayloadEvidence),('lift',LiftEvidence),('unload',UnloadEvidence))
        self.evidence_publishers={name:self.create_publisher(kind,'phase2c/'+name+'_evidence',STATE_QOS) for name,kind in kinds}
        self.create_subscription(TargetBinding,'phase2c/binding_request',self.bind,STATE_QOS)
        self.create_subscription(CycleStatus,'task/cycle_status',self.cycle,QOS)
        for key,kind,topic,qos in (
                ('geometry',GrabGeometrySafetyState,'perception/grab_geometry_safety',QOS),
                ('grab',GrabState,'perception/grab_state',QOS),('hoist',HoistState,'hardware/hoist_state',QOS),
                ('io',GrabIoState,'hardware/grab_io_state',QOS),('load',LoadState,'hardware/load_state',QOS),
                ('proposal',GraspProposal,'planning/grasp_proposal_shadow',QOS),
                ('context',PlanningSpatialContext,'planning/spatial_context',QOS),
                ('refresh',SurfaceRefreshEvidence,'perception/surface_refresh_evidence',STATE_QOS),
                ('cart',UnloadingCartState,'perception/unloading_cart_state',QOS),('zone',UnloadSafeZone,'planning/unload_safe_zone',QOS)):
            self.create_subscription(kind,topic,latest_callback(self.inputs,key),qos)
        self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def now(self):return self.get_clock().now().nanoseconds/1e9

    def bind(self,m):
        if self.engine is None:return
        try:
            b=binding_from_message(m)
            self.engine.bind(b,self.now());self.binding_pub.publish(binding_to_message(b))
            self.error='SHADOW_ONLY'
        except (ValueError,TypeError,KeyError,AttributeError) as exc:
            self.error=str(exc);self.get_logger().warning(self.error)

    def cycle(self,m):
        if self.engine is None or self.engine.binding is None:return
        try:
            b=self.engine.binding;e=observation_evidence(m)
            ttl=self.config.get('evidence_ttl_sec') or 0.
            if (m.run_id!=b.run_id or m.task_id!=b.task_id or m.cycle_id!=b.cycle_id
                    or not e.fresh(self.now(),ttl,self.config['source_type'])):
                raise ValueError('CYCLE_BINDING_IDENTITY_OR_FRESHNESS_CONFLICT')
            self.phase=CycleState(m.state).name;self.cycle_input=(m,time.monotonic())
        except (ValueError,TypeError,KeyError,AttributeError) as exc:self.error=str(exc)

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
            if self.cycle_input is None:raise ValueError('CYCLE_INPUT_MISSING')
            cycle,cycle_arrival=self.cycle_input
            ce=observation_evidence(cycle,time.monotonic()-cycle_arrival)
            if (time.monotonic()-cycle_arrival>(self.config.get('evidence_ttl_sec') or 0.)
                    or not ce.fresh(self.now(),self.config.get('evidence_ttl_sec') or 0.,self.config['source_type'])
                    or (cycle.run_id,cycle.task_id,cycle.cycle_id,cycle.cycle_index)!=(b.run_id,b.task_id,b.cycle_id,b.cycle_index)):
                raise ValueError('CYCLE_STATE_STALE_OR_BINDING_CONFLICT')
            geometry,ge=self.input('geometry');grab,se=self.input('grab')
            if (geometry.header!=grab.header or ge.counter!=se.counter
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
            refresh,re=self.input('refresh',True)
            try:r=refresh_from_message(refresh) if refresh is not None else None
            except ValueError:r=None
            if r is not None:r=replace(r,evidence=re)
            cart,ce=self.input('cart',True);zone,ze=self.input('zone',True)
            ca=CartObservation(ce,cart.station_id,cart.present,cart.aligned,cart.full,cart.overflow) if cart is not None else None
            zo=ZoneObservation(ze,(zone.x_min,zone.x_max,zone.y_min,zone.y_max,zone.z_min,zone.z_max),
                zone.geometry_version,zone.flow_projection_verified) if zone is not None else None
            lineage=b.lineage
            if 'proposal' in self.inputs:
                proposal,_=self.inputs['proposal']
                if proposal.validity in (3,4) and proposal.selected_candidate_id:
                    lineage=tuple(getattr(proposal,key) for key in ('map_id','semantic_version','pit_id','surface_session_id',
                        'spatial_calibration_id','geofence_id','geofence_version','geofence_config_sha256','kernel_id'))
            context,live=self.input('context')
            if (not live.fresh(self.now(),self.config['evidence_ttl_sec'],self.config['source_type'])
                    or context.target_frame!=b.evidence.frame_id or context.source_type!=self.config['source_type']
                    or (context.map_id,context.semantic_version,context.pit_id,context.calibration_id)!=
                       (b.map_id,b.semantic_version,b.pit_id,b.spatial_calibration_id)):
                raise ValueError('LIVE_SPATIAL_LINEAGE_STALE_OR_CONFLICT')
            result=self.engine.evaluate(self.phase,g,h,i,l,self.now(),lineage,r,ca,zo)
            derived=replace(ge,calibration_id=b.spatial_calibration_id,config_version=self.config['config_version'])
            state=result_to_message(result,b,derived,self.config)
            self.state_pub.publish(state)
            for key,publisher in self.evidence_publishers.items():publisher.publish(getattr(state,key))
        except (ValueError,TypeError,KeyError,AttributeError) as exc:
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
