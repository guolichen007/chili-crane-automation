"""ROS proposal shadow only, independent of SafetySupervisor/ActionExecutor."""
import copy
import time
import yaml
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.executors import ExternalShutdownException
from visualization_msgs.msg import Marker,MarkerArray
from chili_crane_msgs.msg import PitSurfaceSnapshot,PlanningSpatialContext,GraspProposal,SurfaceDisturbance
from chili_crane_perception.surface_runtime import QOS,SNAPSHOT_QOS,PROPOSAL_QOS,EVENT_QOS,configure,source_now,line_marker
from rclpy.serialization import serialize_message
from chili_crane_perception.surface_transport import (snapshot_from_message,context_from_message,proposal_to_message,seconds)
from chili_crane_perception.surface_disturbance import SurfaceDisturbanceTracker
from chili_crane_control.geofence import OperationalGeofence
from chili_crane_control.grasp_planning import GraspPlanner
from .surface_session import SurfaceSessionGate


class GraspProposalShadowNode(Node):
    def __init__(self):
        super().__init__('grasp_proposal_shadow_node')
        self.config,self.planner={},None
        self.error='NOT_CONFIGURED'
        self.declare_parameter('cycle_snapshot_mode',False)
        self.snapshot_mode=bool(self.get_parameter('cycle_snapshot_mode').value)
        try:
            self.config,semantic=configure(self)
            geofence=OperationalGeofence(yaml.safe_load(Path(self.config['geofence_file']).read_text(encoding='utf-8')),semantic)
            self.planner=GraspPlanner(self.config,geofence)
            self.recent=SurfaceDisturbanceTracker(self.config)
        except (OSError,ValueError,TypeError,KeyError) as exc:
            self.error=str(exc)
        self.latest,self.spatial_context_input=None,None
        self.generation_floor=-1
        self.last_counter=-1
        self.pending_event=None
        self.surface_sessions=SurfaceSessionGate()
        self.pending_surface=None
        self.decoded_message,self.decoded_snapshot=None,None
        self.serialized_proposal,self.last_result=None,None
        from chili_crane_control.snapshot_cycle import SnapshotCycleGate
        self.cycle_gate=SnapshotCycleGate(self.config.get('source_type'),self.config.get('frame_ttl_sec',0.))
        if self.snapshot_mode:
            from chili_crane_msgs.msg import CycleStatus,TargetBinding
            self.create_subscription(CycleStatus,'task/cycle_status',self.cycle,QOS)
            self.create_subscription(TargetBinding,'phase2c/target_binding',self.freeze_binding,SNAPSHOT_QOS)
        self.pub=self.create_publisher(GraspProposal,'planning/grasp_proposal_shadow',PROPOSAL_QOS)
        self.markers=self.create_publisher(MarkerArray,'planning/grasp_proposal_markers',QOS)
        self.create_subscription(PitSurfaceSnapshot,'perception/pit_surface_snapshot',self.surface,SNAPSHOT_QOS)
        self.create_subscription(PlanningSpatialContext,'planning/spatial_context',lambda m:setattr(self,'spatial_context_input',(m,time.monotonic())),QOS)
        # This mirrors penalty ONLY. Authoritative invalidation happens in the surface node.
        self.create_subscription(SurfaceDisturbance,'planning/surface_disturbance_shadow',self.event,EVENT_QOS)
        self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def surface(self,m):
        if self.snapshot_mode:
            from chili_crane_control.snapshot_cycle_transport import proof_from_status
            try:
                if not self.cycle_gate.accepts(proof_from_status(m.status),source_now(self,self.config)):return
            except (ValueError,TypeError,KeyError):return
        decision=self.surface_sessions.begin(m.status.surface_session_id)
        if decision not in ('NEW_SESSION','SAME_SESSION'):
            if decision == 'SESSION_HISTORY_CAPACITY_EXCEEDED':
                self.reset_surface_state()
            return
        if decision == 'NEW_SESSION':
            # The ID alone never grants admission. Block old state immediately;
            # only validated fresh evidence below can establish a new baseline.
            self.reset_surface_state()
        if not self.surface_sessions.note(m.status.surface_generation):
            return
        self.latest=None
        self.pending_surface=(m,time.monotonic())
        self.admit_surface()

    def cycle(self,m):
        from chili_crane_control.snapshot_cycle_transport import cycle_observation
        try:
            reset=self.cycle_gate.observe(cycle_observation(m),source_now(self,self.config))
            if reset or not self.cycle_gate.planning(source_now(self,self.config)):self.reset_surface_state()
        except (ValueError,TypeError,KeyError):self.reset_surface_state()

    def freeze_binding(self,m):
        try:self.cycle_gate.freeze((m.run_id,m.task_id,m.cycle_id,int(m.cycle_index)))
        except ValueError:self.cycle_gate.blocked=True
        self.reset_surface_state()

    def reset_surface_state(self):
        self.latest,self.pending_surface=None,None
        self.decoded_message,self.decoded_snapshot=None,None
        self.serialized_proposal,self.last_result=None,None
        self.generation_floor,self.last_counter=-1,-1
        self.pending_event=None
        if self.planner is not None:
            self.planner.reset_cache()
            self.recent=SurfaceDisturbanceTracker(self.config)

    def admit_surface(self):
        if self.snapshot_mode and not self.cycle_gate.planning(source_now(self,self.config)):return
        if self.pending_surface is None or self.planner is None or self.spatial_context_input is None:
            return
        m,arrival=self.pending_surface
        try:
            now=source_now(self,self.config)
            ctx_msg,ctx_arrival=self.spatial_context_input
            if (time.monotonic()-arrival > self.config['surface_ttl_sec']
                    or time.monotonic()-ctx_arrival > self.config['frame_ttl_sec']
                    or not 0 <= ctx_msg.evidence.evidence_age_sec <= self.config['frame_ttl_sec']):
                raise ValueError('SESSION_INPUT_RESIDENCE_STALE')
            snapshot=snapshot_from_message(m,self.config)
            ctx=context_from_message(ctx_msg)
            ctx.validate(self.config,self.planner.geofence.semantic,ctx.source_frame,ctx.measurement_epoch,now)
            snapshot.context.validate(self.config,self.planner.geofence.semantic,
                                      snapshot.context.source_frame,snapshot.window_end,now)
            if (snapshot.surface_session_id != self.surface_sessions.session_id
                    or snapshot.config_sha256 != self.planner.config_hash or snapshot.context.key != ctx.key
                    or snapshot.validity not in ('VALID','DEGRADED')
                    or not 0 <= now-snapshot.window_end <= self.config['surface_ttl_sec']):
                raise ValueError('SESSION_SNAPSHOT_PROVENANCE_OR_FRESHNESS_INVALID')
        except (ValueError,TypeError,KeyError,AttributeError):
            # Keep one pending latest snapshot so a late context callback can
            # validate TRANSIENT_LOCAL delivery after a planner-only restart.
            if time.monotonic()-arrival > self.config['surface_ttl_sec']:
                self.pending_surface=None
            return
        if self.pending_event is not None:
            event_id,counter=self.pending_event
            if m.status.disturbance_counter < counter or m.status.last_disturbance_id != event_id:
                self.latest=None
                self.pending_surface=None
                return
            self.pending_event=None
        if m.status.surface_generation < self.generation_floor:
            self.pending_surface=None
            return
        if not self.surface_sessions.accept(m.status.surface_generation,m.status.evidence.source_counter):
            self.pending_surface=None
            return
        self.generation_floor=m.status.surface_generation
        self.last_counter=m.status.evidence.source_counter
        self.latest=(m,arrival)
        self.decoded_message,self.decoded_snapshot=m,snapshot
        self.pending_surface=None

    def event(self,m):
        if self.planner is None or self.latest is None:
            return
        try:
            snapshot=snapshot_from_message(self.latest[0],self.config)
            if m.surface_session_id != self.surface_sessions.session_id:
                raise ValueError('RECENT_EVENT_SESSION_MISMATCH')
            if (m.validity != 4 or m.source_type not in ('SYNTHETIC','REPLAY') or m.source_type != snapshot.context.source_type
                    or m.header.frame_id != snapshot.context.target_frame
                    or m.map_id != snapshot.context.map_id or m.semantic_version != snapshot.context.semantic_version
                    or m.pit_id != snapshot.context.pit_id or m.transform_id != snapshot.context.transform_id
                    or m.kernel_id != self.planner.kernel.kernel_id or m.source_counter <= self.recent.counter
                    or seconds(m.header.stamp) <= self.recent.epoch
                    or not 0 <= source_now(self,self.config)-seconds(m.header.stamp) <= self.recent.ttl):
                raise ValueError('RECENT_EVENT_REJECTED')
            from shapely.geometry import Polygon
            fp=Polygon([(p.x,p.y) for p in m.footprint.points])
            if not fp.is_valid or fp.is_empty:
                raise ValueError('RECENT_FOOTPRINT_INVALID')
            self.recent.events.append((seconds(m.header.stamp),fp,snapshot.context.key))
            self.recent.counter=m.source_counter
            self.recent.epoch=seconds(m.header.stamp)
            # Immediately block old generation until authoritative invalidated snapshot arrives.
            self.generation_floor=max(self.generation_floor,snapshot.generation+1)
            self.pending_event=(m.event_id,m.source_counter)
            self.latest=None
        except (ValueError,TypeError,KeyError) as exc:
            self.get_logger().warning(str(exc))

    def tick(self):
        try:
            if self.snapshot_mode and not self.cycle_gate.planning(source_now(self,self.config)):
                raise ValueError('PLANNING_FROZEN_ACTIVE_CYCLE' if self.cycle_gate.cycle is not None
                    and (self.cycle_gate.bound or self.cycle_gate.cycle.phase.value>=2) else 'SCAN_REQUIRED')
            self.admit_surface()
            if self.planner is None or self.latest is None or self.spatial_context_input is None:
                raise ValueError(self.error if self.planner is None else 'SURFACE_OR_CONTEXT_MISSING')
            message,arrival=self.latest
            ctx_msg,ctx_arrival=self.spatial_context_input
            ttl=self.config['frame_ttl_sec']
            if (time.monotonic()-arrival > self.config['surface_ttl_sec'] or time.monotonic()-ctx_arrival > ttl
                    or not 0 <= ctx_msg.evidence.evidence_age_sec <= ttl):
                raise ValueError('SURFACE_CONTEXT_RESIDENCE_STALE')
            now=source_now(self,self.config)
            if message is not self.decoded_message:
                self.decoded_snapshot=snapshot_from_message(message,self.config)
                self.decoded_message=message
            snapshot=self.decoded_snapshot
            ctx=context_from_message(ctx_msg)
            result=self.planner.plan(snapshot,now,ctx,self.generation_floor,self.recent)
            publish_now=source_now(self,self.config)
            age=max(publish_now-snapshot.window_end,message.status.evidence_age_sec+time.monotonic()-arrival)
            result=self.planner.publication_gate(result,snapshot,publish_now,age,ctx)
            if result is not self.last_result:
                tick=time.perf_counter()
                self.serialized_proposal=proposal_to_message(result,message,age)
                self.planner.stage_ms['proposal_serialization']=(time.perf_counter()-tick)*1000
                for key in ('candidate_centers_scanned','shortlist_size','esdf_checked_count',
                            'esdf_reject_count','exact_geometry_check_count'):
                    setattr(self.serialized_proposal,key,self.planner.metrics.get(key,0))
                self.planner.metrics['proposal_serialized_bytes']=len(serialize_message(self.serialized_proposal))
                self.last_result=result
            output=self.serialized_proposal
            output.evidence.evidence_age_sec=age
            if self.snapshot_mode:
                from chili_crane_control.snapshot_cycle_transport import annotate_cycle
                self.cycle_gate.snapshot_ready=True;annotate_cycle(output,self.cycle_gate)
            output.planning_compute_count=self.planner.compute_count
            output.planning_cache_hit_count=self.planner.cache_hit_count
            self.pub.publish(output)
            markers=[Marker(action=Marker.DELETEALL)]
            region=self.planner.geofence.safe_region
            polygons=[region] if region.geom_type=='Polygon' else list(region.geoms)
            for i,p in enumerate(polygons):
                markers.append(line_marker(output.header,list(p.exterior.coords)[:-1],i,(0.,1.,1.,1.)))
            lookup={c.candidate_id:c for c in result.candidates}
            accepted=[lookup[i] for i in result.ranked_candidate_ids]
            for i,c in enumerate(accepted[:10]):
                selected=result.selected is not None and result.selected.candidate_id==c.candidate_id
                markers.append(line_marker(output.header,c.footprint_xy,100+i,(0.,1.,0.,1.) if selected else (.3,.5,1.,.6),c.surface_upper_m))
            label=Marker(header=copy.deepcopy(output.header),ns='SHADOW_NOT_FOR_CONTROL',id=999,
                type=Marker.TEXT_VIEW_FACING,action=Marker.ADD,text='SHADOW / NOT FOR CONTROL')
            label.pose.orientation.w=1.;label.pose.position.z=4.;label.scale.z=.3;label.color.a=1.;label.color.r=1.
            markers.append(label)
            self.markers.publish(MarkerArray(markers=markers))
        except (ValueError,TypeError,KeyError,AttributeError) as exc:
            m=GraspProposal(validity=2 if 'STALE' in str(exc) else 1,reason=str(exc),valid_for_control=False)
            m.surface_session_id=self.surface_sessions.session_id or ''
            if self.latest is not None:
                m.header=copy.deepcopy(self.latest[0].status.header)
                m.evidence=copy.deepcopy(self.latest[0].status.evidence)
            m.evidence.evidence_age_sec=1e9;m.evidence.validity=m.validity
            if self.snapshot_mode:
                from chili_crane_control.snapshot_cycle_transport import annotate_cycle
                annotate_cycle(m,self.cycle_gate)
            self.pub.publish(m)
            self.markers.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))


def proposal_main():
    rclpy.init();node=GraspProposalShadowNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
