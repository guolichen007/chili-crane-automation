"""ROS2 Phase2B surface shadow. No permit, motor, task or hardware output API."""
import copy
from pathlib import Path
import time
import uuid
from collections import deque
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2,PointField
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point
from chili_crane_msgs.msg import (PlanningSpatialContext,PitSurfaceSnapshot,PitSurfaceStatus,SurfaceDisturbance,
    GrabGeometrySafetyState,GrabClearanceState,EvidenceMetadata)
from chili_crane_slam.pointcloud import CANONICAL,validate_points
from .surface_mask import PitSurfaceBuilder,GrabMask
from .surface_disturbance import SurfaceDisturbanceTracker,SurfaceGraspEvent
from .surface_transport import seconds,stamp,context_from_message,snapshot_to_message,SOURCE_IDS
from .surface_time import scan_epochs

QOS=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
# Raw sensors and rapidly expiring evidence remain sensor-compatible. Large
# generation snapshots are reliable latest-state; late joiners get only one.
SNAPSHOT_QOS=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)
PROPOSAL_QOS=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
EVENT_QOS=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)


def configure(node):
    node.declare_parameter('config_file','')
    c=yaml.safe_load(Path(node.get_parameter('config_file').value).read_text(encoding='utf-8'))
    if c.get('time_domain') not in ('HOST_UTC','ROS_TIME') or c.get('source_type') == 'PHYSICAL' and (
            c.get('time_domain') != 'HOST_UTC' or node.get_parameter('use_sim_time').value):
        raise ValueError('TIME_DOMAIN_NOT_CONFIGURED')
    semantic=yaml.safe_load(Path(c['semantic_map_file']).read_text(encoding='utf-8'))
    return c,semantic


def source_now(node,c):
    return time.time() if c['time_domain']=='HOST_UTC' else node.get_clock().now().nanoseconds/1e9


def decode_cloud(m,maximum):
    offsets=dict(zip(CANONICAL.names,(0,4,8,12,16,18,24)))
    datatypes=dict(zip(CANONICAL.names,(7,7,7,7,4,2,8)))
    if (m.point_step!=32 or m.is_bigendian or not 0 < m.width*m.height <= maximum
            or len(m.data)!=m.row_step*m.height or m.row_step < m.width*32 or len(m.fields)!=7
            or any(not any(f.name==k and f.offset==v and f.count==1 and f.datatype==datatypes[k] for f in m.fields)
                   for k,v in offsets.items())):
        raise ValueError('CANONICAL_XYZIRT_REQUIRED')
    p=np.ndarray((m.height,m.width),dtype=CANONICAL,buffer=m.data,strides=(m.row_step,32)).reshape(-1).copy()
    validate_points(p)
    scan_epochs(seconds(m.header.stamp),p['timestamp'],p['sensor_id'])
    return p


def debug_cloud(header,rows):
    dtype=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('roughness','<f4'),('coverage','<f4')])
    p=np.array(rows,dtype=dtype)
    m=PointCloud2(header=copy.deepcopy(header),height=1,width=len(p),is_bigendian=False,
        point_step=20,row_step=20*len(p),is_dense=False,data=p.tobytes())
    m.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(dtype.names)]
    return m


def line_marker(header,xy,identifier,color,z=0.):
    m=Marker(header=copy.deepcopy(header),ns='SHADOW_NOT_FOR_CONTROL',id=identifier,type=Marker.LINE_STRIP,action=Marker.ADD)
    m.pose.orientation.w=1.;m.scale.x=.03
    m.color.r,m.color.g,m.color.b,m.color.a=color
    m.points=[Point(x=float(x),y=float(y),z=float(z)) for x,y in list(xy)+[xy[0]]]
    return m


class PitSurfaceShadowNode(Node):
    def __init__(self):
        super().__init__('pit_surface_shadow_node')
        self.surface_session_id=uuid.uuid4().hex
        self.declare_parameter('cycle_snapshot_mode',False)
        self.snapshot_mode=bool(self.get_parameter('cycle_snapshot_mode').value)
        self.config,self.builder,self.disturbance={},None,None
        self.error='NOT_CONFIGURED'
        try:
            self.config,semantic=configure(self)
            self.semantic=semantic
            self.builder=PitSurfaceBuilder(self.config,semantic,self.surface_session_id)
            self.disturbance=SurfaceDisturbanceTracker(self.config)
        except (OSError,ValueError,TypeError,KeyError) as exc:
            self.error=str(exc)
        self.latest=None
        self.inputs={}
        from .grab_mask_pairs import GrabMaskPairs
        self.mask_pairs=GrabMaskPairs()
        self.last_receive=None
        self.last_grab=None
        self.last_event=None
        self.last_scan_mid_epoch=0.
        self.last_snapshot_message=None
        self.last_published_generation=-1
        self.next_debug=0.
        self.interaction_lifecycle=None
        from chili_crane_control.snapshot_cycle import SnapshotCycleGate
        self.cycle_gate=SnapshotCycleGate(self.config.get('source_type'),self.config.get('frame_ttl_sec',0.))
        if self.snapshot_mode and self.builder is not None:
            from .snapshot_surface import CycleSnapshotWindow
            self.snapshot_window=CycleSnapshotWindow(self.config,self.semantic,self.surface_session_id)
            self.builder=self.snapshot_window.builder;self.cycle_gate=self.snapshot_window.gate
        if self.snapshot_mode:
            from chili_crane_msgs.msg import CycleStatus,TargetBinding
            self.create_subscription(CycleStatus,'task/cycle_status',self.cycle,QOS)
            self.create_subscription(TargetBinding,'phase2c/target_binding',self.freeze_binding,SNAPSHOT_QOS)
        self.frozen_snapshot_history=deque(maxlen=32)
        self.declare_parameter('interaction_config_file','')
        interaction_file=self.get_parameter('interaction_config_file').value
        if interaction_file and self.builder is not None and not self.snapshot_mode:
            from .surface_interaction import SurfaceInteractionLifecycle
            from chili_crane_msgs.msg import TargetBinding,SurfaceInteractionEvidence,LiftEvidence,SurfaceRefreshEvidence
            try:
                ic=yaml.safe_load(Path(interaction_file).read_text(encoding='utf-8'))
                if ic['source_type']!=self.config['source_type']:raise ValueError('INTERACTION_SOURCE_PROFILE_MISMATCH')
                if any(ic.get(k) is not False for k in ('physical_output_enabled','automatic_control_enabled','valid_for_control')):
                    raise ValueError('INTERACTION_SHADOW_FLAGS_REQUIRED')
                self.interaction_lifecycle=SurfaceInteractionLifecycle(self.builder.grid,ic)
                self.refresh_pub=self.create_publisher(SurfaceRefreshEvidence,'perception/surface_refresh_evidence',EVENT_QOS)
                self.create_subscription(TargetBinding,'phase2c/target_binding',self.interaction_binding,SNAPSHOT_QOS)
                self.create_subscription(SurfaceInteractionEvidence,'phase2c/interaction_evidence',self.interaction_event,EVENT_QOS)
                self.create_subscription(LiftEvidence,'phase2c/lift_evidence',self.interaction_clear,EVENT_QOS)
            except (OSError,ValueError,KeyError,TypeError) as exc:self.get_logger().warning(str(exc))
        self.pub=self.create_publisher(PitSurfaceSnapshot,'perception/pit_surface_snapshot',SNAPSHOT_QOS)
        self.status_pub=self.create_publisher(PitSurfaceStatus,'perception/pit_surface_status',QOS)
        self.debug_pub=self.create_publisher(PointCloud2,'perception/pit_surface_observed_debug',QOS)
        self.unknown_pub=self.create_publisher(PointCloud2,'perception/pit_surface_unknown_debug',QOS)
        self.marker_pub=self.create_publisher(MarkerArray,'perception/pit_surface_markers',QOS)
        self.create_subscription(PointCloud2,'lidar/merged_points',self.cloud,QOS)
        for kind,topic,key in ((EvidenceMetadata,'perception/pit_surface_input_evidence','meta'),
                (PlanningSpatialContext,'planning/spatial_context','context'),
                (GrabGeometrySafetyState,'perception/grab_geometry_safety','grab'),
                (GrabClearanceState,'perception/grab_clearance','clearance')):
            self.create_subscription(kind,topic,lambda m,key=key:self.input_evidence(key,m),QOS)
        self.create_subscription(SurfaceDisturbance,'planning/surface_disturbance_shadow',self.event,EVENT_QOS)
        self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def input_evidence(self,key,message):
        arrival=time.monotonic()
        self.inputs[key]=(message,arrival)
        if key in ('grab','clearance'):self.mask_pairs.add(key,message,arrival)

    def cloud(self,m):
        if self.builder is None:return
        if self.snapshot_mode and not self.cycle_gate.scanning(source_now(self,self.config)):return
        self.latest=(m,time.monotonic())  # latest-wins, no unbounded pending queue.

    def cycle(self,m):
        from chili_crane_control.snapshot_cycle_transport import cycle_observation
        try:
            if self.builder is not None and self.snapshot_window.observe(cycle_observation(m),source_now(self,self.config)):
                self.builder=self.snapshot_window.builder
                self.latest=None;self.last_receive=None;self.last_scan_mid_epoch=0.
                self.last_snapshot_message=None;self.last_published_generation=-1
                self.inputs.pop('meta',None);self.inputs.pop('context',None)
        except (ValueError,TypeError,KeyError) as exc:self.error=str(exc)

    def freeze_binding(self,m):
        try:self.cycle_gate.freeze((m.run_id,m.task_id,m.cycle_id,int(m.cycle_index)))
        except ValueError as exc:self.cycle_gate.blocked=True;self.error=str(exc)

    def publish_frozen(self,now):
        """No grid.snapshot/expire/fusion while frozen, even after original TTL."""
        from chili_crane_control.snapshot_cycle_transport import annotate_cycle
        message=self.last_snapshot_message
        if message is None:
            status=PitSurfaceStatus(validity=1,reason=self.cycle_gate.reason,
                surface_session_id=self.surface_session_id,valid_for_control=False)
            status.evidence.validity=1;status.evidence.evidence_age_sec=status.evidence_age_sec=1e9
        else:
            status=copy.deepcopy(message.status)
            status.evidence_age_sec=max(0.,now-seconds(status.header.stamp))
            status.evidence.evidence_age_sec=status.evidence_age_sec
            status.reason=self.cycle_gate.state
            if not self.cycle_gate.planning(now):status.spatial_eligible=False
            if status.evidence_age_sec>self.config['surface_ttl_sec']:
                status.validity=status.evidence.validity=2;status.spatial_eligible=False
        self.status_pub.publish(annotate_cycle(status,self.cycle_gate))

    def interaction_binding(self,m):
        from chili_crane_control.interaction_transport import binding_from_message
        try:
            b=binding_from_message(m)
            if self.interaction_lifecycle.binding is not None and b.binding_digest_sha256==self.interaction_lifecycle.binding.binding_digest_sha256:
                return
            snapshots=[s for s in self.frozen_snapshot_history if s.snapshot_id==b.surface_snapshot_id]
            if len(snapshots)!=1:raise ValueError('FROZEN_SNAPSHOT_NOT_IN_BOUNDED_HISTORY')
            self.interaction_lifecycle.bind(b,snapshots[0],source_now(self,self.config))
        except (ValueError,TypeError,KeyError,AttributeError) as exc:self.get_logger().warning(str(exc))

    def interaction_event(self,m):
        from chili_crane_control.interaction_transport import observation_evidence
        try:
            life=self.interaction_lifecycle;b=life.binding
            if (b is None or m.state!=m.INTERACTION_COMMITTED or not m.surface_invalidation_required or m.valid_for_control
                    or (m.binding_id,m.binding_digest_sha256,m.run_id,m.task_id,m.cycle_id)!=
                       (b.binding_id,b.binding_digest_sha256,b.run_id,b.task_id,b.cycle_id)
                    or (m.map_id,m.semantic_version,m.pit_id,m.surface_session_id,m.spatial_calibration_id,m.bound_surface_snapshot_id,
                        m.bound_surface_generation)!=(b.map_id,b.semantic_version,b.pit_id,b.surface_session_id,b.spatial_calibration_id,
                                                     b.surface_snapshot_id,b.surface_generation)
                    or m.collapse_margin_m!=life.config['collapse_margin_m']
                    or tuple((p.x,p.y) for p in m.capture_footprint.points)!=b.capture_footprint):
                raise ValueError('INTERACTION_CONTRACT_OR_BINDING_CONFLICT')
            life.interact(m.interaction_id,seconds(m.interaction_stamp),int(m.interaction_source_counter),
                tuple(tuple((p.x,p.y) for p in fp.points) for fp in m.swept_footprints),observation_evidence(m),source_now(self,self.config))
        except (ValueError,TypeError,KeyError,AttributeError) as exc:self.get_logger().warning(str(exc))

    def interaction_clear(self,m):
        from chili_crane_control.interaction_transport import observation_evidence
        try:
            if not m.clear_of_bound_surface or m.valid_for_control:return
            self.interaction_lifecycle.allow_observation(m.binding_id,m.binding_digest_sha256,observation_evidence(m),
                m.observed_rise_m,m.bound_clearance_m,source_now(self,self.config))
        except (ValueError,TypeError,KeyError,AttributeError) as exc:self.get_logger().warning(str(exc))

    def event(self,m):
        if self.snapshot_mode:return  # New scans replace the whole window in R2.
        if self.disturbance is None:
            return
        try:
            if m.validity != 4:
                raise ValueError('DISTURBANCE_VALIDITY_REJECTED')
            if m.surface_session_id != self.surface_session_id:
                raise ValueError('DISTURBANCE_SESSION_MISMATCH')
            event=SurfaceGraspEvent(m.event_id,m.source_counter,seconds(m.header.stamp),m.source_type,
                m.map_id,m.semantic_version,m.pit_id,m.transform_id,m.kernel_id,
                tuple((p.x,p.y) for p in m.footprint.points))
            if m.header.frame_id != self.builder.grid.semantic['frame_id']:
                raise ValueError('DISTURBANCE_FRAME_MISMATCH')
            self.disturbance.apply(self.builder.grid,event,source_now(self,self.config))
            self.last_event=(m,time.monotonic())
        except (ValueError,TypeError,KeyError) as exc:
            self.get_logger().warning(str(exc))

    def tick(self):
        if self.builder is None:
            status=PitSurfaceStatus(validity=1,reason=self.error,valid_for_control=False,spatial_eligible=False,
                                    evidence_age_sec=1e9,surface_session_id=self.surface_session_id)
            status.evidence.validity=1;status.evidence.evidence_age_sec=1e9
            self.status_pub.publish(status)
            return
        b,c=self.builder,self.config
        now=source_now(self,c)
        if self.snapshot_mode and not self.cycle_gate.scanning(now):
            self.latest=None;self.publish_frozen(now);return
        interaction_pause=False
        if self.interaction_lifecycle is not None and self.interaction_lifecycle.binding is not None and 'grab' in self.inputs:
            geometry,received_grab=self.inputs['grab']
            interaction_pause=(geometry.validity in (3,4) and geometry.tracking_state=='TRACKED'
                and geometry.measurement_accepted and not geometry.motion_stable
                and time.monotonic()-received_grab<=c['frame_ttl_sec']
                and 0<=now-seconds(geometry.header.stamp)<=c['frame_ttl_sec'])
        item,self.latest=self.latest,None
        if item is not None:
            cloud,arrival=item
            try:
                if not all(k in self.inputs for k in ('meta','context','grab','clearance')):
                    if time.monotonic()-arrival <= c['frame_ttl_sec']:
                        self.latest=item
                    raise ValueError('INPUT_EVIDENCE_MISSING')
                meta,meta_arrival=self.inputs['meta']
                context_msg,ctx_arrival=self.inputs['context']
                epoch=seconds(cloud.header.stamp)
                if self.snapshot_mode and int(round(epoch*1e9))<=self.cycle_gate.scan_start_ns:
                    raise ValueError('FRAME_NOT_AFTER_CURRENT_SCAN_START')
                if meta.measurement_stamp != cloud.header.stamp or context_msg.header.stamp != cloud.header.stamp:
                    if time.monotonic()-arrival <= c['frame_ttl_sec']:
                        self.latest=item
                    raise ValueError('INPUT_EPOCH_PAIR_PENDING')
                ctx=context_from_message(context_msg)
                if (meta.validity != 4 or meta.header.frame_id != cloud.header.frame_id
                        or meta.calibration_id != ctx.calibration_id or meta.config_version != c['config_version']
                        or SOURCE_IDS.get(ctx.source_type) != meta.source_type
                        or not np.isfinite(meta.evidence_age_sec) or not 0 <= meta.evidence_age_sec <= c['frame_ttl_sec']
                        or not np.isfinite(context_msg.evidence.evidence_age_sec) or context_msg.evidence.evidence_age_sec < 0
                        or max(time.monotonic()-arrival,time.monotonic()-meta_arrival,time.monotonic()-ctx_arrival,
                               context_msg.evidence.evidence_age_sec) > c['frame_ttl_sec']):
                    raise ValueError('INPUT_PROVENANCE_OR_RESIDENCE_STALE')
                points=decode_cloud(cloud,c['maximum_input_points'])
                timing=scan_epochs(epoch,points['timestamp'],points['sensor_id'])
                (geometry,grab_arrival),(clearance,clear_arrival)=self.mask_pairs.closest(
                    timing.mid_epoch,c['maximum_grab_mask_time_skew_sec'])
                if (geometry.validity not in (3,4) or clearance.validity not in (3,4)
                        or geometry.model_id in ('','UNKNOWN','NOT_CONFIGURED')
                        or geometry.geometry_version in ('','UNKNOWN','NOT_CONFIGURED')
                        or geometry.header != clearance.header or geometry.evidence.measurement_stamp != geometry.header.stamp
                        or clearance.evidence.measurement_stamp != clearance.header.stamp
                        or clearance.extrinsic_calibration_id != geometry.extrinsic_calibration_id
                        or clearance.model_id != geometry.model_id or clearance.geometry_version != geometry.geometry_version
                        or clearance.evidence.source_type != SOURCE_IDS.get(ctx.source_type)
                        or geometry.evidence.source_type != SOURCE_IDS.get(ctx.source_type)
                        or not all(np.isfinite(age) and age >= 0 for age in
                            (clearance.evidence_age_sec,geometry.evidence_age_sec,
                             clearance.evidence.evidence_age_sec,geometry.evidence.evidence_age_sec))
                        or clearance.evidence_age_sec != clearance.evidence.evidence_age_sec
                        or geometry.evidence_age_sec != geometry.evidence.evidence_age_sec
                        or max(time.monotonic()-grab_arrival,time.monotonic()-clear_arrival,clearance.evidence_age_sec,
                               geometry.evidence_age_sec) > c['frame_ttl_sec']):
                    raise ValueError('GRAB_CLEARANCE_EVIDENCE_STALE_OR_MISMATCH')
                grab=GrabMask(seconds(geometry.header.stamp),geometry.header.frame_id,clearance.map_id,
                    clearance.semantic_version,clearance.pit_id,geometry.extrinsic_calibration_id,geometry.source_type,
                    geometry.tracking_state,geometry.measurement_accepted,geometry.motion_stable,
                    tuple((p.x,p.y) for p in geometry.conservative_footprint.points),geometry.bottom_lower_bound_m,
                    geometry.top_upper_bound_m,max(geometry.evidence_age_sec,time.monotonic()-grab_arrival),geometry.extrinsic_state)
                if timing.mid_epoch <= self.last_scan_mid_epoch:
                    raise ValueError('POINT_TIME_FRAME_REGRESSION')
                frame=dict(epoch=epoch,source_counter=int(meta.source_counter),frame_id=cloud.header.frame_id,
                    mid_epoch=timing.mid_epoch,frame_start_epoch=timing.start_epoch,frame_end_epoch=timing.end_epoch,
                    span_sec=timing.span_sec,source_type=ctx.source_type,calibration_id=meta.calibration_id)
                if self.interaction_lifecycle is not None:
                    p=np.c_[points['x'],points['y'],points['z']]
                    transform=np.asarray(ctx.transform,dtype=float).reshape(4,4)
                    keep=self.interaction_lifecycle.filter_points(p@transform[:3,:3].T+transform[:3,3],points['timestamp'])
                    points=points[keep]
                    if not len(points):raise ValueError('ALL_POINTS_BLOCKED_BY_INTERACTION_LIFECYCLE')
                if interaction_pause:
                    # Opt-in bound cycle only: no fusion while moving, and no
                    # wholesale clear merely for motion. Unaffected samples still
                    # expire by their ORIGINAL epochs in the frozen grid.
                    self.last_receive=arrival;self.last_scan_mid_epoch=timing.mid_epoch
                elif (self.snapshot_window.ingest if self.snapshot_mode else b.ingest)(
                        np.c_[points['x'],points['y'],points['z']],points['sensor_id'],frame,ctx,grab,now):
                    self.last_receive=arrival
                    self.last_grab=grab
                    self.last_scan_mid_epoch=timing.mid_epoch
            except (ValueError,TypeError,KeyError,AttributeError) as exc:
                b.grid.clear(str(exc))
        elif self.last_receive is None or time.monotonic()-self.last_receive > c['frame_ttl_sec']:
            if b.grid.context is not None:
                b.grid.clear('INPUT_TIMEOUT')
        if b.grid.context is not None:
            try:
                context_msg,received=self.inputs['context']
                current_context=context_from_message(context_msg)
                if current_context.key != b.grid.context.key or time.monotonic()-received > c['frame_ttl_sec']:
                    raise ValueError('SPATIAL_IDENTITY_CHANGED_OR_STALE')
                current_context.validate(c,b.grid.semantic,current_context.source_frame,
                    current_context.measurement_epoch,now)
                geometry,received_grab=self.inputs['grab']
                if (geometry.tracking_state != 'TRACKED' or not geometry.motion_stable and not interaction_pause
                        or time.monotonic()-received_grab > c['frame_ttl_sec']):
                    raise ValueError('GRAB_MASK_STALE_OR_MOVING')
            except (ValueError,TypeError,KeyError) as exc:
                b.grid.clear(str(exc))
        snapshot=self.snapshot_window.snapshot(now) if self.snapshot_mode else b.grid.snapshot(now)
        if self.interaction_lifecycle is not None:
            snapshot=self.interaction_lifecycle.planning_snapshot(snapshot,now)
        if snapshot.generation != self.last_published_generation:
            if self.interaction_lifecycle is not None:self.frozen_snapshot_history.append(snapshot)
            tick=time.perf_counter()
            message=snapshot_to_message(snapshot,source_now(self,c),b.grid.last_counter)
            if self.snapshot_mode:
                from chili_crane_control.snapshot_cycle_transport import annotate_cycle
                self.cycle_gate.snapshot_ready=message.status.spatial_eligible
                annotate_cycle(message.status,self.cycle_gate)
            b.grid.stage_ms['snapshot_serialization']=(time.perf_counter()-tick)*1000
            tick=time.perf_counter()
            payload=serialize_message(message)
            b.grid.stage_ms['dds_payload_preparation']=(time.perf_counter()-tick)*1000
            b.grid.metrics.update(serialized_cells=len(message.cells),serialized_bytes=len(payload))
            self.pub.publish(message)
            self.last_snapshot_message=message
            self.last_published_generation=snapshot.generation
        else:
            message=self.last_snapshot_message
        # Heartbeat changes only residence age, not header, counter or source epoch.
        status=copy.deepcopy(message.status)
        status.evidence_age_sec=max(0.,now-snapshot.window_end) if now>=snapshot.window_end>0 else 1e9
        status.evidence.evidence_age_sec=status.evidence_age_sec
        self.status_pub.publish(status)
        if self.interaction_lifecycle is not None:
            from chili_crane_control.interaction_transport import refresh_to_message
            refreshed=self.interaction_lifecycle.refresh(snapshot,source_now(self,c))
            if refreshed is not None:self.refresh_pub.publish(refresh_to_message(refreshed,self.interaction_lifecycle))
        if time.monotonic() < self.next_debug:
            return
        self.next_debug=time.monotonic()+1.
        observed=[];unknown=[]
        for cell in snapshot.cells:
            x,y=b.grid.center(cell.index)
            if cell.observed:
                observed.append((x,y,cell.z_median,cell.roughness,cell.neighborhood_coverage))
            elif b.grid.pit_active_cell_mask[cell.index]:
                unknown.append((x,y,b.grid.pit['pit_bottom_z_m'],0.,0.))
        self.debug_pub.publish(debug_cloud(message.status.header,observed))
        self.unknown_pub.publish(debug_cloud(message.status.header,unknown))
        markers=[line_marker(message.status.header,list(b.grid.polygon.exterior.coords)[:-1],0,(1.,.7,0.,1.))]
        if self.last_grab is not None and b.grid.context is not None:
            markers.append(line_marker(message.status.header,self.last_grab.footprint_xy,1,(1.,0.,0.,1.),self.last_grab.bottom_lower_bound_m))
        self.marker_pub.publish(MarkerArray(markers=markers))


def surface_main():
    rclpy.init();node=PitSurfaceShadowNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
