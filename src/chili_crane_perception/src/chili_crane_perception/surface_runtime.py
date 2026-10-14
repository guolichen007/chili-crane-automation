"""ROS2 Phase2B surface shadow. No permit, motor, task or hardware output API."""
import copy
from pathlib import Path
import time
import uuid
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
        self.config,self.builder,self.disturbance={},None,None
        self.error='NOT_CONFIGURED'
        try:
            self.config,semantic=configure(self)
            self.builder=PitSurfaceBuilder(self.config,semantic,self.surface_session_id)
            self.disturbance=SurfaceDisturbanceTracker(self.config)
        except (OSError,ValueError,TypeError,KeyError) as exc:
            self.error=str(exc)
        self.latest=None
        self.inputs={}
        self.last_receive=None
        self.last_grab=None
        self.last_event=None
        self.last_scan_mid_epoch=0.
        self.last_snapshot_message=None
        self.last_published_generation=-1
        self.next_debug=0.
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
            self.create_subscription(kind,topic,lambda m,key=key:self.inputs.update({key:(m,time.monotonic())}),QOS)
        self.create_subscription(SurfaceDisturbance,'planning/surface_disturbance_shadow',self.event,EVENT_QOS)
        self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def cloud(self,m):
        self.latest=(m,time.monotonic())  # latest-wins, no unbounded pending queue.

    def event(self,m):
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
                geometry,grab_arrival=self.inputs['grab']
                clearance,clear_arrival=self.inputs['clearance']
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
                points=decode_cloud(cloud,c['maximum_input_points'])
                timing=scan_epochs(epoch,points['timestamp'],points['sensor_id'])
                if timing.mid_epoch <= self.last_scan_mid_epoch:
                    raise ValueError('POINT_TIME_FRAME_REGRESSION')
                frame=dict(epoch=epoch,source_counter=int(meta.source_counter),frame_id=cloud.header.frame_id,
                    mid_epoch=timing.mid_epoch,frame_start_epoch=timing.start_epoch,frame_end_epoch=timing.end_epoch,
                    span_sec=timing.span_sec,source_type=ctx.source_type,calibration_id=meta.calibration_id)
                if b.ingest(np.c_[points['x'],points['y'],points['z']],points['sensor_id'],frame,ctx,grab,now):
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
                if geometry.tracking_state != 'TRACKED' or not geometry.motion_stable or time.monotonic()-received_grab > c['frame_ttl_sec']:
                    raise ValueError('GRAB_MASK_STALE_OR_MOVING')
            except (ValueError,TypeError,KeyError) as exc:
                b.grid.clear(str(exc))
        snapshot=b.grid.snapshot(now)
        if snapshot.generation != self.last_published_generation:
            tick=time.perf_counter()
            message=snapshot_to_message(snapshot,source_now(self,c),b.grid.last_counter)
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
