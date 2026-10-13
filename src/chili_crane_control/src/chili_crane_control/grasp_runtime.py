"""ROS proposal shadow only, independent of SafetySupervisor/ActionExecutor."""
import copy
import time
import yaml
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from visualization_msgs.msg import Marker,MarkerArray
from chili_crane_msgs.msg import PitSurfaceSnapshot,PlanningSpatialContext,GraspProposal,SurfaceDisturbance
from chili_crane_perception.surface_runtime import QOS,configure,source_now,line_marker
from chili_crane_perception.surface_transport import (snapshot_from_message,context_from_message,proposal_to_message,seconds)
from chili_crane_perception.surface_disturbance import SurfaceDisturbanceTracker
from .geofence import OperationalGeofence
from .grasp_planning import GraspPlanner


class GraspProposalShadowNode(Node):
    def __init__(self):
        super().__init__('grasp_proposal_shadow_node')
        self.config,self.planner={},None
        self.error='NOT_CONFIGURED'
        try:
            self.config,semantic=configure(self)
            geofence=OperationalGeofence(yaml.safe_load(Path(self.config['geofence_file']).read_text(encoding='utf-8')),semantic)
            self.planner=GraspPlanner(self.config,geofence)
            self.recent=SurfaceDisturbanceTracker(self.config)
        except (OSError,ValueError,TypeError,KeyError) as exc:
            self.error=str(exc)
        self.latest,self.context=None,None
        self.generation_floor=-1
        self.last_counter=-1
        self.pub=self.create_publisher(GraspProposal,'planning/grasp_proposal_shadow',QOS)
        self.markers=self.create_publisher(MarkerArray,'planning/grasp_proposal_markers',QOS)
        self.create_subscription(PitSurfaceSnapshot,'perception/pit_surface_snapshot',self.surface,QOS)
        self.create_subscription(PlanningSpatialContext,'planning/spatial_context',lambda m:setattr(self,'context',(m,time.monotonic())),QOS)
        # This mirrors penalty ONLY. Authoritative invalidation happens in the surface node.
        self.create_subscription(SurfaceDisturbance,'planning/surface_disturbance_shadow',self.event,QOS)
        self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def surface(self,m):
        if (m.status.surface_generation < self.generation_floor or m.status.evidence.source_counter < self.last_counter):
            self.latest=None
            return
        self.generation_floor=m.status.surface_generation
        self.last_counter=m.status.evidence.source_counter
        self.latest=(m,time.monotonic())

    def event(self,m):
        if self.planner is None or self.latest is None:
            return
        try:
            snapshot=snapshot_from_message(self.latest[0],self.config)
            if (m.validity != 4 or m.source_type not in ('SYNTHETIC','REPLAY') or m.source_type != snapshot.context.source_type
                    or m.map_id != snapshot.context.map_id or m.semantic_version != snapshot.context.semantic_version
                    or m.pit_id != snapshot.context.pit_id or m.transform_id != snapshot.context.transform_id
                    or m.kernel_id != self.planner.kernel.kernel_id or m.source_counter <= self.recent.counter
                    or not 0 <= source_now(self,self.config)-seconds(m.header.stamp) <= self.recent.ttl):
                raise ValueError('RECENT_EVENT_REJECTED')
            from shapely.geometry import Polygon
            fp=Polygon([(p.x,p.y) for p in m.footprint.points])
            if not fp.is_valid or fp.is_empty:
                raise ValueError('RECENT_FOOTPRINT_INVALID')
            self.recent.events.append((seconds(m.header.stamp),fp,snapshot.context.key))
            self.recent.counter=m.source_counter
            # Immediately block old generation until authoritative invalidated snapshot arrives.
            self.generation_floor=max(self.generation_floor,snapshot.generation+1)
            self.latest=None
        except (ValueError,TypeError,KeyError) as exc:
            self.get_logger().warning(str(exc))

    def tick(self):
        try:
            if self.planner is None or self.latest is None or self.context is None:
                raise ValueError(self.error if self.planner is None else 'SURFACE_OR_CONTEXT_MISSING')
            message,arrival=self.latest
            ctx_msg,ctx_arrival=self.context
            ttl=self.config['frame_ttl_sec']
            if (time.monotonic()-arrival > self.config['surface_ttl_sec'] or time.monotonic()-ctx_arrival > ttl
                    or not 0 <= ctx_msg.evidence.evidence_age_sec <= ttl):
                raise ValueError('SURFACE_CONTEXT_RESIDENCE_STALE')
            now=source_now(self,self.config)
            snapshot=snapshot_from_message(message,self.config)
            ctx=context_from_message(ctx_msg)
            result=self.planner.plan(snapshot,now,ctx,self.generation_floor,self.recent)
            age=max(now-snapshot.window_end,message.status.evidence_age_sec+time.monotonic()-arrival)
            output=proposal_to_message(result,message,age)
            self.pub.publish(output)
            markers=[Marker(action=Marker.DELETEALL)]
            region=self.planner.geofence.safe_region
            polygons=[region] if region.geom_type=='Polygon' else list(region.geoms)
            for i,p in enumerate(polygons):
                markers.append(line_marker(output.header,list(p.exterior.coords)[:-1],i,(0.,1.,1.,1.)))
            accepted=[c for c in result.candidates if c.accepted]
            accepted.sort(key=lambda c:(-c.material_volume_proxy_lower_bound_m3,-c.observed_area_fraction,
                                       c.roughness_m,c.slope_rad,c.recent_grab_penalty,c.candidate_id))
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
            if self.latest is not None:
                m.header=copy.deepcopy(self.latest[0].status.header)
                m.evidence=copy.deepcopy(self.latest[0].status.evidence)
            m.evidence.evidence_age_sec=1e9;m.evidence.validity=m.validity
            self.pub.publish(m)
            self.markers.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))


def proposal_main():
    rclpy.init();node=GraspProposalShadowNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node();rclpy.shutdown()
