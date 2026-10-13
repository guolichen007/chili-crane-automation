#!/usr/bin/env python3
"""Ubuntu-only installed Phase2B transport/expiry/identity acceptance, synthetic."""
import argparse
import copy
from dataclasses import replace
import json
from pathlib import Path
import subprocess
import tempfile
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from chili_crane_msgs.msg import (EvidenceMetadata,GrabGeometrySafetyState,GrabClearanceState,PlanningSpatialContext,
    PitSurfaceSnapshot,PitSurfaceStatus,GraspProposal,SurfaceDisturbance,SafetyPermit,AuthorizedCommand,ActuationRequest)
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from chili_crane_perception.phase2b_synthetic import fixture,scene
from chili_crane_perception.pit_surface import PitSurfaceGrid
from chili_crane_perception.surface_runtime import QOS
from chili_crane_perception.surface_transport import context_to_message,polygon,stamp,seconds
from chili_crane_slam.dual_lidar import Cloud
from chili_crane_slam.dual_lidar_node import cloud_message
from chili_crane_slam.pointcloud import CANONICAL
ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--evidence-dir',type=Path,required=True)
    args=parser.parse_args();args.evidence_dir.mkdir(parents=True,exist_ok=True)
    for package,name in (('chili_crane_perception','pit_surface_shadow_node.py'),('chili_crane_perception','grasp_proposal_shadow_node.py')):
        assert name in subprocess.check_output(['ros2','pkg','executables',package],text=True)
    c,s,gc,ctx,grab=fixture(ROOT/'config/safety/pit_operational_geofence.template.yaml')
    with tempfile.TemporaryDirectory(prefix='phase2b-synthetic-') as tmp:
        tmp=Path(tmp)
        semantic=tmp/'semantic.yaml';geofence=tmp/'geofence.yaml';config=tmp/'surface.yaml'
        c.update(semantic_map_file=str(semantic),geofence_file=str(geofence))
        for path,value in ((semantic,s),(geofence,gc),(config,c)):
            path.write_text(yaml.safe_dump(value),'utf-8')
        log=(args.evidence_dir/'phase2b-launch.log').open('w',encoding='utf-8')
        child=subprocess.Popen(['ros2','launch','chili_crane_bringup','phase2b_surface_shadow.launch.py',
            'config_file:='+str(config)],stdout=log,stderr=subprocess.STDOUT)
        rclpy.init();node=Node('phase2b_transport_acceptance',namespace='crane_01')
        received={};forbidden=[];subs=[]
        for kind,topic,key in ((PitSurfaceSnapshot,'perception/pit_surface_snapshot','snapshot'),
            (PitSurfaceStatus,'perception/pit_surface_status','status'),(GraspProposal,'planning/grasp_proposal_shadow','proposal'),
            (PointCloud2,'perception/pit_surface_observed_debug','debug'),(PointCloud2,'perception/pit_surface_unknown_debug','unknown'),
            (MarkerArray,'planning/grasp_proposal_markers','markers')):
            subs.append(node.create_subscription(kind,topic,lambda m,key=key:received.update({key:m}),QOS))
        for kind,topic in ((SafetyPermit,'safety/permit'),(AuthorizedCommand,'control/authorized_command'),
                           (ActuationRequest,'control/actuation_request')):
            subs.append(node.create_subscription(kind,topic,lambda m:forbidden.append(type(m).__name__),QOS))
        pubs={key:node.create_publisher(kind,topic,QOS) for key,kind,topic in (
            ('cloud',PointCloud2,'lidar/merged_points'),('meta',EvidenceMetadata,'perception/pit_surface_input_evidence'),
            ('context',PlanningSpatialContext,'planning/spatial_context'),('geometry',GrabGeometrySafetyState,'perception/grab_geometry_safety'),
            ('clearance',GrabClearanceState,'perception/grab_clearance'),('event',SurfaceDisturbance,'planning/surface_disturbance_shadow'))}
        grid=PitSurfaceGrid(c,s);xyz,sensors=scene(grid,'grab_connected')
        counter=0

        def send_frame(bad_map=False):
            nonlocal counter
            epoch=node.get_clock().now().nanoseconds/1e9-.02
            counter+=1
            p=np.zeros(len(xyz),dtype=CANONICAL)
            for i,k in enumerate(('x','y','z')):p[k]=xyz[:,i]
            p['sensor_id']=sensors;p['intensity']=20;p['timestamp']=np.linspace(epoch-.01,epoch+.01,len(p))
            cloud=cloud_message(Cloud(epoch,time.monotonic(),ctx.source_frame,p))
            context=context_to_message(replace(ctx,measurement_epoch=seconds(cloud.header.stamp),
                map_id='WRONG_MAP' if bad_map else ctx.map_id))
            meta=EvidenceMetadata(validity=4,source_type=4,source_counter=counter,calibration_id=ctx.calibration_id,
                config_version=c['config_version'],evidence_age_sec=.02)
            meta.header=copy.deepcopy(cloud.header);meta.measurement_stamp=copy.deepcopy(cloud.header.stamp)
            meta.receive_stamp=node.get_clock().now().to_msg()
            context.header=copy.deepcopy(cloud.header);context.evidence.header=copy.deepcopy(cloud.header)
            context.evidence.measurement_stamp=copy.deepcopy(cloud.header.stamp);context.evidence.evidence_age_sec=.02
            geometry=GrabGeometrySafetyState(validity=3,reason='SYNTHETIC_NORMALIZED_PHASE2A_EVIDENCE',
                source_type='SYNTHETIC',tracking_state='TRACKED',measurement_accepted=True,motion_stable=True,
                model_id='synthetic-model',geometry_version='synthetic-v1',extrinsic_calibration_id=ctx.calibration_id,
                extrinsic_state='SYNTHETIC_VALIDATED',bottom_lower_bound_m=grab.bottom_lower_bound_m,
                top_upper_bound_m=grab.top_upper_bound_m,evidence_age_sec=.02,valid_for_control=False)
            geometry.header=copy.deepcopy(cloud.header);geometry.header.frame_id='map'
            geometry.evidence=copy.deepcopy(meta);geometry.evidence.header=copy.deepcopy(geometry.header)
            geometry.conservative_footprint=polygon(grab.footprint_xy)
            clearance=GrabClearanceState(validity=3,reason='SHADOW_GEOMETRY_ONLY',map_id=ctx.map_id,
                semantic_version=ctx.semantic_version,pit_id=ctx.pit_id,model_id=geometry.model_id,
                geometry_version=geometry.geometry_version,extrinsic_calibration_id=ctx.calibration_id,
                evidence_age_sec=.02,valid_for_control=False)
            clearance.header=copy.deepcopy(geometry.header);clearance.evidence=copy.deepcopy(geometry.evidence)
            for key,value in (('meta',meta),('context',context),('geometry',geometry),('clearance',clearance),('cloud',cloud)):
                pubs[key].publish(value)

        def wait(condition,timeout=8.,frames=False,bad_map=False):
            end=time.monotonic()+timeout;next_frame=0.
            while time.monotonic()<end:
                assert child.poll() is None,'installed Phase2B launch exited'
                if frames and time.monotonic()>=next_frame:
                    send_frame(bad_map);next_frame=time.monotonic()+.18
                rclpy.spin_once(node,timeout_sec=.01)
                if condition():return
            raise RuntimeError('Phase2B condition timeout: '+json.dumps({k:getattr(v,'reason','received') for k,v in received.items()}))
        try:
            wait(lambda:all(pub.get_subscription_count()>0 for pub in pubs.values()),timeout=8.)
            wait(lambda:'proposal' in received and bool(received['proposal'].selected_candidate_id),timeout=12.,frames=True)
            proposal=received['proposal'];snapshot=received['snapshot']
            assert not proposal.valid_for_control and not forbidden
            assert proposal.header.stamp==proposal.evidence.measurement_stamp
            assert all(not c.valid_for_control for c in proposal.candidates)
            selected=next(c for c in proposal.candidates if c.candidate_id==proposal.selected_candidate_id)
            assert selected.hard_gates_passed and selected.unknown_area_fraction<1e-8
            assert snapshot.status.valid_cell_count < snapshot.status.total_cell_count
            assert received['unknown'].width>0 and received['debug'].width>0
            assert snapshot.status.header.stamp==snapshot.status.evidence.measurement_stamp
            assert snapshot.status.evidence_age_sec==snapshot.status.evidence.evidence_age_sec
            assert proposal.transform_id==ctx.transform_id and proposal.spatial_calibration_id==ctx.calibration_id
            original_generation=snapshot.status.surface_generation
            event=SurfaceDisturbance(validity=4,reason='SYNTHETIC_NOT_EXECUTION',event_id='installed-event',source_counter=1,
                source_type='SYNTHETIC',map_id=ctx.map_id,semantic_version=ctx.semantic_version,pit_id=ctx.pit_id,
                transform_id=ctx.transform_id,kernel_id=c['kernel']['kernel_id'],footprint=copy.deepcopy(selected.footprint))
            event.header.frame_id='map';event.header.stamp=node.get_clock().now().to_msg()
            pubs['event'].publish(event)
            wait(lambda:received['snapshot'].status.surface_generation>original_generation
                and received['snapshot'].status.last_disturbance_id==event.event_id,timeout=3.)
            disturbed=received['snapshot']
            from shapely.geometry import Polygon,box
            disturbed_polygon=Polygon([(p.x,p.y) for p in event.footprint.points]).buffer(c['planning']['disturbance_dilation_m'])
            disturbed_indices=[]
            for cell in disturbed.cells:
                row,col=divmod(cell.index,disturbed.width);r=c['resolution_m']
                cell_poly=box(disturbed.origin_x_m+col*r,disturbed.origin_y_m+row*r,
                    disturbed.origin_x_m+(col+1)*r,disturbed.origin_y_m+(row+1)*r)
                if cell_poly.intersects(disturbed_polygon):
                    assert not cell.observed
                    disturbed_indices.append(cell.index)
            assert disturbed_indices,'disturbance did not affect any planning cells'
            wait(lambda:bool(received['proposal'].selected_candidate_id)
                and all(received['snapshot'].cells[i].observed
                        and seconds(received['snapshot'].cells[i].last_observed_epoch)>seconds(event.header.stamp)
                        for i in disturbed_indices),timeout=8.,frames=True)
            # Independent spatial mismatch is not rehabilitated by an attractive pile.
            send_frame(True)
            wait(lambda:not received['proposal'].selected_candidate_id,timeout=3.,frames=True,bad_map=True)
            wait(lambda:bool(received['proposal'].selected_candidate_id),timeout=8.,frames=True)
            # Drain the last already-published generation before stopping input.
            # Otherwise a queued newer scan, not a heartbeat, can legitimately advance epoch.
            wait(lambda:received['snapshot'].status.evidence.source_counter==counter,timeout=3.)
            epoch=copy.deepcopy(received['snapshot'].status.header.stamp)
            wait(lambda:not received['proposal'].selected_candidate_id and not received['status'].spatial_eligible,timeout=3.)
            assert received['snapshot'].status.header.stamp==epoch,'expiry renewed measurement epoch'
            assert not forbidden
            print(json.dumps(dict(ROS2_PHASE2B_SHADOW_STATUS='PASS_INSTALLED_SYNTHETIC',
                OBSERVED_MASK_TRANSPORT_STATUS='PASS',PROPOSAL_IDENTITY_STATUS='PASS',
                DISTURBANCE_INVALIDATION_STATUS='PASS',REOBSERVATION_STATUS='PASS',
                SPATIAL_MISMATCH_REJECTION_STATUS='PASS',SURFACE_EXPIRY_STATUS='PASS',
                CONTROL_TOPIC_MESSAGES=len(forbidden),PHYSICAL_ACTUATION_STATUS='NOT_RUN',valid_for_control=False)))
        finally:
            node.destroy_node();rclpy.shutdown();child.send_signal(2)
            try:child.wait(timeout=8)
            except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=5)
            log.close()


if __name__=='__main__':
    main()
