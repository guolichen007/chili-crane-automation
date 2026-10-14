#!/usr/bin/env python3
"""Installed two-P1 closeout: unchanged merger FIRST_POINT time and real restarts."""
import argparse
import copy
from dataclasses import replace
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from chili_crane_msgs.msg import (EvidenceMetadata,GrabGeometrySafetyState,GrabClearanceState,PlanningSpatialContext,
    PitSurfaceSnapshot,GraspProposal,SafetyPermit,AuthorizedCommand,ActuationRequest)
from sensor_msgs.msg import PointCloud2
from chili_crane_perception.phase2b_synthetic import fixture,scene
from chili_crane_perception.pit_surface import PitSurfaceGrid
from chili_crane_perception.surface_runtime import QOS,SNAPSHOT_QOS,PROPOSAL_QOS
from chili_crane_perception.surface_time import scan_epochs
from chili_crane_perception.surface_transport import context_to_message,polygon,stamp,seconds
from chili_crane_slam.dual_lidar import Cloud,DualLidarMerger,Extrinsic
from chili_crane_slam.dual_lidar_node import cloud_message
from chili_crane_slam.pointcloud import CANONICAL
ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--evidence-dir',type=Path,required=True)
    args=parser.parse_args();args.evidence_dir.mkdir(parents=True,exist_ok=True)
    c,s,gc,ctx,grab=fixture(ROOT/'config/safety/pit_operational_geofence.template.yaml')
    # NON_PRODUCTION only. Allow real 100ms scans and process startup without
    # conflating restart recovery with TTL expiry; original R1 expiry stays intact.
    c.update(frame_ttl_sec=5.,surface_ttl_sec=6.,window_sec=6.,maximum_window_frames=2,maximum_frame_span_sec=.12)
    namespace='phase2b_r11_'+uuid.uuid4().hex[:12]
    with tempfile.TemporaryDirectory(prefix='phase2b-r11-') as temporary:
        tmp=Path(temporary);config=tmp/'surface.yaml'
        c.update(semantic_map_file=str(tmp/'semantic.yaml'),geofence_file=str(tmp/'geofence.yaml'))
        for path,value in ((config,c),(tmp/'semantic.yaml',s),(tmp/'geofence.yaml',gc)):
            path.write_text(yaml.safe_dump(value),encoding='utf-8')
        processes={};logs=[];launch_counts={'surface':0,'planner':0};planner_started_ns=0

        def start(kind):
            nonlocal planner_started_ns
            launch_counts[kind]+=1
            if kind=='planner':planner_started_ns=time.time_ns()
            name='pit_surface_shadow_node.py' if kind=='surface' else 'grasp_proposal_shadow_node.py'
            log=(args.evidence_dir/(kind+'-'+str(launch_counts[kind])+'.log')).open('w',encoding='utf-8');logs.append(log)
            processes[kind]=subprocess.Popen(['ros2','run','chili_crane_perception',name,'--ros-args',
                '-r','__ns:=/'+namespace,'-p','config_file:='+str(config)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)

        def stop(kind):
            child=processes.pop(kind,None)
            if child is None:return
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)

        rclpy.init();node=Node('phase2b_r11_acceptance',namespace=namespace)
        received={};forbidden=[];snapshot_history=[]
        def snapshot_callback(m):
            received['snapshot']=m
            snapshot_history.append((m.surface_session_id,m.status.surface_generation,m.status.evidence.source_counter))
        def proposal_callback(m,info):
            received['proposal']=m
            # Humble message-info source_timestamp is DDS publication time.
            # Acceptance-test origin proof ONLY, never a measurement epoch.
            received['proposal_publication_ns']=info['source_timestamp']
        subscriptions=[node.create_subscription(PitSurfaceSnapshot,'perception/pit_surface_snapshot',snapshot_callback,SNAPSHOT_QOS),
            node.create_subscription(GraspProposal,'planning/grasp_proposal_shadow',proposal_callback,PROPOSAL_QOS)]
        for kind,topic in ((SafetyPermit,'safety/permit'),(AuthorizedCommand,'control/authorized_command'),
                           (ActuationRequest,'control/actuation_request')):
            subscriptions.append(node.create_subscription(kind,topic,lambda m:forbidden.append(type(m).__name__),QOS))
        pubs={key:node.create_publisher(kind,topic,QOS) for key,kind,topic in (
            ('cloud',PointCloud2,'lidar/merged_points'),('meta',EvidenceMetadata,'perception/pit_surface_input_evidence'),
            ('context',PlanningSpatialContext,'planning/spatial_context'),('geometry',GrabGeometrySafetyState,'perception/grab_geometry_safety'),
            ('clearance',GrabClearanceState,'perception/grab_clearance'))}
        grid=PitSurfaceGrid(c,s);xyz,sensors=scene(grid,'grab_connected')
        extrinsics=[Extrinsic.from_config(dict(config_state='VALID',calibration_id='synthetic-only-'+str(i),
            source_frame='sensor-'+str(i),target_frame=ctx.source_frame,rotation_matrix=np.eye(3).tolist(),translation_m=[0,0,0])) for i in (0,1)]
        merger=DualLidarMerger(*extrinsics)
        counter=0;last_context=None

        def send(mode='mid',span=.1,phase=.004):
            nonlocal counter,last_context
            now=node.get_clock().now().nanoseconds/1e9;first=now-span-phase-.02;counter+=1
            scans=[]
            for sensor in (0,1):
                values=xyz[sensors==sensor];p=np.zeros(len(values),dtype=CANONICAL)
                for i,k in enumerate(('x','y','z')):p[k]=values[:,i]
                p['sensor_id']=sensor;p['intensity']=20
                start_epoch=first+sensor*phase
                p['timestamp']=np.linspace(start_epoch,start_epoch+span,len(p))
                scans.append(Cloud(start_epoch,time.monotonic(),'sensor-'+str(sensor),p,4))
            merged=merger.merge(tuple(scans));cloud=cloud_message(merged)
            timing=scan_epochs(seconds(cloud.header.stamp),merged.points['timestamp'],merged.points['sensor_id'])
            assert abs(timing.header_epoch-max(scan.stamp for scan in scans))<1e-6
            assert abs(timing.mid_epoch-timing.header_epoch)>c['maximum_grab_mask_time_skew_sec']
            context=context_to_message(replace(ctx,measurement_epoch=timing.header_epoch))
            context.header=copy.deepcopy(cloud.header);context.evidence.header=copy.deepcopy(cloud.header)
            context.evidence.measurement_stamp=copy.deepcopy(cloud.header.stamp)
            context.evidence.evidence_age_sec=now-timing.header_epoch;last_context=context
            meta=EvidenceMetadata(validity=4,source_type=4,source_counter=counter,calibration_id=ctx.calibration_id,
                config_version=c['config_version'],evidence_age_sec=now-timing.header_epoch)
            meta.header=copy.deepcopy(cloud.header);meta.measurement_stamp=copy.deepcopy(cloud.header.stamp)
            measurement=timing.mid_epoch if mode not in ('first','old') else timing.header_epoch if mode=='first' else timing.mid_epoch-.1
            geometry=GrabGeometrySafetyState(validity=3,reason='SYNTHETIC_TRUE_MID_SCAN',source_type='SYNTHETIC',
                tracking_state='TRACKED',measurement_accepted=True,motion_stable=True,model_id='synthetic-model',
                geometry_version='synthetic-v1',extrinsic_calibration_id=ctx.field_extrinsic_id,extrinsic_state='SYNTHETIC_VALIDATED',
                bottom_lower_bound_m=grab.bottom_lower_bound_m,top_upper_bound_m=grab.top_upper_bound_m,
                evidence_age_sec=now-measurement,valid_for_control=False)
            geometry.header.frame_id='map';geometry.header.stamp=stamp(measurement)
            geometry.evidence=copy.deepcopy(meta);geometry.evidence.header=copy.deepcopy(geometry.header)
            geometry.evidence.measurement_stamp=copy.deepcopy(geometry.header.stamp)
            geometry.evidence.evidence_age_sec=geometry.evidence_age_sec
            geometry.conservative_footprint=polygon(grab.footprint_xy)
            clearance=GrabClearanceState(validity=3,reason='SHADOW_ONLY',map_id=ctx.map_id,semantic_version=ctx.semantic_version,
                pit_id=ctx.pit_id,model_id=geometry.model_id,geometry_version=geometry.geometry_version,
                extrinsic_calibration_id=ctx.field_extrinsic_id,evidence_age_sec=geometry.evidence_age_sec,valid_for_control=False)
            clearance.header=copy.deepcopy(geometry.header);clearance.evidence=copy.deepcopy(geometry.evidence)
            if mode in ('regression','nonfinite'):
                bad=merged.points.copy()
                bad['timestamp'][5]=bad['timestamp'][4]-.01 if mode=='regression' else np.nan
                cloud.data=bad.tobytes()
            for key,value in (('meta',meta),('context',context),('geometry',geometry),('clearance',clearance),('cloud',cloud)):
                pubs[key].publish(value)

        def refresh_context():
            if last_context is None:return
            message=copy.deepcopy(last_context)
            message.evidence.evidence_age_sec=node.get_clock().now().nanoseconds/1e9-seconds(message.header.stamp)
            pubs['context'].publish(message)  # Same measurement, no fabricated epoch.

        def wait(condition,timeout=10.,frames=False,mode='mid',span=.1,phase=.004,context_only=False):
            end=time.monotonic()+timeout;next_send=0.
            while time.monotonic()<end:
                assert all(child.poll() is None for child in processes.values()),'installed node exited'
                if time.monotonic()>=next_send:
                    if frames:send(mode,span,phase)
                    elif context_only:refresh_context()
                    next_send=time.monotonic()+.25
                rclpy.spin_once(node,timeout_sec=.01)
                if condition():return
            raise RuntimeError('R1.1 timeout: '+json.dumps({k:getattr(v,'reason','') for k,v in received.items()}))

        def selected():
            return ('proposal' in received and bool(received['proposal'].selected_candidate_id)
                    and received.get('proposal_publication_ns',0)>planner_started_ns)

        def drain_valid_generation():
            # Stop sending before recording replay/cache baselines. Otherwise a
            # legitimate queued source generation could look like replay work.
            wait(selected,frames=True)
            wait(lambda:selected() and 'snapshot' in received
                 and received['snapshot'].status.evidence.source_counter==counter
                 and received['proposal'].surface_snapshot_id==received['snapshot'].status.snapshot_id,
                 timeout=5.)

        try:
            start('surface');start('planner')
            wait(lambda:all(pub.get_subscription_count()>0 for pub in pubs.values()),timeout=12.)
            for span in (.08,.1):
                received.pop('proposal',None)
                wait(selected,frames=True,span=span)
                p=received['proposal'];m=received['snapshot']
                assert p.surface_session_id==m.surface_session_id==m.status.surface_session_id
                assert p.surface_snapshot_id.startswith(p.surface_session_id+':')
                assert not p.valid_for_control
            for mode in ('first','old','regression','nonfinite'):
                send(mode)
                wait(lambda:not selected() and 'snapshot' in received and not received['snapshot'].status.spatial_eligible,
                     timeout=5.,frames=True,mode=mode)
                wait(selected,frames=True)
            # Source-only restart. Keep the planner process and its old high floor.
            old=copy.deepcopy(received['snapshot']);old_proposal=copy.deepcopy(received['proposal'])
            planner_pid=processes['planner'].pid
            stop('surface');start('surface')
            wait(lambda:selected() and received['proposal'].surface_session_id!=old.surface_session_id,
                 timeout=12.,frames=True)
            current=received['proposal'];new_id=current.surface_session_id
            assert processes['planner'].pid==planner_pid
            assert current.proposal_id!=old_proposal.proposal_id
            # Replay one old writer sample, then remove its transient-local cache.
            replay=node.create_publisher(PitSurfaceSnapshot,'perception/pit_surface_snapshot',SNAPSHOT_QOS)
            wait(lambda:replay.get_subscription_count()>=2,timeout=3.,frames=True)
            drain_valid_generation()
            baseline=copy.deepcopy(received['proposal']);old.status.surface_generation=baseline.surface_generation
            history=len(snapshot_history);replay.publish(old)
            wait(lambda:len(snapshot_history)>history,timeout=2.,context_only=True)
            wait(lambda:selected() and received['proposal'].surface_session_id==new_id
                 and received['proposal'].planning_cache_hit_count>baseline.planning_cache_hit_count,context_only=True)
            assert received['proposal'].planning_compute_count==baseline.planning_compute_count
            assert received['proposal'].proposal_id==baseline.proposal_id
            node.destroy_publisher(replay)
            # Same-session generation regression also cannot replace latest.
            replay=node.create_publisher(PitSurfaceSnapshot,'perception/pit_surface_snapshot',SNAPSHOT_QOS)
            wait(lambda:replay.get_subscription_count()>=2,timeout=3.,frames=True)
            drain_valid_generation()
            baseline=copy.deepcopy(received['proposal']);latest=copy.deepcopy(received['snapshot'])
            latest.status.surface_generation=max(0,baseline.surface_generation-1)
            history=len(snapshot_history);replay.publish(latest)
            wait(lambda:len(snapshot_history)>history,timeout=2.,context_only=True)
            wait(lambda:selected() and received['proposal'].planning_cache_hit_count>baseline.planning_cache_hit_count,context_only=True)
            assert received['proposal'].surface_generation==baseline.surface_generation
            assert received['proposal'].planning_compute_count==baseline.planning_compute_count
            node.destroy_publisher(replay)
            # Planner-only restart, NO new cloud/snapshot. Only retransmit genuine
            # current context with unchanged measurement stamp for the new reader.
            drain_valid_generation()
            stable=copy.deepcopy(received['snapshot']);source_pid=processes['surface'].pid
            before=len(snapshot_history);stop('planner')
            wait(lambda:node.count_publishers('planning/grasp_proposal_shadow')==0,timeout=3.,context_only=True)
            received.pop('proposal',None);start('planner')
            wait(lambda:selected() and received['proposal'].surface_session_id==stable.surface_session_id,
                 timeout=4.,context_only=True)
            assert processes['surface'].pid==source_pid
            assert len(snapshot_history)==before,'planner-only restart needed a new producer snapshot'
            assert received['proposal'].surface_snapshot_id==stable.status.snapshot_id
            # Both restart: new session and a wholly new cache baseline.
            previous_id=received['proposal'].surface_session_id
            stop('planner');stop('surface');received.pop('proposal',None);start('surface');start('planner')
            wait(lambda:selected() and received['proposal'].surface_session_id!=previous_id,timeout=12.,frames=True)
            assert not forbidden
            result=dict(MID_SCAN_TIME_ANCHOR_STATUS='PASS_UNCHANGED_MERGER',GRAB_MASK_MID_SCAN_ALIGNMENT_STATUS='PASS_INSTALLED',
                SURFACE_SESSION_ID_STATUS='PASS',SURFACE_RESTART_RECOVERY_STATUS='PASS_INSTALLED_ALL_THREE',
                OLD_SESSION_REPLAY_REJECTION_STATUS='PASS',SAME_SESSION_REGRESSION_STATUS='PASS',
                SAME_GENERATION_DIFFERENT_SESSION_STATUS='PASS',SNAPSHOT_ID_UNIQUENESS_STATUS='PASS',
                PLANNER_TRANSIENT_LOCAL_RESTART_STATUS='PASS_WITHOUT_NEW_SNAPSHOT',
                RESTART_PUBLICATION_ORIGIN_STATUS='PASS_NEW_DDS_PUBLICATION_AFTER_PROCESS_START',
                CACHE_SESSION_ISOLATION_STATUS='PASS',CONTROL_TOPIC_MESSAGES=len(forbidden),
                PHYSICAL_ACTUATION_STATUS='NOT_RUN',valid_for_control=False)
            (args.evidence_dir/'result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
            print(json.dumps(result))
        finally:
            stop('planner');stop('surface');node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
            for log in logs:log.close()


if __name__=='__main__':main()
