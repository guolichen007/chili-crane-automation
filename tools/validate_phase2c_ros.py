#!/usr/bin/env python3
"""Installed Humble synthetic graph; owns only its isolated processes/namespace.

Real DDS types, unchanged dual-LiDAR merger, B planner/grid, C evidence/lifecycle,
and local HTTP/ROS console are exercised. No hardware transport is constructed.
"""
import argparse
import copy
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from urllib.request import Request,urlopen
from urllib.error import HTTPError
import uuid

import numpy as np
import yaml
from shapely.geometry import Polygon
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from chili_crane_msgs.msg import (EvidenceMetadata,GrabGeometrySafetyState,GrabClearanceState,GrabState,
    PlanningSpatialContext,HoistState,GrabIoState,LoadState,CycleStatus,UnloadingCartState,UnloadSafeZone,
    PitSurfaceSnapshot,GraspProposal,TargetBinding,GraspInteractionState,SurfaceRefreshEvidence,
    RemoteControlState,CommissioningShadowOutput,SafetyPermit,AuthorizedCommand,ActuationRequest)
from chili_crane_perception.phase2b_synthetic import fixture,scene
from chili_crane_perception.phase2c_synthetic import post_grab_observations
from chili_crane_perception.pit_surface import PitSurfaceGrid
from chili_crane_perception.surface_runtime import QOS,SNAPSHOT_QOS,PROPOSAL_QOS
from chili_crane_perception.surface_transport import context_to_message,polygon,stamp,seconds
from chili_crane_perception.surface_time import scan_epochs
from chili_crane_control.interaction_transport import binding_from_proposal,binding_to_message
from chili_crane_control.interaction_node import STATE_QOS,BIND_QOS
from chili_crane_control.cycle import TaskCycles,CycleState
from chili_crane_control.grasp_interaction import evidence_allows_advance
from types import SimpleNamespace
from chili_crane_slam.dual_lidar import Cloud,DualLidarMerger,Extrinsic
from chili_crane_slam.dual_lidar_node import cloud_message
from chili_crane_slam.pointcloud import CANONICAL

ROOT=Path(__file__).resolve().parents[1]


class InstalledRun:
    def __init__(self,directory,name,owner_driven=False):
        self.directory=directory/name;self.directory.mkdir()
        self.namespace='phase2c_'+uuid.uuid4().hex[:12];self.name=name
        self.cycles=TaskCycles('installed-run','installed-task') if owner_driven else None
        self.c,self.semantic,self.gc,self.ctx,self.initial_grab=fixture(ROOT/'config/safety/pit_operational_geofence.template.yaml')
        self.c.update(frame_ttl_sec=2.,surface_ttl_sec=2.5,window_sec=3.,maximum_window_frames=4)
        self.ic=yaml.safe_load((ROOT/'config/algorithms/phase2c_interaction.synthetic.yaml').read_text(encoding='utf-8'))
        self.ic['semantic_map_file']=str(self.directory/'semantic.yaml')
        self.c.update(semantic_map_file=self.ic['semantic_map_file'],geofence_file=str(self.directory/'geofence.yaml'))
        for name,value in (('surface',self.c),('semantic',self.semantic),('geofence',self.gc),('interaction',self.ic)):
            (self.directory/(name+'.yaml')).write_text(yaml.safe_dump(value),encoding='utf-8')
        self.node=Node('phase2c_installed_acceptance',namespace=self.namespace)
        self.received={};self.forbidden=[];self.subscriptions=[];self.history=[]
        self.snapshot_history=[];self.invalidation_generation=None;self.source_epochs=[]
        for kind,topic,key,qos in (
            (PitSurfaceSnapshot,'perception/pit_surface_snapshot','snapshot',SNAPSHOT_QOS),
            (GraspProposal,'planning/grasp_proposal_shadow','proposal',PROPOSAL_QOS),
            (TargetBinding,'phase2c/target_binding','binding',BIND_QOS),
            (GraspInteractionState,'phase2c/interaction_state','state',STATE_QOS),
            (SurfaceRefreshEvidence,'perception/surface_refresh_evidence','refresh',STATE_QOS),
            (RemoteControlState,'commissioning/simulation/remote_control_state','remote',STATE_QOS),
            (CommissioningShadowOutput,'commissioning/simulation/expected_output','expected',STATE_QOS)):
            def callback_for(key):
                def callback(message):
                    self.received[key]=message
                    if key=='state':self.history.append(message)
                    if key=='snapshot':self.snapshot_history.append(message)
                    if key=='refresh' and message.invalidation_acknowledged and self.invalidation_generation is None:
                        self.invalidation_generation=message.surface_generation
                return callback
            self.subscriptions.append(self.node.create_subscription(kind,topic,callback_for(key),qos))
        for kind,topic in ((SafetyPermit,'safety/permit'),(AuthorizedCommand,'control/authorized_command'),
                           (ActuationRequest,'control/actuation_request')):
            self.subscriptions.append(self.node.create_subscription(kind,topic,lambda m:self.forbidden.append(type(m).__name__),QOS))
        self.pubs={key:self.node.create_publisher(kind,topic,STATE_QOS if key=='binding' else QOS) for key,kind,topic in (
            ('cloud',PointCloud2,'lidar/merged_points'),('meta',EvidenceMetadata,'perception/pit_surface_input_evidence'),
            ('context',PlanningSpatialContext,'planning/spatial_context'),('geometry',GrabGeometrySafetyState,'perception/grab_geometry_safety'),
            ('clearance',GrabClearanceState,'perception/grab_clearance'),('grab',GrabState,'perception/grab_state'),
            ('hoist',HoistState,'hardware/hoist_state'),('io',GrabIoState,'hardware/grab_io_state'),
            ('load',LoadState,'hardware/load_state'),('cycle',CycleStatus,'task/cycle_status'),
            ('cart',UnloadingCartState,'perception/unloading_cart_state'),('zone',UnloadSafeZone,'planning/unload_safe_zone'),
            ('binding',TargetBinding,'phase2c/binding_request'))}
        self.grid=PitSurfaceGrid(self.c,self.semantic);self.xyz,self.sensors=scene(self.grid,'flat')
        extrinsics=[Extrinsic.from_config(dict(config_state='VALID',calibration_id='synthetic-only-'+str(i),
            source_frame='sensor-'+str(i),target_frame=self.ctx.source_frame,rotation_matrix=np.eye(3).tolist(),translation_m=[0,0,0])) for i in (0,1)]
        self.merger=DualLidarMerger(*extrinsics);self.counter=0;self.binding=None;self.last_messages={}
        self.processes=[];self.logs=[];self.url=None
        self.hold_post_cloud=False;self.visibility='full';self.chains={}
        self.hold_pit_context=False
        log=(self.directory/'commissioning_console.py.log').open('w',encoding='utf-8');self.logs.append(log)
        self.processes.append(subprocess.Popen(['ros2','launch','chili_crane_bringup','phase2c_shadow_stack.launch.py',
            'namespace:='+self.namespace,'surface_config_file:='+str(self.directory/'surface.yaml'),
            'interaction_config_file:='+str(self.directory/'interaction.yaml'),'commissioning:=true',
            'commissioning_config_file:='+str(ROOT/'config/commissioning/console.yaml'),
            'git_sha:='+subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
            'runs_directory:='+str(self.directory/'runs'),'commissioning_port:=0','commissioning_run_id:=installed-run'],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True))

    def start(self,package,executable,arguments):
        assert executable in subprocess.check_output(['ros2','pkg','executables',package],text=True)
        prefix=subprocess.check_output(['ros2','pkg','prefix',package],text=True).strip()
        path=Path(prefix)/'lib'/package/executable
        assert os.access(path,os.X_OK),'installed/symlink entry is not executable: '+str(path)
        log=(self.directory/(executable+'.log')).open('w',encoding='utf-8');self.logs.append(log)
        self.processes.append(subprocess.Popen([str(path)]+arguments,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))

    def spin(self,duration=.1):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            assert all(p.poll() is None for p in self.processes),'owned installed process exited; inspect logs'
            rclpy.spin_once(self.node,timeout_sec=.01)

    def wait(self,condition,timeout=10.,send=None):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if send:send()
            else:self.spin()
            if condition():return
        diagnostics={k:getattr(v,'reason','received') for k,v in self.received.items()}
        transitions=[]
        for m in self.history:
            identity=(m.task_cycle_state,m.reason,m.abort_required)
            if not transitions or identity!=transitions[-1]['identity']:
                transitions.append(dict(identity=identity,stamp=seconds(m.header.stamp),
                    counter=m.evidence.source_counter,gates=json.loads(m.gate_diagnostics_json or '{}'),
                    load=json.loads(m.load_features_json or '{}')))
        diagnostics['state_transitions']=transitions[-128:]
        diagnostics['source_epoch_gaps_sec']=[b-a for a,b in zip(self.source_epochs,self.source_epochs[1:])][-256:]
        state=self.received.get('state')
        if state is not None:
            diagnostics.update(task_cycle_state=state.task_cycle_state,contact_state=state.contact.state,
                closure_state=state.closure.state,payload_state=state.payload.state,unload_state=state.unload.state,
                load_features_json=state.load_features_json,clear_bound=state.lift.clear_of_bound_surface,
                clear_updated=state.lift.clear_of_updated_surface,clear_pit=state.lift.clear_of_pit,
                source_counter=state.evidence.source_counter)
            diagnostics['gates']=json.loads(state.gate_diagnostics_json or '{}')
            if not diagnostics['gates']:
                prior=next((m for m in reversed(self.history) if m.gate_diagnostics_json),None)
                if prior is not None:
                    diagnostics['last_valid_gates']=json.loads(prior.gate_diagnostics_json)
                    diagnostics['last_valid_gates_stamp']=seconds(prior.header.stamp)
        (self.directory/'timeout-diagnostics.json').write_text(json.dumps(diagnostics,indent=2)+'\n',encoding='utf-8')
        raise RuntimeError('Phase2C timeout '+json.dumps(diagnostics))

    def annotate(self,message,epoch,calibration,frame='map',validity=4):
        message.header.frame_id=frame;message.header.stamp=stamp(epoch);message.validity=validity
        if hasattr(message,'evidence_age_sec'):message.evidence_age_sec=max(0.,self.node.get_clock().now().nanoseconds/1e9-epoch)
        e=EvidenceMetadata(validity=validity,source_type=4,source_counter=self.counter,calibration_id=calibration,
            config_version=self.ic['config_version'],evidence_age_sec=max(0.,self.node.get_clock().now().nanoseconds/1e9-epoch))
        e.header=copy.deepcopy(message.header);e.measurement_stamp=copy.deepcopy(message.header.stamp)
        e.receive_stamp=self.node.get_clock().now().to_msg()
        message.evidence=e
        if hasattr(message,'evidence_age_sec'):message.evidence_age_sec=e.evidence_age_sec
        return message

    def send(self,phase=None,z=4.,motion=0,opening=1,closed=False,opened=True,value=100.,stable=True,empty=False):
        phase=phase or (self.cycles.state.name if self.cycles is not None else 'WAIT_STABLE')
        if self.cycles is not None:assert phase==self.cycles.state.name
        self.counter+=1;now=self.node.get_clock().now().nanoseconds/1e9;first=now-.04
        scans=[];points=self.xyz.copy()
        if self.binding and 'refresh' in self.received and self.received['refresh'].observation_enabled:
            # Actual new observations, no interpolation. Entire physical fixture
            # has changed height; only locally invalidated cells gate C refresh.
            visible=post_grab_observations(points,self.binding,self.visibility)
        else:visible=np.ones(len(points),dtype=bool)
        for sensor in (0,1):
            values=points[(self.sensors==sensor)&visible];p=np.zeros(len(values),dtype=CANONICAL)
            if len(p)==0:continue
            for i,k in enumerate(('x','y','z')):p[k]=values[:,i]
            p['sensor_id']=sensor;p['intensity']=20;p['timestamp']=np.linspace(first+sensor*.001,first+sensor*.001+.02,len(p))
            scans.append(Cloud(first+sensor*.001,time.monotonic(),'sensor-'+str(sensor),p,4))
        # No cloud when occluded, but normalized motion/load evidence continues.
        if not scans:
            first_scan=now-.04
            timing=type('Timing',(),dict(header_epoch=first_scan,mid_epoch=first_scan+.01))()
            cloud=PointCloud2();cloud.header.frame_id=self.ctx.source_frame;cloud.header.stamp=stamp(first_scan)
        else:
            merged=self.merger.merge(tuple(scans));cloud=cloud_message(merged)
            timing=scan_epochs(seconds(cloud.header.stamp),merged.points['timestamp'],merged.points['sensor_id'])
            assert abs(timing.header_epoch-max(s.stamp for s in scans))<1e-6
        epoch=timing.mid_epoch
        self.source_epochs.append(epoch)
        context=context_to_message(replace(self.ctx,measurement_epoch=timing.header_epoch))
        context.header=copy.deepcopy(cloud.header);context.evidence.header=copy.deepcopy(cloud.header)
        context.evidence.measurement_stamp=copy.deepcopy(cloud.header.stamp);context.evidence.source_counter=self.counter
        context.evidence.config_version=self.c['config_version'];context.evidence.receive_stamp=self.node.get_clock().now().to_msg()
        context.evidence.evidence_age_sec=now-timing.header_epoch
        meta=EvidenceMetadata(validity=4,source_type=4,source_counter=self.counter,calibration_id=self.ctx.calibration_id,
            config_version=self.c['config_version'],evidence_age_sec=now-timing.header_epoch)
        meta.header=copy.deepcopy(cloud.header);meta.measurement_stamp=copy.deepcopy(cloud.header.stamp)
        footprint=self.binding.capture_footprint if self.binding else self.initial_grab.footprint_xy
        geometry=self.annotate(GrabGeometrySafetyState(reason='SYNTHETIC_TRUE_MID_SCAN',source_type='SYNTHETIC',
            tracking_state='TRACKED',measurement_accepted=True,motion_stable=stable,model_id='synthetic-model',
            geometry_version='synthetic-v1',extrinsic_calibration_id=self.ctx.field_extrinsic_id,extrinsic_state='SYNTHETIC_VALIDATED',
            opening_state={1:'OPEN',2:'CLOSED',3:'TRANSITION'}[opening],bottom_lower_bound_m=z-.05,bottom_upper_bound_m=z+.05,
            top_upper_bound_m=z+.8,conservative_footprint=polygon(footprint),valid_for_control=False),epoch,'synthetic-extrinsic',validity=3)
        clearance=self.annotate(GrabClearanceState(map_id=self.ctx.map_id,semantic_version=self.ctx.semantic_version,pit_id=self.ctx.pit_id,
            model_id=geometry.model_id,geometry_version=geometry.geometry_version,extrinsic_calibration_id=self.ctx.field_extrinsic_id,
            valid_for_control=False),epoch,self.ctx.calibration_id,validity=3)
        grab=self.annotate(GrabState(tracking_state=2,opening_state=opening,source=1,motion_stable=stable),epoch,'synthetic-extrinsic',validity=3)
        grab.pose.pose.position.z=z
        grab.pose.pose.orientation.w=1.
        if self.binding:
            grab.pose.pose.position.x,grab.pose.pose.position.y=self.binding.target_suspension_map[:2]
        grab.bottom_z_m=z-.05;grab.top_z_m=z+.8;grab.footprint_map=copy.deepcopy(geometry.conservative_footprint)
        hoist=self.annotate(HoistState(motion=motion,running=motion!=0),epoch,'synthetic-hoist')
        io=self.annotate(GrabIoState(open_limit=opened,closed_limit=closed,known_signals=['open_limit','closed_limit']),epoch,'synthetic-io')
        load=self.annotate(LoadState(value=value,unit='raw',calibration_id='synthetic-load',load_state=1 if empty else 3),epoch,'synthetic-load')
        cycle_id=self.cycles.cycle_id if self.cycles is not None else 'installed-cycle:1'
        cycle_index=self.cycles.cycle_index if self.cycles is not None else 1
        cycle=self.annotate(CycleStatus(run_id='installed-run',task_id='installed-task',cycle_id=cycle_id,cycle_index=cycle_index,
            state=getattr(CycleStatus,phase)),epoch,'synthetic-cycle')
        cart=self.annotate(UnloadingCartState(station_id='synthetic-only',present=True,aligned=True,full=False,overflow=False),epoch,'synthetic-cart')
        zone=self.annotate(UnloadSafeZone(frame_id='map',x_min=-3.,x_max=3.,y_min=-3.,y_max=3.,z_min=3.5,z_max=5.,
            geometry_version='synthetic-zone',flow_projection_verified=True),epoch,'synthetic-zone')
        for key,message in (('meta',meta),('context',context),('geometry',geometry),('clearance',clearance),('grab',grab),
            ('hoist',hoist),('io',io),('load',load),('cycle',cycle),('cart',cart),('zone',zone),('cloud',cloud)):
            if key=='cloud' and (self.hold_post_cloud or not scans):continue
            if key=='context' and self.hold_pit_context:continue
            self.pubs[key].publish(message);self.last_messages[key]=message
        # Publish normalized fixture evidence at 20 Hz, independently of the
        # runtime's unchanged 0.2s maximum load gap and 9s success waits.
        self.spin(.05)

    def initialize(self):
        self.wait(lambda:all(p.get_subscription_count()>0 for key,p in self.pubs.items() if key!='binding'))
        self.wait(lambda:'proposal' in self.received and bool(self.received['proposal'].selected_candidate_id),send=self.send)
        cycle_id=self.cycles.cycle_id if self.cycles is not None else 'installed-cycle:1'
        self.binding=binding_from_proposal(self.received['proposal'],'installed-run','installed-task',cycle_id,1,
                                           self.node.get_clock().now().nanoseconds/1e9)
        self.pubs['binding'].publish(binding_to_message(self.binding))
        self.initial_binding=self.binding
        self.wait(lambda:'binding' in self.received,send=self.send)
        assert self.received['binding'].binding_digest_sha256==self.binding.binding_digest_sha256
        for _ in range(18):self.send(empty=True)
        self.wait(lambda:self.received['state'].validity==3,send=lambda:self.send(empty=True))
        features=json.loads(self.received['state'].load_features_json);assert features['baseline_valid'],features
        if self.cycles is not None:
            assert self.cycles.accept_scan(self.received['proposal'].evidence.source_counter,fresh=True)

    def owner_admission(self):
        state=self.received['state']
        result=SimpleNamespace(phase=state.task_cycle_state,abort_required=state.abort_required or state.validity!=3,
            payload=dict(state=state.payload.reason),interaction=dict(state=state.interaction.reason),
            closure=dict(state=state.closure.reason),unload=dict(state=state.unload.reason),
            lift=dict(pit_exit_confirmed=state.lift.pit_exit_confirmed))
        return self.cycles.advance(evidence_verified=evidence_allows_advance(self.cycles,result))

    def interact(self):
        if self.cycles is not None:
            assert self.cycles.advance(evidence_verified=True)  # mock positioning policy
            self.send(empty=True)
            assert self.cycles.advance(evidence_verified=True)
            self.send(empty=True)
            assert self.cycles.advance(evidence_verified=True)
            self.send()
            assert self.cycles.advance(evidence_verified=self.last_messages['io'].open_limit)
        for i in range(22):self.send('LOWER',z=4.-3.2*(i+1)/22,motion=-1,stable=False)
        for i in range(6):self.send('LOWER',z=.8-.006*i,motion=-1,stable=False)
        self.wait(lambda:'refresh' in self.received and self.received['refresh'].invalidation_acknowledged,send=lambda:
                  self.send('LOWER',z=.76,motion=-1,stable=False))
        assert self.received['refresh'].affected_cell_count>0
        assert not self.received['refresh'].refresh_ready,'old disturbed observations rehabilitated before lift'
        if self.cycles is not None:assert self.owner_admission()
        for _ in range(8):self.send('CLOSE',z=.76,opening=2,closed=True,opened=False)
        self.wait(lambda:self.received['state'].closure.state==self.received['state'].closure.CLOSURE_CONFIRMED,send=lambda:
                  self.send('CLOSE',z=.76,opening=2,closed=True,opened=False))
        if self.cycles is not None:
            assert self.owner_admission()
            self.send('VERIFY_LOAD',z=.76,opening=2,closed=True,opened=False)
            assert self.owner_admission()

    def lift(self,value=101.,acceleration=False):
        for i in range(25):
            z=.76+3.24*(i+1)/25 if not acceleration else .76+.009*(i+1)**2
            self.send('RAISE',z=z,motion=1,opening=2,closed=True,opened=False,value=value,stable=False)
            assert self.received['state'].payload.state!=self.received['state'].payload.PAYLOAD_CONFIRMED
        if not acceleration:
            expected=2 if value>100.5 else 3
            self.hold_post_cloud=True
            self.wait(lambda:self.received['state'].payload.state==expected,
                timeout=9.,send=lambda:self.send('RAISE',opening=2,closed=True,opened=False,value=value))
            assert not self.received['state'].lift.surface_refresh_ready
            self.chains['PAYLOAD_CHAIN']='PASS' if expected==2 else 'EXPECTED_EMPTY_GRASP'
            self.hold_post_cloud=False
            if self.visibility!='full':return
            self.wait(lambda:self.received['refresh'].generation_ready and self.received['refresh'].refresh_ready,
                timeout=9.,send=lambda:self.send('RAISE',opening=2,closed=True,opened=False,value=value))
            self.chains['SURFACE_REFRESH_CHAIN']='PASS'
            self.wait(lambda:self.received['state'].lift.clear_of_updated_surface,
                timeout=9.,send=lambda:self.send('RAISE',opening=2,closed=True,opened=False,value=value))
            r=self.received['refresh'];assert r.refresh_ready and r.surface_generation>self.binding.surface_generation
            assert r.qualified_cell_count==r.affected_cell_count
            assert self.received['state'].lift.clear_of_pit and self.received['state'].lift.safe_lift_height
            self.wait(lambda:self.received['proposal'].surface_session_id==self.binding.surface_session_id
                and self.received['proposal'].surface_generation>self.binding.surface_generation
                and bool(self.received['proposal'].selected_candidate_id) and self.received['state'].next_cycle_surface_ready,
                send=lambda:self.send('RAISE',opening=2,closed=True,opened=False,value=value))
            self.chains['NEXT_PLAN_CHAIN']='PASS'

    def visibility_recovery(self):
        for _ in range(35):self.send('RAISE',opening=2,closed=True,opened=False,value=101.)
        assert self.received['state'].payload.state==2
        assert not self.received['proposal'].selected_candidate_id
        if self.visibility=='partial':
            r=self.received['refresh'];assert r.refreshed_cell_count>0 and r.unknown_cell_count>0
        else:assert not self.received['refresh'].refresh_ready
        self.visibility='recovery'
        self.wait(lambda:self.received['proposal'].surface_generation>self.binding.surface_generation
            and bool(self.received['proposal'].selected_candidate_id) and self.received['state'].next_cycle_surface_ready,
            send=lambda:self.send('RAISE',opening=2,closed=True,opened=False,value=101.))
        r=self.received['refresh'];assert r.generation_ready and r.unknown_cell_count>0
        assert not self.received['state'].lift.clear_of_updated_surface
        self.chains.update(SURFACE_REFRESH_CHAIN='PASS',NEXT_PLAN_CHAIN='PASS')

    def complete_partial_current_cycle(self):
        for _ in range(35):self.send('RAISE',opening=2,closed=True,opened=False,value=101.)
        state=self.received['state'];r=self.received['refresh']
        assert (r.affected_cell_count,r.refreshed_cell_count,r.unknown_cell_count)==(9,2,7)
        assert state.payload.state==2 and state.lift.pit_exit_confirmed
        assert not state.lift.clear_of_updated_surface and not state.next_cycle_surface_ready
        # Independent DDS arrivals wait for pairing; no mixed observation can
        # revoke the exit or create new evidence. Same-observation conflicts
        # remain fatal in the node.
        delayed=copy.deepcopy(self.last_messages['grab']);self.counter+=1
        delayed.evidence.source_counter=self.counter
        self.pubs['grab'].publish(delayed)
        self.wait(lambda:self.received['state'].reason=='GRAB_GEOMETRY_OBSERVATION_PAIR_PENDING',timeout=2.)
        assert not self.received['state'].abort_required
        self.wait(lambda:self.received['state'].lift.pit_exit_confirmed and self.received['state'].payload.state==2,
            send=lambda:self.send('RAISE',opening=2,closed=True,opened=False,value=101.))
        assert self.owner_admission() and self.cycles.state==CycleState.MOVE_TO_UNLOAD
        self.hold_post_cloud=True;self.hold_pit_context=True
        for _ in range(25):self.send(opening=2,closed=True,opened=False,value=101.)
        state=self.received['state'];assert state.lift.pit_exit_confirmed and not state.lift.surface_refresh_ready
        assert self.cycles.advance(evidence_verified=True)  # mock transport arrival
        self.send(opening=2,closed=True,opened=False,value=101.)
        assert self.cycles.advance(evidence_verified=self.received['state'].unload.aligned)
        self.send(opening=2,closed=True,opened=False,value=101.)
        assert self.cycles.advance(evidence_verified=self.received['state'].unload.in_safe_zone)
        self.send(value=100.)
        assert self.cycles.advance(evidence_verified=self.received['state'].unload.open_confirmed)
        self.wait(lambda:self.received['state'].unload.state==self.received['state'].unload.UNLOAD_CONFIRMED,
                  timeout=10.,send=lambda:self.send(value=100.))
        assert not self.received['state'].next_cycle_surface_ready and self.owner_admission()
        self.send(value=100.);assert self.cycles.state==CycleState.COMPLETE
        assert self.cycles.next_cycle(continue_task=True) and self.cycles.state==CycleState.SCAN
        old=self.binding;now_ns=self.node.get_clock().now().nanoseconds
        attempt=replace(old,cycle_id=self.cycles.cycle_id,cycle_index=self.cycles.cycle_index,binding_id='old-next-attempt',
            evidence=replace(old.evidence,epoch_ns=now_ns,reported_age=0.),binding_epoch_ns=now_ns,binding_digest_sha256='')
        self.pubs['binding'].publish(binding_to_message(attempt));self.spin(.2)
        assert self.received['binding'].binding_digest_sha256==old.binding_digest_sha256
        self.hold_post_cloud=False;self.hold_pit_context=False;self.visibility='recovery'
        self.wait(lambda:self.received['proposal'].surface_generation>old.surface_generation
                  and bool(self.received['proposal'].selected_candidate_id),send=lambda:self.send(value=100.))
        fresh=binding_from_proposal(self.received['proposal'],old.run_id,old.task_id,self.cycles.cycle_id,
            self.cycles.cycle_index,self.node.get_clock().now().nanoseconds/1e9)
        self.pubs['binding'].publish(binding_to_message(fresh))
        self.wait(lambda:self.received['binding'].binding_digest_sha256==fresh.binding_digest_sha256,send=lambda:self.send(value=100.))
        self.binding=fresh
        self.chains.update(RAISE_CURRENT_CYCLE_ADVANCE='PASS',UNLOAD_WITH_STALE_REFRESH='PASS',
            COMPLETE_WITHOUT_NEXT_PLAN='PASS',NEXT_BIND_WITHOUT_NEW_B_PROPOSAL='REJECTED',NEXT_BIND_WITH_NEW_B_PROPOSAL='PASS')

    def old_sample_reuse_count(self):
        assert self.invalidation_generation is not None
        region=Polygon(self.initial_binding.capture_footprint).buffer(self.ic['collapse_margin_m']/math.cos(math.pi/64),resolution=16)
        affected={i for i,active in enumerate(self.grid.pit_active_cell_mask)
                  if active and region.intersects(self.grid.cell_polygon(i))}
        boundary=self.received['refresh'].observation_after_epoch
        return sum(cell.observed and cell.index in affected and (boundary==0 or seconds(cell.last_observed_epoch)<=boundary)
            for snapshot in self.snapshot_history if snapshot.status.surface_generation>=self.invalidation_generation
            for cell in snapshot.cells)

    def http(self,path,body=None):
        if self.url is None:
            def ready():
                text=(self.directory/'commissioning_console.py.log').read_text(encoding='utf-8')
                self.url=next((s.split('LOCAL_URL=',1)[1].strip() for s in text.splitlines() if 'LOCAL_URL=' in s),None)
                return bool(self.url)
            self.wait(ready)
        headers={}
        if body is not None:
            headers={'Content-Type':'application/json','X-CSRF-Token':self.http('/api/state')['csrf_token'],'Origin':self.url}
        request=Request(self.url+path,data=json.dumps(body).encode() if body is not None else None,headers=headers)
        with urlopen(request,timeout=3.) as response:return json.load(response)

    def console_checks(self):
        state=self.http('/api/state');assert not state['bench_enabled'] and not state['physical_output_enabled']
        self.http('/api/simulation',dict(signal='y_left',active=True))
        self.wait(lambda:'remote' in self.received and self.received['remote'].y_left)
        assert self.received['remote'].evidence.source_type==2
        self.wait(lambda:'expected' in self.received and self.received['expected'].expected_do[0])
        assert sum(self.received['expected'].expected_do)==1
        self.http('/api/stop',{})
        self.wait(lambda:self.received['remote'].stop_requested and not any(self.received['expected'].expected_do))
        self.http('/api/simulation',dict(signal='g_close',active=True))
        self.wait(lambda:self.received['remote'].g_close)
        self.wait(lambda:self.received['remote'].stop_requested,timeout=2.)
        try:self.http('/api/bench/pulse',dict(signal='y_left',duration_ms=500))
        except HTTPError as exc:assert exc.code==403
        else:raise AssertionError('unarmed physical bench accepted')
        assert not self.forbidden
        state=self.http('/api/state');assert 'phase2c/interaction_state' in state['devices']
        assert not state['automatic_control_enabled'] and not state['camera_control_authority']
        for event in ('TARGET_BOUND','INTERACTION_COMMITTED','SURFACE_INVALIDATED','CLOSE_STARTED','CLOSURE_CONFIRMED','RAISE_STARTED'):
            assert event in {e['event_type'] for e in state['events']},event

    def close(self):
        for p in reversed(self.processes):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=8.)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5.)
        for log in self.logs:log.close()
        self.node.destroy_node()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--evidence-dir',type=Path,required=True);args=parser.parse_args()
    args.evidence_dir.mkdir(parents=True,exist_ok=True);rclpy.init();results=[]
    try:
        for name in ('SUCCESSFUL_GRASP','FAILED_GRASP_AFTER_INTERACTION','LOAD_ACCELERATION_FALSE_POSITIVE',
                     'PARTIAL_POST_GRAB_VISIBILITY','POST_GRAB_OCCLUSION_THEN_RECOVERY','PARTIAL_REFRESH_CURRENT_CYCLE_EXIT'):
            run=InstalledRun(args.evidence_dir,name,owner_driven=name=='PARTIAL_REFRESH_CURRENT_CYCLE_EXIT')
            try:
                run.initialize()
                old_cycle=copy.deepcopy(run.last_messages['cycle'])
                old_cycle.evidence.source_counter=0
                old_cycle.state=CycleStatus.CLOSE
                run.pubs['cycle'].publish(old_cycle)
                run.wait(lambda:'CYCLE_OBSERVATION_CHRONOLOGY_CONFLICT' in run.received['state'].reason,timeout=2.)
                run.wait(lambda:run.received['state'].validity==3,send=lambda:run.send(empty=True))
                now_ns=run.node.get_clock().now().nanoseconds
                attack=replace(run.binding,candidate_id='swap-attack',binding_epoch_ns=now_ns,
                    evidence=replace(run.binding.evidence,epoch_ns=now_ns,reported_age=0.),binding_digest_sha256='')
                run.pubs['binding'].publish(binding_to_message(attack));run.spin(.2)
                assert run.received['binding'].binding_digest_sha256==run.binding.binding_digest_sha256
                run.interact()
                if name in ('PARTIAL_POST_GRAB_VISIBILITY','POST_GRAB_OCCLUSION_THEN_RECOVERY','PARTIAL_REFRESH_CURRENT_CYCLE_EXIT'):
                    run.visibility='partial' if name in ('PARTIAL_POST_GRAB_VISIBILITY','PARTIAL_REFRESH_CURRENT_CYCLE_EXIT') else 'occluded'
                run.lift(value=100. if name=='FAILED_GRASP_AFTER_INTERACTION' else 101.,
                    acceleration=name=='LOAD_ACCELERATION_FALSE_POSITIVE')
                payload=run.received['state'].payload.state
                if name=='SUCCESSFUL_GRASP':
                    run.wait(lambda:run.received['state'].unload.state==run.received['state'].unload.UNLOAD_CONFIRMED,
                        timeout=10.,send=lambda:run.send('VERIFY_EMPTY',value=100.))
                    run.console_checks()
                elif name=='LOAD_ACCELERATION_FALSE_POSITIVE':
                    assert not run.received['refresh'].refresh_ready
                    assert payload!=run.received['state'].payload.PAYLOAD_CONFIRMED
                elif name=='PARTIAL_REFRESH_CURRENT_CYCLE_EXIT':run.complete_partial_current_cycle()
                else:
                    if name in ('PARTIAL_POST_GRAB_VISIBILITY','POST_GRAB_OCCLUSION_THEN_RECOVERY'):run.visibility_recovery()
                assert not run.forbidden
                reused=run.old_sample_reuse_count();assert reused==0
                assert all(not m.valid_for_control and not m.contact.valid_for_control and not m.payload.valid_for_control for m in run.history)
                results.append(dict(scenario=name,status='PASS_INSTALLED_SYNTHETIC',payload_after_lift=payload,
                    interaction_acknowledged=run.received['refresh'].invalidation_acknowledged,
                    refresh_ready=run.received['refresh'].refresh_ready,control_topic_messages=len(run.forbidden),
                    unknown_cells=run.received['refresh'].unknown_cell_count,
                    refreshed_cells=run.received['refresh'].refreshed_cell_count,old_samples_reused=reused,**run.chains))
            finally:run.close()
    finally:
        if rclpy.ok():rclpy.shutdown()
    result=dict(status='PASS_INSTALLED_SYNTHETIC',scope='HUMBLE_SOFTWARE_SHADOW_NOT_FIELD',scenarios=results,
        target_swap_accepted_count=0,control_topic_messages=0,physical_output_enabled=False,automatic_control_enabled=False,
        BENCH_DO_STATUS='NOT_RUN',FIELD_GRASP_VALIDATION='NOT_RUN',FULL_AUTO='NOT_RUN')
    (args.evidence_dir/'result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result))


if __name__=='__main__':main()
