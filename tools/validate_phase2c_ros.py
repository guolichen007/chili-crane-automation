#!/usr/bin/env python3
"""R2 installed snapshot-cycle acceptance. Synthetic observations, no actuators."""
import argparse
import copy
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import subprocess
import time
import uuid
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from chili_crane_msgs.msg import (EvidenceMetadata,GrabGeometrySafetyState,GrabClearanceState,GrabState,
    PlanningSpatialContext,HoistState,GrabIoState,LoadState,CycleStatus,UnloadingCartState,UnloadSafeZone,
    PitSurfaceStatus,PitSurfaceSnapshot,GraspProposal,TargetBinding,GraspInteractionState,
    SafetyPermit,AuthorizedCommand,ActuationRequest)
from chili_crane_control.cycle import TaskCycles,CycleState
from chili_crane_control.grasp_interaction import evidence_allows_advance
from chili_crane_control.interaction_transport import binding_from_proposal,binding_to_message
from chili_crane_control.interaction_node import STATE_QOS,BIND_QOS
from chili_crane_perception.phase2b_synthetic import fixture,scene
from chili_crane_perception.pit_surface import PitSurfaceGrid
from chili_crane_perception.surface_runtime import QOS,SNAPSHOT_QOS,PROPOSAL_QOS
from chili_crane_perception.surface_transport import context_to_message,polygon,stamp,seconds
from chili_crane_perception.surface_time import scan_epochs
from chili_crane_slam.dual_lidar import Cloud,DualLidarMerger,Extrinsic
from chili_crane_slam.dual_lidar_node import cloud_message
from chili_crane_slam.pointcloud import CANONICAL
from types import SimpleNamespace
from owned_ros_process import start_owned,stop_owned,owned_process_leak_count
from lidar_frame_bundle import AtomicLidarPublisher

ROOT=Path(__file__).resolve().parents[1]


class SnapshotRun:
    def __init__(self,root,name,individual=False,missing_rim=False):
        self.directory=root/name;self.directory.mkdir()
        self.namespace='phase2c_r2_'+uuid.uuid4().hex[:12]
        self.cycles=TaskCycles('installed-run','installed-task')
        self.c,self.semantic,self.gc,self.ctx,self.initial_grab=fixture(ROOT/'config/safety/pit_operational_geofence.template.yaml')
        self.c.update(frame_ttl_sec=2.,surface_ttl_sec=2.5,window_sec=3.,maximum_window_frames=4,
                      architecture_mode='SNAPSHOT_PER_CYCLE')
        self.ic=yaml.safe_load((ROOT/'config/algorithms/phase2c_interaction.synthetic.yaml').read_text(encoding='utf-8'))
        if missing_rim:self.ic['pit_geometry_state']='NOT_CONFIGURED'
        self.ic['semantic_map_file']=str(self.directory/'semantic.yaml')
        self.c.update(semantic_map_file=self.ic['semantic_map_file'],geofence_file=str(self.directory/'geofence.yaml'))
        for key,value in (('surface',self.c),('semantic',self.semantic),('geofence',self.gc),('interaction',self.ic)):
            (self.directory/(key+'.yaml')).write_text(yaml.safe_dump(value),encoding='utf-8')
        self.node=Node('snapshot_cycle_acceptance',namespace=self.namespace)
        self.received={};self.forbidden=[];self.history=[];self.bindings=[];self.subs=[]
        self.active_frozen=False;self.frozen_generation=None;self.frozen_cells=None
        self.mutations=0;self.active_proposals=0;self.old_reuse=0;self.checks={}
        self.binding=None;self.counter=0;self.frame_counter=0;self.hold_bundle=False;self.poison=False
        self.invalid_bundle_faults=0
        self.cycle_discovery_ready=False
        self.processes={};self.logs={};self.commands={}
        for kind,topic,key,qos in (
            (PitSurfaceStatus,'perception/pit_surface_status','surface',QOS),
            (PitSurfaceSnapshot,'perception/pit_surface_snapshot','snapshot',SNAPSHOT_QOS),
            (GraspProposal,'planning/grasp_proposal_shadow','proposal',PROPOSAL_QOS),
            (TargetBinding,'phase2c/target_binding','binding',BIND_QOS),
            (GraspInteractionState,'phase2c/interaction_state','state',STATE_QOS)):
            def callback_for(key):
                def callback(m):
                    self.received[key]=m
                    if key=='state':
                        self.history.append(m)
                        assert not m.valid_for_control and not m.contact.valid_for_control and not m.payload.valid_for_control
                    if key=='binding':self.bindings.append(m)
                    if self.active_frozen:
                        if key=='surface' and m.surface_generation!=self.frozen_generation:self.mutations+=1
                        if key=='snapshot' and (m.status.surface_generation!=self.frozen_generation
                                or hashlib.sha256(repr(m.cells).encode()).hexdigest()!=self.frozen_cells):self.mutations+=1
                        if key=='proposal' and m.selected_candidate_id:self.active_proposals+=1
                return callback
            self.subs.append(self.node.create_subscription(kind,topic,callback_for(key),qos))
        for kind,topic in ((SafetyPermit,'safety/permit'),(AuthorizedCommand,'control/authorized_command'),
                           (ActuationRequest,'control/actuation_request')):
            self.subs.append(self.node.create_subscription(kind,topic,lambda m:self.forbidden.append(type(m).__name__),QOS))
        self.pubs={key:self.node.create_publisher(kind,topic,STATE_QOS if key in ('binding','cycle') else QOS) for key,kind,topic in (
            ('cloud',PointCloud2,'lidar/merged_points'),('meta',EvidenceMetadata,'perception/pit_surface_input_evidence'),
            ('context',PlanningSpatialContext,'planning/spatial_context'),('geometry',GrabGeometrySafetyState,'perception/grab_geometry_safety'),
            ('clearance',GrabClearanceState,'perception/grab_clearance'),('grab',GrabState,'perception/grab_state'),
            ('hoist',HoistState,'hardware/hoist_state'),('io',GrabIoState,'hardware/grab_io_state'),('load',LoadState,'hardware/load_state'),
            ('cycle',CycleStatus,'task/cycle_status'),('cart',UnloadingCartState,'perception/unloading_cart_state'),
            ('zone',UnloadSafeZone,'planning/unload_safe_zone'),('binding',TargetBinding,'phase2c/binding_request'))}
        self.bundle=AtomicLidarPublisher(self.pubs)
        grid=PitSurfaceGrid(self.c,self.semantic);self.xyz,self.sensors=scene(grid,'flat')
        extrinsics=[Extrinsic.from_config(dict(config_state='VALID',calibration_id='synthetic-only-'+str(i),
            source_frame='sensor-'+str(i),target_frame=self.ctx.source_frame,rotation_matrix=np.eye(3).tolist(),translation_m=[0,0,0])) for i in (0,1)]
        self.merger=DualLidarMerger(*extrinsics)
        if individual:
            for key,package,executable,file in (('surface','chili_crane_perception','pit_surface_shadow_node.py','surface'),
                    ('planner','chili_crane_perception','grasp_proposal_shadow_node.py','surface'),
                    ('interaction','chili_crane_control','grasp_interaction_shadow_node.py','interaction')):
                command=['ros2','run',package,executable,'--ros-args','-r','__ns:=/'+self.namespace,
                    '-p','config_file:='+str(self.directory/(file+'.yaml'))]
                if key!='interaction':command+=['-p','cycle_snapshot_mode:=true']
                self.start(key,command)
        else:
            self.start('stack',['ros2','launch','chili_crane_bringup','phase2c_shadow_stack.launch.py',
                'namespace:='+self.namespace,'surface_config_file:='+str(self.directory/'surface.yaml'),
                'interaction_config_file:='+str(self.directory/'interaction.yaml'),'commissioning:=true',
                'commissioning_config_file:='+str(ROOT/'config/commissioning/console.yaml'),
                'git_sha:='+subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
                'runs_directory:='+str(self.directory/'runs'),'commissioning_port:=0','commissioning_run_id:=installed-run'])

    def start(self,key,command):
        self.commands[key]=command
        log=(self.directory/(key+'-'+uuid.uuid4().hex[:6]+'.log')).open('w',encoding='utf-8')
        self.logs.setdefault(key,[]).append(log)
        self.processes[key]=start_owned(command,stdout=log,stderr=subprocess.STDOUT)

    def spin(self,duration=.05):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            assert all(p.poll() is None for p in self.processes.values()),'owned ROS process exited'
            rclpy.spin_once(self.node,timeout_sec=.005)

    def wait(self,condition,send=None,timeout=10.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if condition():return
            if send:send()
            else:self.spin()
        state=self.received.get('state');surface=self.received.get('surface');proposal=self.received.get('proposal')
        diagnostics=dict(owner=self.cycles.state.name,cycle_id=self.cycles.cycle_id,
            received={k:getattr(v,'reason','received') for k,v in self.received.items()},
            gates=json.loads(state.gate_diagnostics_json or '{}') if state else {},
            generation=surface.surface_generation if surface else None,
            observed_cycle=surface.observed_cycle_state if surface else None,
            proposal_generation=proposal.surface_generation if proposal else None,
            scan_cycle=proposal.scan_cycle_id if proposal else None,bundle_counts=self.bundle.counts,
            state_transitions=[(m.task_cycle_state,m.reason,m.abort_required) for m in self.history[-32:]])
        (self.directory/'timeout-diagnostics.json').write_text(json.dumps(diagnostics,indent=2)+'\n',encoding='utf-8')
        raise RuntimeError('R2 timeout '+json.dumps(diagnostics))

    def annotate(self,m,epoch,calibration,validity=4):
        m.header.frame_id='map';m.header.stamp=stamp(epoch);m.validity=validity
        age=max(0.,self.node.get_clock().now().nanoseconds/1e9-epoch)
        e=EvidenceMetadata(validity=validity,source_type=4,source_counter=self.counter,
            calibration_id=calibration,config_version=self.ic['config_version'],evidence_age_sec=age)
        e.header=copy.deepcopy(m.header);e.measurement_stamp=copy.deepcopy(m.header.stamp)
        e.receive_stamp=self.node.get_clock().now().to_msg();m.evidence=e
        if hasattr(m,'evidence_age_sec'):m.evidence_age_sec=age
        return m

    def make_bundle(self,now):
        first=now-.04;scans=[];points=self.xyz.copy()
        if self.poison:points[:,2]=1000.
        for sensor in (0,1):
            values=points[self.sensors==sensor];p=np.zeros(len(values),dtype=CANONICAL)
            for i,k in enumerate(('x','y','z')):p[k]=values[:,i]
            p['sensor_id']=sensor;p['intensity']=20
            p['timestamp']=np.linspace(first+sensor*.001,first+sensor*.001+.02,len(p))
            scans.append(Cloud(first+sensor*.001,time.monotonic(),'sensor-'+str(sensor),p,4))
        merged=self.merger.merge(tuple(scans));cloud=cloud_message(merged)
        timing=scan_epochs(seconds(cloud.header.stamp),merged.points['timestamp'],merged.points['sensor_id'])
        assert abs(timing.mid_epoch-(now-.0295))<1e-6
        self.frame_counter+=1
        context=context_to_message(replace(self.ctx,measurement_epoch=timing.header_epoch))
        context.header=copy.deepcopy(cloud.header);context.evidence.header=copy.deepcopy(cloud.header)
        context.evidence.measurement_stamp=copy.deepcopy(cloud.header.stamp)
        context.evidence.source_counter=self.frame_counter;context.evidence.config_version=self.c['config_version']
        context.evidence.receive_stamp=self.node.get_clock().now().to_msg();context.evidence.evidence_age_sec=now-timing.header_epoch
        meta=EvidenceMetadata(validity=4,source_type=4,source_counter=self.frame_counter,
            calibration_id=self.ctx.calibration_id,config_version=self.c['config_version'],evidence_age_sec=now-timing.header_epoch)
        meta.header=copy.deepcopy(cloud.header);meta.measurement_stamp=copy.deepcopy(cloud.header.stamp)
        return cloud,meta,context

    def send(self,z=4.,motion=0,opening=1,closed=False,opened=True,value=100.,stable=True,empty=False):
        self.counter+=1;now=self.node.get_clock().now().nanoseconds/1e9;epoch=now-.0295
        footprint=self.binding.capture_footprint if self.binding else self.initial_grab.footprint_xy
        geometry=self.annotate(GrabGeometrySafetyState(reason='SYNTHETIC_TRUE_MID_SCAN',source_type='SYNTHETIC',tracking_state='TRACKED',
            measurement_accepted=True,motion_stable=stable,model_id='synthetic-model',geometry_version='synthetic-v1',
            extrinsic_calibration_id=self.ctx.field_extrinsic_id,extrinsic_state='SYNTHETIC_VALIDATED',
            opening_state={1:'OPEN',2:'CLOSED',3:'TRANSITION'}[opening],bottom_lower_bound_m=z-.05,bottom_upper_bound_m=z+.05,
            top_upper_bound_m=z+.8,conservative_footprint=polygon(footprint),valid_for_control=False),epoch,'synthetic-extrinsic',3)
        clearance=self.annotate(GrabClearanceState(map_id=self.ctx.map_id,semantic_version=self.ctx.semantic_version,pit_id=self.ctx.pit_id,
            model_id=geometry.model_id,geometry_version=geometry.geometry_version,extrinsic_calibration_id=self.ctx.field_extrinsic_id,
            valid_for_control=False),epoch,self.ctx.calibration_id,3)
        grab=self.annotate(GrabState(tracking_state=2,opening_state=opening,source=1,motion_stable=stable),epoch,'synthetic-extrinsic',3)
        grab.pose.pose.position.z=z;grab.pose.pose.orientation.w=1.;grab.bottom_z_m=z-.05;grab.top_z_m=z+.8
        if self.binding:grab.pose.pose.position.x,grab.pose.pose.position.y=self.binding.target_suspension_map[:2]
        grab.footprint_map=copy.deepcopy(geometry.conservative_footprint)
        messages=dict(
            cycle=self.annotate(CycleStatus(run_id=self.cycles.run_id,task_id=self.cycles.task_id,cycle_id=self.cycles.cycle_id,
                cycle_index=self.cycles.cycle_index,state=int(self.cycles.state),needs_new_scan=self.cycles.needs_new_scan),now,'synthetic-cycle'),
            geometry=geometry,clearance=clearance,grab=grab,
            hoist=self.annotate(HoistState(motion=motion,running=motion!=0),epoch,'synthetic-hoist'),
            io=self.annotate(GrabIoState(open_limit=opened,closed_limit=closed,known_signals=['open_limit','closed_limit']),epoch,'synthetic-io'),
            load=self.annotate(LoadState(value=value,unit='raw',calibration_id='synthetic-load',load_state=1 if empty else 3),epoch,'synthetic-load'),
            cart=self.annotate(UnloadingCartState(station_id='synthetic-only',present=True,aligned=True),epoch,'synthetic-cart'),
            zone=self.annotate(UnloadSafeZone(frame_id='map',x_min=-3.,x_max=3.,y_min=-3.,y_max=3.,z_min=3.5,z_max=5.,
                geometry_version='synthetic-zone',flow_projection_verified=True),epoch,'synthetic-zone'))
        for key,m in messages.items():self.pubs[key].publish(m)
        if not self.hold_bundle:self.bundle.publish_due(time.monotonic(),lambda:self.make_bundle(now))
        self.spin(.05)

    def scan_and_bind(self):
        if not self.cycle_discovery_ready:
            required={'pit_surface_shadow_node','grasp_proposal_shadow_node','grasp_interaction_shadow_node'}
            def discovered():
                endpoints=self.node.get_subscriptions_info_by_topic('task/cycle_status')
                names={info.node_name for info in endpoints if info.node_namespace=='/'+self.namespace}
                return required<=names and self.pubs['binding'].get_subscription_count()>0
            # No SCAN heartbeat before all three real consumers have joined.
            self.wait(discovered)
            self.cycle_discovery_ready=True
        def surface_drained():
            s=self.received.get('surface');m=self.received.get('snapshot')
            return (s is not None and m is not None and s.spatial_eligible and s.support_qualified_cell_count>0
                and s.scan_cycle_id==self.cycles.cycle_id and s.snapshot_id==m.status.snapshot_id
                and s.evidence.source_counter==self.frame_counter)
        self.wait(surface_drained,send=lambda:self.send(empty=True))
        self.hold_bundle=True  # All published frames consumed before owner leaves SCAN.
        self.wait(lambda:'proposal' in self.received and bool(self.received['proposal'].selected_candidate_id)
            and self.received['proposal'].scan_cycle_id==self.cycles.cycle_id
            and self.received['proposal'].surface_snapshot_id==self.received['surface'].snapshot_id,
            send=lambda:self.send(empty=True))
        assert self.cycles.accept_scan(int(self.received['proposal'].evidence.source_counter),fresh=True)
        self.send(empty=True)
        self.wait(lambda:'surface' in self.received and self.received['surface'].observed_cycle_state==int(CycleState.PLAN_PICK)
            and self.received['proposal'].surface_generation==self.received['surface'].surface_generation
            and bool(self.received['proposal'].selected_candidate_id),send=lambda:self.send(empty=True))
        p=self.received['proposal'];s=self.received['surface']
        assert seconds(s.window_start)>seconds(s.scan_start_stamp)
        assert seconds(s.window_end)>seconds(s.scan_start_stamp)
        self.saved_proposal=copy.deepcopy(p)
        b=binding_from_proposal(p,self.cycles.run_id,self.cycles.task_id,self.cycles.cycle_id,self.cycles.cycle_index,
            self.node.get_clock().now().nanoseconds/1e9)
        self.pubs['binding'].publish(binding_to_message(b))
        next_retry=time.monotonic()+.2
        def await_binding():
            nonlocal next_retry
            self.send(empty=True)
            if time.monotonic()>=next_retry:
                # Exact same request only: never select a replacement candidate.
                self.pubs['binding'].publish(binding_to_message(b));next_retry=time.monotonic()+.2
        self.wait(lambda:'binding' in self.received and self.received['binding'].binding_digest_sha256==b.binding_digest_sha256,
            send=await_binding)
        self.binding=b
        self.wait(lambda:self.received['proposal'].reason=='PLANNING_FROZEN_ACTIVE_CYCLE',send=lambda:self.send(empty=True))
        self.hold_bundle=False
        self.frozen_generation=s.surface_generation;self.frozen_cells=hashlib.sha256(repr(self.received['snapshot'].cells).encode()).hexdigest()
        self.active_frozen=True;self.poison=True
        for _ in range(12):self.send(empty=True)
        self.wait(lambda:json.loads(self.received['state'].load_features_json or '{}').get('baseline_valid',False),
            send=lambda:self.send(empty=True))
        self.checks.update(SCAN_BUILD_SNAPSHOT='PASS',PLAN_BIND_FREEZES_SURFACE='PASS')

    def admission(self):
        m=self.received['state']
        r=SimpleNamespace(phase=m.task_cycle_state,abort_required=m.abort_required or m.validity!=3,
            interaction=dict(state=m.interaction.reason),closure=dict(state=m.closure.reason),payload=dict(state=m.payload.reason),
            lift=dict(pit_exit_confirmed=m.lift.pit_exit_confirmed),unload=dict(state=m.unload.reason))
        return self.cycles.advance(evidence_verified=evidence_allows_advance(self.cycles,r))

    def state_is_current(self):
        m=self.received.get('state')
        return m is not None and (m.run_id,m.task_id,m.cycle_id,m.task_cycle_state)==(
            self.cycles.run_id,self.cycles.task_id,self.cycles.cycle_id,self.cycles.state.name)

    def grasp(self):
        self.scan_and_bind()
        assert self.cycles.advance(evidence_verified=True);self.send(empty=True)  # mock positioning
        assert self.cycles.advance(evidence_verified=True);self.send(empty=True)
        assert self.cycles.advance(evidence_verified=True);self.send()
        assert self.cycles.advance(evidence_verified=True)
        for i in range(22):self.send(z=4.-3.2*(i+1)/22,motion=-1,stable=False)
        for i in range(6):self.send(z=.8-.006*i,motion=-1,stable=False)
        self.wait(lambda:self.received['state'].interaction.reason=='INTERACTION_COMMITTED',send=lambda:self.send(z=.76,motion=-1,stable=False))
        assert not self.received['state'].interaction.surface_invalidation_required
        assert self.admission()
        for _ in range(8):self.send(z=.76,opening=2,closed=True,opened=False)
        self.wait(lambda:self.received['state'].closure.reason=='CLOSURE_CONFIRMED',send=lambda:self.send(z=.76,opening=2,closed=True,opened=False))
        assert self.admission()
        self.wait(self.state_is_current,send=lambda:self.send(z=.76,opening=2,closed=True,opened=False))
        assert self.admission()
        for i in range(25):self.send(z=.76+3.24*(i+1)/25,motion=1,opening=2,closed=True,opened=False,value=101.,stable=False)
        self.wait(lambda:self.received['state'].payload.reason=='PAYLOAD_CONFIRMED',timeout=9.,
            send=lambda:self.send(opening=2,closed=True,opened=False,value=101.))
        assert not self.received['state'].next_cycle_surface_ready
        assert not self.received['state'].lift.surface_refresh_ready
        assert not self.received['state'].lift.clear_of_updated_surface
        self.checks.update(CLOUD_DURING_LOWER_IGNORED='PASS',CLOUD_DURING_CLOSE_IGNORED='PASS',CLOUD_DURING_RAISE_IGNORED='PASS',
            SURFACE_GENERATION_STABLE_DURING_ACTIVE_GRAB='PASS',PLANNER_DISABLED_DURING_ACTIVE_GRAB='PASS')

    def happy_cycle(self):
        self.grasp();assert self.received['state'].lift.pit_exit_confirmed;assert self.admission()
        for _ in range(3):self.send(opening=2,closed=True,opened=False,value=101.)
        for _ in range(3):
            assert self.cycles.advance(evidence_verified=True)  # mock arrival/cart positioning
            self.send(opening=2,closed=True,opened=False,value=101.)
        for _ in range(12):self.send(value=100.)
        assert self.cycles.advance(evidence_verified=True)
        self.wait(lambda:self.state_is_current() and self.received['state'].unload.reason=='UNLOAD_CONFIRMED',
            timeout=10.,send=lambda:self.send(value=100.))
        assert not self.received['state'].next_cycle_surface_ready;assert self.admission();self.send()
        assert self.cycles.state==CycleState.COMPLETE
        self.wait(lambda:self.state_is_current()
            and self.received['surface'].observed_cycle_state==int(CycleState.COMPLETE)
            and self.received['proposal'].observed_cycle_state==int(CycleState.COMPLETE),send=self.send)
        self.checks.update(PIT_EXIT_USES_BOUND_SURFACE='PASS',COMPLETE_DOES_NOT_REQUIRE_NEXT_PLAN='PASS')
        old=self.binding;old_proposal=copy.deepcopy(self.saved_proposal)
        assert self.cycles.next_cycle(continue_task=True)
        self.active_frozen=False;self.poison=False;self.hold_bundle=True
        self.xyz[:,2]=.65  # A genuinely new synthetic material observation in cycle 2.
        for _ in range(5):self.send(empty=True)
        before=len(self.bindings)
        replay=binding_from_proposal(old_proposal,self.cycles.run_id,self.cycles.task_id,self.cycles.cycle_id,
            self.cycles.cycle_index,self.node.get_clock().now().nanoseconds/1e9)
        self.pubs['binding'].publish(binding_to_message(replay))
        for _ in range(5):self.send(empty=True)
        assert len(self.bindings)==before,'old proposal bound without a new scan'
        assert not self.received['proposal'].selected_candidate_id
        self.hold_bundle=False;self.scan_and_bind()
        if self.binding.surface_session_id==old.surface_session_id and self.binding.surface_generation<=old.surface_generation:self.old_reuse+=1
        assert self.binding.surface_nominal_m<old.surface_nominal_m-.1,'new scan did not replace the old material window'
        assert self.old_reuse==0
        self.checks.update(NEW_CYCLE_REQUIRES_NEW_SCAN='PASS',OLD_PROPOSAL_CANNOT_BIND_NEXT_CYCLE='PASS',NEW_SCAN_NEW_PROPOSAL_CAN_BIND='PASS')

    def restart_abort(self):
        self.grasp();old=self.received['surface'].surface_session_id
        stop_owned(self.processes.pop('surface'));self.active_frozen=False
        self.start('surface',self.commands['surface'])
        self.wait(lambda:self.received['surface'].surface_session_id!=old and self.received['state'].abort_required,
            send=lambda:self.send(opening=2,closed=True,opened=False,value=101.))
        assert not self.received['surface'].spatial_eligible and not self.received['proposal'].selected_candidate_id
        self.checks['SURFACE_RESTART_ACTIVE_CYCLE_ABORT']='PASS'

    def strict_pairing(self):
        self.hold_bundle=True
        self.wait(lambda:all(self.pubs[k].get_subscription_count()>0 for k in ('cloud','meta','context','cycle')),send=self.send)
        self.wait(lambda:'surface' in self.received and self.received['surface'].scan_cycle_id==self.cycles.cycle_id
            and seconds(self.received['surface'].scan_start_stamp)>0,send=self.send)
        for _ in range(3):self.send()
        now=self.node.get_clock().now().nanoseconds/1e9
        cloud,meta,context=self.make_bundle(now);meta.measurement_stamp=stamp(now-1.)
        self.pubs['meta'].publish(meta);self.pubs['context'].publish(context);self.pubs['cloud'].publish(cloud)
        self.invalid_bundle_faults+=1
        self.wait(lambda:'surface' in self.received and 'INPUT_EPOCH_PAIR_PENDING' in self.received['surface'].reason,send=self.send,timeout=2.)
        assert not self.received['surface'].spatial_eligible
        self.hold_bundle=False
        self.wait(lambda:'proposal' in self.received and bool(self.received['proposal'].selected_candidate_id),send=self.send)
        self.checks['STRICT_FRAME_PAIRING_PRESERVED']='PASS'

    def close(self):
        errors=[]
        for process in list(self.processes.values()):
            try:stop_owned(process)
            except (OSError,RuntimeError) as exc:errors.append(str(exc))
        for logs in self.logs.values():
            for log in logs:log.close()
        self.node.destroy_node()
        if errors:raise RuntimeError('; '.join(errors))
        assert owned_process_leak_count()==0

    def result(self):
        assert not self.forbidden and self.mutations==0 and self.active_proposals==0 and self.old_reuse==0
        assert len(set(self.bundle.counts.values()))==1 and self.bundle.counts['cloud']>0
        self.checks.update(LIDAR_BUNDLE_ATOMIC='PASS',NO_INTERFRAME_META_CONTEXT_OVERWRITE='PASS',VALIDATOR_PROCESS_LEAK_ZERO='PASS')
        return dict(status='PASS_INSTALLED_R2_SYNTHETIC',checks=self.checks,
            ACTIVE_GRAB_SURFACE_MUTATION_COUNT=self.mutations,ACTIVE_GRAB_NEW_PROPOSAL_COUNT=self.active_proposals,
            NEXT_CYCLE_OLD_SURFACE_REUSE_COUNT=self.old_reuse,CONTROL_TOPIC_MESSAGES=len(self.forbidden),
            VALIDATOR_OWNED_PROCESS_LEAK_COUNT=owned_process_leak_count(),bundle_counts=self.bundle.counts,
            intentional_invalid_bundle_count=self.invalid_bundle_faults,
            physical_output_enabled=False,automatic_control_enabled=False,valid_for_control=False)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--evidence-dir',type=Path,required=True)
    parser.add_argument('--probe',type=int,default=0,choices=(0,10));args=parser.parse_args()
    args.evidence_dir.mkdir(parents=True,exist_ok=True);rclpy.init();results=[]
    cases=[('CRITICAL_'+str(i+1),'happy') for i in range(args.probe)] if args.probe else [
        ('SNAPSHOT_CYCLE_WITH_RESCAN','happy'),('SURFACE_RESTART_ACTIVE_CYCLE_ABORT','restart'),
        ('MISSING_PIT_GEOMETRY_BLOCKS_ADVANCE','rim'),('STRICT_SCAN_FRAME_PAIRING','pairing')]
    try:
        for name,kind in cases:
            run=SnapshotRun(args.evidence_dir,name,individual=kind=='restart',missing_rim=kind=='rim')
            try:
                if kind=='happy':run.happy_cycle()
                elif kind=='restart':run.restart_abort()
                elif kind=='rim':
                    run.grasp();assert not run.received['state'].lift.pit_exit_confirmed;assert not run.admission()
                    run.checks['MISSING_PIT_GEOMETRY_BLOCKS_ADVANCE']='PASS'
                else:run.strict_pairing()
            finally:run.close()
            result=run.result();result['scenario']=name;results.append(result)
    finally:
        if rclpy.ok():rclpy.shutdown()
    result=dict(status='PASS_INSTALLED_R2_SYNTHETIC',scope='SOFTWARE_SHADOW_NOT_FIELD',
        git_sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        probe_passes=len(results) if args.probe else None,scenarios=results,
        BENCH_DO_STATUS='NOT_RUN',FIELD_STATUS='NOT_RUN',FULL_AUTO='NOT_RUN')
    (args.evidence_dir/'result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result))


if __name__=='__main__':main()
