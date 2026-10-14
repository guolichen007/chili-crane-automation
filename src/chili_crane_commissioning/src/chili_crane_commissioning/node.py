"""ROS monitor bridge and explicitly local-only headless software preview."""
import argparse
import copy
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import threading
import time
import uuid
import yaml
from .backend import ConsoleBackend
from .bench import BenchService
from .recording import RunRecorder
from .bag import BagRecorder
from .server import ConsoleServer


def main():
    parser=argparse.ArgumentParser(description='Local software-shadow commissioning console; no production control')
    parser.add_argument('--config',required=True);parser.add_argument('--git-sha',required=True)
    parser.add_argument('--run-id',default=None);parser.add_argument('--runs-directory',default='runs')
    parser.add_argument('--crane-namespace',default='crane_01');parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=8080);parser.add_argument('--allow-nonlocal',action='store_true')
    parser.add_argument('--allowed-host',action='append',default=[])
    parser.add_argument('--bench-config');parser.add_argument('--enable-isolated-bench-output',action='store_true')
    parser.add_argument('--confirm-isolated-bench',action='store_true');parser.add_argument('--confirm-exclusive-device-session',action='store_true')
    parser.add_argument('--recording-config');parser.add_argument('--headless-preview',action='store_true')
    args,ros_args=parser.parse_known_args()
    config=yaml.safe_load(Path(args.config).read_text(encoding='utf-8'))
    if args.headless_preview:
        if args.enable_isolated_bench_output:raise PermissionError('HEADLESS_PREVIEW_FORBIDS_PHYSICAL_OUTPUT')
        config=dict(config,mode='HEADLESS_PREVIEW_ROS_NOT_CONNECTED')
    bench_config=yaml.safe_load(Path(args.bench_config).read_text(encoding='utf-8')) if args.bench_config else {}
    recording_config=yaml.safe_load(Path(args.recording_config).read_text(encoding='utf-8')) if args.recording_config else {}
    if args.headless_preview:recording_config={}
    config_bundle=dict(console=config,bench=bench_config,recording=recording_config,
        bench_cli_flags=dict(enable=args.enable_isolated_bench_output,isolated=args.confirm_isolated_bench,
                             exclusive=args.confirm_exclusive_device_session))
    config_hash=hashlib.sha256(json.dumps(config_bundle,sort_keys=True,allow_nan=False).encode()).hexdigest()
    recorder=RunRecorder(args.runs_directory,args.run_id or uuid.uuid4().hex,args.git_sha,config_hash,config['recording_profile'])
    bench=BenchService(bench_config,recorder,args.enable_isolated_bench_output,args.confirm_isolated_bench,args.confirm_exclusive_device_session)
    backend=ConsoleBackend(config,recorder,bench)
    backend.bag=BagRecorder(recording_config,recorder,args.crane_namespace)
    ros_node=None;shutdown_exceptions=(KeyboardInterrupt,)
    if not args.headless_preview:
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import ExternalShutdownException
        shutdown_exceptions+=(ExternalShutdownException,)
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        from chili_crane_msgs.msg import (RemoteControlState,RunManifest,TaskStatus,CycleStatus,SystemModeStatus,SystemReadiness,
            GrabState,GrabGeometrySafetyState,PitSurfaceStatus,PitSurfaceSnapshot,GraspProposal,TargetBinding,
            GraspInteractionState,ContactEvidence,SurfaceInteractionEvidence,ClosureEvidence,PayloadEvidence,LiftEvidence,UnloadEvidence,
            SurfaceRefreshEvidence,GrabIoState,HoistState,LoadState,SafetyEvent,ControlBoardState,
            CommissioningShadowOutput,BenchOutputEvidence)
        rclpy.init(args=ros_args);ros_node=Node('commissioning_console',namespace=args.crane_namespace)
        recorder.ros_now=lambda:ros_node.get_clock().now().nanoseconds/1e9
        started_stamp=ros_node.get_clock().now().to_msg()
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)
        latest=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        remote_pub=ros_node.create_publisher(RemoteControlState,'commissioning/simulation/remote_control_state',reliable)
        expected_pub=ros_node.create_publisher(CommissioningShadowOutput,'commissioning/simulation/expected_output',reliable)
        bench_pub=ros_node.create_publisher(BenchOutputEvidence,'commissioning/bench/do_readback',reliable)
        manifest_pub=ros_node.create_publisher(RunManifest,'commissioning/run_manifest',reliable)
        def publish_remote(state):
            m=RemoteControlState(validity=4,reason='ISOLATED_SIMULATION_OBSERVATION_ONLY',receiver_ready=True,
                safety_ok=True,e_stop_known=True,control_mode='REMOTE_SIMULATED',release_automatic_outputs=True,
                discard_pending_commands=True,stop_requested=state['stop_requested'],evidence_age_sec=0.)
            if state['active']:setattr(m,state['signal'],True)
            m.known_signals=list(('y_left','y_right','z_up','z_down','g_open','g_close','stop_requested'))
            m.header.frame_id='commissioning_simulation';m.header.stamp=ros_node.get_clock().now().to_msg()
            m.evidence.header=m.header;m.evidence.measurement_stamp=m.header.stamp;m.evidence.receive_stamp=m.header.stamp
            m.evidence.validity=4;m.evidence.source_type=m.evidence.SOURCE_SIMULATED
            m.evidence.source_counter=state['source_counter'];m.evidence.calibration_id='SIMULATED_REMOTE_NOT_FIELD'
            m.evidence.config_version=config['config_version'];m.evidence.evidence_age_sec=0.
            remote_pub.publish(m)
            expected=CommissioningShadowOutput(validity=3,reason='SHADOW_EXPECTATION_NOT_DO_WRITE',
                semantic_signal=state['signal'],arbitration_mode=state['control_mode'],expected_do=state['expected_do_preview'],
                software_only=True,physical_output_enabled=False,automatic_control_enabled=False)
            expected.header=copy.deepcopy(m.header);expected.evidence=copy.deepcopy(m.evidence);expected.evidence.validity=3
            expected_pub.publish(expected)
        backend.publish_remote=publish_remote
        def publish_bench(record):
            d=record['details'];m=BenchOutputEvidence(validity=3,reason='HOST_READBACK_NOT_VOLTAGE_OR_FIELD',
                bench_test_id=d['bench_test_id'],run_id=recorder.run_id,git_sha=args.git_sha,config_hash=config_hash,
                semantic_signal=d['semantic_signal'],device=d['device'],channel=d['channel'],
                requested_duration_ms=d['requested_duration_ms'],pre_readback_json=json.dumps(d['pre_readback']),
                active_readback_json=json.dumps(d['active_readback']),post_readback_json=json.dumps(d['post_readback']),
                result=d['result'],voltage_measured=False)
            m.header.stamp=ros_node.get_clock().now().to_msg();m.header.frame_id='isolated_bench'
            m.evidence.header=m.header;m.evidence.measurement_stamp=m.header.stamp;m.evidence.receive_stamp=m.header.stamp
            m.evidence.validity=3;m.evidence.source_type=4 if record['source_type']=='SYNTHETIC' else 1
            m.evidence.calibration_id='BENCH_LOGICAL_REGISTER_READBACK_NOT_FIELD';m.evidence.config_version=bench_config.get('config_version','BENCH_EXPLICIT_CONFIG')
            bench_pub.publish(m)
        bench.publish_record=publish_bench
        selected=('validity','reason','state','task_cycle_state','run_id','task_id','cycle_id','binding_id','binding_digest_sha256',
            'selected_candidate_id','snapshot_id','surface_generation','opening_state','tracking_state','motion_stable',
            'value','delta','unit','load_state','motion','fault','event_types','abort_required','next_cycle_surface_ready',
            'refresh_ready','invalidation_acknowledged','observation_enabled','qualified_cell_count','affected_cell_count',
            'map_id','semantic_version','pit_id','surface_session_id','surface_snapshot_id','spatial_calibration_id',
            'geofence_id','geofence_version','kernel_id','load_features_json','gate_diagnostics_json',
            'refreshed_cell_count','unknown_cell_count','observed_fraction','generation_ready')
        def mirror(topic,m):
            summary={name:getattr(m,name) for name in selected if hasattr(m,name)}
            if hasattr(m,'target_suspension_map'):
                p=m.target_suspension_map;summary['target_xyz']=[p.x,p.y,p.z]
            for name in ('contact','interaction','closure','payload','lift','unload'):
                if hasattr(m,name):
                    v=getattr(m,name);summary[name]={key:getattr(v,key) for key in ('state','reason','clear_of_bound_surface',
                        'clear_of_updated_surface','clear_of_pit','safe_lift_height') if hasattr(v,key)}
            if hasattr(m,'status'):
                summary.update(validity=m.status.validity,reason=m.status.reason,surface_generation=m.status.surface_generation,
                               surface_session_id=m.status.surface_session_id)
            e=getattr(m,'evidence',getattr(getattr(m,'status',None),'evidence',None))
            source={1:'PHYSICAL',2:'SIMULATED',3:'REPLAY',4:'SYNTHETIC'}.get(e.source_type,'UNKNOWN') if e else 'UNKNOWN'
            epoch=m.header.stamp.sec+m.header.stamp.nanosec/1e9 if hasattr(m,'header') else None
            if e:
                summary.update(reported_age_sec=e.evidence_age_sec,evidence_source_counter=e.source_counter,
                    evidence_calibration_id=e.calibration_id,evidence_config_version=e.config_version)
            backend.mirror(topic,summary,source,epoch)
        topics=(('task/status',TaskStatus,qos),('task/cycle_status',CycleStatus,qos),('system/mode',SystemModeStatus,qos),
            ('system/readiness',SystemReadiness,qos),('perception/grab_state',GrabState,qos),
            ('perception/grab_geometry_safety',GrabGeometrySafetyState,qos),('perception/pit_surface_status',PitSurfaceStatus,qos),
            ('perception/pit_surface_snapshot',PitSurfaceSnapshot,latest),('planning/grasp_proposal_shadow',GraspProposal,reliable),
            ('phase2c/target_binding',TargetBinding,latest),('phase2c/interaction_state',GraspInteractionState,reliable),
            ('phase2c/contact_evidence',ContactEvidence,reliable),('phase2c/interaction_evidence',SurfaceInteractionEvidence,reliable),
            ('phase2c/closure_evidence',ClosureEvidence,reliable),('phase2c/payload_evidence',PayloadEvidence,reliable),
            ('phase2c/lift_evidence',LiftEvidence,reliable),('phase2c/unload_evidence',UnloadEvidence,reliable),
            ('perception/surface_refresh_evidence',SurfaceRefreshEvidence,reliable),('hardware/grab_io_state',GrabIoState,qos),
            ('hardware/hoist_state',HoistState,qos),('hardware/load_state',LoadState,qos),
            ('hardware/remote_control_state',RemoteControlState,qos),('hardware/control_board_state',ControlBoardState,qos),
            ('safety/event',SafetyEvent,reliable))
        def callback_for(topic):
            def callback(message):
                mirror(topic,message)
            return callback
        subscriptions=[ros_node.create_subscription(kind,topic,callback_for(topic),profile) for topic,kind,profile in topics]
        manifest_counter=0
        def manifest():
            nonlocal manifest_counter
            manifest_counter+=1
            m=RunManifest(validity=3,reason='COMMISSIONING_RECORD_NOT_FIELD_ACCEPTANCE',run_id=recorder.run_id,
                git_sha=args.git_sha,config_hash=config_hash,schema_version='RUN_MANIFEST_V3',phase='2C')
            for key,value in backend.bag.status().items():setattr(m,key,value)
            m.header.stamp=ros_node.get_clock().now().to_msg();m.evidence.header=m.header
            m.started_stamp=started_stamp;m.task_id=backend.context['task_id'];m.cycle_id=backend.context['cycle_id']
            m.evidence.measurement_stamp=m.header.stamp;m.evidence.receive_stamp=m.header.stamp
            m.evidence.validity=3;m.evidence.source_type=2;m.evidence.source_counter=manifest_counter
            m.evidence.calibration_id='COMMISSIONING_RECORD_NOT_SENSOR_CALIBRATION'
            m.evidence.config_version=config['config_version']
            manifest_pub.publish(m)
        ros_node.create_timer(1.,manifest)
    server=ConsoleServer((args.host,args.port),backend,allow_nonlocal=args.allow_nonlocal,allowed_hosts=args.allowed_host)
    server_thread=threading.Thread(target=server.serve_forever,daemon=True);server_thread.start()
    print('LOCAL_URL=http://'+args.host+':'+str(server.server_port),flush=True)
    print('SCOPE=SOFTWARE_SHADOW; PHYSICAL_OUTPUT=false; BENCH_DO_STATUS=NOT_RUN',flush=True)
    try:
        while ros_node is None or rclpy.ok():
            if ros_node is not None:rclpy.spin_once(ros_node,timeout_sec=.05)
            else:time.sleep(.05)
            backend.tick()
    except shutdown_exceptions:pass
    finally:
        try:backend.shutdown()
        finally:
            server.shutdown();server.server_close();server_thread.join(timeout=2.)
            if ros_node is not None:
                ros_node.destroy_node()
                if rclpy.ok():rclpy.shutdown()
