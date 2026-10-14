from contextlib import contextmanager
import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from urllib.request import Request,urlopen
from urllib.error import HTTPError
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/chili_crane_commissioning/src'),str(ROOT/'src/chili_crane_control/src'),str(ROOT/'src/chili_crane_hardware/src')]
from chili_crane_commissioning.recording import RunRecorder
from chili_crane_commissioning.backend import ConsoleBackend
from chili_crane_commissioning.bench import BenchService,expected_outputs
from chili_crane_commissioning.server import ConsoleServer
from chili_crane_commissioning.bag import BagRecorder
from chili_crane_hardware.adam.adam6052_driver import Adam6052Driver
from chili_crane_hardware.adam.register_map import Adam6052RegisterMap as Registers


class FakeTransport:
    def __init__(self):self.outputs=[False]*8;self.calls=[];self.fail_on=False;self.exclusive=0
    def read_coils(self,offset,count):return tuple(self.outputs)
    def write_all_off(self,offset,count):self.calls.append('OFF');self.outputs=[False]*8
    def write_single_coil(self,offset,value):
        self.calls.append(('ON',offset));self.outputs[offset-Registers.DO_OFFSET]=value
        if self.fail_on:raise IOError('injected transport failure after ON')
    @contextmanager
    def exclusive_session(self):self.exclusive+=1;yield


def bench_config():
    return dict(source_type='SYNTHETIC',physical_output_enabled=True,automatic_control_enabled=False,
        acceptance=dict(ADAM_FSV_CONFIGURED=True,ADAM_WATCHDOG_CONFIGURED=True,ADAM_SAFE_OUTPUT_VERIFIED=True,
                        watchdog_timeout_sec=.5,verification_evidence='SYNTHETIC_ONLY_NOT_FIELD'))


class CommissioningTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.recorder=RunRecorder(self.temp.name,'test-run','a'*40,'b'*64)
        self.config=yaml.safe_load((ROOT/'config/commissioning/console.yaml').read_text())
    def tearDown(self):self.temp.cleanup()

    def bench(self,config=None,**flags):
        transport=FakeTransport()
        args=dict(enable_cli=True,isolated_cli=True,exclusive_cli=True);args.update(flags)
        service=BenchService(config or bench_config(),self.recorder,driver_factory=lambda gate:Adam6052Driver(transport,gate),**args)
        return service,transport

    def test_default_bench_disabled_and_missing_every_gate(self):
        service=BenchService({},self.recorder)
        self.assertFalse(service.enabled)
        with self.assertRaises(PermissionError):service.pulse('y_left')
        for key in ('ADAM_FSV_CONFIGURED','ADAM_WATCHDOG_CONFIGURED','ADAM_SAFE_OUTPUT_VERIFIED','verification_evidence','watchdog_timeout_sec'):
            c=bench_config();c['acceptance'][key]=False
            service,transport=self.bench(c)
            with self.assertRaises(PermissionError):service.pulse('y_left')
            self.assertFalse(transport.calls)
        for key in ('enable_cli','isolated_cli','exclusive_cli'):
            service,transport=self.bench(**{key:False})
            with self.assertRaises(PermissionError):service.pulse('y_left')
            self.assertFalse(transport.calls)

    def test_six_mappings_reserved_and_pulse_limits(self):
        for signal,channel in (('y_left',0),('y_right',1),('z_up',2),('z_down',3),('g_open',4),('g_close',5)):
            self.assertEqual(expected_outputs(signal).index(True),channel)
            service,transport=self.bench();record=service.pulse(signal,500,sleeper=lambda seconds:None)
            self.assertFalse(any(transport.outputs));self.assertGreaterEqual(transport.calls.count('OFF'),2)
            self.assertEqual(record['details']['channel'],channel)
            self.assertEqual(record['details']['active_readback'],list(expected_outputs(signal)))
            self.assertEqual(record['details']['post_readback'],[False]*8)
            self.assertIsNone(record['details']['measured_voltage'])
        for signal in ('x_positive','do6','do7','y_left,z_up',['y_left','z_up']):
            with self.assertRaises(ValueError):expected_outputs(signal)
        service,transport=self.bench()
        for duration in (1,0,-1,1001,500.5,True):
            with self.assertRaises(ValueError):service.pulse('y_left',duration)

    def test_mock_source_cannot_construct_real_transport_or_claim_physical_evidence(self):
        service=BenchService(bench_config(),self.recorder,True,True,True)
        with self.assertRaisesRegex(PermissionError,'REAL_TRANSPORT_REQUIRES'):
            service.pulse('y_left')
        config=dict(bench_config(),source_type='PHYSICAL')
        service,transport=self.bench(config)
        with self.assertRaisesRegex(PermissionError,'MOCK_FACTORY_REQUIRES'):
            service.pulse('y_left')
        self.assertFalse(transport.calls)

        service=BenchService(dict(bench_config(),source_type='PHYSICAL'),self.recorder,True,True,True)
        with self.assertRaisesRegex(PermissionError,'BENCH_PROFILE_NOT_CONFIGURED'):
            service.pulse('y_left')

    def test_exception_always_off_and_record_not_voltage(self):
        service,transport=self.bench();transport.fail_on=True
        with self.assertRaises(IOError):service.pulse('y_left',500,sleeper=lambda seconds:None)
        self.assertFalse(any(transport.outputs));self.assertGreaterEqual(transport.calls.count('OFF'),2)
        self.assertEqual(service.last['details']['result'],'FAILED')
        self.assertIsNone(service.last['details']['measured_voltage'])

    def test_concurrent_direction_rejected(self):
        service,transport=self.bench();entered=threading.Event();leave=threading.Event()
        def sleeper(seconds):entered.set();leave.wait(1.)
        worker=threading.Thread(target=lambda:service.pulse('y_left',500,sleeper=sleeper));worker.start()
        self.assertTrue(entered.wait(1.))
        with self.assertRaisesRegex(ValueError,'ALREADY_ACTIVE'):service.pulse('z_up',500)
        leave.set();worker.join(2.);self.assertFalse(any(transport.outputs))

    def test_simulation_uses_arbitration_and_never_real_do(self):
        published=[];backend=ConsoleBackend(self.config,self.recorder,publish_remote=published.append)
        state=backend.remote('z_up',True)
        self.assertEqual(state['source_type'],'SIMULATED')
        self.assertTrue(state['expected_do_preview'][2]);self.assertFalse(state['physical_output_enabled'])
        with self.assertRaises(ValueError):backend.remote('y_left',True)
        self.assertEqual(backend.active,'stop');self.assertFalse(backend.bench.enabled)
        backend.remote('g_close',True);backend.release('UI_BLUR')
        self.assertEqual(published[-1]['signal'],'stop')

    def test_release_disconnect_lease_and_shutdown(self):
        config=dict(self.config,simulation_lease_sec=.025);backend=ConsoleBackend(config,self.recorder)
        with patch('chili_crane_commissioning.backend.time.monotonic',return_value=100.):
            backend.remote('y_left',True)
        with patch('chili_crane_commissioning.backend.time.monotonic',return_value=100.024):backend.tick()
        self.assertEqual(backend.active,'y_left')
        with patch('chili_crane_commissioning.backend.time.monotonic',return_value=100.026):backend.tick()
        self.assertEqual(backend.active,'stop')
        backend.remote('y_right',True);backend.shutdown();self.assertEqual(backend.active,'stop')
        self.assertIn(b'UI_DISCONNECT_OR_LEASE_EXPIRED',self.recorder.read('events'))

    def test_append_only_provenance_and_owned_paths(self):
        self.recorder.append('events','ONE');first=self.recorder.read('events')
        self.recorder.append('events','TWO');second=self.recorder.read('events')
        self.assertTrue(second.startswith(first));self.assertEqual(len(second.splitlines()),2)
        record=json.loads(first)
        for key in ('wall_timestamp','ros_timestamp','monotonic_timestamp','run_id','task_id','cycle_id',
                    'binding_id','git_sha','config_hash','source_type','event_type','old_state','new_state','reason'):
            self.assertIn(key,record)
        with self.assertRaises(ValueError):RunRecorder(self.temp.name,'../escape','a'*40,'b'*64)
        with self.assertRaises(FileExistsError):RunRecorder(self.temp.name,'test-run','a'*40,'b'*64)
        with self.assertRaises(ValueError):self.recorder.read('../../manifest')

    def test_generated_run_records_are_forbidden_public_delivery(self):
        import importlib.util
        spec=importlib.util.spec_from_file_location('delivery_check',ROOT/'tools/check_delivery.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        for name in ('runs/test/manifest.json','runs/test/events.jsonl','runs/test/bench_measurements.jsonl'):
            self.assertTrue(module.forbidden_path(name),name)

    def test_surface_invalidation_and_refresh_are_recorded_once_per_binding(self):
        backend=ConsoleBackend(self.config,self.recorder)
        summary=dict(binding_id='binding-1',invalidation_acknowledged=True,observation_enabled=True,
                     refresh_ready=True,surface_generation=2,reason='SYNTHETIC_ONLY')
        for _ in range(4):backend.mirror('perception/surface_refresh_evidence',summary,'SYNTHETIC',10.)
        events=[json.loads(line)['event_type'] for line in self.recorder.read('events').splitlines()]
        for name in ('SURFACE_INVALIDATED','SURFACE_REFRESH_STARTED','SURFACE_REFRESH_READY'):
            self.assertEqual(events.count(name),1)

    def test_bounded_backend_and_no_runtime_enable_flags(self):
        c=dict(self.config,maximum_events=3,maximum_plot_samples=3)
        backend=ConsoleBackend(c,self.recorder)
        for _ in range(10):backend.remote('stop',False);backend.mirror('hardware/load_state',{'value':None})
        self.assertLessEqual(len(backend.timeline),3);self.assertEqual(len(backend.samples),3)
        with self.assertRaises(ValueError):ConsoleBackend(dict(c,physical_output_enabled=True),self.recorder)

    def test_bag_rotation_namespace_raw_cloud_gate(self):
        recorder=BagRecorder({},self.recorder,'crane_01',executable='ros2')
        with self.assertRaises(PermissionError):recorder.command('state_only')
        c=dict(config_state='VALID',enabled=True,storage_id='sqlite3',max_bag_size_bytes=1000000,max_bag_duration_sec=20,
            global_recording_budget_bytes=10000000,minimum_free_space_bytes=0,retention_policy='STOP_ONLY',
            storage_check_interval_sec=1.,
            max_total_storage_bytes=2000000,max_recording_duration_sec=30,profiles={'state_only':{'topics':['phase2c/interaction_state']}})
        recorder=BagRecorder(c,self.recorder,'crane_01',executable='ros2')
        command=recorder.command('state_only');self.assertIn('--max-bag-size',command);self.assertIn('--max-bag-duration',command)
        self.assertIn('/crane_01/phase2c/interaction_state',command)
        c['profiles']['state_only']['topics']=['lidar/merged_points']
        with self.assertRaises(PermissionError):recorder.command('state_only')

    def test_http_same_origin_token_bounded_api_and_page(self):
        backend=ConsoleBackend(self.config,self.recorder);server=ConsoleServer(('127.0.0.1',0),backend)
        worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
        root='http://127.0.0.1:'+str(server.server_port)
        try:
            with urlopen(root,timeout=2) as response:
                html=response.read().decode();self.assertIn('ISOLATED BENCH ONLY',html);self.assertIn('模拟遥控',html)
            with urlopen(root+'/api/state',timeout=2) as response:state=json.load(response)
            body=json.dumps({'signal':'y_left','active':True}).encode()
            with self.assertRaises(HTTPError):urlopen(Request(root+'/api/simulation',data=body),timeout=2)
            headers={'X-CSRF-Token':state['csrf_token'],'Content-Type':'application/json'}
            with urlopen(Request(root+'/api/simulation',data=body,headers=headers),timeout=2) as response:
                self.assertEqual(json.load(response)['source_type'],'SIMULATED')
            with self.assertRaises(HTTPError):
                urlopen(Request(root+'/api/bench/pulse',data=b'{"signal":"y_left","physical_output_enabled":true}',headers=headers),timeout=2)
            with self.assertRaises(HTTPError):
                urlopen(Request(root+'/api/stop',data=b'{}',headers=dict(headers,Origin='http://evil.invalid')),timeout=2)
            with urlopen(Request(root+'/api/stop',data=b'{}',headers=headers),timeout=2) as response:
                self.assertEqual(json.load(response)['signal'],'stop')
        finally:backend.shutdown();server.shutdown();server.server_close();worker.join(2.)

    def test_frontend_release_hooks_and_production_dependency_isolation(self):
        javascript=(ROOT/'src/chili_crane_commissioning/src/chili_crane_commissioning/web/app.js').read_text(encoding='utf-8')
        for event in ('pointerdown','pointerup','pointercancel','lostpointercapture','blur','beforeunload','visibilitychange'):
            self.assertIn(event,javascript)
        import xml.etree.ElementTree as ET
        for package in (ROOT/'src').glob('*/package.xml'):
            if package.parent.name in ('chili_crane_commissioning','chili_crane_bringup'):continue
            self.assertNotIn('chili_crane_commissioning',{n.text for n in ET.parse(package).getroot()})
        for path in (ROOT/'src/chili_crane_bringup/launch').glob('*production*'):
            self.assertNotIn('commissioning',path.read_text(encoding='utf-8'))
