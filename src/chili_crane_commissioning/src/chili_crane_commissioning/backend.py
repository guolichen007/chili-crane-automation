"""Bounded commissioning state and isolated simulated observation lease."""
from collections import deque
from datetime import datetime,timezone
import math
import secrets
import threading
import time
from chili_crane_control.mode_policy import evaluate_mode
from chili_crane_control.evidence_policy import RuntimePolicy,SourceType
from .bench import BenchService,SIGNALS,expected_outputs
from .recording import clean


class ConsoleBackend:
    def __init__(self,config,recorder,bench=None,publish_remote=None):
        self.config=dict(config);self.recorder=recorder
        if any(config.get(k) is not False for k in ('physical_output_enabled','automatic_control_enabled','camera_control_authority')):
            raise ValueError('CONSOLE_RUNTIME_OUTPUTS_MUST_BE_DISABLED')
        self.bench=bench or BenchService({},recorder)
        from .bag import BagRecorder
        self.bag=BagRecorder({},recorder,'crane_01')
        self.publish_remote=publish_remote;self.lock=threading.RLock();self.token=secrets.token_urlsafe(32)
        self.active='stop';self.lease_until=0.;self.counter=0;self.devices={};self.timeline=deque(maxlen=config['maximum_events'])
        self.last_remote={};self.context=dict(task_id='',cycle_id='',binding_id='');self.samples=deque(maxlen=config['maximum_plot_samples'])
        self.recorded_refresh=set()

    def remote(self,signal,active=False):
        if not isinstance(signal,str) or signal not in set(SIGNALS)|{'stop'} or type(active) is not bool:raise ValueError('SIMULATION_SIGNAL_INVALID')
        with self.lock:
            if active and self.active not in ('stop',signal):
                self.release('SIMULATED_DIRECTION_CONFLICT');raise ValueError('SIMULTANEOUS_DIRECTIONS_REJECTED')
            self.active=signal if active and signal!='stop' else 'stop'
            self.lease_until=time.monotonic()+self.config['simulation_lease_sec'] if self.active!='stop' else 0.
            state=self._remote_state()
            event=self.recorder.append('operator_actions','SIMULATED_REMOTE_PRESS' if active else 'SIMULATED_REMOTE_RELEASE',
                **self.context,reason='OBSERVATION_ONLY_NO_PHYSICAL_OUTPUT',semantic_signal=signal,active=active)
            self.timeline.append(event)
            if self.publish_remote:self.publish_remote(state)
            return state

    def _remote_state(self):
        self.counter+=1
        mode=evaluate_mode(False,True,self.active!='stop',True)
        allowed=RuntimePolicy(mode='shadow_control').sources_allowed([SourceType.SIMULATED])
        expected=expected_outputs(self.active) if allowed and mode.mode=='REMOTE' else (False,)*8
        self.last_remote=dict(source_type='SIMULATED',source_counter=self.counter,signal=self.active,
            active=self.active!='stop',stop_requested=self.active=='stop',control_mode=mode.mode,
            release_automatic_outputs=True,discard_pending_commands=True,expected_do_preview=list(expected),
            physical_output_enabled=False,automatic_control_enabled=False,reason='SHADOW_EXPECTATION_NOT_DO_WRITE')
        return dict(self.last_remote)

    def release(self,reason):
        with self.lock:
            was=self.active;self.active='stop';self.lease_until=0.;state=self._remote_state()
            if was!='stop':
                self.timeline.append(self.recorder.append('events','SIMULATED_REMOTE_STOP',**self.context,reason=reason))
            if self.publish_remote:self.publish_remote(state)
            return state

    def tick(self):
        with self.lock:
            if self.active!='stop' and time.monotonic()>=self.lease_until:self.release('UI_DISCONNECT_OR_LEASE_EXPIRED')
        self.bag.tick()

    def mirror(self,topic,summary,source_type='UNKNOWN',ros_timestamp=None):
        with self.lock:
            value=clean(summary);self.devices[topic]=dict(value=value,received_monotonic=time.monotonic(),source_type=source_type)
            for key in self.context:
                if value.get(key):self.context[key]=value[key]
            if topic=='hardware/load_state':
                self.samples.append(dict(epoch=ros_timestamp,value=value.get('value'),delta=value.get('delta')))
            if topic=='phase2c/interaction_state':
                for event in value.get('event_types',[]):
                    self.timeline.append(self.recorder.append('events',event,source_type,ros_timestamp,**self.context,
                        old_state='',new_state=value.get('task_cycle_state',''),reason=value.get('reason',''),evidence_summary=value))
            elif topic=='phase2c/target_binding':
                self.timeline.append(self.recorder.append('events','TARGET_BOUND',source_type,ros_timestamp,**self.context,
                    reason='IMMUTABLE_CYCLE_CONTRACT',binding_digest=value.get('binding_digest_sha256','')))
            elif topic=='perception/surface_refresh_evidence':
                for flag,event in (('invalidation_acknowledged','SURFACE_INVALIDATED'),
                                   ('observation_enabled','SURFACE_REFRESH_STARTED'),('refresh_ready','SURFACE_REFRESH_READY')):
                    key=(value.get('binding_id'),event)
                    if value.get(flag) and key not in self.recorded_refresh:
                        if len(self.recorded_refresh)>=self.config['maximum_events']:
                            raise ValueError('RECORDING_EVENT_CAPACITY_REQUIRES_NEW_RUN')
                        self.recorded_refresh.add(key)
                        self.timeline.append(self.recorder.append('events',event,source_type,ros_timestamp,**self.context,
                            reason=value.get('reason',''),surface_generation=value.get('surface_generation')))

    def voltage(self,body):
        for name in ('supply_voltage','measured_do_voltage'):
            if body.get(name) is not None and (type(body[name]) not in (int,float) or not math.isfinite(body[name])):
                raise ValueError('VOLTAGE_MUST_BE_FINITE_OR_UNMEASURED')
        note=body.get('operator_note','')
        if not isinstance(note,str) or len(note)>4096:raise ValueError('OPERATOR_NOTE_BOUND')
        test_id=body.get('bench_test_id','')
        if not self.bench.last or test_id!=self.bench.last['details']['bench_test_id']:
            raise ValueError('VOLTAGE_REQUIRES_EXISTING_OWNED_BENCH_TEST')
        record=self.recorder.append('bench_measurements','OPERATOR_VOLTAGE_ENTRY',source_type='OPERATOR_REPORTED',
            **self.context,bench_test_id=test_id,supply_voltage=body.get('supply_voltage'),measured_voltage=body.get('measured_do_voltage'),
            operator_note=note,result='OPERATOR_REPORTED_NOT_FIELD_ACCEPTANCE',software_readback_is_voltage=False)
        self.timeline.append(record);return record

    def state(self):
        self.tick()
        with self.lock:
            return dict(run_id=self.recorder.run_id,git_sha=self.recorder.git_sha,config_hash=self.recorder.config_hash,
                csrf_token=self.token,mode=self.config.get('mode','SOFTWARE_SHADOW'),phase='2C',
                physical_output_enabled=False,automatic_control_enabled=False,camera_control_authority=False,
                BENCH_DO_STATUS='NOT_RUN',FULL_AUTO='NOT_RUN',context=dict(self.context),remote=dict(self.last_remote),
                bench_enabled=self.bench.enabled,bench_label='ISOLATED BENCH ONLY / 隔离台架专用',bench_last=self.bench.last,
                devices={k:dict(v,transport_residence_sec=max(0.,time.monotonic()-v['received_monotonic'])) for k,v in self.devices.items()},
                events=list(self.timeline),plot_samples=list(self.samples),**self.bag.status())

    def shutdown(self):
        try:self.release('CONSOLE_SHUTDOWN')
        finally:
            self.bench.shutdown();self.bag.stop()
            self.recorder.append('events','RUN_STOPPED',**self.context,reason='CONSOLE_CLOSED_NO_FIELD_CLAIM')
