"""Reuse the hardware gate/pulse/session, never a UI-owned coil writer."""
from contextlib import nullcontext
import math
import threading
import time
from types import SimpleNamespace
import uuid
from chili_crane_hardware.adam.adam6052_driver import BenchOutputGate,Adam6052Driver
from chili_crane_hardware.adam.session import output_recovery_latch
from chili_crane_hardware.bench.adam_do_test import pulse_once
from chili_crane_hardware.bench.common import tcp_from_config
from chili_crane_hardware.actuation import request_to_do

SIGNALS={'y_left':('Y',-1),'y_right':('Y',1),'z_up':('Z',1),'z_down':('Z',-1),
         'g_open':('G',1),'g_close':('G',-1)}


def expected_outputs(signal):
    if not isinstance(signal,str):raise ValueError('UNMAPPED_OR_RESERVED_SIGNAL')
    if signal=='stop':return (False,)*8
    if signal not in SIGNALS:raise ValueError('UNMAPPED_OR_RESERVED_SIGNAL')
    axis,direction=SIGNALS[signal]
    return request_to_do(SimpleNamespace(enable=True,axis=axis,direction=direction,speed_command_valid=False))


class AuditedDriver(Adam6052Driver):
    def __init__(self,driver):
        super().__init__(driver.transport,driver.gate);self.trace={};self.phase='pre';self.activated=False

    def read_do(self):
        value=super().read_do();self.trace[self.phase]=list(value);return value

    def bench_on(self,channel):
        self.phase='active';self.activated=True;super().bench_on(channel)

    def all_off(self):
        self.phase='post' if self.activated else 'pre';return super().all_off()


class BenchService:
    def __init__(self,config,recorder,enable_cli=False,isolated_cli=False,exclusive_cli=False,driver_factory=None):
        self.config=dict(config);self.recorder=recorder
        acceptance=config.get('acceptance',{})
        self.gate=BenchOutputGate(physical_output_enabled=enable_cli and config.get('physical_output_enabled') is True,
            isolated_bench_confirmed=isolated_cli,adam_fsv_configured=acceptance.get('ADAM_FSV_CONFIGURED') is True,
            adam_watchdog_configured=acceptance.get('ADAM_WATCHDOG_CONFIGURED') is True,
            adam_safe_output_verified=acceptance.get('ADAM_SAFE_OUTPUT_VERIFIED') is True)
        self.exclusive_cli=exclusive_cli;self.driver_factory=driver_factory
        self.lock=threading.Lock();self.stop=threading.Event();self.idle=threading.Event();self.idle.set()
        self.last=None
        self.logging_failed=False
        self.publish_record=None

    def require(self):
        if self.logging_failed:raise PermissionError('BENCH_LOGGING_FAILED_RECOMMISSION_REQUIRED')
        self.gate.require()
        a=self.config.get('acceptance',{})
        watchdog=a.get('watchdog_timeout_sec')
        if self.exclusive_cli is not True or self.config.get('automatic_control_enabled') is not False:
            raise PermissionError('EXCLUSIVE_ISOLATED_SESSION_AND_AUTO_OFF_REQUIRED')
        if (type(watchdog) not in (int,float) or not math.isfinite(watchdog) or not 0<watchdog<=2
                or not isinstance(a.get('verification_evidence'),str)
                or a.get('verification_evidence') in ('','NOT_CONFIGURED','UNKNOWN')):
            raise PermissionError('VERIFIED_WATCHDOG_AND_OFF_EVIDENCE_REQUIRED')

    @property
    def enabled(self):
        try:self.require();return True
        except PermissionError:return False

    def pulse(self,signal,duration_ms=500,sleeper=None):
        self.require()
        if type(duration_ms) is not int or not 1<duration_ms<=1000:raise ValueError('PULSE_REQUIRES_1_MS_LT_DURATION_LE_1000_MS')
        outputs=expected_outputs(signal)
        if sum(outputs)!=1:raise ValueError('EXACTLY_ONE_DIRECTION_REQUIRED')
        if not self.lock.acquire(blocking=False):raise ValueError('BENCH_PULSE_ALREADY_ACTIVE')
        self.idle.clear();self.stop.clear();driver=None;reason='';result='FAILED'
        test_id=uuid.uuid4().hex;started=time.monotonic()
        try:
            base=self.driver_factory(self.gate) if self.driver_factory else Adam6052Driver(tcp_from_config(self.config),self.gate)
            driver=AuditedDriver(base)
            source=self.config.get('source_type','PHYSICAL')
            # A test factory may use a null crash latch only for explicit mock
            # SOURCE_SYNTHETIC. Real transports ALWAYS use the existing latch.
            latch=nullcontext() if source=='SYNTHETIC' and self.driver_factory else output_recovery_latch(
                driver.transport.host,driver.transport.port,driver.transport.unit_id)
            with driver.transport.exclusive_session(),latch:
                try:pulse_once(driver,outputs.index(True),duration_ms/1000.,sleeper=sleeper or self.stop.wait)
                finally:driver.all_off()
            result='PASS_SOFTWARE_MOCK' if source=='SYNTHETIC' else 'BENCH_READBACK_COMPLETED_NOT_VOLTAGE_OR_FIELD'
        except BaseException as exc:
            reason=str(exc);raise
        finally:
            try:
                record=self.recorder.append('bench_measurements','BENCH_PULSE',source_type=self.config.get('source_type','PHYSICAL'),
                    reason=reason,bench_test_id=test_id,semantic_signal=signal,device='ADAM-6052',channel=outputs.index(True),
                    requested_duration_ms=duration_ms,elapsed_sec=time.monotonic()-started,
                    pre_readback=driver.trace.get('pre') if driver else None,active_readback=driver.trace.get('active') if driver else None,
                    post_readback=driver.trace.get('post') if driver else None,supply_voltage=None,measured_voltage=None,result=result,
                    voltage_state='NOT_MEASURED',FIELD_MOTOR_DIRECTION='NOT_RUN',FIELD_LIMIT_CHAIN='NOT_RUN',FULL_AUTO='NOT_RUN')
                self.last=record
                if self.publish_record:self.publish_record(record)
            except BaseException:
                self.logging_failed=True;raise
            finally:self.lock.release();self.idle.set()
        return record

    def shutdown(self):
        self.stop.set();self.idle.wait(timeout=3.)
