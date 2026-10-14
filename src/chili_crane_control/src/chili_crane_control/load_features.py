"""Time-ordered, bounded robust signal features. No kg or device assumptions."""
from collections import deque
from dataclasses import dataclass, replace
import math
import numpy as np
from .interaction_contracts import Evidence


def configured_number(config,key,positive=False):
    value=config.get(key)
    return (type(value) in (int,float) and math.isfinite(value) and (value > 0 if positive else value >= 0))


@dataclass(frozen=True)
class LoadSample:
    evidence: Evidence
    value: float
    unit: str
    fault: bool = False
    suspended_empty_known: bool = False


@dataclass(frozen=True)
class LoadFeatures:
    fresh: bool = False
    reason: str = 'NOT_CONFIGURED'
    measurement_epoch: float = 0.
    sample_count: int = 0
    rolling_median: float = 0.
    short_median: float = 0.
    mad: float = 0.
    robust_scale: float = 0.
    empty_baseline: float = 0.
    baseline_valid: bool = False
    delta: float = 0.
    normalized_delta: float = 0.
    slope: float = 0.
    change_persistence_sec: float = 0.
    change_latched: bool = False
    settling_sec: float = 0.
    unit: str = 'NOT_CONFIGURED'
    calibration_confirmed_for_source: bool = False


class LoadFeatureEstimator:
    def __init__(self,config):
        self.config=dict(config)
        maximum=config.get('maximum_samples')
        if type(maximum) is not int or not 4 <= maximum <= 100000:
            raise ValueError('LOAD_BUFFER_BOUND_INVALID')
        self.samples=deque(maxlen=maximum)
        self.empty_samples=deque(maxlen=maximum)
        self.last_sample=None
        self.baseline=None
        self.baseline_epoch=0.
        self.change_start,self.settle_start=None,None
        self.latched=False
        self.features=LoadFeatures()

    def reset(self,reason):
        self.samples.clear();self.empty_samples.clear();self.baseline=None
        self.baseline_epoch=0.;self.change_start=self.settle_start=None;self.latched=False
        self.features=LoadFeatures(reason=reason)
        return self.features

    def calibrated(self,evidence):
        required='FIELD_VALIDATED' if evidence.source_type=='PHYSICAL' else 'SYNTHETIC_VALIDATED'
        return (self.config.get('calibration_state')==required
                and evidence.calibration_id==self.config.get('calibration_id')
                and self.config.get('load_polarity') in (-1,1)
                and configured_number(self.config,'contact_abs_threshold',True)
                and configured_number(self.config,'payload_abs_threshold',True))

    def current(self,now):
        ttl=self.config.get('sensor_ttl_sec')
        if (self.last_sample is None or not configured_number(self.config,'sensor_ttl_sec',True)
                or not self.last_sample.evidence.fresh(now,ttl,self.config.get('source_type'))):
            return self.reset('LOAD_SENSOR_STALE_OR_UNCONFIGURED')
        baseline_fresh=self.baseline is not None and 0 <= now-self.baseline_epoch <= self.config['baseline_ttl_sec']
        return replace(self.features,baseline_valid=baseline_fresh)

    def observe(self,sample,now,empty_eligible=False):
        keys=('short_window_sec','long_window_sec','baseline_ttl_sec','sensor_ttl_sec','scale_floor',
              'normalized_enter','normalized_exit','debounce_sec','maximum_sample_gap_sec','settle_slope_limit')
        if not all(configured_number(self.config,k,True) for k in keys):
            return self.reset('LOAD_FEATURES_NOT_CONFIGURED')
        c=self.config
        if (c['short_window_sec'] > c['long_window_sec'] or c['normalized_exit'] >= c['normalized_enter']
                or type(c.get('minimum_samples')) is not int or not 3 <= c['minimum_samples'] <= c['maximum_samples']):
            return self.reset('LOAD_FEATURE_CONFIG_INVALID')
        if (sample.fault or not math.isfinite(sample.value) or sample.unit!=c.get('unit')
                or not sample.evidence.fresh(now,c['sensor_ttl_sec'],c.get('source_type'))
                or sample.evidence.calibration_id!=c.get('calibration_id')):
            return self.reset('LOAD_SENSOR_FAULT_STALE_OR_PROVENANCE')
        e=sample.evidence;epoch=e.epoch
        if self.last_sample is not None:
            last=self.last_sample
            if e.counter==last.evidence.counter and e.epoch_ns==last.evidence.epoch_ns and (
                    sample.value==last.value and sample.unit==last.unit and e.calibration_id==last.evidence.calibration_id):
                return self.current(now)  # Duplicate heartbeat never grows persistence/baseline.
            if e.counter <= last.evidence.counter or e.epoch_ns <= last.evidence.epoch_ns:
                return self.reset('LOAD_TIME_OR_COUNTER_REGRESSION')
            if epoch-last.evidence.epoch > c['maximum_sample_gap_sec']:
                self.reset('LOAD_SAMPLE_GAP')
        self.last_sample=sample
        self.samples.append((epoch,float(sample.value)))
        while self.samples and epoch-self.samples[0][0] > c['long_window_sec']:
            self.samples.popleft()
        if empty_eligible is True and sample.suspended_empty_known is True and self.calibrated(e):
            self.empty_samples.append((epoch,float(sample.value)))
            while self.empty_samples and epoch-self.empty_samples[0][0] > c['long_window_sec']:
                self.empty_samples.popleft()
            if len(self.empty_samples) >= c['minimum_samples']:
                self.baseline=float(np.median([v for _,v in self.empty_samples]));self.baseline_epoch=epoch
        short=[(t,v) for t,v in self.samples if epoch-t <= c['short_window_sec']]
        values=np.array([v for _,v in self.samples]);median=float(np.median(values))
        mad=float(np.median(np.abs(values-median)))
        scale=max(1.482602218505602*mad,c['scale_floor'])
        short_med=float(np.median([v for _,v in short]))
        baseline_valid=self.baseline is not None and 0 <= epoch-self.baseline_epoch <= c['baseline_ttl_sec']
        delta=short_med-self.baseline if baseline_valid else 0.
        normalized=delta/scale if baseline_valid else 0.
        t=np.array([t for t,_ in short]);v=np.array([v for _,v in short]);t=t-t.mean()
        slope=float(np.dot(t,v-v.mean())/np.dot(t,t)) if len(short)>=2 and np.dot(t,t)>0 else 0.
        enough=len(short)>=c['minimum_samples']
        above=baseline_valid and enough and abs(normalized)>=c['normalized_enter']
        below=not baseline_valid or not enough or abs(normalized)<=c['normalized_exit']
        if below:self.change_start=None;self.latched=False
        elif above:
            if self.change_start is None:self.change_start=epoch
            if epoch-self.change_start>=c['debounce_sec']:self.latched=True
        if enough and abs(slope)<=c['settle_slope_limit']:
            if self.settle_start is None:self.settle_start=epoch
        else:self.settle_start=None
        self.features=LoadFeatures(True,'ROBUST_SIGNAL_NOT_PAYLOAD_MASS',epoch,len(self.samples),median,short_med,mad,scale,
            self.baseline if self.baseline is not None else 0.,baseline_valid,delta,normalized,slope,
            0. if self.change_start is None else epoch-self.change_start,self.latched,
            0. if self.settle_start is None else epoch-self.settle_start,sample.unit,self.calibrated(e))
        return self.current(now)


@dataclass(frozen=True)
class VerticalFeatures:
    valid: bool = False
    velocity: float = 0.
    acceleration: float = 0.
    rise: float = 0.
    reason: str = 'INSUFFICIENT_REAL_MEASUREMENTS'


class VerticalMotionEstimator:
    def __init__(self,config):
        self.config=dict(config)
        self.samples=deque(maxlen=config['maximum_samples'])
        self.last_counter=-1
        self.anchor=None
        self.features=VerticalFeatures()

    def observe(self,evidence,z):
        if not math.isfinite(z):
            self.samples.clear();self.features=VerticalFeatures(reason='Z_NONFINITE');return self.features
        if self.samples and evidence.counter==self.last_counter and evidence.epoch==self.samples[-1][0]:
            return self.features
        if self.samples and (evidence.counter<=self.last_counter or evidence.epoch<=self.samples[-1][0]):
            self.samples.clear();self.features=VerticalFeatures(reason='Z_CHRONOLOGY_CONFLICT');return self.features
        if self.anchor is None:self.anchor=z
        self.last_counter=evidence.counter;self.samples.append((evidence.epoch,z))
        window=self.config.get('window_sec');minimum=self.config.get('minimum_samples')
        if not configured_number(self.config,'window_sec',True) or type(minimum) is not int or minimum<3:
            self.features=VerticalFeatures(reason='Z_MODEL_NOT_CONFIGURED');return self.features
        while self.samples and evidence.epoch-self.samples[0][0]>window:self.samples.popleft()
        if len(self.samples)<minimum:return VerticalFeatures(rise=z-self.anchor)
        t=np.array([a-evidence.epoch for a,_ in self.samples]);v=np.array([b for _,b in self.samples])
        matrix=np.c_[np.ones(len(t)),t,t*t];weights=np.ones(len(t))
        for _ in range(3):
            coeff=np.linalg.lstsq(matrix*weights[:,None],v*weights,rcond=None)[0]
            residual=v-matrix@coeff;scale=max(float(np.median(np.abs(residual-np.median(residual)))),np.finfo(float).eps)
            weights=np.minimum(1.,1.5*scale/np.maximum(np.abs(residual),np.finfo(float).eps))
        self.features=VerticalFeatures(True,float(coeff[1]),float(2*coeff[2]),z-self.anchor,'ROBUST_WINDOW_QUADRATIC')
        return self.features
