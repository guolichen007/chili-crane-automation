"""Immutable Phase2C evidence contracts, independent of ROS and device registers."""
from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
import re
import struct


def identity(value):
    return isinstance(value, str) and value not in ('', 'UNKNOWN', 'NOT_CONFIGURED') and len(value) <= 4096


@dataclass(frozen=True)
class Evidence:
    epoch_ns: int
    counter: int
    source_type: str
    calibration_id: str
    config_version: str
    frame_id: str = 'map'
    validity: int = 4
    reported_age: float = 0.
    receive_ns: int = 0
    reason: str = ''

    @property
    def epoch(self):
        return self.epoch_ns / 1e9

    @classmethod
    def at(cls, epoch, counter=1, source_type='SYNTHETIC', calibration_id='synthetic-calibration',
           config_version='synthetic-v1', **kwargs):
        return cls(int(round(epoch*1e9)), counter, source_type, calibration_id, config_version, **kwargs)

    def fresh(self, now, ttl, source_type=None):
        return (type(self.epoch_ns) is int and self.epoch_ns > 0 and type(self.counter) is int and self.counter >= 0
                and self.validity in (3, 4) and self.source_type in ('PHYSICAL', 'SIMULATED', 'REPLAY', 'SYNTHETIC')
                and (source_type is None or self.source_type == source_type)
                and all(identity(v) for v in (self.calibration_id, self.config_version, self.frame_id))
                and math.isfinite(now) and math.isfinite(ttl) and ttl > 0 and 0 <= now-self.epoch <= ttl
                and math.isfinite(self.reported_age) and 0 <= self.reported_age <= ttl)


def canonical(value):
    """Versioned exact binary64 digest encoding; never Python's randomized hash."""
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError('NONFINITE_BINDING_NUMBER')
        return {'$f64': (0. if value == 0 else value).hex()}
    if isinstance(value, dict):
        return {key: canonical(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [canonical(item) for item in value]
    return value


@dataclass(frozen=True)
class TargetBinding:
    run_id: str
    task_id: str
    cycle_id: str
    cycle_index: int
    binding_id: str
    proposal_id: str
    candidate_id: str
    map_id: str
    semantic_version: str
    pit_id: str
    surface_snapshot_id: str
    surface_session_id: str
    surface_generation: int
    transform_id: str
    spatial_calibration_id: str
    geofence_id: str
    geofence_version: str
    geofence_config_sha256: str
    kernel_id: str
    target_suspension_map: tuple
    capture_footprint: tuple
    surface_nominal_m: float
    surface_lower_m: float
    surface_upper_m: float
    material_volume_proxy_lower_bound_m3: float
    binding_epoch_ns: int
    binding_source_counter: int
    evidence: Evidence
    binding_schema_version: str = 'PHASE2C_BINDING_CANONICAL_V1'
    binding_digest_sha256: str = ''
    valid_for_control: bool = False

    def __post_init__(self):
        # Polygon uses ROS Point32. Canonicalize exactly at the contract boundary
        # so a generated-message roundtrip cannot silently change the digest.
        try:
            fp=tuple(tuple(float(struct.unpack('!f', struct.pack('!f', v))[0]) for v in xy) for xy in self.capture_footprint)
        except (OverflowError,TypeError,struct.error) as exc:
            raise ValueError('BINDING_FOOTPRINT_INVALID') from exc
        object.__setattr__(self, 'capture_footprint', fp)
        object.__setattr__(self, 'target_suspension_map', tuple(float(v) for v in self.target_suspension_map))
        for name in ('surface_nominal_m','surface_lower_m','surface_upper_m','material_volume_proxy_lower_bound_m3'):
            object.__setattr__(self,name,float(getattr(self,name)))
        fields=asdict(self)
        if (not all(identity(fields[k]) for k in ('run_id','task_id','cycle_id','binding_id','proposal_id','candidate_id',
                'map_id','semantic_version','pit_id','surface_snapshot_id','surface_session_id','transform_id',
                'spatial_calibration_id','geofence_id','geofence_version','geofence_config_sha256','kernel_id'))
                or type(self.cycle_index) is not int or self.cycle_index < 1
                or type(self.surface_generation) is not int or self.surface_generation < 0
                or type(self.binding_epoch_ns) is not int or self.binding_epoch_ns < self.evidence.epoch_ns
                or self.binding_source_counter != self.evidence.counter or self.valid_for_control is not False
                or re.fullmatch('[0-9a-f]{32}',self.surface_session_id) is None
                or not self.surface_snapshot_id.startswith(self.surface_session_id+':')
                or re.fullmatch('[0-9a-f]{64}',self.geofence_config_sha256) is None
                or self.binding_schema_version != 'PHASE2C_BINDING_CANONICAL_V1'
                or len(self.target_suspension_map) != 3 or len(fp) < 3 or len(fp) > 256
                or any(len(xy)!=2 for xy in fp)
                or not self.surface_lower_m <= self.surface_nominal_m <= self.surface_upper_m
                or self.material_volume_proxy_lower_bound_m3 < 0):
            raise ValueError('TARGET_BINDING_CONTRACT_INVALID')
        digest=self.digest()
        if self.binding_digest_sha256 and self.binding_digest_sha256 != digest:
            raise ValueError('TARGET_BINDING_DIGEST_MISMATCH')
        object.__setattr__(self,'binding_digest_sha256',digest)

    def digest(self):
        fields=asdict(self);fields.pop('binding_digest_sha256')
        raw=json.dumps(canonical(fields),sort_keys=True,ensure_ascii=False,separators=(',',':'),allow_nan=False).encode('utf-8')
        return hashlib.sha256(raw).hexdigest()

    @property
    def lineage(self):
        return tuple(getattr(self,name) for name in ('map_id','semantic_version','pit_id','surface_session_id',
            'spatial_calibration_id','geofence_id','geofence_version','geofence_config_sha256','kernel_id'))


def require_observation_successor(previous,previous_payload,current,payload):
    """Accept exact duplicates, never regress epoch/counter or change producer identity."""
    if previous is None:return
    identity=lambda e:(e.source_type,e.frame_id,e.calibration_id,e.config_version)
    if identity(previous)!=identity(current):raise ValueError('OBSERVATION_SOURCE_IDENTITY_CONFLICT')
    old=(previous.epoch_ns,previous.counter);new=(current.epoch_ns,current.counter)
    if new==old and payload==previous_payload:return
    if new[0]<=old[0] or new[1]<=old[1]:raise ValueError('OBSERVATION_CHRONOLOGY_CONFLICT')


class BindingLedger:
    """No eviction/rebinding. A new run may create a new deliberate ledger."""
    def __init__(self, maximum=1024):
        if type(maximum) is not int or not 1 <= maximum <= 100000:
            raise ValueError('BINDING_LEDGER_BOUND_INVALID')
        self.maximum=maximum
        self.bindings={}
        self.run_id=None

    def bind(self, binding):
        if self.run_id is not None and binding.run_id!=self.run_id:
            raise ValueError('BINDING_LEDGER_RUN_MISMATCH')
        key=(binding.run_id,binding.task_id,binding.cycle_id)
        if key in self.bindings:
            raise ValueError('TARGET_ALREADY_BOUND_ABORT_AND_NEW_CYCLE_REQUIRED')
        if len(self.bindings) >= self.maximum:
            raise ValueError('BINDING_LEDGER_CAPACITY_EXCEEDED')
        self.bindings[key]=binding
        self.run_id=binding.run_id
        return binding

    def check_lineage(self, binding, lineage):
        if tuple(lineage)!=binding.lineage:
            raise ValueError('BINDING_LINEAGE_CONFLICT_ABORT_REQUIRED')
