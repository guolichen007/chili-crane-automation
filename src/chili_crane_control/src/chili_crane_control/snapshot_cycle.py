"""Cycle observations gate snapshots; TaskCycles alone owns transitions."""
from dataclasses import dataclass
import math
from .cycle import CycleState
from .interaction_contracts import Evidence, identity, require_observation_successor


@dataclass(frozen=True)
class CycleObservation:
    evidence: Evidence
    run_id: str
    task_id: str
    cycle_id: str
    cycle_index: int
    phase: CycleState

    @property
    def key(self):return (self.run_id,self.task_id,self.cycle_id,self.cycle_index)


@dataclass(frozen=True)
class ScanProof:
    key: tuple
    scan_start_ns: int
    window_start_ns: int
    window_end_ns: int
    session: str
    generation: int
    snapshot_id: str
    evidence: Evidence

    def matches_binding(self,b,now,ttl,source):
        return (self.key==(b.run_id,b.task_id,b.cycle_id,b.cycle_index)
            and 0<self.scan_start_ns<self.window_start_ns<=self.window_end_ns
            and self.window_end_ns==b.evidence.epoch_ns
            and (self.session,self.generation,self.snapshot_id)==
                (b.surface_session_id,b.surface_generation,b.surface_snapshot_id)
            and all(getattr(self.evidence,k)==getattr(b.evidence,k) for k in
                    ('epoch_ns','counter','source_type','calibration_id','config_version','frame_id','validity'))
            and self.evidence.fresh(now,ttl,source) and b.evidence.fresh(now,ttl,source))


class SnapshotCycleGate:
    """Bounded observer, not a second FSM. Stale/conflicting input blocks work."""
    def __init__(self,source,ttl):
        self.source=source
        self.ttl=ttl if type(ttl) in (int,float) and math.isfinite(ttl) and ttl>0 else 0.
        self.cycle=None;self.scan_start_ns=0;self.bound=False;self.blocked=True
        self.conflict=False;self.reason='SCAN_REQUIRED';self.snapshot_ready=False

    def observe(self,current,now):
        previous=self.cycle
        try:
            if (self.source not in ('PHYSICAL','SIMULATED','REPLAY','SYNTHETIC')
                    or not all(identity(v) for v in current.key[:3]) or type(current.cycle_index) is not int
                    or current.cycle_index<1 or not isinstance(current.phase,CycleState)
                    or not current.evidence.fresh(now,self.ttl,self.source)):
                raise ValueError('CYCLE_STATUS_STALE_OR_INVALID')
            changed=previous is None or current.key!=previous.key
            if previous is not None:
                require_observation_successor(previous.evidence,(previous.key,previous.phase),
                                              current.evidence,(current.key,current.phase))
                if changed:
                    if previous.phase not in (CycleState.COMPLETE,CycleState.ABORTED) or current.phase!=CycleState.SCAN:
                        raise ValueError('ACTIVE_CYCLE_IDENTITY_CONFLICT_ABORT_REQUIRED')
                    if (current.key[:2]==previous.key[:2] and current.cycle_index<=previous.cycle_index):
                        raise ValueError('CYCLE_INDEX_REGRESSION_CONFLICT')
                elif current.phase<previous.phase:
                    raise ValueError('CYCLE_PHASE_REGRESSION_CONFLICT')
            self.cycle=current
            reset=changed and current.phase==CycleState.SCAN
            if reset:
                self.scan_start_ns=current.evidence.epoch_ns
                self.bound=False;self.snapshot_ready=False;self.conflict=False
            self.blocked=self.conflict
            self.reason='CYCLE_OBSERVATION_CONFLICT' if self.conflict else self.state
            return reset
        except ValueError as exc:
            self.blocked=True
            if 'CONFLICT' in str(exc):self.conflict=True
            self.reason=str(exc)
            raise

    def fresh(self,now):
        return (not self.blocked and self.cycle is not None
                and self.cycle.evidence.fresh(now,self.ttl,self.source))

    def scanning(self,now):
        return self.fresh(now) and self.scan_start_ns>0 and not self.bound and self.cycle.phase==CycleState.SCAN

    def planning(self,now):
        return (self.fresh(now) and self.scan_start_ns>0 and not self.bound
                and self.cycle.phase in (CycleState.SCAN,CycleState.PLAN_PICK))

    def accepts(self,proof,now):
        # DDS observers can join on different SCAN heartbeats. The producer's
        # scan_start is provenance; our first SCAN is an independent lower bound.
        # Require the whole window after both, without rewriting producer proof.
        return (self.planning(now) and proof.key==self.cycle.key
                and 0<proof.scan_start_ns<proof.window_start_ns<=proof.window_end_ns
                and self.scan_start_ns<proof.window_start_ns
                and proof.evidence.epoch_ns==proof.window_end_ns
                and proof.evidence.fresh(now,self.ttl,self.source))

    def freeze(self,key):
        if self.cycle is None or tuple(key)!=self.cycle.key or not self.scan_start_ns:
            raise ValueError('BINDING_SCAN_IDENTITY_CONFLICT')
        self.bound=True

    @property
    def state(self):
        if self.cycle is None or not self.scan_start_ns:return 'SCAN_REQUIRED'
        if self.bound or self.cycle.phase>=CycleState.POSITION_Y:return 'FROZEN_ACTIVE_CYCLE'
        return 'SNAPSHOT_READY' if self.snapshot_ready else 'SCANNING'
