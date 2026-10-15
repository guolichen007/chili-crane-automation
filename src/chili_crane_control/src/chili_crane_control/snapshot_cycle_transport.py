"""R2 cycle message envelopes shared by perception and interaction adapters."""
from chili_crane_control.cycle import CycleState
from chili_crane_control.snapshot_cycle import CycleObservation, ScanProof
from chili_crane_control.interaction_transport import observation_evidence, ns, stamp


def cycle_observation(m):
    return CycleObservation(observation_evidence(m),m.run_id,m.task_id,m.cycle_id,int(m.cycle_index),CycleState(m.state))


def annotate_cycle(m,gate):
    m.surface_cycle_state=gate.state
    if gate.cycle is not None:
        m.observed_cycle_state=int(gate.cycle.phase)
        m.scan_run_id,m.scan_task_id,m.scan_cycle_id,m.scan_cycle_index=gate.cycle.key
        m.scan_start_stamp=stamp(gate.scan_start_ns)
    return m


def proof_from_status(s):
    return ScanProof((s.scan_run_id,s.scan_task_id,s.scan_cycle_id,int(s.scan_cycle_index)),
        ns(s.scan_start_stamp),ns(s.window_start),ns(s.window_end),s.surface_session_id,
        int(s.surface_generation),s.snapshot_id,observation_evidence(s))


def proof_from_proposal(p):
    return ScanProof((p.scan_run_id,p.scan_task_id,p.scan_cycle_id,int(p.scan_cycle_index)),
        ns(p.scan_start_stamp),ns(p.surface_window_start),ns(p.surface_window_end),p.surface_session_id,
        int(p.surface_generation),p.surface_snapshot_id,observation_evidence(p))
