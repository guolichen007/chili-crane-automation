"""Pure mode handover policy. Observation does not grant an action permit."""
from dataclasses import dataclass


@dataclass(frozen=True)
class ModeDecision:
    mode: str
    release_automatic_outputs: bool = True
    discard_pending_commands: bool = True
    require_new_task: bool = True


def evaluate_mode(mode_auto, safety_ok, remote_active, inputs_fresh,
                  stopped_verified=False, outputs_off_verified=False):
    if not inputs_fresh or mode_auto is None or safety_ok is not True:
        return ModeDecision("BLOCKED")
    if mode_auto is False:
        return ModeDecision("REMOTE")
    if remote_active is not False:
        return ModeDecision("CONFLICT")
    if not stopped_verified or not outputs_off_verified:
        return ModeDecision("AUTO_PENDING")
    return ModeDecision("AUTO_READY")
