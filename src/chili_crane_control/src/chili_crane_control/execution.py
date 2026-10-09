"""Mock action execution and authority epochs; no device protocol or I/O."""
import math
import uuid
from dataclasses import dataclass
from .contracts import SystemMode, AxisCapability, DriveProfile, StopDistance, SystemReadiness


@dataclass(frozen=True)
class AuthorizedAction:
    command_id: str = ""
    intent_id: str = ""
    permit_id: str = ""
    permit_generation: int = 0
    session_id: str = ""
    command_epoch: int = 0
    sequence: int = 0
    issued_stamp: float = 0.0
    expire_stamp: float = 0.0
    valid: bool = False
    axis: str = "ALL"
    direction: int = 0
    target: float | None = None


@dataclass(frozen=True)
class ActuationRequest:
    axis: str = "ALL"
    direction: int = 0
    enable: bool = False
    speed_command_valid: bool = False
    speed_command: float = 0.0
    source_command_id: str = ""
    session_id: str = ""
    command_epoch: int = 0
    sequence: int = 0
    issued_stamp: float = 0.0
    expire_stamp: float = 0.0
    reason: str = "NOT_CONFIGURED"


@dataclass(frozen=True)
class PermitEvidence:
    permit_id: str = ""
    permit_generation: int = 0
    evaluated_intent_id: str = ""
    session_id: str = ""
    command_epoch: int = 0
    issued_stamp: float = 0.0
    expire_stamp: float = 0.0
    valid: bool = False
    direction_allowed: bool = False

    def authorizes(self, action, now):
        stamps = (self.issued_stamp, self.expire_stamp, now)
        return (self.valid is True and self.direction_allowed is True
                and all(type(v) in (int, float) and math.isfinite(v) for v in stamps)
                and 0 < self.issued_stamp <= now < self.expire_stamp
                and bool(self.permit_id) and self.permit_id == action.permit_id
                and self.permit_generation == action.permit_generation
                and self.evaluated_intent_id == action.intent_id
                and self.session_id == action.session_id
                and self.command_epoch == action.command_epoch)


class ControlAuthority:
    def __init__(self, session_id=None):
        self.session_id = session_id or uuid.uuid4().hex
        self.command_epoch = 1
        self.mode = SystemMode.BOOT
        self.last_sequence = 0
        self.cycle_aborted = False

    def transition(self, mode, *, stopped=False, outputs_off=False, safety_fresh=False,
                   perception_ready=False, localization_ready=False, reset_authorized=False):
        if self.mode in (SystemMode.FAULT_LATCHED, SystemMode.EMERGENCY_STOP):
            if reset_authorized is not True or not all(v is True for v in (stopped, outputs_off, safety_fresh)):
                return self.mode
        if mode == SystemMode.AUTO_ACTIVE:
            return self.mode  # Only activate_new_task may enter AUTO_ACTIVE.
        if mode == SystemMode.AUTO_READY and not all(v is True for v in (
                stopped, outputs_off, safety_fresh, perception_ready, localization_ready)):
            mode = SystemMode.AUTO_PENDING
        if mode != self.mode:
            self.command_epoch += 1
            self.last_sequence = 0
            self.cycle_aborted = True  # Re-entry never resumes an old cycle.
            self.mode = mode
        return self.mode

    def admit(self, action, now, permit, readiness):
        # STOP has no energizing fields and bypasses authorization only to release.
        if action.direction == 0:
            return True, "STOP_DOMINANCE"
        if self.mode != SystemMode.AUTO_ACTIVE:
            return False, "NOT_AUTO_ACTIVE"
        if not (action.valid is True and permit.authorizes(action, now) and readiness is True):
            return False, "AUTHORIZATION_OR_READINESS_INVALID"
        if not all((action.command_id, action.intent_id, action.permit_id)):
            return False, "IDENTITY_MISSING"
        if action.session_id != self.session_id or action.command_epoch != self.command_epoch:
            return False, "OLD_SESSION_OR_EPOCH"
        if type(action.sequence) is not int or action.sequence <= self.last_sequence:
            return False, "DUPLICATE_OR_REORDERED"
        stamps = (now, action.issued_stamp, action.expire_stamp)
        if not (all(type(v) in (int, float) and math.isfinite(v) for v in stamps)
                and 0 < action.issued_stamp <= now < action.expire_stamp):
            return False, "EXPIRED_OR_INVALID_TIME"
        self.last_sequence = action.sequence
        return True, "AUTHORIZED"

    def activate_new_task(self, new_task=False):
        if self.mode != SystemMode.AUTO_READY or new_task is not True:
            return False
        self.command_epoch += 1
        self.last_sequence = 0
        self.cycle_aborted = False
        self.mode = SystemMode.AUTO_ACTIVE
        return True


@dataclass(frozen=True)
class MotionDecision:
    direction: int = 0
    enable: bool = False
    speed_command_valid: bool = False
    speed_command: float = 0.0
    phase: str = "STOP"
    reason: str = "NOT_CONFIGURED"


class VariableSpeedStrategy:
    def __init__(self, capability=AxisCapability(), bands=()):
        self.capability, self.bands = capability, tuple(bands)

    def step(self, position, target, *, feedback_fresh=False, permitted=False, **unused):
        if (self.capability.profile != DriveProfile.VARIABLE_SPEED or not self.capability.ready()
                or feedback_fresh is not True or permitted is not True
                or not all(type(v) in (int, float) and math.isfinite(v) for v in (position, target))):
            return MotionDecision(reason="CAPABILITY_OR_FEEDBACK_INVALID")
        if not self.bands or any(
                len(b) != 3 or type(b[0]) not in (int, float) or type(b[1]) not in (int, float)
                or not math.isfinite(b[0]) or not math.isfinite(b[1]) or b[0] < 0 or b[1] <= 0
                for b in self.bands):
            return MotionDecision(reason="SPEED_BANDS_NOT_CONFIGURED")
        ordered = sorted(self.bands, key=lambda band: band[0])
        distance = abs(target - position)
        if distance <= ordered[0][0]:
            return MotionDecision(phase="STOP", reason="TARGET_TOLERANCE")
        selected = ordered[0]
        for band in ordered:
            if distance > band[0]:
                selected = band
        return MotionDecision(1 if target > position else -1, True, True, selected[1],
                              selected[2], "POSITION_FEEDBACK")


class FixedSlowStrategy:
    def __init__(self, capability=AxisCapability(), stop_model=StopDistance(),
                 tolerance=None, settle_sec=None):
        self.capability, self.stop_model = capability, stop_model
        self.tolerance, self.settle_sec = tolerance, settle_sec
        self.stopping_since = None

    def step(self, position, target, *, speed=None, now=None, feedback_fresh=False, permitted=False):
        if (self.capability.profile != DriveProfile.FIXED_SLOW or not self.capability.ready()
                or feedback_fresh is not True or permitted is not True
                or not all(type(v) in (int, float) and math.isfinite(v)
                           for v in (position, target, now, self.tolerance, self.settle_sec))
                or self.tolerance <= 0 or self.settle_sec <= 0):
            return MotionDecision(reason="CAPABILITY_OR_FEEDBACK_INVALID")
        stopping = self.stop_model.distance(speed)
        if stopping is None:
            return MotionDecision(reason="STOP_MODEL_NOT_CONFIGURED")
        distance = abs(target - position)
        if self.stopping_since is not None:
            if now < self.stopping_since:
                return MotionDecision(reason="CLOCK_REGRESSION")
            if now - self.stopping_since < self.settle_sec:
                return MotionDecision(phase="SETTLING", reason="WAIT_STABLE")
            return MotionDecision(phase="AT_TARGET" if distance <= self.tolerance else "REPLAN_REQUIRED",
                                  reason="REMEASURED_NO_AUTOMATIC_PULSE_JOG")
        if distance <= max(stopping, self.tolerance):
            self.stopping_since = now
            return MotionDecision(phase="SETTLING", reason="EARLY_STOP")
        return MotionDecision(1 if target > position else -1, True, False, 0.0,
                              "FIXED_SLOW", "POSITION_FEEDBACK")


class AxisExecutor:
    axis = "ALL"

    def __init__(self, authority, strategy=None):
        self.authority, self.strategy = authority, strategy

    def execute(self, action, now, *, readiness=SystemReadiness(), permit=PermitEvidence(),
                position=None, feedback_fresh=False, speed=None):
        axis_ready = {
            "X": readiness.x_auto_ready, "Y": readiness.y_auto_ready,
            "Z": readiness.z_lower_ready if action.direction < 0 else readiness.z_raise_ready,
            "G": readiness.grab_ready,
        }.get(self.axis, False)
        allowed, reason = self.authority.admit(action, now, permit, axis_ready)
        decision = MotionDecision(reason=reason)
        if allowed and action.direction != 0:
            if action.axis != self.axis or action.direction not in (-1, 1):
                decision = MotionDecision(reason="AXIS_OR_DIRECTION_MISMATCH")
            elif self.strategy is not None:
                decision = self.strategy.step(position, action.target, speed=speed, now=now,
                                              feedback_fresh=feedback_fresh, permitted=True)
            else:
                decision = MotionDecision(reason="EXECUTION_STRATEGY_NOT_CONFIGURED")
            if decision.enable and decision.direction != action.direction:
                decision = MotionDecision(reason="AUTHORIZED_DIRECTION_MISMATCH")
        return ActuationRequest(self.axis, decision.direction, decision.enable,
                                decision.speed_command_valid, decision.speed_command,
                                action.command_id, self.authority.session_id,
                                self.authority.command_epoch, action.sequence,
                                now, now if not decision.enable else min(action.expire_stamp, permit.expire_stamp),
                                decision.reason)


class BridgeServoExecutor(AxisExecutor):
    axis = "X"


class TrolleyExecutor(AxisExecutor):
    axis = "Y"


class HoistExecutor(AxisExecutor):
    axis = "Z"


class GrabExecutor(AxisExecutor):
    axis = "G"
