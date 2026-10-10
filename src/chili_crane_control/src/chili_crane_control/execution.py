"""Accept-once mock execution with bounded leases; no device protocol or I/O."""
import math
import uuid
from dataclasses import dataclass, replace
from typing import Protocol, runtime_checkable
from .contracts import SystemMode, AxisCapability, DriveProfile, StopDistance, SystemReadiness


def finite(*values):
    return all(type(v) in (int, float) and math.isfinite(v) for v in values)


@dataclass(frozen=True)
class AuthorizedAction:
    command_id: str = ""
    intent_id: str = ""
    permit_id: str = ""
    permit_generation: int = 0
    session_id: str = ""
    command_epoch: int = 0
    sequence: int = 0  # Admission command_sequence, not an actuation tick counter.
    issued_stamp: float = 0.0
    expire_stamp: float = 0.0
    valid: bool = False
    axis: str = "ALL"
    direction: int = 0
    target: float | None = None
    action: int = 0  # AuthorizedCommand ACTION_* enum; unknown never grants motion.
    cycle_phase: str = ""


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
    command_sequence: int = 0
    actuation_sequence: int = 0
    issued_stamp: float = 0.0
    expire_stamp: float = 0.0
    reason: str = "NOT_CONFIGURED"


ACTION_BY_DIRECTION = {("X", 1): 2, ("X", -1): 2, ("Y", 1): 3, ("Y", -1): 3,
                       ("Z", 1): 4, ("Z", -1): 5, ("G", 1): 6, ("G", -1): 7}
PERMISSION_FIELDS = {
    ("X", 1): "allow_x_positive", ("X", -1): "allow_x_negative",
    ("Y", 1): "allow_y_positive", ("Y", -1): "allow_y_negative",
    ("Z", 1): "allow_raise", ("Z", -1): "allow_lower",
    ("G", 1): "allow_grab_open", ("G", -1): "allow_grab_close",
}


def direction_permission(permit, axis, direction):
    """Single SafetyPermit mapping. STOP never grants an energizing permission."""
    if type(direction) is not int or not isinstance(axis, str) or axis not in {"X", "Y", "Z", "G", "ALL"}:
        return False
    if direction == 0:
        return True
    field = PERMISSION_FIELDS.get((axis, direction))
    return field is not None and getattr(permit, field, False) is True


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
    allow_x_positive: bool = False
    allow_x_negative: bool = False
    allow_y_positive: bool = False
    allow_y_negative: bool = False
    allow_lower: bool = False
    allow_raise: bool = False
    allow_grab_open: bool = False
    allow_grab_close: bool = False
    allow_auto_task: bool = False
    allow_unload: bool = False

    def authorizes(self, action, now):
        return (self.valid is True and ActionAuthorizationPolicy.authorizes(self, action)
                and finite(self.issued_stamp, self.expire_stamp, now)
                and 0 < self.issued_stamp <= now < self.expire_stamp
                and type(self.permit_generation) is int and self.permit_generation > 0
                and type(action.permit_generation) is int
                and type(self.command_epoch) is int and self.command_epoch > 0
                and type(action.command_epoch) is int
                and bool(self.permit_id) and self.permit_id == action.permit_id
                and self.permit_generation == action.permit_generation
                and self.evaluated_intent_id == action.intent_id
                and self.session_id == action.session_id
                and self.command_epoch == action.command_epoch)


class ActionAuthorizationPolicy:
    @staticmethod
    def authorizes(permit, action):
        if action.direction == 0:
            return True
        if (permit.allow_auto_task is not True
                or not direction_permission(permit, action.axis, action.direction)):
            return False
        if action.axis == "G" and action.direction > 0:
            if action.cycle_phase == "OPEN_GRAB":
                return permit.allow_unload is True
            return action.cycle_phase == "ENSURE_GRAB_OPEN"
        return True


# Safety exits are explicit for every non-latched state, including startup.
SAFETY_EXITS = {SystemMode.PROTECTIVE_STOP, SystemMode.FAULT_LATCHED,
                SystemMode.EMERGENCY_STOP, SystemMode.REMOTE}
MODE_TRANSITIONS = {
    SystemMode.BOOT: {SystemMode.SELF_CHECK},
    SystemMode.SELF_CHECK: {SystemMode.SAFE_IDLE},
    SystemMode.SAFE_IDLE: {SystemMode.AUTO_PENDING, SystemMode.MAINTENANCE},
    SystemMode.AUTO_PENDING: {SystemMode.AUTO_READY, SystemMode.SAFE_IDLE},
    SystemMode.AUTO_READY: {SystemMode.SAFE_IDLE, SystemMode.AUTO_PENDING},
    SystemMode.AUTO_ACTIVE: {SystemMode.SAFE_IDLE},
    SystemMode.REMOTE: {SystemMode.SAFE_IDLE, SystemMode.SELF_CHECK},
    SystemMode.PROTECTIVE_STOP: {SystemMode.SAFE_IDLE, SystemMode.SELF_CHECK},
    SystemMode.MAINTENANCE: {SystemMode.SAFE_IDLE, SystemMode.SELF_CHECK},
    SystemMode.FAULT_LATCHED: {SystemMode.SELF_CHECK, SystemMode.SAFE_IDLE,
                              SystemMode.EMERGENCY_STOP},
    SystemMode.EMERGENCY_STOP: {SystemMode.SELF_CHECK, SystemMode.SAFE_IDLE},
}


class ControlAuthority:
    def __init__(self, session_id=None):
        self.session_id = session_id or uuid.uuid4().hex
        self.command_epoch = 1
        self.mode = SystemMode.BOOT
        self.last_sequence = 0
        self.actuation_sequence = 0
        self.accepted_ids = set()
        self.cycle_aborted = False

    def transition(self, mode, *, stopped=False, outputs_off=False, safety_fresh=False,
                   perception_ready=False, localization_ready=False, reset_authorized=False,
                   self_check_passed=False):
        if not isinstance(mode, SystemMode) or mode == self.mode:
            return self.mode
        latched = self.mode in (SystemMode.FAULT_LATCHED, SystemMode.EMERGENCY_STOP)
        allowed = MODE_TRANSITIONS[self.mode] | (set() if latched else SAFETY_EXITS)
        if mode not in allowed:
            return self.mode
        # A fault may always escalate to E-stop; reset may never jump to AUTO.
        if latched and mode != SystemMode.EMERGENCY_STOP and not (
                reset_authorized is True
                and all(v is True for v in (stopped, outputs_off, safety_fresh))):
            return self.mode
        if mode == SystemMode.SAFE_IDLE and not all(
                v is True for v in (stopped, outputs_off, safety_fresh)):
            return self.mode
        if self.mode == SystemMode.SELF_CHECK and mode == SystemMode.SAFE_IDLE and self_check_passed is not True:
            return self.mode
        if mode == SystemMode.AUTO_READY and not all(v is True for v in (
                stopped, outputs_off, safety_fresh, perception_ready, localization_ready)):
            return self.mode
        self.command_epoch += 1
        self.last_sequence = 0
        self.accepted_ids.clear()
        self.cycle_aborted = True
        self.mode = mode
        return self.mode

    def validate_active(self, action, now, permit, readiness):
        if self.mode != SystemMode.AUTO_ACTIVE:
            return False, "NOT_AUTO_ACTIVE"
        if not (action.valid is True and permit.authorizes(action, now) and readiness is True):
            return False, "AUTHORIZATION_OR_READINESS_INVALID"
        if not all((action.command_id, action.intent_id, action.permit_id)):
            return False, "IDENTITY_MISSING"
        if action.session_id != self.session_id or action.command_epoch != self.command_epoch:
            return False, "OLD_SESSION_OR_EPOCH"
        if not finite(now, action.issued_stamp, action.expire_stamp) or not (
                0 < action.issued_stamp <= now < action.expire_stamp):
            return False, "EXPIRED_OR_INVALID_TIME"
        return True, "AUTHORIZED"

    def admit(self, action, now, permit, readiness):
        if (not isinstance(action.axis, str) or action.axis not in {"X", "Y", "Z", "G"}
                or type(action.direction) is not int or action.direction not in (-1, 1)
                or not finite(action.target) or type(action.action) is not int
                or action.action != ACTION_BY_DIRECTION.get((action.axis, action.direction))):
            return False, "AXIS_DIRECTION_OR_TARGET_INVALID"
        allowed, reason = self.validate_active(action, now, permit, readiness)
        if not allowed:
            return allowed, reason
        if (type(action.sequence) is not int or action.sequence <= self.last_sequence
                or action.command_id in self.accepted_ids):
            return False, "DUPLICATE_OR_REORDERED"
        self.last_sequence = action.sequence
        self.accepted_ids.add(action.command_id)
        return True, "AUTHORIZED"

    def activate_new_task(self, new_task=False):
        if self.mode != SystemMode.AUTO_READY or new_task is not True:
            return False
        self.command_epoch += 1
        self.last_sequence = 0
        self.accepted_ids.clear()
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


@runtime_checkable
class ExecutionStrategy(Protocol):
    def configured(self) -> bool: ...
    def supports_axis(self, axis: str) -> bool: ...
    def begin(self, execution_key) -> None: ...
    def step(self, position, target, **evidence) -> MotionDecision: ...


class VariableSpeedStrategy:
    def __init__(self, capability=AxisCapability(), bands=()):
        self.capability, self.bands = capability, tuple(bands)

    def supports_axis(self, axis):
        return axis in {"Y", "Z"}

    def begin(self, execution_key):
        pass

    def configured(self):
        return (self.capability.profile == DriveProfile.VARIABLE_SPEED and self.capability.ready()
                and bool(self.bands) and all(isinstance(b, (tuple, list)) and len(b) == 3
                    and finite(b[0], b[1]) and b[0] >= 0 and b[1] > 0 for b in self.bands))

    def step(self, position, target, *, feedback_fresh=False, permitted=False, **unused):
        if (self.capability.profile != DriveProfile.VARIABLE_SPEED or not self.capability.ready()
                or feedback_fresh is not True or permitted is not True or not finite(position, target)):
            return MotionDecision(reason="CAPABILITY_OR_FEEDBACK_INVALID")
        if not self.bands or any(
                len(b) != 3 or not finite(b[0], b[1]) or b[0] < 0 or b[1] <= 0
                for b in self.bands):
            return MotionDecision(reason="SPEED_BANDS_NOT_CONFIGURED")
        ordered = sorted(self.bands, key=lambda band: band[0])
        distance = abs(target - position)
        if distance <= ordered[0][0]:
            return MotionDecision(phase="AT_TARGET", reason="TARGET_TOLERANCE")
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
        self.execution_key = None

    def supports_axis(self, axis):
        return axis in {"Y", "Z"}

    def configured(self):
        return (self.capability.profile == DriveProfile.FIXED_SLOW and self.capability.ready()
                and self.stop_model.distance(0) is not None
                and finite(self.tolerance, self.settle_sec)
                and self.tolerance > 0 and self.settle_sec > 0)

    def begin(self, execution_key):
        # Reset exactly once for a new admitted command/epoch, never on AT_TARGET ticks.
        if execution_key != self.execution_key:
            self.execution_key = execution_key
            self.stopping_since = None

    def step(self, position, target, *, speed=None, now=None, feedback_fresh=False, permitted=False):
        if (self.capability.profile != DriveProfile.FIXED_SLOW or not self.capability.ready()
                or feedback_fresh is not True or permitted is not True
                or not finite(position, target, now, self.tolerance, self.settle_sec)
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


@dataclass(frozen=True)
class ActiveExecutionContext:
    action: AuthorizedAction
    state: str = "ACCEPTED"
    phase: str = "ACCEPTED"
    last_tick: float = 0.0


TERMINAL_STATES = {"COMPLETED", "FAILED", "CANCELLED", "ABORTED"}


class AxisExecutor:
    axis = "ALL"

    def __init__(self, authority, strategy=None, lease_sec=None):
        self.authority, self.strategy, self.lease_sec = authority, strategy, lease_sec
        self.context = None

    def _ready(self, readiness, direction):
        return {"X": readiness.x_auto_ready, "Y": readiness.y_auto_ready,
                "Z": readiness.z_lower_ready if direction < 0 else readiness.z_raise_ready,
                "G": readiness.grab_ready}.get(self.axis, False)

    def accept(self, action, now, *, readiness=SystemReadiness(), permit=PermitEvidence(),
               position=None, feedback_fresh=False):
        # Route and validate before any global sequence is consumed.
        if (action.axis != self.axis or type(action.direction) is not int
                or action.direction not in (-1, 1) or type(action.action) is not int
                or action.action != ACTION_BY_DIRECTION.get((self.axis, action.direction))
                or not finite(action.target, position)
                or feedback_fresh is not True):
            return False, "AXIS_DIRECTION_TARGET_OR_FEEDBACK_INVALID"
        if (action.target - position) * action.direction < 0:
            return False, "AUTHORIZED_DIRECTION_MISMATCH"
        if (not isinstance(self.strategy, ExecutionStrategy)
                or not self.strategy.supports_axis(self.axis)
                or not self.strategy.configured() or not finite(self.lease_sec) or self.lease_sec <= 0):
            return False, "STRATEGY_OR_LEASE_NOT_CONFIGURED"
        if self.context is not None and self.context.state not in TERMINAL_STATES:
            return False, "EXECUTION_ALREADY_ACTIVE"
        allowed, reason = self.authority.admit(action, now, permit, self._ready(readiness, action.direction))
        if allowed:
            self.context = ActiveExecutionContext(action)
            self.strategy.begin((action.session_id, action.command_epoch, action.command_id))
        return allowed, reason

    def _request(self, now, decision, action=None, permit=None):
        self.authority.actuation_sequence += 1
        expiry = now
        if decision.enable:
            expiry = min(now + self.lease_sec, action.expire_stamp, permit.expire_stamp)
        return ActuationRequest(
            axis=self.axis, direction=decision.direction, enable=decision.enable,
            speed_command_valid=decision.speed_command_valid, speed_command=decision.speed_command,
            source_command_id=action.command_id if action else "",
            session_id=self.authority.session_id, command_epoch=self.authority.command_epoch,
            command_sequence=action.sequence if action else 0,
            actuation_sequence=self.authority.actuation_sequence,
            issued_stamp=now, expire_stamp=expiry, reason=decision.reason)

    def cancel(self, now, *, aborted=False, reason="EXPLICIT_CANCEL"):
        if self.context is not None and self.context.state not in TERMINAL_STATES:
            self.context = replace(self.context, state="ABORTED" if aborted else "CANCELLED", phase=reason)
        return self._request(now, MotionDecision(reason=reason),
                             self.context.action if self.context else None)

    def tick(self, now, *, readiness=SystemReadiness(), permit=PermitEvidence(),
             position=None, feedback_fresh=False, speed=None):
        if self.context is None:
            return self._request(now, MotionDecision(reason="NO_ACTIVE_EXECUTION"))
        action = self.context.action
        if self.context.state in TERMINAL_STATES:
            return self._request(now, MotionDecision(reason=self.context.state), action)
        allowed, reason = self.authority.validate_active(
            action, now, permit, self._ready(readiness, action.direction))
        if (not allowed or feedback_fresh is not True or not finite(now, position)
                or now < self.context.last_tick):
            return self.cancel(now, aborted=True, reason=reason if not allowed else "FEEDBACK_OR_CLOCK_INVALID")
        decision = self.strategy.step(position, action.target, speed=speed, now=now,
                                      feedback_fresh=True, permitted=True)
        if decision.enable and decision.direction != action.direction:
            decision = MotionDecision(reason="AUTHORIZED_DIRECTION_MISMATCH")
        state = "EXECUTING"
        if decision.phase == "AT_TARGET":
            state = "COMPLETED"
        elif decision.phase == "REPLAN_REQUIRED" or (not decision.enable and decision.phase != "SETTLING"):
            state = "FAILED"
        self.context = replace(self.context, state=state, phase=decision.phase, last_tick=now)
        return self._request(now, decision, action, permit)

    def execute(self, action, now, **evidence):
        """One-shot convenience: repeated admission is rejected; use tick thereafter."""
        if action.direction == 0 and type(action.direction) is int and action.axis in {self.axis, "ALL"}:
            return self.cancel(now, reason="STOP_DOMINANCE")
        allowed, reason = self.accept(action, now, **{
            k: v for k, v in evidence.items() if k != "speed"})
        if not allowed:
            return self._request(now, MotionDecision(reason=reason), action)
        return self.tick(now, **evidence)


class BridgeServoExecutor(AxisExecutor):
    axis = "X"


class TrolleyExecutor(AxisExecutor):
    axis = "Y"


class HoistExecutor(AxisExecutor):
    axis = "Z"


class GrabExecutor(AxisExecutor):
    axis = "G"
