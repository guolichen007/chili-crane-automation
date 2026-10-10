"""Pure adapter lease seam. Not a physical watchdog or an output writer."""
import math
import time
from .actuation import request_to_do, direction_only_supported


class ActuationLeaseGuard:
    def __init__(self, session_id="", command_epoch=0, max_lease_sec=None,
                 clock=time.monotonic, supports=direction_only_supported):
        # Session/epoch must be established through a trusted control handshake.
        self.session_id = session_id
        self.command_epoch = command_epoch
        self.max_lease_sec = max_lease_sec
        self.last_sequence = 0
        self.current = None
        self.last_poll = None
        self.clock, self.supports = clock, supports
        self.deadline = None

    def receive(self, request, now, monotonic_now=None):
        self.current = None  # Any invalid, reordered or STOP frame releases the lease.
        self.deadline = None
        observed = self.clock() if monotonic_now is None else monotonic_now
        identity_valid = (
            bool(self.session_id) and type(self.command_epoch) is int and self.command_epoch > 0
            and request.session_id == self.session_id and request.command_epoch == self.command_epoch
            and type(request.actuation_sequence) is int and request.actuation_sequence > self.last_sequence)
        if request.enable is not True:
            # A trusted newer OFF is also a replay barrier for delayed pre-STOP ON frames.
            if identity_valid:
                self.last_sequence = request.actuation_sequence
            return False
        numbers = (now, observed, request.issued_stamp, request.expire_stamp, self.max_lease_sec)
        if not all(type(v) in (int, float) and math.isfinite(v) for v in numbers):
            return False
        if (not identity_valid or self.last_poll is not None and observed < self.last_poll
                or not self.supports(request)
                or type(request.command_sequence) is not int or request.command_sequence <= 0
                or not request.source_command_id or self.max_lease_sec <= 0
                or not 0 < request.issued_stamp <= now < request.expire_stamp
                or request.expire_stamp - request.issued_stamp > self.max_lease_sec
                or request.axis not in {"X", "Y", "Z", "G"}
                or type(request.direction) is not int or request.direction not in (-1, 1)):
            return False
        self.last_sequence = request.actuation_sequence
        self.current = request
        # Audit/source clock is checked only at admission. NTP cannot extend this lease.
        self.deadline = observed + min(self.max_lease_sec, request.expire_stamp - now)
        return True

    def poll(self, now=None):
        # Caller must poll at a verified rate; real disconnect safety needs hardware WDT/FSV.
        now = self.clock() if now is None else now
        valid_time = type(now) in (int, float) and math.isfinite(now)
        if (not valid_time or self.last_poll is not None and now < self.last_poll
                or self.current is not None and not now < self.deadline):
            self.current = None
        if valid_time:
            self.last_poll = now
        return self.current

    def logical_outputs(self, now):
        request = self.poll(now)
        return (False,) * 8 if request is None else request_to_do(request)
