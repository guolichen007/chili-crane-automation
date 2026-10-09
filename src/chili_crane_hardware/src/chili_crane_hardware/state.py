"""Monotonic evidence health; failed polls never refresh old samples."""
from dataclasses import dataclass
import math
import time


@dataclass(frozen=True)
class Evidence:
    value: object
    observed_monotonic: float
    source_counter: int = 0

    def age(self, now=None):
        now = time.monotonic() if now is None else now
        delta = now - self.observed_monotonic
        return delta if math.isfinite(delta) and delta >= 0 else float("inf")

    def is_fresh(self, timeout_sec, now=None):
        return self.age(now) <= timeout_sec


class EvidenceTracker:
    def __init__(self):
        self.sample = None
        self.communication_ok = False
        self.reason = "NOT_CONFIGURED"
        self._counter = 0

    def accept(self, value, now=None):
        self._counter += 1
        self.sample = Evidence(
            value, time.monotonic() if now is None else now, self._counter)
        self.communication_ok = True
        self.reason = "read_success"

    def failed(self, reason):
        self.communication_ok = False
        self.reason = str(reason)

    def validity(self, timeout_sec, now=None):
        if self.sample is None:
            return "NOT_CONFIGURED" if self.reason == "NOT_CONFIGURED" else "DEGRADED"
        if not self.sample.is_fresh(timeout_sec, now):
            return "STALE"
        return "VALID" if self.communication_ok else "DEGRADED"
