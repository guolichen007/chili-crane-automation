"""Point-time intervals and explicit clock evidence; no spatial or control authority."""
from dataclasses import dataclass
import math
import numpy as np


def bounded(value, name, upper=10.0, allow_zero=False):
    if (type(value) not in (int, float) or not math.isfinite(value)
            or value < 0 or (value == 0 and not allow_zero) or value > upper):
        raise ValueError(name + " NOT_CONFIGURED/out of bounds")
    return float(value)


@dataclass(frozen=True)
class FrameTime:
    header: float
    start: float
    end: float
    mid: float

    @classmethod
    def from_points(cls, header, timestamps):
        if not math.isfinite(header) or header <= 0:
            raise ValueError("invalid header timestamp")
        ts = np.asarray(timestamps)
        if not ts.size or not np.isfinite(ts).all() or (ts <= 0).any():
            raise ValueError("invalid point timestamps")
        start, end = float(ts.min()), float(ts.max())
        return cls(float(header), start, end, start + (end - start) / 2)

    @property
    def span(self):
        return self.end - self.start

    def check(self, now, maximum_future_skew, stale, first_point_tolerance=None):
        future = bounded(maximum_future_skew, "maximum_future_skew_sec", 1.0, True)
        bounded(stale, "stale_timeout_sec")
        if not math.isfinite(now) or now <= 0:
            raise ValueError("invalid receive clock")
        if max(self.header, self.end) > now + future:
            raise ValueError("FUTURE_TIMESTAMP")
        if now - min(self.header, self.start) > stale:
            raise ValueError("STALE_TIMESTAMP")
        if first_point_tolerance is not None:
            tolerance = bounded(first_point_tolerance, "header_first_point_tolerance_sec", 1.0, True)
            if abs(self.header - self.start) > tolerance + 1e-9:
                raise ValueError("FIRST_POINT_HEADER_MISMATCH")

    def overlap(self, other):
        intersection = max(0.0, min(self.end, other.end) - max(self.start, other.start))
        union = max(self.end, other.end) - min(self.start, other.start)
        ratio = intersection / union if union > 0 else float(self.mid == other.mid)
        return intersection, ratio


@dataclass(frozen=True)
class ClockContract:
    mode: str
    state: str
    domain: str
    evidence_id: str = ""

    @classmethod
    def from_config(cls, cfg):
        mode, state, domain = (cfg.get(k, "NOT_CONFIGURED")
                               for k in ("clock_mode", "clock_sync_state", "clock_domain"))
        if mode not in {"SENSOR_PTP", "HOST_DERIVED"} or not isinstance(domain, str) or domain in {"", "NOT_CONFIGURED"}:
            raise ValueError("CLOCK_DOMAIN_NOT_CONFIGURED")
        evidence = cfg.get("time_evidence_id") or ""
        if mode == "HOST_DERIVED" and state != "PROVISIONAL":
            raise ValueError("HOST_DERIVED_MUST_BE_PROVISIONAL")
        if mode == "SENSOR_PTP" and (state != "VALID" or not evidence or evidence == "NOT_CONFIGURED"):
            raise ValueError("PTP_EVIDENCE_NOT_VERIFIED")
        return cls(mode, state, domain, evidence)

    @property
    def ptp_verified(self):
        return self.mode == "SENSOR_PTP" and self.state == "VALID" and bool(self.evidence_id)

    def compatible(self, other):
        return self.mode == other.mode and self.domain == other.domain
