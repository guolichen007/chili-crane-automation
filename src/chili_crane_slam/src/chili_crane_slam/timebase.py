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

    def check_span(self, maximum_frame_span):
        if maximum_frame_span is not None:
            maximum = bounded(maximum_frame_span, "maximum_frame_span_sec")
            if self.span > maximum + 1e-9:
                raise ValueError("FRAME_SPAN_EXCEEDED")

    def check(self, now, maximum_future_skew, stale, first_point_tolerance=None,
              maximum_frame_span=None):
        self.check_span(maximum_frame_span)
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
    host_relation: str = "NOT_CONFIGURED"
    host_evidence_id: str = ""
    host_domain: str = "NOT_CONFIGURED"

    @classmethod
    def from_config(cls, cfg):
        mode, state, domain = (cfg.get(k, "NOT_CONFIGURED")
                               for k in ("clock_mode", "clock_sync_state", "clock_domain"))
        if mode not in {"SENSOR_PTP", "HOST_DERIVED"} or not isinstance(domain, str):
            raise ValueError("CLOCK_DOMAIN_NOT_CONFIGURED")
        def evidence_value(key):
            value = cfg.get(key)
            return value if isinstance(value, str) and value.strip() and value != "NOT_CONFIGURED" else ""
        evidence = evidence_value("time_evidence_id")
        relation = cfg.get("host_clock_relation", "NOT_CONFIGURED")
        host_evidence = evidence_value("host_clock_relation_evidence_id")
        host_domain = cfg.get("host_clock_domain", "NOT_CONFIGURED")
        if relation not in {"NOT_CONFIGURED", "PROBING", "VALID", "DEGRADED"}:
            raise ValueError("HOST_CLOCK_RELATION_INVALID")
        if mode == "HOST_DERIVED" and state != "PROVISIONAL":
            raise ValueError("HOST_DERIVED_MUST_BE_PROVISIONAL")
        if not domain or domain == "NOT_CONFIGURED":
            if not (mode == "SENSOR_PTP" and state == "PROBING"):
                raise ValueError("CLOCK_DOMAIN_NOT_CONFIGURED")
        if mode == "SENSOR_PTP" and state not in {"PROBING", "VALID"}:
            raise ValueError("PTP_EVIDENCE_NOT_VERIFIED")
        result = cls(mode, state, domain, evidence, relation, host_evidence, host_domain)
        if mode == "SENSOR_PTP" and state == "VALID" and not result.ptp_verified:
            raise ValueError("PTP_HOST_RELATION_OR_TIME_EVIDENCE_NOT_VERIFIED")
        return result

    @property
    def ptp_verified(self):
        return (self.mode == "SENSOR_PTP" and self.state == "VALID"
                and self.domain not in {"", "NOT_CONFIGURED"} and bool(self.evidence_id)
                and self.host_relation == "VALID" and bool(self.host_evidence_id)
                and self.host_domain == self.domain)

    @property
    def pairing_ready(self):
        return (self.mode == "HOST_DERIVED" and self.state == "PROVISIONAL"
                and self.domain not in {"", "NOT_CONFIGURED"}) or self.ptp_verified

    def compatible(self, other):
        return self.mode == other.mode and self.domain == other.domain
