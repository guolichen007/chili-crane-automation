"""Point-time intervals and explicit clock evidence; no spatial or control authority."""
from dataclasses import dataclass
import math
import numpy as np
from .timescale import TimeScale, TimeScaleRelation, known


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
    raw_time_scale: str = "UNKNOWN"
    relation: TimeScaleRelation = TimeScaleRelation()

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
        scale = cfg.get("raw_time_scale", "UNKNOWN")
        if scale not in {s.value for s in TimeScale}:
            raise ValueError("RAW_TIME_SCALE_INVALID")
        relation = TimeScaleRelation.from_config(cfg.get("time_scale_relation"))
        if relation.bench_allowed and relation.source_domain != domain:
            raise ValueError("TIME_SCALE_SOURCE_DOMAIN_MISMATCH")
        if mode == "HOST_DERIVED" and state != "PROVISIONAL":
            raise ValueError("HOST_DERIVED_MUST_BE_PROVISIONAL")
        if mode == "HOST_DERIVED" and scale not in {"UNKNOWN", "HOST_UTC"}:
            raise ValueError("HOST_DERIVED_SCALE_MISMATCH")
        if mode == "SENSOR_PTP" and scale not in {"UNKNOWN", "PTP_TIMESCALE"}:
            raise ValueError("SENSOR_PTP_SCALE_MISMATCH")
        if not domain or domain == "NOT_CONFIGURED":
            if not (mode == "SENSOR_PTP" and state == "PROBING"):
                raise ValueError("CLOCK_DOMAIN_NOT_CONFIGURED")
        if mode == "SENSOR_PTP" and state not in {"PROBING", "LOCAL_VERIFIED", "VALID"}:
            raise ValueError("PTP_EVIDENCE_NOT_VERIFIED")
        if mode == "SENSOR_PTP" and state in {"LOCAL_VERIFIED", "VALID"} and not evidence:
            raise ValueError("PTP_TIME_EVIDENCE_NOT_VERIFIED")
        return cls(mode, state, domain, evidence, scale, relation)

    @property
    def ptp_verified(self):
        return (self.mode == "SENSOR_PTP" and self.state in {"LOCAL_VERIFIED", "VALID"}
                and known(self.domain) and bool(self.evidence_id)
                and self.raw_time_scale == "PTP_TIMESCALE" and self.relation.bench_allowed)

    @property
    def pairing_ready(self):
        return (self.mode == "HOST_DERIVED" and self.state == "PROVISIONAL"
                and known(self.domain) and self.raw_time_scale == "HOST_UTC") or self.ptp_verified

    @property
    def production_ready(self):
        return self.state == "VALID" and self.ptp_verified and self.relation.production_allowed

    def compatible(self, other):
        if self.mode != other.mode or self.domain != other.domain:
            return False
        if self.relation.bench_allowed and other.relation.bench_allowed:
            return (self.relation.target_domain == other.relation.target_domain
                    and self.relation.utc_offset_sec == other.relation.utc_offset_sec)
        return True
