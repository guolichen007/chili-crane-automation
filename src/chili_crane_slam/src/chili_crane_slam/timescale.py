"""Explicit time scales and evidence-based conversion; never a vendor correction."""
from dataclasses import dataclass
from enum import Enum
import math
import numpy as np


class TimeScale(str, Enum):
    UNKNOWN = "UNKNOWN"
    HOST_UTC = "HOST_UTC"
    PTP_TIMESCALE = "PTP_TIMESCALE"
    BOOT_RELATIVE = "BOOT_RELATIVE"


def known(value):
    return isinstance(value, str) and bool(value.strip()) and value.strip() not in {"UNKNOWN", "NOT_CONFIGURED"}


@dataclass(frozen=True)
class TimeScaleRelation:
    source_scale: str = "UNKNOWN"
    target_scale: str = "UNKNOWN"
    utc_offset_sec: float | None = None
    relation_state: str = "NOT_CONFIGURED"
    evidence_id: str = "NOT_CONFIGURED"
    offset_source: str = "NOT_CONFIGURED"
    current_utc_offset_valid: bool | None = None
    time_traceable: bool | None = None
    source_domain: str = "NOT_CONFIGURED"
    target_domain: str = "NOT_CONFIGURED"

    @classmethod
    def from_config(cls, cfg):
        cfg = cfg or {}
        if not isinstance(cfg, dict):
            raise ValueError("TIME_SCALE_RELATION_INVALID")
        result = cls(**{k: cfg[k] for k in cls.__dataclass_fields__ if k in cfg})
        if result.source_scale not in {s.value for s in TimeScale} or result.target_scale not in {s.value for s in TimeScale}:
            raise ValueError("TIME_SCALE_UNKNOWN")
        if result.relation_state not in {"NOT_CONFIGURED", "PROBING", "LOCAL_VERIFIED", "VALID", "DEGRADED"}:
            raise ValueError("TIME_SCALE_RELATION_STATE_INVALID")
        for flag in (result.current_utc_offset_valid, result.time_traceable):
            if flag is not None and type(flag) is not bool:
                raise ValueError("TIME_SCALE_TRACEABILITY_FLAG_INVALID")
        if result.utc_offset_sec is not None and (
                type(result.utc_offset_sec) not in (int, float) or not math.isfinite(result.utc_offset_sec)
                or abs(result.utc_offset_sec) > 128 or int(result.utc_offset_sec) != result.utc_offset_sec):
            raise ValueError("UTC_OFFSET_NOT_INTEGER_SECONDS")
        if result.relation_state in {"LOCAL_VERIFIED", "VALID"} and not result.bench_allowed:
            raise ValueError("TIME_SCALE_RELATION_EVIDENCE_MISSING")
        if result.relation_state == "VALID" and not result.production_allowed:
            raise ValueError("UTC_PRODUCTION_TRACEABILITY_NOT_VERIFIED")
        return result

    @property
    def bench_allowed(self):
        return (self.source_scale == "PTP_TIMESCALE" and self.target_scale == "HOST_UTC"
                and self.relation_state in {"LOCAL_VERIFIED", "VALID"}
                and type(self.utc_offset_sec) in (int, float) and math.isfinite(self.utc_offset_sec)
                and abs(self.utc_offset_sec) <= 128 and int(self.utc_offset_sec) == self.utc_offset_sec
                and known(self.evidence_id) and known(self.offset_source)
                and known(self.source_domain) and known(self.target_domain))

    @property
    def production_allowed(self):
        return (self.bench_allowed and self.relation_state == "VALID"
                and self.current_utc_offset_valid is True and self.time_traceable is True)


def normalize_times(raw_header_ns, points, raw_scale, relation=None):
    """Copy points; subtract the same integer offset from header and every point."""
    if type(raw_header_ns) is not int or raw_header_ns <= 0:
        raise ValueError("RAW_HEADER_INVALID")
    if raw_scale == "HOST_UTC":
        offset = 0  # Host-derived LiDAR and HOST_UTC camera never inherit a PTP offset.
    elif raw_scale == "PTP_TIMESCALE" and relation and relation.bench_allowed:
        offset = int(relation.utc_offset_sec)
    else:
        raise ValueError("TIME_SCALE_RELATION_NOT_VERIFIED")
    normalized = points.copy()
    normalized["timestamp"] -= offset
    header = raw_header_ns - offset * 1000000000
    if header <= 0 or not np.isfinite(normalized["timestamp"]).all() or (normalized["timestamp"] <= 0).any():
        raise ValueError("NORMALIZED_TIMESTAMP_INVALID")
    return header, normalized, offset
