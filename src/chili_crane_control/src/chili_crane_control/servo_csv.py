"""Pure helpers for normalized servo CSV replay.

Wall-clock scheduling, source sample time, ROS publication time, and evidence
age are intentionally separate concepts.
"""

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List


VALID = "VALID"
DEGRADED = "DEGRADED"
NOT_CONFIGURED = "NOT_CONFIGURED"


@dataclass(frozen=True)
class ServoCsvSample:
    stamp_sec: float
    raw_position: float
    position_m: float
    velocity_mps: float
    valid: bool
    homed: bool
    fault: bool


def parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes"}:
        return True
    if normalized in {"0", "false", "no"}:
        return False
    raise ValueError("invalid boolean value: {!r}".format(value))


def load_samples(csv_path: Path) -> List[ServoCsvSample]:
    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {
            "stamp_sec",
            "raw_position",
            "position_m",
            "velocity_mps",
            "valid",
            "homed",
            "fault",
        }
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(
                "missing required CSV columns: " + ", ".join(sorted(missing))
            )
        samples = [
            ServoCsvSample(
                stamp_sec=float(row["stamp_sec"]),
                raw_position=float(row["raw_position"]),
                position_m=float(row["position_m"]),
                velocity_mps=float(row["velocity_mps"]),
                valid=parse_bool(row["valid"]),
                homed=parse_bool(row["homed"]),
                fault=parse_bool(row["fault"]),
            )
            for row in reader
        ]
    if not samples:
        raise ValueError("servo CSV contains no samples")
    for sample in samples:
        numeric_values = (
            sample.stamp_sec,
            sample.raw_position,
            sample.position_m,
            sample.velocity_mps,
        )
        if not all(math.isfinite(value) for value in numeric_values):
            raise ValueError("servo CSV numeric values must be finite")
    if any(
        right.stamp_sec <= left.stamp_sec
        for left, right in zip(samples, samples[1:])
    ):
        raise ValueError("servo CSV timestamps must be strictly increasing")
    return samples


def classify_sample(calibration_id: str, sample: ServoCsvSample) -> str:
    if calibration_id.strip() in {"", "NOT_CONFIGURED"}:
        return NOT_CONFIGURED
    if sample.valid and sample.homed and not sample.fault:
        return VALID
    return DEGRADED


def scheduled_monotonic_time(
    wall_start_sec: float,
    source_start_sec: float,
    source_sample_sec: float,
) -> float:
    return wall_start_sec + (source_sample_sec - source_start_sec)


def mapped_sample_time(
    ros_start_sec: float,
    source_start_sec: float,
    source_sample_sec: float,
) -> float:
    return ros_start_sec + (source_sample_sec - source_start_sec)


def evidence_age_seconds(
    publish_time_sec: float,
    evidence_time_sec: float,
) -> float:
    if not math.isfinite(publish_time_sec) or not math.isfinite(
        evidence_time_sec
    ):
        raise ValueError("evidence timestamps must be finite")
    return max(0.0, publish_time_sec - evidence_time_sec)
