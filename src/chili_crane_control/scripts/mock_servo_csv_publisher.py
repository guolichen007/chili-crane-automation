#!/usr/bin/env python3
"""Replay normalized servo samples from CSV for interface development.

Required columns:
stamp_sec, raw_position, position_m, velocity_mps, valid, homed, fault
"""

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import List

import rospy

from chili_crane_msgs.msg import ServoState


@dataclass(frozen=True)
class ServoCsvSample:
    stamp_sec: float
    raw_position: float
    position_m: float
    velocity_mps: float
    valid: bool
    homed: bool
    fault: bool


def _parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes"}:
        return True
    if normalized in {"0", "false", "no"}:
        return False
    raise ValueError(f"invalid boolean value: {value!r}")


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
                valid=_parse_bool(row["valid"]),
                homed=_parse_bool(row["homed"]),
                fault=_parse_bool(row["fault"]),
            )
            for row in reader
        ]
    if not samples:
        raise ValueError("servo CSV contains no samples")
    if any(
        right.stamp_sec <= left.stamp_sec
        for left, right in zip(samples, samples[1:])
    ):
        raise ValueError("servo CSV timestamps must be strictly increasing")
    return samples


def main() -> None:
    rospy.init_node("mock_servo_csv_publisher")
    csv_file = str(rospy.get_param("~csv_file", "")).strip()
    if not csv_file:
        raise RuntimeError("~csv_file is required; no default data is invented")

    samples = load_samples(Path(csv_file))
    frame_id = str(rospy.get_param("~frame_id", "crane_01/base"))
    calibration_id = str(
        rospy.get_param("~calibration_id", "NOT_CONFIGURED")
    )
    calibration_configured = calibration_id not in {
        "",
        "NOT_CONFIGURED",
    }
    publisher = rospy.Publisher(
        "hardware/servo_state", ServoState, queue_size=10
    )
    start_wall = rospy.Time.now()
    start_sample = samples[0].stamp_sec

    for sample in samples:
        if rospy.is_shutdown():
            break
        target_elapsed = sample.stamp_sec - start_sample
        while (
            not rospy.is_shutdown()
            and (rospy.Time.now() - start_wall).to_sec() < target_elapsed
        ):
            rospy.sleep(0.001)

        message = ServoState()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = frame_id
        if not calibration_configured:
            message.validity = ServoState.NOT_CONFIGURED
        elif sample.valid and not sample.fault and sample.homed:
            message.validity = ServoState.VALID
        else:
            message.validity = ServoState.DEGRADED
        message.reason = (
            "csv_sample_valid"
            if message.validity == ServoState.VALID
            else "csv_calibration_not_configured"
            if message.validity == ServoState.NOT_CONFIGURED
            else "csv_sample_invalid_unhomed_or_fault"
        )
        message.raw_position = sample.raw_position
        message.position_m = sample.position_m
        message.velocity_mps = sample.velocity_mps
        message.homed = sample.homed
        message.running = abs(sample.velocity_mps) > 1.0e-6
        message.fault = sample.fault
        message.calibration_id = calibration_id
        message.evidence_age_sec = 0.0
        publisher.publish(message)


if __name__ == "__main__":
    main()
