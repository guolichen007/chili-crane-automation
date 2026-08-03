#!/usr/bin/env python3
"""Replay normalized servo samples from CSV for interface development.

Required columns:
stamp_sec, raw_position, position_m, velocity_mps, valid, homed, fault
"""

import time
from pathlib import Path

import rospy

from chili_crane_control.servo_csv import (
    NOT_CONFIGURED,
    VALID,
    classify_sample,
    evidence_age_seconds,
    load_samples,
    mapped_sample_time,
    scheduled_monotonic_time,
)
from chili_crane_msgs.msg import ServoState


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
    stamp_policy = str(rospy.get_param("~stamp_policy", "mapped")).strip()
    if stamp_policy not in {"mapped", "source"}:
        raise RuntimeError("~stamp_policy must be 'mapped' or 'source'")
    publisher = rospy.Publisher(
        "hardware/servo_state", ServoState, queue_size=10
    )
    start_wall = time.monotonic()
    start_sample = samples[0].stamp_sec
    start_ros = rospy.Time.now().to_sec()

    for source_counter, sample in enumerate(samples, start=1):
        if rospy.is_shutdown():
            break
        scheduled_time = scheduled_monotonic_time(
            start_wall, start_sample, sample.stamp_sec
        )
        while not rospy.is_shutdown() and time.monotonic() < scheduled_time:
            time.sleep(0.001)

        if stamp_policy == "source":
            evidence_stamp_sec = sample.stamp_sec
        else:
            evidence_stamp_sec = mapped_sample_time(
                start_ros, start_sample, sample.stamp_sec
            )
        publish_ros_sec = rospy.Time.now().to_sec()
        effective_publish_sec = (
            publish_ros_sec if publish_ros_sec > 0.0 else evidence_stamp_sec
        )

        message = ServoState()
        message.header.stamp = rospy.Time.from_sec(evidence_stamp_sec)
        message.header.frame_id = frame_id
        classification = classify_sample(calibration_id, sample)
        if classification == NOT_CONFIGURED:
            message.validity = ServoState.NOT_CONFIGURED
        elif classification == VALID:
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
        message.drive_ready = classification == VALID
        message.servo_enabled = False
        message.positive_limit = False
        message.negative_limit = False
        message.communication_ok = classification == VALID
        message.fault_code = -1 if sample.fault else 0
        message.source_counter = source_counter
        message.calibration_id = calibration_id
        message.evidence_age_sec = evidence_age_seconds(
            effective_publish_sec, evidence_stamp_sec
        )
        publisher.publish(message)


if __name__ == "__main__":
    main()
