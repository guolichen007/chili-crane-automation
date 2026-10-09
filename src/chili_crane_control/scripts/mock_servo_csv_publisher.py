#!/usr/bin/env python3
"""CSV replay is scheduled with monotonic time, never ROS /clock."""
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from chili_crane_control.qos import state_qos
from chili_crane_control.servo_csv import (
    classify_sample, evidence_age_seconds, load_samples,
    mapped_sample_time, scheduled_monotonic_time,
)
from chili_crane_msgs.msg import ServoState


class ServoCsvPublisher(Node):
    def __init__(self):
        super().__init__("mock_servo_csv_publisher")
        for name, value in {
            "csv_file": "", "frame_id": "crane_01/base",
            "calibration_id": "NOT_CONFIGURED", "stamp_policy": "mapped",
        }.items():
            self.declare_parameter(name, value)
        csv_file = self.get_parameter("csv_file").value
        if not csv_file:
            raise ValueError("csv_file is required")
        self._samples = load_samples(Path(csv_file))
        self._policy = self.get_parameter("stamp_policy").value
        if self._policy not in {"source", "mapped"}:
            raise ValueError("stamp_policy must be source or mapped")
        self._publisher = self.create_publisher(
            ServoState, "hardware/servo_state", state_qos(depth=10))
        self._wall = time.monotonic()
        self._source = self._samples[0].stamp_sec
        self._ros = self.get_clock().now().nanoseconds / 1.0e9
        self._index = 0
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(0.002, self._tick, clock=self._steady_clock)

    def _tick(self):
        if self._index >= len(self._samples):
            self._timer.cancel()
            return
        sample = self._samples[self._index]
        if time.monotonic() < scheduled_monotonic_time(self._wall, self._source, sample.stamp_sec):
            return
        evidence = sample.stamp_sec if self._policy == "source" else mapped_sample_time(
            self._ros, self._source, sample.stamp_sec)
        now = self.get_clock().now().nanoseconds / 1.0e9
        message = ServoState()
        message.header.stamp = Time(nanoseconds=int(evidence * 1.0e9)).to_msg()
        message.header.frame_id = self.get_parameter("frame_id").value
        calibration = self.get_parameter("calibration_id").value
        classification = classify_sample(calibration, sample)
        message.validity = getattr(ServoState, classification)
        message.reason = "csv_" + classification.lower()
        message.raw_position = sample.raw_position
        message.position_m = sample.position_m
        message.velocity_mps = sample.velocity_mps
        message.homed = sample.homed
        message.running = abs(sample.velocity_mps) > 1.0e-6
        message.fault = sample.fault
        message.drive_ready = classification == "VALID"
        message.communication_ok = classification == "VALID"
        message.fault_code = -1 if sample.fault else 0
        message.source_counter = self._index + 1
        message.calibration_id = calibration
        if now <= 0.0 or evidence > now:
            message.validity = ServoState.STALE
            message.reason = "csv_ros_clock_unavailable_or_future"
            message.evidence_age_sec = 1.0e9
        else:
            message.evidence_age_sec = evidence_age_seconds(now, evidence)
        self._publisher.publish(message)
        self._index += 1


def main():
    rclpy.init()
    node = ServoCsvPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
