#!/usr/bin/env python3
"""Publish manifest periodically so VOLATILE bag subscribers receive provenance."""
from pathlib import Path
import yaml
import json
import rclpy
from rclpy.node import Node
from chili_crane_control.qos import state_qos
from chili_crane_msgs.msg import RunManifest, CalibrationRef


class ManifestNode(Node):
    def __init__(self):
        super().__init__("run_manifest")
        self.declare_parameter("manifest_file", "")
        self._data = yaml.safe_load(Path(self.get_parameter("manifest_file").value).read_text())
        self._publisher = self.create_publisher(RunManifest, "system/run_manifest", state_qos())
        self._timer = self.create_timer(1, self._publish)

    def _publish(self):
        msg = RunManifest()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.validity = msg.DEGRADED
        msg.reason = "PHASE1" + self._data.get("phase", "A").removeprefix("1") + "_CAPTURE_NOT_PRODUCTION_ACCEPTANCE"
        for field in ("run_id", "git_sha", "config_hash"):
            setattr(msg, field, self._data[field])
        msg.timebase_json = json.dumps(self._data.get("timebase", {}), allow_nan=False)
        msg.required_sensor_ids = ["er1_204", "er1_205", "hik_01" if self._data.get("phase") == "1B" else "pull_wire_y"]
        for sensor, data in self._data["calibrations"].items():
            ref = CalibrationRef()
            ref.sensor_id = sensor
            ref.calibration_id = data.get("calibration_id", "NOT_CONFIGURED")
            ref.extrinsic_version = data.get("calibration_id", "NOT_CONFIGURED")
            ref.intrinsic_version = "NOT_CONFIGURED"
            msg.calibrations.append(ref)
        msg.evidence.source_type = getattr(msg.evidence, "SOURCE_" + self._data["source_type"])
        self._publisher.publish(msg)


def main():
    rclpy.init()
    node = ManifestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
