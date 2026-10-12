#!/usr/bin/env python3
"""Optional Linux AF_PACKET observer: no UDP bind conflict, no writes, no payload log."""
from pathlib import Path
import socket
import time
import uuid
import yaml
import rclpy
from rclpy.node import Node
from chili_crane_control.qos import state_qos
from chili_crane_msgs.msg import Rse1ClockStatus
from chili_crane_hardware.er1_time_status import RuntimeClockTracker, udp_datagram, clock_fields


class Er1ClockObserver(Node):
    def __init__(self):
        super().__init__("er1_clock_observer")
        self.declare_parameter("site_config", "")
        site = Path(self.get_parameter("site_config").value)
        network = yaml.safe_load((site / "network/interfaces.yaml").read_text())["lidar"]
        self.configs = {s: yaml.safe_load((site / ("sensors/er1_" + s + ".yaml")).read_text()) for s in ("204", "205")}
        self.trackers = {s: RuntimeClockTracker(c.get("runtime_status_timeout_sec")) for s, c in self.configs.items()}
        self.pubs = {s: self.create_publisher(Rse1ClockStatus, "lidar/er1_" + s + "/clock_status", state_qos()) for s in self.configs}
        self.received = {}
        self.boot_id = str(uuid.uuid4())
        self.capture = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0003))
        self.capture.bind((network["interface"], 0))
        self.capture.setblocking(False)
        self.poll_timer = self.create_timer(.005, self.poll)
        self.status_timer = self.create_timer(.1, self.publish_all)

    def poll(self):
        for _ in range(256):  # bounded work per callback
            try:
                item = udp_datagram(self.capture.recv(65535))
            except BlockingIOError:
                break
            if item is None:
                continue
            source, destination, port, payload = item
            fields = clock_fields(payload)
            if fields is None:
                continue
            for sensor, cfg in self.configs.items():
                expected_port = cfg.get("difop_port" if fields["kind"] == "DIFOP" else "msop_port")
                if (source != cfg["host"] or destination != cfg.get("destination_address")
                        or cfg.get("port_roles") != "VALID" or port != expected_port):
                    continue
                previous = dict(self.trackers[sensor].fields)
                if self.trackers[sensor].observe(payload, time.monotonic()):
                    self.received[(sensor, fields["kind"])] = self.get_clock().now().to_msg()
                    if previous != self.trackers[sensor].fields:
                        self.publish(sensor)  # status change, including failure, without waiting for timer

    def publish(self, sensor):
        state = self.trackers[sensor].state(time.monotonic())
        msg = Rse1ClockStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.configs[sensor]["frame_id"]
        msg.sensor_id, msg.source_type = "er1_" + sensor, 1  # PHYSICAL, never synthetic inference
        msg.clock_state, msg.scan_phase_state = state["clock_state"], state["scan_phase_state"]
        for key in ("timesync_mode", "timesync_status", "time_mode", "frame_sync"):
            setattr(msg, key, state.get(key, 255))
        msg.clock_sync_valid, msg.scan_phase_sync_valid = state["clock_sync_valid"], state["scan_phase_sync_valid"]
        msg.observation_sequence = state["observation_sequence"]
        msg.observer_boot_id = self.boot_id
        msg.validity = msg.VALID if msg.clock_sync_valid else msg.DEGRADED
        msg.reason = msg.clock_state
        for kind, key in (("DIFOP", "difop_received_at"), ("MSOP", "msop_received_at")):
            if (sensor, kind) in self.received:
                setattr(msg, key, self.received[(sensor, kind)])
        msg.evidence.source_type, msg.evidence.validity, msg.evidence.reason = msg.source_type, msg.validity, msg.reason
        msg.evidence.measurement_stamp = msg.difop_received_at
        msg.evidence.receive_stamp = msg.msop_received_at
        msg.evidence.source_counter = msg.observation_sequence
        self.pubs[sensor].publish(msg)

    def publish_all(self):
        for sensor in self.configs:
            self.publish(sensor)

    def destroy_node(self):
        self.capture.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = Er1ClockObserver()  # CAP_NET_RAW must be granted by owner; never auto-escalate.
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
