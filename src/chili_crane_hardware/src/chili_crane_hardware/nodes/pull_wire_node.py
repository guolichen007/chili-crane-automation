"""Y position wrapper; limits remain separate DI evidence."""
import math
import time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from chili_crane_control.qos import state_qos
from chili_crane_hardware.trolley.pull_wire_driver import PullWireDriver
from chili_crane_hardware.state import EvidenceTracker
from chili_crane_hardware.evidence import fill_evidence, SourceType
from chili_crane_msgs.msg import TrolleyState, DigitalInputState
from chili_crane_hardware.mapping import RawInput, normalize, derive_logical_inputs


class PullWireNode(Node):
    def __init__(self):
        super().__init__("pull_wire")
        self.declare_parameter("device_config", "")
        self.declare_parameter("mapping_config", "")
        path = self.get_parameter("device_config").value
        config = yaml.safe_load(Path(path).read_text(encoding="utf-8")) if path else {}
        map_path = self.get_parameter("mapping_config").value
        mapping_config = yaml.safe_load(Path(map_path).read_text(encoding="utf-8")) if map_path else {}
        self._mapping = derive_logical_inputs(mapping_config)
        self._di_stale = float(mapping_config.get("stale_timeout_sec", 1.0))
        self._di_samples = {}
        self._subscriptions_by_device = [
            self.create_subscription(
                DigitalInputState, "hardware/" + device + "/raw_di",
                lambda msg, device=device: self._receive(device, msg), state_qos())
            for device in ("adam6052", "adam6251")]
        self._driver = None
        self._tracker = EvidenceTracker()
        self._measurement_stamp = self.get_clock().now().to_msg()
        self._stale = float(config.get("stale_timeout_sec", 1.0))
        rate = float(config.get("poll_rate_hz", 5.0))
        if not (all(math.isfinite(x) and 0 < x <= 10 for x in (self._stale, self._di_stale))
                and math.isfinite(rate) and 0.1 <= rate <= 50):
            raise ValueError("invalid polling/freshness configuration")
        try:
            self._driver = PullWireDriver.from_config(config)
        except (KeyError, ValueError, TypeError) as exc:
            self.get_logger().warning("NOT_CONFIGURED: " + str(exc))
        self._publisher = self.create_publisher(TrolleyState, "hardware/trolley_state", state_qos())
        self._previous = None
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(1.0 / rate, self._poll, clock=self._steady_clock)

    def _receive(self, device, message):
        self._di_samples[device] = RawInput(
            tuple(message.raw_di), time.monotonic(), message.evidence_age_sec,
            message.device_id == device and message.validity == DigitalInputState.VALID
            and message.communication_ok)

    def _poll(self):
        velocity = 0.0
        if self._driver is not None:
            try:
                raw, position = self._driver.read()
                observed = time.monotonic()
                if self._previous is not None:
                    previous_pos, previous_time = self._previous
                    velocity = (position - previous_pos) / (observed - previous_time)
                self._previous = (position, observed)
                self._tracker.accept((raw, position, velocity), observed)
                self._measurement_stamp = self.get_clock().now().to_msg()
            except (OSError, ValueError, RuntimeError) as exc:
                self._tracker.failed(exc)
                self._previous = None
        sample = self._tracker.sample
        msg = TrolleyState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.validity = getattr(TrolleyState, self._tracker.validity(self._stale))
        msg.reason = self._tracker.reason
        msg.position_m = float(sample.value[1]) if sample else 0.0
        msg.velocity_mps = float(sample.value[2]) if sample else 0.0
        msg.running = abs(msg.velocity_mps) > 1.0e-6 and msg.validity == TrolleyState.VALID
        msg.calibration_id = self._driver.calibration.calibration_id if self._driver else "NOT_CONFIGURED"
        msg.evidence_age_sec = sample.age() if sample else 1.0e9
        values = normalize(self._mapping, self._di_samples, time.monotonic(), self._di_stale)
        msg.left_limit = values.get("y_left_limit") is True
        msg.right_limit = values.get("y_right_limit") is True
        msg.fault = values.get("y_fault") is True
        names = ("y_left_limit", "y_right_limit", "y_fault")
        msg.known_signals = [name for name in names if values.get(name) is not None]
        msg.unknown_signals = [name for name in names if values.get(name) is None]
        msg.auto_evidence_ready = (not msg.unknown_signals and self._driver is not None
                                   and self._driver.calibration.production_ready())
        fill_evidence(msg, SourceType.PHYSICAL, self._measurement_stamp, self._measurement_stamp,
                      sample.source_counter if sample else 0, msg.evidence_age_sec,
                      msg.calibration_id, "site-crane01-20261010")
        self._publisher.publish(msg)


def main():
    rclpy.init()
    node = PullWireNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
