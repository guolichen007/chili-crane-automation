"""Read-only ADAM nodes publish all 8/16 raw channels and health."""
import math
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from chili_crane_control.qos import state_qos
from chili_crane_hardware.adam.adam6052_driver import Adam6052Driver
from chili_crane_hardware.adam.adam6251_driver import Adam6251Driver
from chili_crane_hardware.adam.modbus_tcp_transport import ModbusTcpTransport, ModbusError
from chili_crane_hardware.state import EvidenceTracker
from chili_crane_msgs.msg import DigitalInputState


class AdamNode(Node):
    def __init__(self, device):
        super().__init__(device)
        self.declare_parameter("device_config", "")
        path = self.get_parameter("device_config").value
        config = yaml.safe_load(Path(path).read_text(encoding="utf-8")) if path else {}
        self._device = device
        self._tracker = EvidenceTracker()
        self._driver = None
        self._stale = float(config.get("stale_timeout_sec", 1.0))
        rate = float(config.get("poll_rate_hz", 5.0))
        if not math.isfinite(self._stale) or not 0 < self._stale <= 10:
            raise ValueError("invalid stale_timeout_sec")
        if not math.isfinite(rate) or not 0.1 <= rate <= 50:
            raise ValueError("invalid poll_rate_hz")
        try:
            transport = ModbusTcpTransport(
                config.get("host"), config.get("port", 502),
                config.get("unit_id"), config.get("timeout_sec", 0.5))
            driver_type = Adam6052Driver if device == "adam6052" else Adam6251Driver
            self._driver = driver_type(transport)
        except (ValueError, TypeError) as exc:
            self.get_logger().warning("NOT_CONFIGURED: " + str(exc))
        self._publisher = self.create_publisher(
            DigitalInputState, "hardware/" + device + "/raw_di", state_qos())
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(1.0 / rate, self._poll, clock=self._steady_clock)

    def _poll(self):
        if self._driver is not None:
            try:
                self._tracker.accept(self._driver.read_di())
            except (ModbusError, OSError, RuntimeError) as exc:
                self._tracker.failed(exc)
        sample = self._tracker.sample
        message = DigitalInputState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.device_id = self._device
        message.validity = getattr(DigitalInputState, self._tracker.validity(self._stale))
        message.reason = self._tracker.reason
        message.raw_di = list(sample.value) if sample else []
        message.communication_ok = self._tracker.communication_ok
        message.source_counter = sample.source_counter if sample else 0
        message.evidence_age_sec = sample.age() if sample else 1.0e9
        self._publisher.publish(message)


def main(device):
    rclpy.init()
    node = AdamNode(device)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
