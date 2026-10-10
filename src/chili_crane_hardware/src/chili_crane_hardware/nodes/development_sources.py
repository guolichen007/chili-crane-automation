"""Explicit simulation topics; never a subscriber to physical actuation."""
from pathlib import Path
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from chili_crane_control.qos import state_qos
from chili_crane_hardware.evidence import fill_evidence, SourceType
from chili_crane_msgs.msg import ServoState, SimulatedIoObservations


class DevelopmentSources(Node):
    def __init__(self):
        super().__init__("development_sources")
        self.declare_parameter("simulation_config", "")
        config = yaml.safe_load(Path(self.get_parameter("simulation_config").value).read_text())
        self._x, self._signals = config["x_fixed_m"], config["signals"]
        if (type(self._x) not in (float, int) or not math.isfinite(self._x)
                or any(type(v) is not bool for v in self._signals.values())):
            raise ValueError("simulation config must be explicit and finite")
        self._counter = 0
        self._xpub = self.create_publisher(ServoState, "sim/servo_state", state_qos())
        self._iopub = self.create_publisher(SimulatedIoObservations, "sim/io_observations", state_qos())
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(.1, self._publish, clock=self._steady)

    def _publish(self):
        self._counter += 1
        stamp = self.get_clock().now().to_msg()
        servo = ServoState()
        servo.header.stamp = stamp
        servo.validity = ServoState.VALID
        servo.reason = "SIMULATED_FIXED_X_NO_AUTO_PERMISSION"
        servo.position_m = float(self._x)
        servo.source_counter = self._counter
        servo.calibration_id = "simulated-fixed-x"
        fill_evidence(servo, SourceType.SIMULATED, stamp, stamp, self._counter, 0,
                      servo.calibration_id, "development-scenario-1")
        self._xpub.publish(servo)
        io = SimulatedIoObservations()
        io.header.stamp = stamp
        io.validity = io.VALID
        io.reason = "SIMULATED_DEVELOPMENT_SCENARIO"
        io.signal_names = sorted(self._signals)
        io.signal_values = [self._signals[n] for n in io.signal_names]
        io.evidence.source_type = io.evidence.SOURCE_SIMULATED
        io.evidence.validity = io.evidence.VALID
        io.evidence.measurement_stamp = stamp
        io.evidence.receive_stamp = stamp
        io.evidence.source_counter = self._counter
        io.evidence.config_version = "development-scenario-1"
        self._iopub.publish(io)


def main():
    rclpy.init()
    node = DevelopmentSources()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
