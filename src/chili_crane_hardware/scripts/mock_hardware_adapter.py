#!/usr/bin/env python3
"""Unconfigured hardware mock rejects all actions including valid commands."""
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from chili_crane_control.qos import command_qos, state_qos
from chili_crane_msgs.msg import (
    AuthorizedCommand, CommandExecutionState, ControlBoardState,
    GrabIoState, HoistState, LoadState, ServoState, TrolleyState,
)


def stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


class MockHardwareAdapter(Node):
    def __init__(self):
        super().__init__("mock_hardware_adapter")
        self.declare_parameter("crane_base_frame", "crane_01/base")
        self._frame = self.get_parameter("crane_base_frame").value
        self._counter = 0
        self._types = {
            "servo_state": ServoState, "trolley_state": TrolleyState,
            "hoist_state": HoistState, "load_state": LoadState,
            "grab_io_state": GrabIoState, "control_board_state": ControlBoardState,
        }
        self._publishers_by_topic = {
            topic: self.create_publisher(kind, "hardware/" + topic, state_qos())
            for topic, kind in self._types.items()
        }
        self._execution_pub = self.create_publisher(
            CommandExecutionState, "control/execution_state", state_qos(depth=10))
        self._command_sub = self.create_subscription(
            AuthorizedCommand, "control/authorized_command",
            self._on_authorized_command, command_qos())
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(0.1, self._publish, clock=self._steady_clock)

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        self._counter += 1
        for topic, kind in self._types.items():
            message = kind()
            message.header.stamp = stamp
            message.header.frame_id = self._frame
            message.validity = kind.NOT_CONFIGURED
            message.reason = "mock_hardware_not_configured"
            message.evidence_age_sec = 1.0e9
            if hasattr(message, "calibration_id"):
                message.calibration_id = "NOT_CONFIGURED"
            if topic == "servo_state":
                message.source_counter = self._counter
                message.communication_ok = False
            elif topic == "hoist_state":
                message.motion = HoistState.MOTION_UNKNOWN
            elif topic == "load_state":
                message.load_state = LoadState.LOAD_UNKNOWN
            elif topic == "control_board_state":
                message.safety_ok = False
                message.safety_ok_known = False
                message.e_stop_known = False
                message.auto_mode = False
                message.io_heartbeat_ok = False
            self._publishers_by_topic[topic].publish(message)

    def _on_authorized_command(self, command):
        now = self.get_clock().now().nanoseconds
        issued = stamp_ns(command.issued_stamp)
        expires = stamp_ns(command.expire_stamp)
        execution = CommandExecutionState()
        execution.header.stamp = self.get_clock().now().to_msg()
        execution.validity = CommandExecutionState.VALID
        execution.command_id = command.command_id
        execution.task_id = command.task_id
        execution.intent_id = command.intent_id
        execution.state = CommandExecutionState.STATE_REJECTED
        execution.accepted = False
        execution.executing = False
        execution.completed = False
        execution.failed = True
        if command.validity != AuthorizedCommand.VALID:
            execution.reason = "authorized_command_not_valid"
        elif not all((command.command_id, command.intent_id, command.permit_id)):
            execution.reason = "authorized_command_identity_missing"
        elif issued <= 0 or expires <= issued or now < issued or now >= expires:
            execution.reason = "authorized_command_time_invalid"
        else:
            execution.reason = "mock_adapter_has_no_physical_output"
        evidence = stamp_ns(command.header.stamp)
        execution.evidence_age_sec = (
            (now - evidence) / 1.0e9 if 0 < evidence <= now else 1.0e9)
        self._execution_pub.publish(execution)


def main():
    rclpy.init()
    node = MockHardwareAdapter()
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
