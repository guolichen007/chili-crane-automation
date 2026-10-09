#!/usr/bin/env python3
"""Phase 0 denies every intent; no physical output."""
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from chili_crane_control.qos import command_qos, state_qos
from chili_crane_msgs.msg import AuthorizedCommand, ControlIntent, SafetyPermit


class FailSafeSafetySupervisor(Node):
    def __init__(self):
        super().__init__("safety_supervisor")
        self._publisher = self.create_publisher(SafetyPermit, "safety/permit", state_qos())
        self._command_pub = self.create_publisher(
            AuthorizedCommand, "control/authorized_command", command_qos())
        self._intent_sub = self.create_subscription(
            ControlIntent, "control/requested_intent", self._reject_intent, command_qos())
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(0.1, self._publish, clock=self._steady_clock)

    def _publish(self):
        permit = SafetyPermit()
        permit.header.stamp = self.get_clock().now().to_msg()
        permit.validity = SafetyPermit.NOT_CONFIGURED
        permit.reason = "phase0_safety_policy_not_implemented"
        permit.permit_id = "phase0-fail-closed"
        permit.permit_generation = 0
        permit.evaluated_intent_id = ""
        permit.issued_stamp = permit.header.stamp
        permit.expire_stamp = permit.header.stamp
        permit.allow_x_positive = False
        permit.allow_x_negative = False
        permit.allow_y_positive = False
        permit.allow_y_negative = False
        permit.allow_lower = False
        permit.allow_raise = False
        permit.allow_grab_open = False
        permit.allow_grab_close = False
        permit.allow_unload = False
        permit.allow_auto_task = False
        permit.blocking_reasons = ["SAFETY_POLICY_NOT_IMPLEMENTED", "HARDWARE_NOT_CONFIGURED"]
        permit.evidence_age_sec = 1.0e9
        self._publisher.publish(permit)

    def _reject_intent(self, intent):
        stamp = self.get_clock().now().to_msg()
        command = AuthorizedCommand()
        command.header.stamp = stamp
        command.validity = AuthorizedCommand.NOT_CONFIGURED
        command.reason = "phase0_authorization_policy_not_implemented"
        command.command_id = "rejected-" + (intent.intent_id or "missing-id")
        command.task_id = intent.task_id
        command.intent_id = intent.intent_id
        command.permit_id = "phase0-fail-closed"
        command.permit_generation = 0
        command.issued_stamp = stamp
        command.expire_stamp = stamp
        command.action = intent.action
        command.target_x_m = intent.target_x_m
        command.target_y_m = intent.target_y_m
        command.target_grab_bottom_z_m = intent.target_grab_bottom_z_m
        command.speed_scale = intent.speed_scale
        self._command_pub.publish(command)


def main():
    rclpy.init()
    node = FailSafeSafetySupervisor()
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
