"""ROS2 fail-closed mock boundary. Real controller integration is not enabled."""
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from chili_crane_msgs.msg import (
    AuthorizedCommand, ActuationRequest, SystemModeStatus, SystemReadiness, SafetyEvent,
    RemoteControlState,
)
from .contracts import SystemMode
from .execution import ControlAuthority
from .qos import state_qos, command_qos


class ActionExecutorNode(Node):
    def __init__(self):
        super().__init__("action_executor")
        self.declare_parameter("capability_config", "")
        path = self.get_parameter("capability_config").value
        self._config = yaml.safe_load(Path(path).read_text(encoding="utf-8")) if path else {}
        self.authority = ControlAuthority()
        self.authority.transition(SystemMode.PROTECTIVE_STOP)
        self._actuation = self.create_publisher(
            ActuationRequest, "control/actuation_request", command_qos())
        self._mode_pub = self.create_publisher(SystemModeStatus, "system/mode", state_qos())
        self._ready_pub = self.create_publisher(SystemReadiness, "system/readiness", state_qos())
        self._event_pub = self.create_publisher(SafetyEvent, "safety/event", state_qos())
        self._command_sub = self.create_subscription(
            AuthorizedCommand, "control/authorized_command", self._command, command_qos())
        self._remote_sub = self.create_subscription(
            RemoteControlState, "hardware/remote_control_state", self._remote, state_qos())
        self._event_first_seen = self.get_clock().now().to_msg()
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(0.1, self._publish, clock=self._steady)

    def _remote(self, message):
        # No configured readiness can make AUTO_READY in the current mock runtime.
        fresh = message.validity == RemoteControlState.VALID and 0 <= message.evidence_age_sec <= 1.0
        mode = (SystemMode.REMOTE if fresh and message.control_mode == "REMOTE"
                else SystemMode.PROTECTIVE_STOP)
        self.authority.transition(mode)
        self._release(reason="REMOTE_OR_READINESS_BLOCK")

    def _release(self, command=None, reason="PHASE05_NOT_READY"):
        message = ActuationRequest()
        message.header.stamp = self.get_clock().now().to_msg()
        message.validity = ActuationRequest.NOT_CONFIGURED
        message.reason = reason
        message.axis = "ALL"
        message.direction = 0
        message.enable = False
        message.speed_command_valid = False
        message.session_id = self.authority.session_id
        message.command_epoch = self.authority.command_epoch
        message.issued_stamp = message.header.stamp
        message.expire_stamp = message.header.stamp
        if command is not None:
            message.source_command_id = command.command_id
            message.intent_id = command.intent_id
            message.run_id = command.run_id
            message.task_id = command.task_id
            message.cycle_id = command.cycle_id
            message.sequence = command.sequence
        message.evidence.validity = message.evidence.NOT_CONFIGURED
        message.evidence.reason = "NO_PHYSICAL_EVIDENCE"
        self._actuation.publish(message)

    def _command(self, command):
        # Typed pure executors are unit-tested; all live readiness inputs are unknown.
        self._release(command, "AUTHORIZATION_OR_EXECUTION_NOT_READY")

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        mode = SystemModeStatus()
        mode.header.stamp = stamp
        mode.validity = SystemModeStatus.NOT_CONFIGURED
        mode.reason = "PHASE05_MOCK_ONLY"
        mode.mode = int(self.authority.mode)
        mode.session_id = self.authority.session_id
        mode.command_epoch = self.authority.command_epoch
        mode.release_automatic_outputs = True
        mode.discard_pending_commands = True
        mode.require_new_task = True
        self._mode_pub.publish(mode)
        ready = SystemReadiness()
        ready.header.stamp = stamp
        ready.validity = SystemReadiness.NOT_CONFIGURED
        ready.reason = "LIVE_READINESS_NOT_CONFIGURED"
        ready.blocking_reasons = ["HARDWARE_NOT_CONFIGURED", "ALGORITHMS_NOT_IMPLEMENTED"]
        ready.evidence.validity = ready.evidence.NOT_CONFIGURED
        self._ready_pub.publish(ready)
        event = SafetyEvent()
        event.header.stamp = stamp
        event.validity = SafetyEvent.NOT_CONFIGURED
        event.reason = "NO_AUTOMATIC_MOTION"
        event.domain = SafetyEvent.PROTECTIVE_STOP
        event.severity = SafetyEvent.WARNING
        event.source = "action_executor"
        event.code = "READINESS_NOT_CONFIGURED"
        event.active = True
        event.first_seen = self._event_first_seen
        event.last_seen = stamp
        self._event_pub.publish(event)
        self._release()


def main():
    rclpy.init()
    node = ActionExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
