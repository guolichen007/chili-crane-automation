#!/usr/bin/env python3
"""Actual ROS2 mock evidence collector; import/run only after an Ubuntu build."""
import argparse
import json
import time

PERMISSIONS = (
    "allow_x_positive", "allow_x_negative", "allow_y_positive", "allow_y_negative",
    "allow_lower", "allow_raise", "allow_grab_open", "allow_grab_close",
    "allow_unload", "allow_auto_task",
)


def main():
    import rclpy
    from rclpy.node import Node
    from chili_crane_control.qos import state_qos, command_qos
    from chili_crane_msgs.msg import (
        SafetyPermit, ServoState, TrolleyState, HoistState, LoadState,
        GrabIoState, ControlBoardState, ControlIntent, AuthorizedCommand, CommandExecutionState,
    )
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="crane_01")
    parser.add_argument("--timeout-sec", type=float, default=25)
    args = parser.parse_args()
    rclpy.init()
    node = Node("phase0_mock_evidence", namespace=args.namespace)
    received = {}
    failure = []
    subscriptions = []

    def permit_callback(msg):
        if msg.validity != SafetyPermit.NOT_CONFIGURED or any(
                getattr(msg, name) is not False for name in PERMISSIONS):
            failure.append("permit must be NOT_CONFIGURED with every permission false")
        received["permit"] = msg

    subscriptions.append(node.create_subscription(SafetyPermit, "safety/permit", permit_callback, state_qos()))
    for topic, kind in {
        "servo_state": ServoState, "trolley_state": TrolleyState,
        "hoist_state": HoistState, "load_state": LoadState,
        "grab_io_state": GrabIoState, "control_board_state": ControlBoardState,
    }.items():
        def hardware_callback(msg, topic=topic, kind=kind):
            if msg.validity != kind.NOT_CONFIGURED:
                failure.append(topic + " is not NOT_CONFIGURED")
            if topic == "control_board_state" and (msg.safety_ok or msg.safety_ok_known or msg.e_stop_known):
                failure.append("mock board must not manufacture safety/estop evidence")
            received[topic] = msg
        subscriptions.append(node.create_subscription(
            kind, "hardware/" + topic, hardware_callback, state_qos()))

    def command_callback(msg):
        if msg.intent_id == "validation-intent":
            if msg.validity != AuthorizedCommand.NOT_CONFIGURED:
                failure.append("supervisor emitted valid authorization")
            received["denied_command"] = msg

    def execution_callback(msg):
        if msg.intent_id == "validation-intent":
            if msg.accepted or msg.executing or msg.completed or not msg.failed:
                failure.append("mock adapter accepted or executed a command")
            if msg.state != CommandExecutionState.STATE_REJECTED:
                failure.append("mock command not rejected")
            received["rejected_execution"] = msg

    subscriptions.append(node.create_subscription(
        AuthorizedCommand, "control/authorized_command", command_callback, command_qos()))
    subscriptions.append(node.create_subscription(
        CommandExecutionState, "control/execution_state", execution_callback, state_qos(depth=10)))
    publisher = node.create_publisher(ControlIntent, "control/requested_intent", command_qos())
    required = {"permit", "servo_state", "trolley_state", "hoist_state", "load_state",
                "grab_io_state", "control_board_state", "denied_command", "rejected_execution"}
    started, next_publish = time.monotonic(), 0.0
    try:
        while time.monotonic() - started < args.timeout_sec:
            now = time.monotonic()
            if now >= next_publish:
                intent = ControlIntent()
                intent.header.stamp = node.get_clock().now().to_msg()
                intent.intent_id = "validation-intent"
                intent.task_id = "validation-mock-only"
                intent.action = ControlIntent.ACTION_STOP_ALL
                intent.validity = ControlIntent.VALID
                publisher.publish(intent)
                next_publish = now + 0.5
            rclpy.spin_once(node, timeout_sec=0.1)
            if failure:
                raise RuntimeError("; ".join(failure))
            if required.issubset(received) and time.monotonic() - started >= 3:
                print(json.dumps({"MOCK_FAIL_CLOSED_STATUS": "PASS", "topics": sorted(received)}))
                return
        raise RuntimeError("missing mock evidence: " + ", ".join(sorted(required - set(received))))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
