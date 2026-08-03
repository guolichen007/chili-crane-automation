#!/usr/bin/env python3
"""Fail-safe mock adapter for interface development.

The default output is NOT_CONFIGURED. Setting configured_valid_mock=true is an
explicit developer action and still does not create a SafetyPermit.
"""

import rospy

from chili_crane_msgs.msg import (
    AuthorizedCommand,
    CommandExecutionState,
    ControlBoardState,
    GrabIoState,
    HoistState,
    LoadState,
    ServoState,
    TrolleyState,
)


class MockHardwareAdapter:
    def __init__(self) -> None:
        self._configured = bool(
            rospy.get_param("~configured_valid_mock", False)
        )
        self._frame_id = str(
            rospy.get_param("~crane_base_frame", "crane_01/base")
        )
        self._servo_position_m = float(
            rospy.get_param("~servo_position_m", 0.0)
        )
        self._trolley_position_m = float(
            rospy.get_param("~trolley_position_m", 0.0)
        )
        self._source_counter = 0
        self._servo_pub = rospy.Publisher(
            "hardware/servo_state", ServoState, queue_size=1
        )
        self._trolley_pub = rospy.Publisher(
            "hardware/trolley_state", TrolleyState, queue_size=1
        )
        self._hoist_pub = rospy.Publisher(
            "hardware/hoist_state", HoistState, queue_size=1
        )
        self._load_pub = rospy.Publisher(
            "hardware/load_state", LoadState, queue_size=1
        )
        self._grab_io_pub = rospy.Publisher(
            "hardware/grab_io_state", GrabIoState, queue_size=1
        )
        self._board_pub = rospy.Publisher(
            "hardware/control_board_state",
            ControlBoardState,
            queue_size=1,
        )
        self._execution_pub = rospy.Publisher(
            "control/execution_state",
            CommandExecutionState,
            queue_size=10,
        )
        self._command_sub = rospy.Subscriber(
            "control/authorized_command",
            AuthorizedCommand,
            self._on_authorized_command,
            queue_size=10,
        )

        publish_rate_hz = max(
            1.0, float(rospy.get_param("~publish_rate_hz", 10.0))
        )
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / publish_rate_hz), self._publish
        )

    def _validity(self, message_type: type) -> int:
        return (
            message_type.VALID
            if self._configured
            else message_type.NOT_CONFIGURED
        )

    def _reason(self) -> str:
        return (
            "explicit_valid_mock"
            if self._configured
            else "mock_hardware_not_configured"
        )

    def _publish(self, _event: rospy.timer.TimerEvent) -> None:
        stamp = rospy.Time.now()
        self._source_counter += 1

        servo = ServoState()
        servo.header.stamp = stamp
        servo.header.frame_id = self._frame_id
        servo.validity = self._validity(ServoState)
        servo.reason = self._reason()
        servo.position_m = self._servo_position_m
        servo.homed = self._configured
        servo.fault = False
        servo.drive_ready = self._configured
        servo.servo_enabled = False
        servo.positive_limit = False
        servo.negative_limit = False
        servo.communication_ok = self._configured
        servo.fault_code = 0
        servo.source_counter = self._source_counter
        servo.calibration_id = (
            "mock_explicit" if self._configured else "NOT_CONFIGURED"
        )
        servo.evidence_age_sec = (
            0.0 if self._configured else 1.0e9
        )
        self._servo_pub.publish(servo)

        trolley = TrolleyState()
        trolley.header.stamp = stamp
        trolley.header.frame_id = self._frame_id
        trolley.validity = self._validity(TrolleyState)
        trolley.reason = self._reason()
        trolley.position_m = self._trolley_position_m
        trolley.fault = False
        trolley.calibration_id = (
            "mock_explicit" if self._configured else "NOT_CONFIGURED"
        )
        trolley.evidence_age_sec = (
            0.0 if self._configured else 1.0e9
        )
        self._trolley_pub.publish(trolley)

        hoist = HoistState()
        hoist.header.stamp = stamp
        hoist.header.frame_id = self._frame_id
        hoist.validity = self._validity(HoistState)
        hoist.reason = self._reason()
        hoist.motion = HoistState.MOTION_STOPPED
        hoist.brake_engaged = True
        hoist.fault = False
        hoist.evidence_age_sec = (
            0.0 if self._configured else 1.0e9
        )
        self._hoist_pub.publish(hoist)

        load = LoadState()
        load.header.stamp = stamp
        load.header.frame_id = self._frame_id
        load.validity = self._validity(LoadState)
        load.reason = self._reason()
        load.load_state = LoadState.LOAD_UNKNOWN
        load.unit = "mock_unit" if self._configured else ""
        load.calibration_id = (
            "mock_explicit" if self._configured else "NOT_CONFIGURED"
        )
        load.evidence_age_sec = 0.0 if self._configured else 1.0e9
        self._load_pub.publish(load)

        grab_io = GrabIoState()
        grab_io.header.stamp = stamp
        grab_io.header.frame_id = self._frame_id
        grab_io.validity = self._validity(GrabIoState)
        grab_io.reason = self._reason()
        grab_io.open_limit = False
        grab_io.closed_limit = False
        grab_io.running = False
        grab_io.fault = False
        grab_io.manual_mode = not self._configured
        grab_io.auto_mode = self._configured
        grab_io.motor_current_valid = False
        grab_io.evidence_age_sec = (
            0.0 if self._configured else 1.0e9
        )
        self._grab_io_pub.publish(grab_io)

        board = ControlBoardState()
        board.header.stamp = stamp
        board.header.frame_id = self._frame_id
        board.validity = self._validity(ControlBoardState)
        board.reason = self._reason()
        board.e_stop_active = False
        board.power_ok = self._configured
        board.manual_mode = not self._configured
        board.auto_mode = self._configured
        board.io_heartbeat_ok = self._configured
        board.fault = False
        board.evidence_age_sec = 0.0 if self._configured else 1.0e9
        self._board_pub.publish(board)

    def _on_authorized_command(self, command: AuthorizedCommand) -> None:
        stamp = rospy.Time.now()
        execution = CommandExecutionState()
        execution.header.stamp = stamp
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
        elif (
            not command.command_id
            or not command.intent_id
            or not command.permit_id
        ):
            execution.reason = "authorized_command_identity_missing"
        elif command.expire_stamp <= command.issued_stamp:
            execution.reason = "authorized_command_expiry_invalid"
        elif stamp >= command.expire_stamp:
            execution.reason = "authorized_command_expired"
        else:
            execution.reason = "mock_adapter_has_no_physical_output"

        if command.header.stamp.is_zero():
            execution.evidence_age_sec = 1.0e9
        else:
            execution.evidence_age_sec = max(
                0.0, (stamp - command.header.stamp).to_sec()
            )
        self._execution_pub.publish(execution)
        rospy.logwarn_throttle(
            2.0,
            "Mock adapter rejected command_id=%s action=%d: %s",
            command.command_id,
            command.action,
            execution.reason,
        )


def main() -> None:
    rospy.init_node("mock_hardware_adapter")
    MockHardwareAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
