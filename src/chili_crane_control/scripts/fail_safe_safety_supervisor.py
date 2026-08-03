#!/usr/bin/env python3
"""Phase 0 safety publisher.

This node deliberately grants no permission. It makes the safety topic and
fail-closed integration behavior available before a real evidence policy is
implemented.
"""

import rospy

from chili_crane_msgs.msg import AuthorizedCommand, ControlIntent, SafetyPermit


class FailSafeSafetySupervisor:
    def __init__(self) -> None:
        self._publisher = rospy.Publisher(
            "safety/permit", SafetyPermit, queue_size=1, latch=True
        )
        self._authorized_command_publisher = rospy.Publisher(
            "control/authorized_command",
            AuthorizedCommand,
            queue_size=10,
        )
        self._intent_subscriber = rospy.Subscriber(
            "control/requested_intent",
            ControlIntent,
            self._reject_intent,
            queue_size=10,
        )
        publish_rate_hz = max(
            1.0, float(rospy.get_param("~publish_rate_hz", 10.0))
        )
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / publish_rate_hz), self._publish
        )

    def _publish(self, _event: rospy.timer.TimerEvent) -> None:
        permit = SafetyPermit()
        permit.header.stamp = rospy.Time.now()
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
        permit.blocking_reasons = [
            "SAFETY_POLICY_NOT_IMPLEMENTED",
            "HARDWARE_NOT_CONFIGURED",
        ]
        permit.evidence_age_sec = 1.0e9
        self._publisher.publish(permit)

    def _reject_intent(self, intent: ControlIntent) -> None:
        """Publish an explicitly invalid command for integration visibility."""
        stamp = rospy.Time.now()
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
        self._authorized_command_publisher.publish(command)
        rospy.logwarn_throttle(
            2.0,
            "Rejected requested intent_id=%s; Phase 0 grants no authorization.",
            intent.intent_id,
        )


def main() -> None:
    rospy.init_node("fail_safe_safety_supervisor")
    FailSafeSafetySupervisor()
    rospy.logwarn(
        "Phase 0 safety supervisor is fail-closed; no automatic action is permitted."
    )
    rospy.spin()


if __name__ == "__main__":
    main()
