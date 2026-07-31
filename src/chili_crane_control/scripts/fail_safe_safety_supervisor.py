#!/usr/bin/env python3
"""Phase 0 safety publisher.

This node deliberately grants no permission. It makes the safety topic and
fail-closed integration behavior available before a real evidence policy is
implemented.
"""

import rospy

from chili_crane_msgs.msg import SafetyPermit


class FailSafeSafetySupervisor:
    def __init__(self) -> None:
        self._publisher = rospy.Publisher(
            "safety/permit", SafetyPermit, queue_size=1, latch=True
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
        permit.allow_x_move = False
        permit.allow_y_move = False
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


def main() -> None:
    rospy.init_node("fail_safe_safety_supervisor")
    FailSafeSafetySupervisor()
    rospy.logwarn(
        "Phase 0 safety supervisor is fail-closed; no automatic action is permitted."
    )
    rospy.spin()


if __name__ == "__main__":
    main()
