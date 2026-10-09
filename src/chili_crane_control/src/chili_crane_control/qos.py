"""Explicit QoS for state and time-bound commands."""
from rclpy.duration import Duration
from rclpy.qos import (
    QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy,
)


def state_qos(depth=1, ttl_sec=1.0):
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST, depth=depth,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        lifespan=Duration(seconds=ttl_sec),
    )


def command_qos():
    return state_qos(depth=1, ttl_sec=0.5)
