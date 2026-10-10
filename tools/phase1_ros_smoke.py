#!/usr/bin/env python3
"""Observe ROS topics; never send control requests or infer actuator state."""
import argparse
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from chili_crane_slam.pointcloud import normalize_cloud


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["raw", "merged"], required=True)
    parser.add_argument("--timeout", type=float, default=15)
    args = parser.parse_args()
    if not 1 <= args.timeout <= 120:
        parser.error("bounded timeout required")
    rclpy.init()
    node = Node("phase1_readonly_observer")
    topics = (["/crane_01/lidar/merged_points"] if args.mode == "merged" else
              ["/crane_01/lidar/er1_204/points", "/crane_01/lidar/er1_205/points"])
    counters, errors, last_stamps = dict.fromkeys(topics, 0), [], dict.fromkeys(topics, 0)
    def receive(topic, msg):
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            now = node.get_clock().now().nanoseconds / 1e9
            if not 0 <= now - stamp <= 1 or stamp <= last_stamps[topic] or not msg.header.frame_id:
                raise ValueError("stale/future/repeated frame")
            normalize_cloud(msg.data, [(f.name,f.offset,f.datatype,f.count) for f in msg.fields],
                            msg.point_step,msg.row_step,msg.width,msg.height,msg.is_bigendian)
            last_stamps[topic] = stamp
            counters[topic] += 1
        except ValueError as exc:
            errors.append(str(exc))
    subscriptions = [node.create_subscription(PointCloud2, topic,
                     lambda msg, topic=topic: receive(topic, msg), qos_profile_sensor_data) for topic in topics]
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.timeout and not all(n >= 3 for n in counters.values()):
            rclpy.spin_once(node, timeout_sec=.1)
        passed = all(n >= 3 for n in counters.values()) and not errors
        print(json.dumps({"scope": "ROS_READONLY_OBSERVATION", "no_do_write": True,
                          "counters": counters, "errors": errors, "result": "PASS" if passed else "FAIL"}))
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
