#!/usr/bin/env python3
"""Ubuntu synthetic ROS2 transport test; no physical endpoints or control publishers."""
import json
import tempfile
import time
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from chili_crane_msgs.msg import DualLidarReadiness, DualLidarTimingState
from chili_crane_control.qos import state_qos
from chili_crane_slam.dual_lidar_node import DualLidarNode, cloud_message
from chili_crane_slam.dual_lidar import Cloud
from chili_crane_slam.pointcloud import CANONICAL


def fixture(site, calibrated, timing_only=False, mixed_domain=False):
    for folder in ("sensors", "calibration"):
        (site / folder).mkdir()
    cfg = {"config_state": "VALID", "consume_once": True, "allow_old_frame_reuse": False,
        "single_lidar_fallback": False, "maximum_pair_delta_sec": .02, "stale_timeout_sec": .35,
        "maximum_queue_size": 4, "clocks_synchronized": True, "coverage_verified": True,
        "point_timestamp_header_tolerance_sec": .1, "source_type": "SYNTHETIC",
        "pairing_basis": "MID_SCAN", "timing_only": timing_only,
        "maximum_future_skew_sec": .01, "header_first_point_tolerance_sec": .01,
        "output_frame": "synthetic/base", "products": {}}
    (site / "sensors/dual_lidar.yaml").write_text(yaml.safe_dump(cfg))
    for sensor in ("204", "205"):
        sensor_cfg = {"frame_id": "synthetic/er1_" + sensor,
            "clock_mode": "HOST_DERIVED", "clock_sync_state": "PROVISIONAL", "clock_domain": "synthetic-host",
            "raw_topic": "/phase1a_synthetic/vendor_" + sensor,
            "normalized_topic": "/phase1a_synthetic/raw_" + sensor}
        if mixed_domain and sensor == "205":
            sensor_cfg["clock_domain"] = "synthetic-other-host"
        (site / "sensors" / ("er1_" + sensor + ".yaml")).write_text(yaml.safe_dump(sensor_cfg))
        extrinsic = {"config_state": "VALID" if calibrated else "NOT_CONFIGURED",
            "calibration_id": "synthetic-only", "source_frame": sensor_cfg["frame_id"],
            "target_frame": cfg["output_frame"], "rotation_matrix": np.eye(3).tolist() if calibrated else None,
            "translation_m": [1, 0, 0] if sensor == "204" else [0, 2, 0]}
        (site / "calibration" / ("er1_" + sensor + "_to_base.yaml")).write_text(yaml.safe_dump(extrinsic))


def scenario(calibrated, timing_only=False, mixed_domain=False):
    with tempfile.TemporaryDirectory(prefix="chili-ros-synthetic-") as tmp:
        fixture(Path(tmp), calibrated, timing_only, mixed_domain)
        rclpy.init(args=["--ros-args", "-p", "site_config:=" + tmp, "-r", "__ns:=/phase1a_synthetic"])
        pipeline = observer = executor = None
        try:
            observer = Node("synthetic_observer")
            counts = {"204": 0, "205": 0, "merged": 0}
            latest = {"ready": None}
            transforms = set()
            subscriptions = []
            publishers = {}
            for sensor in ("204", "205"):
                publishers[sensor] = observer.create_publisher(PointCloud2,
                    "/phase1a_synthetic/vendor_" + sensor, qos_profile_sensor_data)
                def raw(msg, sensor=sensor):
                    assert msg.width == 2 and msg.point_step == 32
                    counts[sensor] += 1
                subscriptions.append(observer.create_subscription(PointCloud2,
                    "/phase1a_synthetic/raw_" + sensor, raw, qos_profile_sensor_data))

            def merged(msg):
                assert msg.header.frame_id == "synthetic/base" and msg.width == 4
                points = np.frombuffer(msg.data, dtype=CANONICAL)
                np.testing.assert_allclose(points["x"], [2, 3, 1, 2])
                np.testing.assert_allclose(points["y"], [0, 0, 2, 2])
                np.testing.assert_array_equal(points["sensor_id"], [0, 0, 1, 1])
                counts["merged"] += 1

            subscriptions.append(observer.create_subscription(PointCloud2,
                "/phase1a_synthetic/lidar/merged_points", merged, qos_profile_sensor_data))
            subscriptions.append(observer.create_subscription(DualLidarReadiness,
                "/phase1a_synthetic/lidar/dual_lidar/readiness",
                lambda msg: latest.update(ready=msg), state_qos()))
            subscriptions.append(observer.create_subscription(DualLidarTimingState,
                "/phase1a_synthetic/lidar/dual_lidar/timing",
                lambda msg: latest.update(timing=msg), state_qos()))
            subscriptions.append(observer.create_subscription(TFMessage, "/tf_static",
                lambda msg: transforms.update(t.child_frame_id for t in msg.transforms),
                QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
            pipeline = DualLidarNode()
            executor = SingleThreadedExecutor()
            executor.add_node(pipeline)
            executor.add_node(observer)

            def pump(seconds, sensors):
                end, next_publish = time.monotonic() + seconds, 0
                while time.monotonic() < end:
                    if time.monotonic() >= next_publish:
                        stamp = observer.get_clock().now().nanoseconds / 1e9 - .02
                        for sensor in sensors:
                            points = np.zeros(2, dtype=CANONICAL)
                            points["x"], points["timestamp"] = [1, 2], stamp
                            header = stamp
                            if timing_only:
                                # Different first-point headers but identical interval midpoints.
                                times = [stamp - .10, stamp] if sensor == "204" else [stamp - .06, stamp - .04]
                                points["timestamp"] = times
                                header = times[0]
                            publishers[sensor].publish(cloud_message(Cloud(header, time.monotonic(),
                                "synthetic/er1_" + sensor, points, 4)))
                        next_publish = time.monotonic() + .05
                    executor.spin_once(timeout_sec=.01)

            pump(2.0, ("204", "205"))
            assert min(counts["204"], counts["205"]) >= 3, counts
            assert latest["ready"] is not None
            assert latest["ready"].evidence.source_type == 4
            assert latest.get("timing") is not None
            if mixed_domain:
                assert not latest["timing"].timing_ready and counts["merged"] == 0 and not transforms
                return {"mixed_clock_domain": True, "result": "PASS", "scope": "SYNTHETIC_ROS2_TRANSPORT_ONLY"}
            assert latest["timing"].timing_ready
            if timing_only:
                assert latest["timing"].header_delta_sec > .03
                assert latest["timing"].mid_delta_sec < .001
            if calibrated and not timing_only:
                assert counts["merged"] >= 3 and latest["ready"].dual_lidar_ready, counts
                assert {"synthetic/er1_204", "synthetic/er1_205"} <= transforms, transforms
                pump(.2, ())  # Drain transport before measuring no-old-frame reuse.
                receive_stamp = latest["ready"].evidence.receive_stamp
                previous = counts["merged"]
                pump(.8, ("204",))
                assert counts["merged"] == previous, "old 205 frame reused"
                assert latest["ready"].evidence.receive_stamp == receive_stamp, "timer refreshed receive evidence"
                assert not latest["ready"].dual_lidar_ready and not latest["ready"].b_fresh
            else:
                assert counts["merged"] == 0 and not latest["ready"].dual_lidar_ready
                assert not latest["ready"].extrinsic_valid
                assert not transforms, "unconfigured calibration published identity TF"
                assert not latest["timing"].spatial_merge_ready and not latest["timing"].dual_lidar_full_ready
                assert latest["timing"].reason == "TEMPORAL_PAIR_VALID"
                pump(.2, ())
                receive_stamp = latest["timing"].evidence.receive_stamp
                pump(.8, ("204",))
                assert not latest["timing"].timing_ready, "lost source did not expire timing"
                assert latest["timing"].evidence.receive_stamp == receive_stamp, "timer refreshed time evidence"
            return {"calibrated_fixture": calibrated, "timing_only": timing_only, "messages": counts,
                "result": "PASS", "scope": "SYNTHETIC_ROS2_TRANSPORT_ONLY"}
        finally:
            if executor:
                executor.shutdown()
            for node in (pipeline, observer):
                if node:
                    node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    print(json.dumps({"scenarios": [scenario(True), scenario(False), scenario(True, True),
        scenario(False, True, True)], "LIVE_ER1_STATUS": "NOT_RUN"}))
