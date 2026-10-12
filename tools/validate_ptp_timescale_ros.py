#!/usr/bin/env python3
"""Exact-SHA ROS2 time-scale transport tests. No sockets, vendor changes or devices."""
import json
from pathlib import Path
import sys
import tempfile
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from chili_crane_msgs.msg import SensorTimingState, DualLidarTimingState, DualLidarReadiness, Rse1ClockStatus
from chili_crane_control.qos import state_qos
from chili_crane_slam.dual_lidar_node import DualLidarNode, cloud_message
from chili_crane_slam.dual_lidar import Cloud
from chili_crane_slam.pointcloud import CANONICAL

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_dual_lidar_ros import fixture


def scenario(name, state="LOCAL_VERIFIED", missing_relation=False, unknown_scale=False,
             failure=None, frame_sync=1, expire=False):
    with tempfile.TemporaryDirectory(prefix="chili-r2-synthetic-") as tmp:
        site = Path(tmp)
        fixture(site, calibrated=False, timing_only=True)
        for sensor in ("204", "205"):
            path = site / ("sensors/er1_" + sensor + ".yaml")
            cfg = yaml.safe_load(path.read_text())
            cfg.update(clock_mode="SENSOR_PTP", clock_sync_state="VALID", clock_domain="ptp-domain-0",
                       raw_time_scale="UNKNOWN" if unknown_scale else "PTP_TIMESCALE",
                       time_evidence_id="synthetic-sync", runtime_status_timeout_sec=.25,
                       time_scale_relation=None if missing_relation else {
                           "source_scale": "PTP_TIMESCALE", "target_scale": "HOST_UTC",
                           "source_domain": "ptp-domain-0", "target_domain": "host-utc",
                           "utc_offset_sec": 37, "relation_state": state,
                           "evidence_id": "synthetic-only", "offset_source": "synthetic-pmc",
                           "current_utc_offset_valid": state == "VALID", "time_traceable": state == "VALID"})
            path.write_text(yaml.safe_dump(cfg))
        rclpy.init(args=["--ros-args", "-p", "site_config:=" + tmp, "-r", "__ns:=/phase1a_synthetic"])
        pipeline = observer = executor = None
        try:
            observer = Node("r2_timescale_observer")
            pipeline = DualLidarNode()
            executor = SingleThreadedExecutor()
            executor.add_node(pipeline)
            executor.add_node(observer)
            counts = {"vendor": 0, "canonical": 0, "merged": 0, "tf": 0}
            latest, sent = {}, {s: {} for s in ("204", "205")}
            subscriptions, clouds, statuses = [], {}, {}
            for sensor in ("204", "205"):
                clouds[sensor] = observer.create_publisher(PointCloud2, "/phase1a_synthetic/vendor_" + sensor, qos_profile_sensor_data)
                statuses[sensor] = observer.create_publisher(Rse1ClockStatus,
                    "/phase1a_synthetic/lidar/er1_" + sensor + "/clock_status", state_qos())
                def vendor(msg):
                    counts["vendor"] += 1
                def canonical(msg, sensor=sensor):
                    ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
                    raw = sent[sensor][ns + 37000000000]
                    np.testing.assert_array_equal(raw["timestamp"] - 37, np.frombuffer(msg.data, dtype=CANONICAL)["timestamp"])
                    counts["canonical"] += 1
                subscriptions.append(observer.create_subscription(PointCloud2, "/phase1a_synthetic/vendor_" + sensor, vendor, qos_profile_sensor_data))
                subscriptions.append(observer.create_subscription(PointCloud2, "/phase1a_synthetic/raw_" + sensor, canonical, qos_profile_sensor_data))
                subscriptions.append(observer.create_subscription(SensorTimingState,
                    "/phase1a_synthetic/lidar/er1_" + sensor + "/timing",
                    lambda msg, sensor=sensor: latest.update({sensor: msg}), state_qos()))
            subscriptions.append(observer.create_subscription(DualLidarTimingState,
                "/phase1a_synthetic/lidar/dual_lidar/timing", lambda msg: latest.update(timing=msg), state_qos()))
            subscriptions.append(observer.create_subscription(DualLidarReadiness,
                "/phase1a_synthetic/lidar/dual_lidar/readiness", lambda msg: latest.update(ready=msg), state_qos()))
            subscriptions.append(observer.create_subscription(PointCloud2, "/phase1a_synthetic/lidar/merged_points",
                lambda _: counts.update(merged=counts["merged"] + 1), qos_profile_sensor_data))
            subscriptions.append(observer.create_subscription(TFMessage, "/tf_static",
                lambda _: counts.update(tf=counts["tf"] + 1),
                QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
            sequence = 0
            saved_status = {}
            def pump(seconds, status=1, repeat_old=False):
                nonlocal sequence
                end, next_publish = time.monotonic() + seconds, 0
                while time.monotonic() < end:
                    if time.monotonic() >= next_publish:
                        sequence += 1
                        now = observer.get_clock().now()
                        for sensor in ("204", "205"):
                            msg = Rse1ClockStatus()
                            msg.header.stamp, msg.header.frame_id = now.to_msg(), "synthetic/er1_" + sensor
                            msg.sensor_id, msg.source_type = "er1_" + sensor, 4
                            msg.timesync_mode = msg.time_mode = 2
                            msg.mode_state = "PTP_MODE"
                            msg.timesync_status, msg.frame_sync = status, frame_sync
                            msg.clock_sync_valid, msg.scan_phase_sync_valid = status == 1, frame_sync == 1
                            msg.validity = msg.VALID if status == 1 else msg.DEGRADED
                            msg.clock_state = {0: "PTP_SYNC_FAILED", 1: "PTP_SYNC_SUCCESS", 2: "PTP_SYNC_TIMEOUT"}[status]
                            msg.difop_received_at = msg.msop_received_at = now.to_msg()
                            msg.observation_sequence, msg.observer_boot_id = sequence, "synthetic-observer-boot"
                            if repeat_old:
                                # Fresh-looking envelope is not fresh evidence; sequence unchanged.
                                msg.observation_sequence = saved_status[sensor]
                            else:
                                saved_status[sensor] = sequence
                            statuses[sensor].publish(msg)
                            raw_ns = now.nanoseconds - 120000000 + 37000000000 + 1
                            pts = np.zeros(2, dtype=CANONICAL)
                            pts["timestamp"] = [raw_ns / 1e9, raw_ns / 1e9 + .0625]
                            sent[sensor][raw_ns] = pts.copy()
                            clouds[sensor].publish(cloud_message(Cloud(raw_ns / 1e9, time.monotonic(),
                                "synthetic/er1_" + sensor, pts, 4, None, raw_ns)))
                        next_publish = time.monotonic() + .04
                    executor.spin_once(timeout_sec=.005)
            pump(1.2)
            assert counts["vendor"] >= 6 and latest.get("timing") is not None
            assert counts["tf"] == counts["merged"] == 0, counts
            assert not latest["ready"].dual_lidar_ready  # No extrinsics/space authority.
            if missing_relation or unknown_scale:
                assert counts["canonical"] == 0 and not latest["timing"].timing_ready, counts
                assert not latest["204"].normalization_valid
            else:
                assert counts["canonical"] >= 6 and latest["timing"].timing_ready, counts
                timing = latest["204"]
                raw_ns = timing.raw_header_stamp.sec * 1000000000 + timing.raw_header_stamp.nanosec
                normalized_ns = timing.normalized_header_stamp.sec * 1000000000 + timing.normalized_header_stamp.nanosec
                assert raw_ns - normalized_ns == 37000000000
                assert timing.raw_time_scale == "PTP_TIMESCALE" and timing.normalized_time_scale == "HOST_UTC"
                assert timing.applied_offset_sec == 37 and timing.normalization_valid
                assert latest["timing"].production_timing_ready == (state == "VALID")
                assert latest["timing"].clock_sync_valid
                assert latest["timing"].scan_phase_sync_valid == (frame_sync == 1)
                assert timing.future_stamp_count == 0
                if failure is not None:
                    pump(.16, status=failure)
                    assert not latest["timing"].timing_ready and not latest["timing"].clock_sync_valid
                    assert latest["timing"].scan_phase_sync_valid == (frame_sync == 1)
                    assert not latest["timing"].production_timing_ready
                if expire:
                    pump(.5, repeat_old=True)
                    assert not latest["timing"].timing_ready and not latest["timing"].clock_sync_valid
            return {"scenario": name, "result": "PASS", "messages": counts, "scope": "SYNTHETIC_ROS2_ONLY"}
        finally:
            if executor:
                executor.shutdown()
            for node in (pipeline, observer):
                if node:
                    node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    print(json.dumps({"scenarios": [
        scenario("LOCAL_VERIFIED_BENCH_NOT_PRODUCTION"),
        scenario("VALID_PRODUCTION_TIMING", state="VALID"),
        scenario("MISSING_RELATION_RAW_ONLY", missing_relation=True),
        scenario("UNKNOWN_SCALE_RAW_ONLY", unknown_scale=True),
        scenario("SYNC_FAILURE_IMMEDIATE", failure=0),
        scenario("SYNC_TIMEOUT_IMMEDIATE", failure=2),
        scenario("FRAME_SYNC_ZERO_CLOCK_SUCCESS", frame_sync=0),
        scenario("DUPLICATE_OBSERVATION_CANNOT_RENEW", expire=True)],
        "LIVE_ER1_STATUS": "NOT_RUN"}))
