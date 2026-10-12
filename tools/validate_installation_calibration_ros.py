#!/usr/bin/env python3
"""ROS transport installation/capture verification; only synthetic evidence."""
import argparse
import importlib.util
import json
from pathlib import Path
import shutil
import tempfile
import time
from types import SimpleNamespace
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from chili_crane_msgs.msg import SensorTimingState, DualLidarTimingState, RigStationarity, Rse1ClockStatus
from chili_crane_control.qos import state_qos
from chili_crane_slam.dual_lidar import Cloud
from chili_crane_slam.dual_lidar_node import cloud_message
from chili_crane_slam.pointcloud import CANONICAL

ROOT = Path(__file__).resolve().parents[1]


def module(name, file):
    spec = importlib.util.spec_from_file_location(name, ROOT / "src/chili_crane_slam/scripts" / file)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-dir", type=Path, required=True)
    args = parser.parse_args()
    args.evidence_dir.mkdir(parents=True, exist_ok=True)
    counts = {"rough": 0, "tf": 0, "merged": 0}
    with tempfile.TemporaryDirectory(prefix="phase1c-synthetic-") as tmp:
        site = Path(tmp) / "site"
        shutil.copytree(ROOT / "config/sites/crane_01", site)
        rig_path = site / "calibration/sensor_rig.yaml"
        rig = yaml.safe_load(rig_path.read_text())
        rig.update(parent_mode="DYNAMIC_PARENT", mount_revision="synthetic-r1")
        rig_path.write_text(yaml.safe_dump(rig))
        rclpy.init(args=["--ros-args", "-r", "__ns:=/crane_01", "-p", "site_config:=" + str(site)])
        executor = SingleThreadedExecutor()
        observer = Node("phase1c_synthetic_observer")
        preview = module("preview_test", "installation_preview_node.py").InstallationPreview()
        capture = module("capture_test", "capture_lidar_calibration_scene.py").Capture(SimpleNamespace(
            site=site, frames=20, max_pair_delta=.001, stable_frames=2, evidence_ttl=.2,
            seconds=None, scene_id="synthetic-transport", split="VALIDATION", operator="CI_SYNTHETIC",
            source_type="SYNTHETIC", repository=ROOT, output=args.evidence_dir / "synthetic-scene"))
        for node in (observer, preview, capture):
            executor.add_node(node)
        latest, subscriptions, pubs = {}, [], {}
        subscriptions.append(observer.create_subscription(String, "installation/diagnostics",
            lambda m: latest.update(diagnostics=json.loads(m.data)), 10))
        for sensor in ("204", "205"):
            cfg = yaml.safe_load((site / ("sensors/er1_" + sensor + ".yaml")).read_text())
            for kind in ("raw", "normalized"):
                pubs[sensor, kind] = observer.create_publisher(PointCloud2, cfg[kind + "_topic"], qos_profile_sensor_data)
            pubs[sensor, "timing"] = observer.create_publisher(SensorTimingState, "lidar/er1_" + sensor + "/timing", state_qos())
            pubs[sensor, "clock"] = observer.create_publisher(Rse1ClockStatus, "lidar/er1_" + sensor + "/clock_status", state_qos())
            subscriptions.append(observer.create_subscription(PointCloud2, "installation/rough/er1_" + sensor,
                lambda _: counts.update(rough=counts["rough"] + 1), qos_profile_sensor_data))
        pubs["dual"] = observer.create_publisher(DualLidarTimingState, "lidar/dual_lidar/timing", state_qos())
        pubs["rig"] = observer.create_publisher(RigStationarity, rig["stationarity_topic"], state_qos())
        subscriptions.append(observer.create_subscription(PointCloud2, "lidar/merged_points",
            lambda _: counts.update(merged=counts["merged"] + 1), qos_profile_sensor_data))
        subscriptions.append(observer.create_subscription(TFMessage, "/tf_static",
            lambda _: counts.update(tf=counts["tf"] + 1), QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)))

        def evidence(msg, stamp):
            msg.header.stamp = stamp
            msg.validity = msg.VALID
            msg.evidence.measurement_stamp = msg.evidence.receive_stamp = stamp
            msg.evidence.source_type = 4

        def pump(duration, stationary=None, clock=1, frame=1, collect=False):
            end, next_pub = time.monotonic() + duration, 0
            while time.monotonic() < end:
                if time.monotonic() >= next_pub:
                    now = observer.get_clock().now()
                    ns = now.nanoseconds - 20000000
                    for s in ("204", "205"):
                        timing = SensorTimingState()
                        evidence(timing, now.to_msg())
                        timing.normalization_valid = timing.clock_sync_valid = timing.scan_phase_sync_valid = True
                        timing.applied_offset_sec = 37
                        pubs[s, "timing"].publish(timing)
                        status = Rse1ClockStatus(timesync_status=clock, frame_sync=frame, source_type=4)
                        status.difop_received_at = status.msop_received_at = now.to_msg()
                        pubs[s, "clock"].publish(status)
                        canonical = np.zeros(3, dtype=CANONICAL)
                        canonical["x"] = [1, 2, 3]
                        canonical["timestamp"] = ns / 1e9 + np.array([0, .01, .02])
                        raw = canonical.copy()
                        raw["timestamp"] += 37
                        # Match the frozen exact float subtraction contract, not idealized algebra.
                        canonical["timestamp"] = raw["timestamp"] - 37
                        for kind, stamp, points in (("raw", ns + 37000000000, raw), ("normalized", ns, canonical)):
                            pubs[s, kind].publish(cloud_message(Cloud(stamp / 1e9, time.monotonic(),
                                "crane_01/er1_" + s, points, 4, None, stamp)))
                    dual = DualLidarTimingState()
                    evidence(dual, now.to_msg())
                    dual.timing_ready = dual.clock_sync_valid = dual.scan_phase_sync_valid = True
                    pubs["dual"].publish(dual)
                    if stationary is not None:
                        motion = RigStationarity(mount_revision="synthetic-r1", stationary=stationary,
                            verification_method="SYNTHETIC_FIXTURE", evidence_id="CI_SYNTHETIC_ONLY")
                        evidence(motion, now.to_msg())
                        pubs["rig"].publish(motion)
                    next_pub = time.monotonic() + .04
                executor.spin_once(timeout_sec=.002)
                if collect:
                    capture.collect()
        try:
            pump(1)
            assert not capture.gate(), "Missing stationarity must block capture"
            assert counts["rough"] == 0, "Unknown seed must not publish identity overlay"
            for s in ("204", "205"):
                path = site / ("calibration/mechanical_seed_er1_" + s + ".yaml")
                seed = yaml.safe_load(path.read_text())
                seed.update(translation_m=[.1 if s == "204" else .2, 0, 0], roll_pitch_yaw_rad=[0, 0, .01],
                    measurement_method="SYNTHETIC_FIXTURE", measurement_uncertainty={"translation_m": .01, "rotation_rad": .01},
                    operator="CI", date="2026-10-12", mount_revision="synthetic-r1")
                path.write_text(yaml.safe_dump(seed))
            pump(1.8, stationary=True, collect=True)
            assert capture.gate() and len(capture.accumulator.rows) == 20
            assert counts["rough"] >= 2 and latest["diagnostics"]["extrinsic_valid"] is False
            manifest = capture.save()
            assert manifest["source_type"] == "SYNTHETIC" and len(manifest["artifacts"]) == 4
            pump(.2, stationary=True, clock=2, collect=True)
            assert not capture.gate() and not capture.accumulator.rows
            pump(.2, stationary=True, frame=0)
            assert not capture.gate(), "Frame phase is independent of clock success"
            pump(.2, stationary=False)
            assert not capture.gate(), "Moving rig must block capture"
            assert counts["tf"] == counts["merged"] == 0
            print(json.dumps({"INSTALLATION_CAPTURE_TRANSPORT": "PASS", "scope": "SYNTHETIC_ONLY",
                "checks": ["unknown-seed-no-overlay", "dynamic-parent-no-static-tf", "missing-rig-blocked",
                    "20-frame-paired-capture", "raw-canonical-hash-provenance", "clock-timeout-blocked",
                    "frame-phase-blocked", "moving-rig-blocked", "no-production-merger"], "counts": counts,
                "INSTALLATION_ON_CRANE": "NOT_RUN", "EXTRINSIC_FIELD_STATUS": "NOT_RUN"}))
        finally:
            executor.shutdown()
            for node in (capture, preview, observer):
                node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
