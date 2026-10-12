#!/usr/bin/env python3
"""Explicit static scene capture; no fresh timing/phase/stationarity means no scene."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from chili_crane_control.qos import state_qos
from chili_crane_msgs.msg import SensorTimingState, DualLidarTimingState, RigStationarity, Rse1ClockStatus
from sensor_msgs.msg import PointCloud2
from chili_crane_slam.pointcloud import normalize_cloud
from chili_crane_slam.calibration_capture import SceneAccumulator, capture_gate


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


class Capture(Node):
    def __init__(self, args):
        super().__init__("capture_lidar_calibration_scene")
        self.args = args
        self.rig = yaml.safe_load((args.site / "calibration/sensor_rig.yaml").read_text())
        self.accumulator = SceneAccumulator(args.frames or 50, args.max_pair_delta, args.stable_frames)
        self.evidence, self.buffers, self.timing = {}, {}, {}
        self.clock_status, self.subs, self.started = {}, [], None
        for s in ("204", "205"):
            self.buffers[s] = {"raw": {}, "canonical": {}}
            cfg = yaml.safe_load((args.site / ("sensors/er1_" + s + ".yaml")).read_text())
            for kind, topic in (("raw", cfg["raw_topic"]), ("canonical", cfg["normalized_topic"])):
                self.subs.append(self.create_subscription(PointCloud2, topic,
                    lambda m, s=s, kind=kind: self.cloud(s, kind, m), qos_profile_sensor_data))
            self.subs.append(self.create_subscription(SensorTimingState, "lidar/er1_" + s + "/timing",
                lambda m, s=s: self.sensor_time(s, m), state_qos()))
            self.subs.append(self.create_subscription(Rse1ClockStatus, "lidar/er1_" + s + "/clock_status",
                lambda m, s=s: self.clock_status.update({s: {"timesync_status": m.timesync_status,
                    "source_type": m.source_type,
                    "frame_sync": m.frame_sync, "sequence": m.observation_sequence,
                    "difop_received_at": seconds(m.difop_received_at), "msop_received_at": seconds(m.msop_received_at)}}), state_qos()))
        self.subs.append(self.create_subscription(DualLidarTimingState, "lidar/dual_lidar/timing", self.dual, state_qos()))
        self.subs.append(self.create_subscription(RigStationarity, self.rig["stationarity_topic"], self.stationary, state_qos()))

    def base(self, msg):
        return {"received_at": self.get_clock().now().nanoseconds / 1e9,
                "measurement_at": seconds(msg.evidence.measurement_stamp),
                "source_type": msg.evidence.source_type, "valid": msg.validity == msg.VALID}

    def sensor_time(self, sensor, msg):
        self.timing[sensor] = msg
        self.evidence[sensor] = {**self.base(msg), "normalization_valid": msg.normalization_valid,
            "clock_valid": msg.clock_sync_valid, "phase_valid": msg.scan_phase_sync_valid}

    def dual(self, msg):
        self.evidence["dual"] = {**self.base(msg), "timing_ready": msg.timing_ready,
                                "clock_valid": msg.clock_sync_valid, "phase_valid": msg.scan_phase_sync_valid}

    def stationary(self, msg):
        self.evidence["rig"] = {**self.base(msg), "stationary": msg.stationary, "mount_revision": msg.mount_revision,
            "verification_method": msg.verification_method, "evidence_id": msg.evidence_id}

    def gate(self):
        self.rig = yaml.safe_load((self.args.site / "calibration/sensor_rig.yaml").read_text())
        if set(self.clock_status) != {"204", "205"}:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        for status in self.clock_status.values():
            if status["source_type"] != (1 if self.args.source_type == "PHYSICAL" else 4):
                return False
            if status["timesync_status"] != 1 or status["frame_sync"] != 1:
                return False
            if any(not 0 <= now - status[k] <= self.args.evidence_ttl for k in ("difop_received_at", "msop_received_at")):
                return False
        return capture_gate(self.get_clock().now().nanoseconds / 1e9, self.evidence,
            self.rig["mount_revision"], self.rig["parent_mode"], self.args.evidence_ttl,
            1 if self.args.source_type == "PHYSICAL" else 4)

    def cloud(self, sensor, kind, msg):
        if msg.width * msg.height > 200000:
            raise ValueError("FRAME_TOO_LARGE")
        cfg = yaml.safe_load((self.args.site / ("sensors/er1_" + sensor + ".yaml")).read_text())
        if msg.header.frame_id != cfg["frame_id"]:
            return
        pts = normalize_cloud(msg.data, [(f.name, f.offset, f.datatype, f.count) for f in msg.fields],
            msg.point_step, msg.row_step, msg.width, msg.height, msg.is_bigendian, int(sensor == "205"))
        ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        buffer = self.buffers[sensor][kind]
        buffer[ns] = pts
        while len(buffer) > 4:
            del buffer[next(iter(buffer))]

    def collect(self):
        if not self.gate():
            self.accumulator.add({}, False)
            self.started = None
            return False
        frames = {}
        now = self.get_clock().now().nanoseconds / 1e9
        for s in ("204", "205"):
            timing = self.timing.get(s)
            if timing is None:
                return False
            offset = timing.applied_offset_sec
            for ns in sorted(self.buffers[s]["canonical"], reverse=True):
                if not 0 <= now - ns / 1e9 <= self.args.evidence_ttl:
                    continue
                raw_ns = ns + round(offset * 1e9)
                if raw_ns in self.buffers[s]["raw"]:
                    frames[s] = {"raw_header_ns": raw_ns, "canonical_header_ns": ns, "applied_offset_sec": offset,
                        "raw_points": self.buffers[s]["raw"][raw_ns], "canonical_points": self.buffers[s]["canonical"][ns]}
                    break
        complete = self.accumulator.add(frames, True)
        if self.accumulator.rows and self.started is None:
            self.started = time.monotonic()
        if self.args.seconds and self.started is not None:
            if time.monotonic() - self.started >= self.args.seconds and len(self.accumulator.rows) >= 20:
                self.accumulator.count = len(self.accumulator.rows)
                return True
            return False
        return complete

    def save(self):
        config = {str(p.relative_to(self.args.site)): hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in self.args.site.rglob("*.yaml")}
        digest = hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()
        sha = subprocess.check_output(["git", "-C", str(self.args.repository), "rev-parse", "HEAD"], text=True).strip()
        manifest = {"scene_id": self.args.scene_id, "split": self.args.split, "source_type": self.args.source_type,
            "source_frame": "crane_01/er1_205", "target_frame": "crane_01/er1_204",
            "mount_revision": self.rig["mount_revision"], "parent_mode": self.rig["parent_mode"],
            "git_sha": sha, "config_hash": digest, "config_files": config, "timing": self.evidence,
            "clock_status": self.clock_status, "network": yaml.safe_load((self.args.site / "network/interfaces.yaml").read_text()),
            "operator": self.args.operator, "physical_output_enabled": False, "automatic_control_enabled": False}
        return self.accumulator.save(self.args.output, manifest, self.args.repository)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--site", type=Path, required=True)
    p.add_argument("--repository", type=Path, required=True)
    p.add_argument("--scene-id", required=True)
    p.add_argument("--split", choices=["CALIBRATION", "VALIDATION"], required=True)
    duration = p.add_mutually_exclusive_group(required=True)
    duration.add_argument("--frames", type=int, choices=range(20, 51))
    duration.add_argument("--seconds", type=float, choices=[2, 3, 4, 5])
    p.add_argument("--stable-frames", type=int, default=20)
    p.add_argument("--max-pair-delta", type=float, required=True)
    p.add_argument("--evidence-ttl", type=float, required=True)
    p.add_argument("--operator", required=True)
    p.add_argument("--source-type", choices=["PHYSICAL", "SYNTHETIC"], default="PHYSICAL")
    p.add_argument("--output", type=Path, required=True)
    a, ros_args = p.parse_known_args()
    if not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", a.scene_id) or a.output.exists():
        p.error("new scene ID/directory required")
    if a.repository.resolve() in a.output.resolve().parents:
        p.error("data must stay outside repository")
    rclpy.init(args=ros_args)
    node = Capture(a)
    try:
        end = time.monotonic() + 90
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
            if node.collect():
                node.save()
                print("SCENE_CAPTURE_COMPLETE; SOURCE=" + a.source_type + "; PHYSICAL_FIELD_ACCEPTANCE_NOT_IMPLIED")
                return
        raise SystemExit("CAPTURE_BLOCKED_OR_INCOMPLETE_NO_MANIFEST")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
