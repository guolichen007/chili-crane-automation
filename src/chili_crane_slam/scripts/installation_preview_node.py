#!/usr/bin/env python3
"""Separate-color, non-production rough/candidate overlays and installation statistics."""
import json
from pathlib import Path
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from chili_crane_slam.pointcloud import normalize_cloud
from chili_crane_slam.dual_lidar import Cloud
from chili_crane_slam.dual_lidar_node import cloud_message
from chili_crane_slam.calibration_geometry import mechanical_seed, transform, coverage, rigid


class InstallationPreview(Node):
    def __init__(self):
        super().__init__("installation_preview")
        self.declare_parameter("site_config", "")
        self.declare_parameter("candidate", "")
        self.site = Path(self.get_parameter("site_config").value)
        self.candidate_path = self.get_parameter("candidate").value
        self.latest, self.subs = {}, []
        self.pubs = {s: self.create_publisher(PointCloud2, "installation/" + ("overlay" if self.candidate_path else "rough") + "/er1_" + s, qos_profile_sensor_data) for s in ("204", "205")}
        self.diagnostics = self.create_publisher(String, "installation/diagnostics", 10)
        for s in ("204", "205"):
            self.subs.append(self.create_subscription(PointCloud2, "lidar/er1_" + s + "/points",
                lambda m, s=s: self.receive(s, m), qos_profile_sensor_data))
        self.timer = self.create_timer(.5, self.render)

    def receive(self, sensor, msg):
        if not 0 < msg.width * msg.height <= 200000:
            self.latest.pop(sensor, None)
            return
        cfg = yaml.safe_load((self.site / ("sensors/er1_" + sensor + ".yaml")).read_text())
        if msg.header.frame_id != cfg["frame_id"]:
            self.latest.pop(sensor, None)
            return
        points = normalize_cloud(msg.data, [(f.name, f.offset, f.datatype, f.count) for f in msg.fields],
            msg.point_step, msg.row_step, msg.width, msg.height, msg.is_bigendian, int(sensor == "205"))
        self.latest[sensor] = (msg, points, time.monotonic())

    def render(self):
        report = {"status": "CANDIDATE" if self.candidate_path else "MECHANICAL_SEED",
                  "extrinsic_valid": False, "dual_lidar_ready": False, "sensor": {}}
        rig = yaml.safe_load((self.site / "calibration/sensor_rig.yaml").read_text())
        clouds = {}
        for s, (msg, points, arrival) in self.latest.items():
            xyz = np.column_stack([points[k] for k in ("x", "y", "z")])
            ranges = np.linalg.norm(xyz, axis=1)
            report["sensor"][s] = {"point_count": len(points), "age_sec": time.monotonic() - arrival,
                "range_m": {str(q): float(np.percentile(ranges, q)) for q in (10, 50, 90, 95)}}
            sensor_cfg = yaml.safe_load((self.site / ("sensors/er1_" + s + ".yaml")).read_text())
            ttl = sensor_cfg.get("runtime_status_timeout_sec")
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            source_age = self.get_clock().now().nanoseconds / 1e9 - stamp
            if ttl is None or time.monotonic() - arrival > ttl or not 0 <= source_age <= ttl:
                report["sensor"][s]["state"] = "STALE"
                continue
            try:
                if self.candidate_path:
                    candidate = yaml.safe_load(Path(self.candidate_path).read_text())
                    if (candidate.get("status") not in {"CANDIDATE", "REVIEWED", "FIELD_VALIDATED"}
                            or candidate.get("mount_revision") != rig["mount_revision"]
                            or candidate.get("source_frame") != "crane_01/er1_205"
                            or candidate.get("target_frame") != "crane_01/er1_204"):
                        raise ValueError("CANDIDATE_INVALIDATED_OR_FRAME_MISMATCH")
                    t = rigid(candidate["T_target_source"]) if s == "205" else np.eye(4)
                    frame = candidate["target_frame"]
                else:
                    seed = yaml.safe_load((self.site / ("calibration/mechanical_seed_er1_" + s + ".yaml")).read_text())
                    t = mechanical_seed(seed, rig["mount_revision"])
                    frame = rig["rig_frame"]
                converted = points.copy()
                moved = transform(xyz, t)
                for i, key in enumerate(("x", "y", "z")):
                    converted[key] = moved[:, i]
                clouds[s] = moved
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                self.pubs[s].publish(cloud_message(Cloud(stamp, arrival, frame, converted)))
            except ValueError as exc:
                report["sensor"][s]["overlay_state"] = str(exc)
        if set(clouds) == {"204", "205"}:
            geometry = yaml.safe_load((self.site / "calibration/installation_geometry.yaml").read_text())
            try:
                if geometry.get("coordinate_frame") != frame:
                    raise ValueError("ROI_FRAME_MISMATCH")
                report["coverage"] = coverage(clouds["204"], clouds["205"], geometry)
                from chili_crane_slam.calibration_geometry import inside
                report["common_static_structure"] = {roi["name"]: {
                    "204_points": int(inside(clouds["204"], roi["bounds_m"]).sum()),
                    "205_points": int(inside(clouds["205"], roi["bounds_m"]).sum())}
                    for roi in geometry.get("static_structure_rois", [])}
            except ValueError as exc:
                report["coverage"] = str(exc)
        else:
            report["coverage"] = "NOT_CONFIGURED"
        report["parent_mode"] = rig["parent_mode"]
        self.diagnostics.publish(String(data=json.dumps(report, ensure_ascii=False, allow_nan=False)))


def main():
    rclpy.init()
    node = InstallationPreview()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
