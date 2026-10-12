#!/usr/bin/env python3
"""Installed-node/launch transport acceptance, synthetic only. Run on Ubuntu."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from chili_crane_msgs.msg import (EvidenceMetadata, GrabIoState, GrabState,
                                 GrabGeometrySafetyState, GrabClearanceState,
                                 SafetyPermit, AuthorizedCommand, ActuationRequest)
from chili_crane_perception.synthetic import fixture, scene, pit
from chili_crane_slam.dual_lidar import Cloud
from chili_crane_slam.dual_lidar_node import cloud_message
ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-dir", type=Path, required=True)
    args = parser.parse_args()
    args.evidence_dir.mkdir(parents=True, exist_ok=True)
    for package, expected in (
            ("chili_crane_perception", ("grab_perception_node.py", "grab_clearance_node.py")),
            ("chili_crane_slam", ("dual_lidar_node.py", "run_manifest_node.py",
                                "installation_preview_node.py", "capture_lidar_calibration_scene.py"))):
        listing = subprocess.check_output(["ros2","pkg","executables",package], text=True)
        assert all(name in listing for name in expected), listing
    subprocess.run(["ros2","run","chili_crane_slam","capture_lidar_calibration_scene.py","--help"],
                   check=True, timeout=20, stdout=subprocess.DEVNULL)
    with tempfile.TemporaryDirectory(prefix="phase2a-ros-synthetic-") as tmp:
        tmp = Path(tmp)
        config = fixture()
        semantic = tmp / "semantic.yaml"
        semantic.write_text(yaml.safe_dump({"config_state":"VALID","frame_id":"map","pits":[pit()]}), encoding="utf-8")
        config.update(config_state="VALID",config_version="synthetic-ci-v1",extrinsic_calibration_id="synthetic-extrinsic",
            physical_output_enabled=False,automatic_control_enabled=False,camera_control_authority=False,
            semantic_map_file=str(semantic),pit_id="SYNTHETIC_PIT")
        file = tmp / "shadow.yaml"
        file.write_text(yaml.safe_dump(config), encoding="utf-8")
        log = (args.evidence_dir / "shadow-launch.log").open("w", encoding="utf-8")
        process = subprocess.Popen(["ros2","launch","chili_crane_bringup","phase2a_perception_shadow.launch.py",
            "config_file:="+str(file)], stdout=log, stderr=subprocess.STDOUT)
        preview_log = (args.evidence_dir / "installed-preview.log").open("w",encoding="utf-8")
        preview = subprocess.Popen(["ros2","run","chili_crane_slam","installation_preview_node.py","--ros-args",
            "-r","__ns:=/crane_01","-p","site_config:="+str(ROOT/"config/sites/crane_01")],
            stdout=preview_log,stderr=subprocess.STDOUT)
        rclpy.init()
        node = Node("phase2a_transport_observer",namespace="crane_01")
        qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        received, forbidden = {}, []
        subs = [node.create_subscription(kind,topic,
            lambda m,topic=topic:received.update({topic:m}),qos)
            for kind,topic in (
                (GrabState,"perception/grab_state"),(GrabGeometrySafetyState,"perception/grab_geometry_safety"),
                (GrabClearanceState,"perception/grab_clearance"))]
        subs.append(node.create_subscription(String,"installation/diagnostics",
            lambda m:received.update(preview=m.data),10))
        for kind,topic in ((SafetyPermit,"safety/permit"),(AuthorizedCommand,"control/authorized_command"),
                           (ActuationRequest,"control/actuation_request")):
            subs.append(node.create_subscription(kind,topic,lambda m:forbidden.append(type(m).__name__),qos))
        pub = node.create_publisher(PointCloud2,"perception/grab_input",qos)
        evidence = node.create_publisher(EvidenceMetadata,"perception/grab_input_evidence",qos)
        io_pub = node.create_publisher(GrabIoState,"hardware/grab_io_state",qos)
        truth = np.eye(4)
        truth[2,3] = 2
        started, next_frame, counter = time.monotonic(),0,0
        try:
            while time.monotonic()-started < 15:
                assert process.poll() is None, "shadow launch exited"
                assert preview.poll() is None, "installed preview exited"
                if time.monotonic() >= next_frame:
                    stamp = node.get_clock().now().nanoseconds/1e9-.07
                    points = scene(config["model"],truth,stamp)
                    cloud = cloud_message(Cloud(stamp,time.monotonic(),"map",points))
                    meta = EvidenceMetadata()
                    meta.header = cloud.header
                    meta.measurement_stamp = cloud.header.stamp
                    meta.receive_stamp = node.get_clock().now().to_msg()
                    counter += 1
                    meta.source_counter = counter
                    meta.validity,meta.source_type = meta.VALID,meta.SOURCE_SYNTHETIC
                    meta.calibration_id,meta.config_version = "synthetic-extrinsic","synthetic-ci-v1"
                    meta.evidence_age_sec = .07
                    io = GrabIoState(validity=GrabIoState.VALID,closed_limit=True,evidence_age_sec=0.)
                    evidence.publish(meta)
                    io_pub.publish(io)
                    pub.publish(cloud)
                    next_frame = time.monotonic()+.15
                rclpy.spin_once(node,timeout_sec=.02)
                state = received.get("perception/grab_state")
                clearance = received.get("perception/grab_clearance")
                if (state and state.tracking_state==state.TRACK_TRACKED
                        and clearance and clearance.reason=="SHADOW_GEOMETRY_ONLY" and "preview" in received):
                    break
            else:
                raise RuntimeError("installed shadow did not TRACK synthetic input; "
                    +json.dumps({k:getattr(v,"reason",str(v)) for k,v in received.items()}))
            geometry = received["perception/grab_geometry_safety"]
            assert not geometry.valid_for_control and not clearance.valid_for_control and not forbidden
            assert state.validity == state.DEGRADED
            assert geometry.evidence.source_type == geometry.evidence.SOURCE_SYNTHETIC
            assert geometry.bottom_lower_bound_m <= 1.4
            assert clearance.conservative_clearance_m <= clearance.raw_horizontal_clearance_m
            # Stop all input; timers must expire output rather than heartbeat-refresh evidence.
            deadline = time.monotonic()+1
            while time.monotonic() < deadline:
                rclpy.spin_once(node,timeout_sec=.02)
            assert received["perception/grab_state"].tracking_state == GrabState.TRACK_LOST
            assert received["perception/grab_clearance"].validity != GrabClearanceState.VALID
            assert not forbidden
            print(json.dumps({"SHADOW_RUNTIME_STATUS":"PASS_SYNTHETIC_ROS",
                "INSTALLED_PREVIEW_STATUS":"PASS_SOFTWARE","INPUT_EXPIRY_STATUS":"PASS",
                "CONTROL_TOPIC_MESSAGES":len(forbidden),"PHYSICAL_ACTUATION_STATUS":"NOT_RUN"}))
        finally:
            node.destroy_node()
            rclpy.shutdown()
            for child in (preview,process):
                child.send_signal(2)
                try:
                    child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    child.terminate()
                    child.wait(timeout=5)
            log.close()
            preview_log.close()


if __name__ == "__main__":
    main()
