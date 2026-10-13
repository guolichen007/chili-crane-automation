#!/usr/bin/env python3
"""Installed-node/launch transport acceptance, synthetic only. Run on Ubuntu."""
import argparse
import copy
import json
import os
from pathlib import Path
import subprocess
import signal
import sys
import tempfile
import time
import uuid
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


def validate_clearance_age(node, config_file, geometry, qos, evidence_dir):
    """One synthetic Geometry publication to a separate INSTALLED clearance node."""
    # normal/symlink invocations must not share a retained source or publisher.
    namespace=node.get_namespace()+"/r11_age_contract_"+uuid.uuid4().hex[:12]
    samples=[]
    received_initial_output=[False]
    marker=geometry.evidence.source_counter+10000
    def observe(m):
        received_initial_output[0]=True
        if m.evidence.source_counter==marker and len(samples)<64:
            samples.append(m)
    sub=node.create_subscription(GrabClearanceState,namespace+"/perception/grab_clearance",
        observe,qos)
    pub=node.create_publisher(GrabGeometrySafetyState,namespace+"/perception/grab_geometry_safety",qos)
    log=(evidence_dir/"clearance-age-contract.log").open("w",encoding="utf-8")
    child=subprocess.Popen(["ros2","run","chili_crane_perception","grab_clearance_node.py","--ros-args",
        "-r","__ns:="+namespace,"-r","__node:=clearance_age_contract","-p","config_file:="+str(config_file)],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        discovery=time.monotonic()+6
        while (pub.get_subscription_count()==0 or not received_initial_output[0]) and time.monotonic()<discovery:
            assert child.poll() is None,"installed clearance age node exited"
            rclpy.spin_once(node,timeout_sec=.02)
        assert pub.get_subscription_count()>0,"clearance age input did not discover"
        assert received_initial_output[0],"clearance age OUTPUT transport did not become ready"
        assert pub.get_subscription_count()==1,"multiple clearance input readers in isolated namespace"
        assert node.count_publishers(namespace+"/perception/grab_clearance")==1,"multiple clearance publishers"
        sample=copy.deepcopy(geometry)
        sample.header.stamp=node.get_clock().now().to_msg()
        sample.evidence.measurement_stamp=copy.deepcopy(sample.header.stamp)
        sample.evidence.receive_stamp=copy.deepcopy(sample.header.stamp)
        sample.evidence.header=copy.deepcopy(sample.header)
        sample.evidence.source_counter=marker
        sample.evidence_age_sec=sample.evidence.evidence_age_sec=0.
        sample.valid_for_control=False
        pub.publish(sample)  # EXACTLY ONCE: no new Geometry can renew its age.
        end=time.monotonic()+1
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
        assert len(samples)>=4,"insufficient clearance timer samples"
        key=lambda s:(s.sec,s.nanosec)
        epochs=key(sample.header.stamp)
        assert all(key(m.header.stamp)==epochs and key(m.evidence.measurement_stamp)==epochs
                   and m.evidence.source_counter==marker for m in samples)
        assert all(abs(m.evidence_age_sec-m.evidence.evidence_age_sec)<1e-9 for m in samples)
        assert all(a.evidence_age_sec<b.evidence_age_sec for a,b in zip(samples,samples[1:]))
        assert any(m.validity==m.DEGRADED and m.reason=="SHADOW_GEOMETRY_ONLY" for m in samples)
        assert samples[-1].validity==samples[-1].STALE and samples[-1].wall_collision_risk
        assert samples[-1].evidence.validity==samples[-1].evidence.STALE
        assert all(not m.valid_for_control for m in samples)
        return dict(CLEARANCE_OUTER_INNER_AGE_MATCH="PASS",CLEARANCE_AGE_MONOTONIC="PASS",
                    CLEARANCE_MEASUREMENT_EPOCH_UNCHANGED="PASS",CLEARANCE_EVENTUALLY_STALE="PASS",
                    geometry_publications=1,clearance_samples=len(samples))
    finally:
        # ros2 run is a wrapper process. Signal its OWN isolated group, including
        # the executable child, rather than leaving a stale timer node behind.
        try:
            os.killpg(child.pid,signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            child.wait(timeout=8)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(child.pid,signal.SIGTERM)
            except ProcessLookupError:
                pass
            child.wait(timeout=5)
        log.close()
        node.destroy_subscription(sub)
        node.destroy_publisher(pub)


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
        semantic.write_text(yaml.safe_dump({"config_state":"VALID","schema_version":1,
            "map_id":"SYNTHETIC_MAP","semantic_version":"synthetic-v1","frame_id":"map","pits":[pit()]}), encoding="utf-8")
        config.update(config_state="VALID",time_domain="ROS_TIME",config_version="synthetic-ci-v1",extrinsic_calibration_id="synthetic-extrinsic",
            synthetic_processing_delay_sec=.01,
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
                    stamp = node.get_clock().now().nanoseconds/1e9-.015
                    points = scene(config["model"],truth,stamp,span=.02)
                    cloud = cloud_message(Cloud(stamp,time.monotonic(),"map",points))
                    meta = EvidenceMetadata()
                    meta.header = cloud.header
                    meta.measurement_stamp = cloud.header.stamp
                    meta.receive_stamp = node.get_clock().now().to_msg()
                    counter += 1
                    meta.source_counter = counter
                    meta.validity,meta.source_type = meta.VALID,meta.SOURCE_SYNTHETIC
                    meta.calibration_id,meta.config_version = "synthetic-extrinsic","synthetic-ci-v1"
                    meta.evidence_age_sec = .015
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
            assert geometry.header.stamp == geometry.evidence.measurement_stamp
            assert clearance.header.stamp == clearance.evidence.measurement_stamp
            assert clearance.map_id == "SYNTHETIC_MAP" and clearance.semantic_version == "synthetic-v1"
            assert clearance.model_id == config["model"]["model_id"]
            assert geometry.publish_latency_sec >= .01
            assert clearance.conservative_clearance_m <= clearance.raw_horizontal_clearance_m
            assert clearance.evidence_age_sec==clearance.evidence.evidence_age_sec
            # Stop all input; timers must expire output rather than heartbeat-refresh evidence.
            deadline = time.monotonic()+1
            while time.monotonic() < deadline:
                rclpy.spin_once(node,timeout_sec=.02)
            assert received["perception/grab_state"].tracking_state == GrabState.TRACK_LOST
            assert received["perception/grab_state"].validity == GrabState.STALE
            assert received["perception/grab_clearance"].validity != GrabClearanceState.VALID
            assert not forbidden
            age_status=validate_clearance_age(node,file,geometry,qos,args.evidence_dir)
            print(json.dumps({"SHADOW_RUNTIME_STATUS":"PASS_SYNTHETIC_ROS",
                "INSTALLED_PREVIEW_STATUS":"PASS_SOFTWARE","INPUT_EXPIRY_STATUS":"PASS",
                "R11_CLEARANCE_AGE_CONTRACT":age_status,
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
