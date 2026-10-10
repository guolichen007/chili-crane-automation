#!/usr/bin/env python3
"""Phase1B requested timebase plus separate measured snapshots; no inferred PTP success."""
import argparse
import os
from pathlib import Path
import yaml
from phase1_manifest import ROOT, create, finalize

TOPICS = ["/crane_01/lidar/er1_204/vendor_points", "/crane_01/lidar/er1_205/vendor_points",
    "/crane_01/lidar/er1_204/points", "/crane_01/lidar/er1_205/points",
    "/crane_01/lidar/er1_204/timing", "/crane_01/lidar/er1_205/timing",
    "/crane_01/lidar/dual_lidar/timing", "/crane_01/lidar/dual_lidar/diagnostics",
    "/crane_01/lidar/dual_lidar/readiness", "/crane_01/camera/hik_01/image_raw",
    "/crane_01/camera/hik_01/camera_info", "/crane_01/camera/hik_01/timing",
    "/crane_01/camera/hik_01/diagnostics", "/crane_01/system/run_manifest"]


def timebase(site, ptp_snapshot=None, camera_snapshot=None):
    lidar = {}
    for sensor in ("er1_204", "er1_205"):
        cfg = yaml.safe_load((site / "sensors" / (sensor + ".yaml")).read_text())
        lidar[sensor] = {k: cfg.get(k) for k in ("clock_mode", "clock_sync_state", "clock_domain",
            "time_evidence_id", "stamp_basis", "host_clock_relation", "host_clock_relation_evidence_id",
            "host_clock_domain", "transport_mode", "destination_address", "host_address", "group_address")}
        lidar[sensor]["use_lidar_clock"] = (True if cfg["clock_mode"] == "SENSOR_PTP" else
                                             False if cfg["clock_mode"] == "HOST_DERIVED" else None)
    cfg = yaml.safe_load((site / "sensors/hik_01.yaml").read_text())
    ptp = {"interface": "enp3s0", "hardware_timestamp_supported": None,
        "transport": "NOT_CONFIGURED", "delay_mechanism": "NOT_CONFIGURED", "domain": None,
        "gm_identity": "NOT_CONFIGURED", "state": "NOT_CONFIGURED", "ptp_verified": False}
    camera = {"requested_timestamp_mode": cfg["timestamp_mode"], "actual_timestamp_mode": "NOT_RUN",
        "expected_ip": cfg["expected_ip"], "expected_serial": cfg["expected_serial"],
        "network_state": cfg.get("network_state", "NOT_CONFIGURED"),
        "ptp_state": "NOT_RUN", "exposure": None, "fps": None, "pixel_format": "NOT_RUN",
        "shutter_type": cfg["shutter_type"], "timing_model": cfg["timing_model"], "camera_control_authority": False}
    return {"lidars": lidar, "ptp": ptp, "camera": camera,
        "ptp_measurement_snapshot": ptp_snapshot, "camera_measurement_snapshot": camera_snapshot,
        "RMW_IMPLEMENTATION": os.environ.get("RMW_IMPLEMENTATION", "DEFAULT_NOT_REPORTED"),
        "physical_output_enabled": False, "automatic_control_enabled": False, "spatial_merge_enabled": False}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("mode", choices=["create", "finalize"])
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--site", type=Path, default=ROOT / "config/sites/crane_01")
    p.add_argument("--source", choices=["PHYSICAL", "SIMULATED", "REPLAY", "SYNTHETIC"])
    p.add_argument("--scene", default="NOT_REPORTED")
    p.add_argument("--ptp-snapshot", type=Path)
    p.add_argument("--camera-snapshot", type=Path)
    a = p.parse_args()
    if a.mode == "create":
        snapshots = [yaml.safe_load(path.read_text()) if path else None for path in (a.ptp_snapshot, a.camera_snapshot)]
        create(a.site, a.output, a.source, a.scene,
               {"phase": "1B", "timebase": timebase(a.site, *snapshots), "required_topics": TOPICS})
    elif not finalize(a.output):
        raise SystemExit("capture incomplete; see per-topic counts; no live acceptance inferred")


if __name__ == "__main__":
    main()
