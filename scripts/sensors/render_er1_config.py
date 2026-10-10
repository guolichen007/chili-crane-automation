#!/usr/bin/env python3
"""Render official driver config only from explicitly verified port/clock facts."""
import argparse
from pathlib import Path
import yaml
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src/chili_crane_slam/src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src/chili_crane_hardware/src"))
from chili_crane_slam.timebase import ClockContract
from chili_crane_hardware.er1_transport import driver_transport


def render(site):
    lidars = []
    used = set()
    clocks = []
    for sensor in ("er1_204", "er1_205"):
        cfg = yaml.safe_load((site / "sensors" / (sensor + ".yaml")).read_text())
        if (cfg.get("driver_enabled") is not True or cfg.get("port_roles") != "VALID"
                or cfg.get("clock_mode") not in {"SENSOR_PTP", "HOST_DERIVED"}):
            raise ValueError(sensor + ": port/clock roles not confirmed; driver remains disabled")
        clocks.append(ClockContract.from_config(cfg))
        ports = (cfg.get("msop_port"), cfg.get("difop_port"))
        if any(type(p) is not int or not 1 <= p <= 65535 or p in used for p in ports) or ports[0] == ports[1]:
            raise ValueError("invalid/shared UDP port")
        used.update(ports)
        lidars.append({"driver": {**driver_transport(cfg), "lidar_type": "RSE1", "msop_port": ports[0], "difop_port": ports[1],
            "use_lidar_clock": cfg["clock_mode"] == "SENSOR_PTP", "dense_points": True,
            "ts_first_point": True}, "ros": {"ros_frame_id": cfg["frame_id"],
            "ros_send_point_cloud_topic": cfg["raw_topic"], "ros_queue_length": 5}})
    if not clocks[0].compatible(clocks[1]):
        raise ValueError("MIXED_CLOCK_MODE_OR_DOMAIN")
    return {"common": {"msg_source": 1, "send_packet_ros": False, "send_point_cloud_ros": True},
            "lidar": lidars}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--site", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = render(args.site)
    if args.output.exists():
        raise SystemExit("refusing to overwrite existing driver config")
    args.output.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
