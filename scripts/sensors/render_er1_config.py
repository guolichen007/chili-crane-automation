#!/usr/bin/env python3
"""Render official driver config only from explicitly verified port/clock facts."""
import argparse
from pathlib import Path
import yaml


def render(site):
    lidars = []
    used = set()
    for sensor in ("er1_204", "er1_205"):
        cfg = yaml.safe_load((site / "sensors" / (sensor + ".yaml")).read_text())
        if (cfg.get("driver_enabled") is not True or cfg.get("port_roles") != "VALID"
                or cfg.get("clock_mode") not in {"SENSOR_SYNCHRONIZED", "HOST_RECEIVE"}):
            raise ValueError(sensor + ": port/clock roles not confirmed; driver remains disabled")
        ports = (cfg.get("msop_port"), cfg.get("difop_port"))
        if any(type(p) is not int or not 1 <= p <= 65535 or p in used for p in ports) or ports[0] == ports[1]:
            raise ValueError("invalid/shared UDP port")
        used.update(ports)
        lidars.append({"driver": {"lidar_type": "RSE1", "msop_port": ports[0], "difop_port": ports[1],
            "use_lidar_clock": cfg["clock_mode"] == "SENSOR_SYNCHRONIZED", "dense_points": True,
            "ts_first_point": True}, "ros": {"ros_frame_id": cfg["frame_id"],
            "ros_send_point_cloud_topic": cfg["raw_topic"], "ros_queue_length": 5}})
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
