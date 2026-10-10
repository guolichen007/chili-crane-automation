#!/usr/bin/env python3
"""Bounded normalized ROS2 time probe; CLI thresholds are experiments, not site edits."""
import argparse
import csv
import json
import math
from pathlib import Path
import sys
import time
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src/chili_crane_slam/src"))
from chili_crane_slam.dual_lidar import Cloud, DualLidarSynchronizer
from chili_crane_slam.pointcloud import normalize_cloud, CANONICAL


def quantiles(values, extra="max"):
    if not values:
        return {k: None for k in ("p50", "p95", "p99", extra)}
    result = {"p" + str(q): float(np.percentile(values, q)) for q in (50, 95, 99)}
    result[extra] = float(min(values) if extra == "min" else max(values))
    return result


class TimingProbe:
    def __init__(self, pair_delta, future, stale):
        self.sync = DualLidarSynchronizer(pair_delta, stale, 16, future)
        self.rows, self.pairs = [], []
        self.counts = {s: 0 for s in ("204", "205")}
        self.invalid = {s: 0 for s in ("204", "205")}
        self.previous = {}

    def receive(self, sensor, cloud, now, mono):
        self.counts[sensor] += 1
        f = cloud.timing
        previous = self.previous.get(sensor)
        period = mono - previous if previous is not None else None
        self.previous[sensor] = mono
        if len(self.rows) >= 10000:
            raise ValueError("probe row bound exceeded")
        self.rows.append({"sensor": sensor, "header": f.header, "start": f.start, "end": f.end,
            "mid": f.mid, "span": f.span, "header_to_first": f.start - f.header,
            "header_to_last": f.end - f.header, "receive": now,
            "receive_latency": now - f.end, "hz": 1 / period if period and period > 0 else None})
        # Keep only time endpoints in bounded pairing queues, not large clouds.
        points = np.zeros(2, dtype=CANONICAL)
        points["timestamp"] = [f.start, f.end]
        thin = Cloud(cloud.stamp, cloud.received_monotonic, cloud.frame_id, points, cloud.source_type, now)
        for a, b in self.sync.push(sensor, thin, now, mono):
            overlap, ratio = a.timing.overlap(b.timing)
            self.pairs.append({"header_delta": abs(a.stamp - b.stamp),
                "mid_delta": abs(a.timing.mid - b.timing.mid), "interval_overlap": overlap,
                "overlap_ratio": ratio})

    def summary(self):
        self.sync.expire(float("inf"), float("inf"))  # account for unpaired tail, no reuse
        result = {"scope": "PROBE_ONLY_NOT_PRODUCTION_ACCEPTANCE", "frame_counts": self.counts,
            "paired_count": self.sync.paired_count, "dropped": self.sync.dropped,
            "duplicate_timestamp": self.sync.duplicates, "clock_regression": self.sync.regressions,
            "future_timestamp": self.sync.future_count, "invalid_timestamp": self.invalid,
            "ptp_verified": False, "site_config_modified": False}
        for sensor in self.counts:
            rows = [r for r in self.rows if r["sensor"] == sensor]
            result[sensor] = {k: quantiles([r[k] for r in rows if r[k] is not None])
                              for k in ("hz", "span", "header_to_first", "header_to_last", "receive_latency")}
        result["pair_statistics"] = {k: quantiles([r[k] for r in self.pairs], "min" if "overlap" in k else "max")
                                     for k in ("header_delta", "mid_delta", "interval_overlap", "overlap_ratio")}
        return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--seconds", type=int, choices=[60, 120], default=60)
    p.add_argument("--pair-delta", type=float, required=True)
    p.add_argument("--future-skew", type=float, required=True)
    p.add_argument("--stale", type=float, required=True)
    p.add_argument("--output-dir", type=Path, required=True)
    a = p.parse_args()
    probe = TimingProbe(a.pair_delta, a.future_skew, a.stale)
    if a.output_dir.exists():
        p.error("new output directory required")
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    rclpy.init()
    node = Node("dual_er1_timing_probe")
    def receive(msg, sensor):
        now = node.get_clock().now().nanoseconds / 1e9
        mono = time.monotonic()
        try:
            pts = normalize_cloud(msg.data, [(f.name, f.offset, f.datatype, f.count) for f in msg.fields],
                msg.point_step, msg.row_step, msg.width, msg.height, msg.is_bigendian, 0 if sensor == "204" else 1)
            probe.receive(sensor, Cloud(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                mono, msg.header.frame_id, pts, 1, now), now, mono)
        except (ValueError, TypeError):
            probe.invalid[sensor] += 1
    subscriptions = [node.create_subscription(PointCloud2, "/crane_01/lidar/er1_" + s + "/points",
                     lambda msg, s=s: receive(msg, s), qos_profile_sensor_data) for s in ("204", "205")]
    try:
        end = time.monotonic() + a.seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    result = probe.summary()
    result["experiment_thresholds"] = {"pair_delta": a.pair_delta, "future_skew": a.future_skew, "stale": a.stale}
    result["duration_sec"] = a.seconds
    a.output_dir.mkdir(parents=True)
    (a.output_dir / "timing.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    with (a.output_dir / "frames.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(probe.rows[0]) if probe.rows else ["sensor", "header"])
        writer.writeheader()
        writer.writerows(probe.rows)
    (a.output_dir / "报告.md").write_text("# 双 ER1 时间探测\n\n仅实验统计，不证明生产同步；配置未修改。\n\n```json\n"
        + json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n```\n", encoding="utf-8")
    return 0 if all(probe.counts.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
