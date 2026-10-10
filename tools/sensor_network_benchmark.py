#!/usr/bin/env python3
"""Read-only NIC/CPU/memory/UDP and optional ROS camera diagnostics, bounded 120s."""
import argparse
from collections import Counter
import importlib.util
import json
from pathlib import Path
import socket
import subprocess
import time
import os

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("udp_probe", ROOT / "scripts/sensors/er1_udp_probe.py")
UDP = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(UDP)


def snapshot(iface):
    root = Path("/sys/class/net") / iface / "statistics"
    nic = {k: int((root / k).read_text()) for k in
           ("rx_bytes", "rx_packets", "rx_dropped", "rx_missed_errors", "rx_fifo_errors")}
    return {"nic": nic, "cpu_stat": Path("/proc/stat").read_text().splitlines()[0],
            "memory": Path("/proc/meminfo").read_text()}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--interface", choices=["enp3s0"], default="enp3s0")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--camera-topic", default="/crane_01/camera/hik_01/diagnostics")
    a = p.parse_args()
    if a.output.exists():
        p.error("refusing overwrite")
    before, counts, samples, camera = snapshot(a.interface), Counter(), [], []
    ethtool = subprocess.run(["ethtool", "-S", a.interface], capture_output=True, text=True, timeout=5)
    node = None
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        rclpy.init()
        node = Node("sensor_network_benchmark")
        def receive(msg):
            try:
                data = json.loads(msg.data)
                camera.append({"receive": time.time(), "frame_number": data.get("frame_number"),
                    "frame_rate_hz": data.get("frame_rate_hz"), "lost_packets": data.get("lost_packets")})
                del camera[:-1000]
            except (ValueError, TypeError):
                pass
        sub = node.create_subscription(String, a.camera_topic, receive, 10)
    except ImportError:
        pass
    start, next_sample = time.monotonic(), 0
    try:
        with socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(3)) as sock:
            sock.bind((a.interface, 0))
            sock.settimeout(.01)
            while time.monotonic() - start < 120:
                try:
                    item = UDP.udp_metadata(sock.recv(65535))
                    if item and item[0] in {"192.168.1.204", "192.168.1.205"}:
                        counts[item[0]] += 1
                except socket.timeout:
                    pass
                if node:
                    rclpy.spin_once(node, timeout_sec=0)
                if time.monotonic() >= next_sample:
                    samples.append(snapshot(a.interface))
                    next_sample = time.monotonic() + 1
    finally:
        if node:
            node.destroy_node()
            rclpy.shutdown()
    after = snapshot(a.interface)
    elapsed = time.monotonic() - start
    result = {"scope": "READ_ONLY_NETWORK_BENCHMARK", "seconds": elapsed, "before": before, "after": after,
        "nic_delta": {k: after["nic"][k] - v for k, v in before["nic"].items()}, "samples": samples,
        "ethtool_S": ethtool.stdout, "ethtool_error": ethtool.stderr,
        "udp_packet_rates": {ip: counts[ip] / elapsed for ip in ("192.168.1.204", "192.168.1.205")},
        "camera_samples": camera, "camera_status": "OBSERVED" if camera else "NOT_CONNECTED",
        "RMW_IMPLEMENTATION": os.environ.get("RMW_IMPLEMENTATION", "DEFAULT_NOT_REPORTED"), "system_modified": False}
    a.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
