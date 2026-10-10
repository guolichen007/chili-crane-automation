#!/usr/bin/env python3
"""Linux AF_PACKET read-only UDP statistics; no packet decoder and no payload export."""
import argparse
import collections
import json
import math
import socket
import struct
import time


def udp_metadata(frame):
    if len(frame) < 14:
        return None
    offset, ethertype = 14, int.from_bytes(frame[12:14], "big")
    while ethertype in (0x8100, 0x88a8):
        if len(frame) < offset + 4:
            return None
        ethertype = int.from_bytes(frame[offset + 2:offset + 4], "big")
        offset += 4
    if ethertype != 0x0800 or len(frame) < offset + 20:
        return None
    ip = frame[offset:]
    ihl = (ip[0] & 15) * 4
    total = int.from_bytes(ip[2:4], "big")
    if (ip[0] >> 4 != 4 or ihl < 20 or len(ip) < total or total < ihl + 8
            or ip[9] != 17 or int.from_bytes(ip[6:8], "big") & 0x3fff):
        return None
    src_port, dst_port, length, _ = struct.unpack("!HHHH", ip[ihl:ihl + 8])
    if length < 8 or ihl + length > total:
        return None
    return socket.inet_ntoa(ip[12:16]), dst_port, length - 8


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="enp3s0", choices=["enp3s0"])
    parser.add_argument("--duration", type=float, default=15)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 300:
        parser.error("duration must be 1..300s")
    stats = collections.defaultdict(collections.Counter)
    start = time.monotonic()
    with socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0003)) as capture:
        capture.bind((args.interface, 0))
        capture.settimeout(.2)
        while time.monotonic() - start < args.duration:
            try:
                metadata = udp_metadata(capture.recv(65535))
            except socket.timeout:
                continue
            if metadata and metadata[0] in {"192.168.1.204", "192.168.1.205"}:
                source, port, length = metadata
                stats[(source, port)][length] += 1
    elapsed = time.monotonic() - start
    print(json.dumps({"scope": "LIVE_UDP_STATISTICS_ONLY", "port_roles": "UNCONFIRMED",
        "duration_sec": elapsed, "streams": [{"source_ip": ip, "destination_port": port,
        "packet_count": sum(sizes.values()), "packet_rate_hz": sum(sizes.values()) / elapsed,
        "payload_length_counts": dict(sizes)} for (ip, port), sizes in sorted(stats.items())]}, indent=2))
    return 0 if {ip for ip, _ in stats} == {"192.168.1.204", "192.168.1.205"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
