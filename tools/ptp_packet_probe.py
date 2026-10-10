#!/usr/bin/env python3
"""Bounded AF_PACKET read-only metadata capture; never store payload or change clocks."""
import argparse
from collections import Counter
import json
from pathlib import Path
import socket
import time

NAMES = {0: "Sync", 1: "Delay_Req", 2: "Pdelay_Req", 3: "Pdelay_Resp", 8: "Follow_Up",
         9: "Delay_Resp", 10: "Pdelay_Resp_Follow_Up", 11: "Announce", 12: "Signaling", 13: "Management"}


def ptp_metadata(packet):
    if len(packet) < 14:
        return None
    kind, pos = int.from_bytes(packet[12:14], "big"), 14
    for _ in range(2):
        if kind in (0x8100, 0x88A8):
            if len(packet) < pos + 4:
                return None
            kind, pos = int.from_bytes(packet[pos + 2:pos + 4], "big"), pos + 4
    transport = "L2"
    if kind == 0x0800:
        if len(packet) < pos + 20 or packet[pos] >> 4 != 4 or packet[pos + 9] != 17:
            return None
        ihl = (packet[pos] & 15) * 4
        if ihl < 20 or int.from_bytes(packet[pos + 6:pos + 8], "big") & 0x3FFF:
            return None
        pos += ihl
        if len(packet) < pos + 8 or not ({int.from_bytes(packet[pos:pos + 2], "big"),
                int.from_bytes(packet[pos + 2:pos + 4], "big")} & {319, 320}):
            return None
        pos += 8
        transport = "UDPv4"
    elif kind == 0x86DD:
        if len(packet) < pos + 48 or packet[pos + 6] != 17:
            return None  # no assumption about IPv6 extension header layout
        pos += 40
        if not ({int.from_bytes(packet[pos:pos + 2], "big"), int.from_bytes(packet[pos + 2:pos + 4], "big")} & {319, 320}):
            return None
        pos += 8
        transport = "UDPv6"
    elif kind != 0x88F7:
        return None
    data = packet[pos:]
    if len(data) < 34 or data[1] & 15 != 2 or not 34 <= int.from_bytes(data[2:4], "big") <= len(data):
        return None
    return {"message_type": NAMES.get(data[0] & 15, "UNKNOWN"), "domain": data[4],
            "transport": transport, "source_clock_identity": data[20:28].hex(),
            "sequence_id": int.from_bytes(data[30:32], "big")}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--interface", default="enp3s0")
    p.add_argument("--seconds", type=int, choices=[60, 120], default=60)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        raise SystemExit("refusing overwrite")
    if not hasattr(socket, "AF_PACKET"):
        raise SystemExit("Linux AF_PACKET required; capture NOT_RUN")
    counts, domains, clocks, transports = Counter(), Counter(), Counter(), Counter()
    with socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(3)) as sock:
        sock.bind((a.interface, 0))
        sock.settimeout(.2)
        end = time.monotonic() + a.seconds
        while time.monotonic() < end:
            try:
                item = ptp_metadata(sock.recv(65535))
            except socket.timeout:
                continue
            if item:
                counts[item["message_type"]] += 1
                domains[item["domain"]] += 1
                clocks[item["source_clock_identity"]] += 1
                transports[item["transport"]] += 1
    data = {"counts": {name: counts[name] for name in NAMES.values()}, "domains": dict(domains),
            "source_clock_identities": dict(clocks), "transports": dict(transports),
            "interface": a.interface, "seconds": a.seconds, "clock_sync_state": "PROBING",
            "ptp_verified": False, "delay_mechanism": "NOT_CONFIGURED", "raw_payload_saved": False}
    a.output.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
