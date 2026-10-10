#!/usr/bin/env python3
"""Interpret ethtool -T conservatively; a PHC alone is not complete HW timestamp proof."""
import argparse
import json
from pathlib import Path
import re
import subprocess


def parse(text, success=True):
    match = re.search(r"PTP Hardware Clock:\s*(-?\d+)", text)
    phc = int(match.group(1)) if match else None
    tx = "hardware-transmit" in text
    rx = "hardware-receive" in text
    raw = "hardware-raw-clock" in text
    supported = bool(success and phc is not None and phc >= 0 and tx and rx and raw)
    return {"hardware_timestamp_supported": supported if success and phc is not None else None,
            "phc_index": phc, "hardware_tx": tx, "hardware_rx": rx, "raw_clock": raw,
            "preferred_timestamping": "hardware" if supported else "REQUIRES_OWNER_REVIEW",
            "clock_sync_state": "NOT_CONFIGURED", "ptp_verified": False}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--interface", default="enp3s0")
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        p.error("refusing overwrite")
    proc = subprocess.run(["ethtool", "-T", a.interface], capture_output=True, text=True, timeout=5)
    result = parse(proc.stdout, proc.returncode == 0)
    result.update(interface=a.interface, ethtool_T=proc.stdout, error=proc.stderr)
    a.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
