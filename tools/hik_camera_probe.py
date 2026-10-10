#!/usr/bin/env python3
"""MVS node access/current value and timestamp evidence probe; never auto-calibrates units."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/chili_crane_hardware/src"))
from chili_crane_hardware.hik_mvs import MvsCamera


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--seconds", type=int, choices=[10, 60, 120], default=10)
    p.add_argument("--capture-frames", action="store_true")
    p.add_argument("--force-slave-only", action="store_true", help="explicit owner-authorized camera setting write")
    a = p.parse_args()
    if a.output.exists():
        p.error("refusing overwrite")
    cfg = yaml.safe_load(a.config.read_text())
    camera = MvsCamera(cfg)
    result = {"features": camera.probe(), "timestamp_units_verified": False,
        "config_sha256": hashlib.sha256(a.config.read_bytes()).hexdigest(),
        "camera": {"serial": camera.serial, "model": camera.model, "ip": camera.ip}, "samples": []}
    try:
        if a.force_slave_only:
            ret = camera.cam.MV_CC_SetBoolValue("GevIEEE1588SlaveOnly", True)
            result["slave_only_force"] = "WRITE_ACCEPTED" if ret == 0 else "SLAVE_ONLY_FORCE_UNSUPPORTED"
            result["features_after_write"] = camera.probe()
        if a.capture_frames:
            camera.start()
            end = time.monotonic() + a.seconds
            while time.monotonic() < end:
                frame = camera.frame(100)
                if frame:
                    metadata, _, encoding = frame
                    metadata.update(receive_CLOCK_REALTIME_sec=time.time(), receive_monotonic_sec=time.monotonic(),
                                    pixel_encoding=encoding)
                    result["samples"].append(metadata)
        result["features_final"] = camera.probe()
        result["conclusion"] = "MEASUREMENT_ONLY_UNIT_EPOCH_AND_PTP_REQUIRE_REVIEW"
    finally:
        camera.close()
    a.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
