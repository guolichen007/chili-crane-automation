#!/usr/bin/env python3
"""Explicit bounded reads only: FC01 ADAM DI, FC03 pull-wire registers."""
import argparse
import json
import sys
import time
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/chili_crane_control/src"))
sys.path.insert(0, str(ROOT / "src/chili_crane_hardware/src"))
from chili_crane_hardware.adam.adam6052_driver import Adam6052Driver
from chili_crane_hardware.adam.adam6251_driver import Adam6251Driver
from chili_crane_hardware.adam.modbus_tcp_transport import ModbusTcpTransport
from chili_crane_hardware.trolley.pull_wire_driver import PullWireDriver


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("device", choices=["adam", "pullwire"])
    parser.add_argument("--run-readonly", action="store_true", required=True)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--site", type=Path, default=ROOT / "config/sites/crane_01")
    args = parser.parse_args()
    if not 1 <= args.samples <= 200:
        parser.error("samples must be 1..200")
    drivers = {}
    if args.device == "adam":
        for name, kind in (("adam6052", Adam6052Driver), ("adam6251", Adam6251Driver)):
            cfg = yaml.safe_load((args.site / "hardware" / (name + ".yaml")).read_text())
            if cfg.get("physical_output_enabled") is not False:
                raise ValueError("read-only profile required")
            drivers[name] = kind(ModbusTcpTransport(cfg["host"], cfg["port"], cfg["unit_id"], cfg["timeout_sec"]))
    else:
        cfg = yaml.safe_load((args.site / "hardware/pull_wire_y.development.yaml").read_text())
        drivers["pull_wire_y"] = PullWireDriver.from_config(cfg)
    failed = 0
    for sample in range(args.samples):
        for name, driver in drivers.items():
            try:
                values = driver.read_di() if args.device == "adam" else driver.read()
                result = {"device": name, "sample": sample, "valid": True, "data": values}
            except (OSError, ValueError, RuntimeError) as exc:
                failed += 1
                result = {"device": name, "sample": sample, "valid": False, "data": None,
                          "required_action": "STOP", "error_type": type(exc).__name__}
            result.update({"source_type": "PHYSICAL", "no_do_write": True,
                           "calibration_class": "DEVELOPMENT_ONLY" if args.device == "pullwire" else None})
            print(json.dumps(result), flush=True)
        time.sleep(.2)
    return int(failed > 0)


if __name__ == "__main__":
    raise SystemExit(main())
