"""Read-only RTU probe; never scans guessed slave IDs or registers."""
import argparse
import json
from chili_crane_hardware.trolley.pull_wire_protocol import PullWireProtocol, ModbusRtuTransport
from chili_crane_hardware.trolley.calibration import PullWireCalibration
from .common import load_config


def main():
    parser = argparse.ArgumentParser(description="Read confirmed pull-wire RTU registers")
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    config = load_config(args.config)
    protocol = PullWireProtocol(**config["protocol_config"])
    protocol.validate()
    registers = ModbusRtuTransport(protocol).read_registers()
    raw = protocol.decode(registers)
    result = {"registers": registers, "raw": raw, "position_status": "NOT_CONFIGURED"}
    try:
        calibration = PullWireCalibration(**config["calibration"])
        result["position_m"] = calibration.position(raw)
        result["position_status"] = "CALIBRATED_SAMPLE"
    except (ValueError, TypeError, KeyError):
        pass
    print(json.dumps(result))


if __name__ == "__main__":
    main()
