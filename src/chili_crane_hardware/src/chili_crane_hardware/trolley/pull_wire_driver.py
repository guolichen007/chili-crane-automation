"""Sensor framework activated only by a complete explicit protocol/calibration."""
from .pull_wire_protocol import PullWireProtocol, ModbusRtuTransport
from .calibration import PullWireCalibration
from .modulo import ModuloPosition
import time


class PullWireDriver:
    def __init__(self, protocol, calibration, transport=None, max_delta_per_sample=None,
                 max_gap_sec=1.0):
        protocol.validate()
        calibration.validate()
        self.protocol, self.calibration = protocol, calibration
        self.transport = transport or ModbusRtuTransport(protocol)
        self.position_tracker = (ModuloPosition(calibration.raw_modulus, calibration.zero_raw,
                                 max_delta_per_sample, max_gap_sec)
                                 if calibration.raw_modulus is not None else None)

    @classmethod
    def from_config(cls, config):
        protocol = PullWireProtocol(**config["protocol_config"])
        calibration = PullWireCalibration(**config["calibration"])
        return cls(protocol, calibration,
                   max_delta_per_sample=config.get("max_delta_per_sample"),
                   max_gap_sec=config.get("stale_timeout_sec", 1.0))

    def read(self, observed=None):
        raw = self.protocol.decode(self.transport.read_registers())
        if self.position_tracker is None:
            return raw, self.calibration.position(raw)
        relative = self.position_tracker.update(
            raw, time.monotonic() if observed is None else observed,
            self.calibration.position_relative)
        return raw, self.calibration.position_relative(relative)
