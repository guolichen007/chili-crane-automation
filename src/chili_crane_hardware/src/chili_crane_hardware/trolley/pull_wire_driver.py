"""Sensor framework activated only by a complete explicit protocol/calibration."""
from .pull_wire_protocol import PullWireProtocol, ModbusRtuTransport
from .calibration import PullWireCalibration


class PullWireDriver:
    def __init__(self, protocol, calibration, transport=None):
        protocol.validate()
        calibration.validate()
        self.protocol, self.calibration = protocol, calibration
        self.transport = transport or ModbusRtuTransport(protocol)

    @classmethod
    def from_config(cls, config):
        protocol = PullWireProtocol(**config["protocol_config"])
        calibration = PullWireCalibration(**config["calibration"])
        return cls(protocol, calibration)

    def read(self):
        raw = self.protocol.decode(self.transport.read_registers())
        return raw, self.calibration.position(raw)
