"""Raw ADAM-6052 access. All writes require an explicit isolated bench gate."""
from dataclasses import dataclass
from .register_map import Adam6052RegisterMap as Registers
from .modbus_tcp_transport import ModbusError


@dataclass(frozen=True)
class BenchOutputGate:
    physical_output_enabled: bool = False
    isolated_bench_confirmed: bool = False
    adam_fsv_configured: bool = False
    adam_watchdog_configured: bool = False
    adam_safe_output_verified: bool = False

    def require(self):
        if not all(value is True for value in (
                self.physical_output_enabled, self.isolated_bench_confirmed,
                self.adam_fsv_configured, self.adam_watchdog_configured,
                self.adam_safe_output_verified)):
            raise PermissionError("bench output prerequisites are not verified")


class Adam6052Driver:
    def __init__(self, transport, gate=None):
        self.transport = transport
        self.gate = gate or BenchOutputGate()

    def read_di(self):
        return self.transport.read_coils(Registers.DI_OFFSET, Registers.DI_COUNT)

    def read_do(self):
        return self.transport.read_coils(Registers.DO_OFFSET, Registers.DO_COUNT)

    def all_off(self):
        self.gate.require()
        self.transport.write_all_off(Registers.DO_OFFSET, Registers.DO_COUNT)
        if any(self.read_do()):
            raise ModbusError("safe output OFF readback failed")

    def bench_on(self, channel):
        self.gate.require()
        if type(channel) is not int or not 0 <= channel < Registers.DO_COUNT:
            raise ValueError("DO channel must be 0..7")
        if any(self.read_do()):
            raise ModbusError("another output is already ON")
        self.transport.write_single_coil(Registers.DO_OFFSET + channel, True)
        expected = tuple(i == channel for i in range(Registers.DO_COUNT))
        if tuple(self.read_do()) != expected:
            raise ModbusError("single-channel ON readback failed")
