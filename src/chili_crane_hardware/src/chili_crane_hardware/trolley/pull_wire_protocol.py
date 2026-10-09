"""Pure read-only Modbus RTU framing and explicit register decoding."""
import math
import struct
import time
from dataclasses import dataclass


def crc16(payload):
    crc = 0xFFFF
    for byte in payload:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


DATA_FORMATS = {"uint16": ("H", 1), "int16": ("h", 1),
                "uint32": ("I", 2), "int32": ("i", 2), "float32": ("f", 2)}


@dataclass(frozen=True)
class PullWireProtocol:
    serial_device: str
    baudrate: int
    parity: str
    stopbits: int
    slave_id: int
    function_code: int
    register_offset: int
    data_type: str
    byte_order: str
    word_order: str
    timeout_sec: float = 0.5

    def validate(self):
        if not isinstance(self.serial_device, str) or not self.serial_device.startswith("/dev/serial/by-id/"):
            raise ValueError("serial_device must use confirmed /dev/serial/by-id path")
        if type(self.baudrate) is not int or self.baudrate <= 0:
            raise ValueError("baudrate is NOT_CONFIGURED")
        if self.parity not in {"N", "E", "O"} or self.stopbits not in {1, 2}:
            raise ValueError("invalid serial framing")
        if type(self.slave_id) is not int or not 1 <= self.slave_id <= 247:
            raise ValueError("slave_id is NOT_CONFIGURED")
        if self.function_code not in {3, 4}:
            raise ValueError("read function_code is NOT_CONFIGURED")
        if type(self.register_offset) is not int or not 0 <= self.register_offset <= 65535:
            raise ValueError("register_offset is NOT_CONFIGURED")
        if self.data_type not in DATA_FORMATS:
            raise ValueError("data_type is NOT_CONFIGURED")
        if self.byte_order not in {"big", "little"} or self.word_order not in {"big", "little"}:
            raise ValueError("byte/word order is NOT_CONFIGURED")
        if not math.isfinite(self.timeout_sec) or not 0 < self.timeout_sec <= 2:
            raise ValueError("invalid read timeout")
        if self.register_offset + DATA_FORMATS[self.data_type][1] > 65536:
            raise ValueError("register range overflow")

    @property
    def register_count(self):
        return DATA_FORMATS[self.data_type][1]

    def decode(self, registers):
        self.validate()
        if len(registers) != self.register_count or any(
                type(value) is not int or not 0 <= value <= 65535 for value in registers):
            raise ValueError("invalid register response")
        words = [struct.pack(">H", value) for value in registers]
        if self.byte_order == "little":
            words = [word[::-1] for word in words]
        if self.word_order == "little":
            words.reverse()
        value = struct.unpack(">" + DATA_FORMATS[self.data_type][0], b"".join(words))[0]
        if not math.isfinite(value):
            raise ValueError("non-finite sensor value")
        return value


class ModbusRtuTransport:
    def __init__(self, protocol, serial_factory=None):
        protocol.validate()
        self.protocol = protocol
        self.serial_factory = serial_factory

    def read_registers(self):
        p = self.protocol
        request = struct.pack(">BBHH", p.slave_id, p.function_code,
                              p.register_offset, p.register_count)
        request += struct.pack("<H", crc16(request))
        if self.serial_factory is None:
            import serial
            factory = serial.Serial
        else:
            factory = self.serial_factory
        expected = 5 + 2 * p.register_count
        with factory(port=p.serial_device, baudrate=p.baudrate, parity=p.parity,
                     stopbits=p.stopbits, bytesize=8, timeout=p.timeout_sec,
                     write_timeout=p.timeout_sec) as port:
            port.reset_input_buffer()
            # RTU silent interval; this driver opens a fresh read-only session.
            time.sleep(max(0.002, 3.5 * 11 / p.baudrate))
            if port.write(request) != len(request):
                raise RuntimeError("short serial write")
            response = bytearray()
            deadline = time.monotonic() + p.timeout_sec
            while len(response) < expected:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("RTU response timeout")
                port.timeout = remaining
                response.extend(port.read(expected - len(response)))
                if len(response) >= 2 and response[1] & 128:
                    expected = 5
            if crc16(response[:-2]) != int.from_bytes(response[-2:], "little"):
                raise ValueError("RTU CRC mismatch")
            if response[0] != p.slave_id:
                raise ValueError("RTU slave mismatch")
            if response[1] & 128:
                raise RuntimeError("RTU exception: " + bytes(response).hex())
            if response[1] != p.function_code or response[2] != 2 * p.register_count:
                raise ValueError("RTU function/byte-count mismatch")
            return struct.unpack(">" + "H" * p.register_count, response[3:-2])
