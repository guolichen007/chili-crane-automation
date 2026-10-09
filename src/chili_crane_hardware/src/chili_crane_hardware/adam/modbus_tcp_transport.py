"""Small synchronous Modbus TCP transport with bounded request deadlines."""
import math
from contextlib import contextmanager, nullcontext
from .session import device_lock
import socket
import struct
import threading
import time


class ModbusError(RuntimeError):
    pass


def unpack_bits(response, function, count):
    byte_count = (count + 7) // 8
    if len(response) != byte_count + 2 or response[:2] != bytes((function, byte_count)):
        raise ModbusError("invalid bit count or function")
    return tuple(bool(response[2 + i // 8] & (1 << (i % 8))) for i in range(count))


class ModbusTcpTransport:
    def __init__(self, host, port, unit_id, timeout_sec=0.5, connector=None):
        if not isinstance(host, str) or host.strip() in {"", "NOT_CONFIGURED"}:
            raise ValueError("host is NOT_CONFIGURED")
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError("invalid TCP port")
        if type(unit_id) is not int or not 0 <= unit_id <= 255:
            raise ValueError("unit_id is NOT_CONFIGURED")
        if not math.isfinite(timeout_sec) or not 0 < timeout_sec <= 2.0:
            raise ValueError("timeout must be finite and <=2 seconds")
        self.host, self.port, self.unit_id = host, port, unit_id
        self.timeout_sec = timeout_sec
        self._connector = connector or socket.create_connection
        self._transaction = 0
        self._lock = threading.Lock()
        self._exclusive_session = False
        self._simulated = connector is not None

    @staticmethod
    def _receive(sock, size, deadline):
        chunks = bytearray()
        while len(chunks) < size:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ModbusError("response timeout")
            sock.settimeout(remaining)
            data = sock.recv(size - len(chunks))
            if not data:
                raise ModbusError("connection closed before complete response")
            chunks.extend(data)
        return bytes(chunks)

    @contextmanager
    def exclusive_session(self):
        if self._exclusive_session:
            raise RuntimeError("nested exclusive session")
        with device_lock(self.host, self.port, self.unit_id, exclusive=True):
            self._exclusive_session = True
            try:
                yield
            finally:
                self._exclusive_session = False

    def request(self, pdu):
        guard = nullcontext() if self._exclusive_session or self._simulated else device_lock(
            self.host, self.port, self.unit_id)
        with self._lock, guard:
            self._transaction = (self._transaction + 1) & 65535
            transaction = self._transaction
            deadline = time.monotonic() + self.timeout_sec
            frame = struct.pack(">HHHB", transaction, 0, len(pdu) + 1, self.unit_id) + pdu
            try:
                with self._connector((self.host, self.port), timeout=self.timeout_sec) as sock:
                    sock.settimeout(max(0.001, deadline - time.monotonic()))
                    sock.sendall(frame)
                    header = self._receive(sock, 7, deadline)
                    tid, protocol, length, unit = struct.unpack(">HHHB", header)
                    if tid != transaction or protocol != 0 or unit != self.unit_id or not 2 <= length <= 254:
                        raise ModbusError("invalid MBAP identity or length")
                    response = self._receive(sock, length - 1, deadline)
            except (OSError, TimeoutError) as exc:
                raise ModbusError("Modbus TCP communication failed: " + str(exc)) from exc
            if response[0] == (pdu[0] | 128):
                raise ModbusError("Modbus exception: " + response.hex())
            if response[0] != pdu[0]:
                raise ModbusError("response function mismatch")
            return response

    def read_coils(self, offset, count):
        if type(offset) is not int or type(count) is not int or not (
                0 <= offset <= 65535 and 1 <= count <= 2000 and offset + count <= 65536):
            raise ValueError("invalid read coil range")
        return unpack_bits(self.request(struct.pack(">BHH", 1, offset, count)), 1, count)

    def write_single_coil(self, offset, enabled):
        if type(offset) is not int or not 0 <= offset <= 65535 or type(enabled) is not bool:
            raise ValueError("invalid coil write")
        request = struct.pack(">BHH", 5, offset, 0xFF00 if enabled else 0)
        if self.request(request) != request:
            raise ModbusError("write acknowledgement mismatch")

    def write_all_off(self, offset, count):
        if not 1 <= count <= 1968 or not 0 <= offset or offset + count > 65536:
            raise ValueError("invalid OFF range")
        payload = bytes((count + 7) // 8)
        request = struct.pack(">BHHB", 15, offset, count, len(payload)) + payload
        if self.request(request) != struct.pack(">BHH", 15, offset, count):
            raise ModbusError("OFF acknowledgement mismatch")
