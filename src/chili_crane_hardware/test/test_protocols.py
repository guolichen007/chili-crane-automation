"""Protocol, polarity, freshness and output tests using only simulated transports."""
import math
import struct
import unittest
from dataclasses import replace
from chili_crane_hardware.adam.register_map import (
    coil_offset, Adam6052RegisterMap as A52, Adam6251RegisterMap as A51,
)
from chili_crane_hardware.adam.modbus_tcp_transport import (
    ModbusTcpTransport, ModbusError, unpack_bits,
)
from chili_crane_hardware.adam.adam6052_driver import Adam6052Driver, BenchOutputGate
from chili_crane_hardware.adam.adam6251_driver import Adam6251Driver
from chili_crane_hardware.mapping import RawInput, normalize
from chili_crane_hardware.state import EvidenceTracker
from chili_crane_hardware.bench.adam_do_test import pulse_once
from chili_crane_hardware.trolley.pull_wire_protocol import PullWireProtocol, crc16, ModbusRtuTransport
from chili_crane_hardware.trolley.calibration import PullWireCalibration
from chili_crane_hardware.trolley.pull_wire_driver import PullWireDriver
from chili_crane_control.mode_policy import evaluate_mode


class FakeCoils:
    def __init__(self):
        self.outputs = [False] * 8
        self.calls = []

    def read_coils(self, offset, count):
        self.calls.append(("read", offset, count))
        if offset == 16:
            return tuple(self.outputs)
        return (False,) * count

    def write_all_off(self, offset, count):
        self.calls.append(("off", offset, count))
        self.outputs = [False] * 8

    def write_single_coil(self, offset, value):
        self.calls.append(("on", offset, value))
        self.outputs[offset - 16] = value


GATE = BenchOutputGate(True, True, True, True, True)


class AddressAndOutputTest(unittest.TestCase):
    def test_1based_manual_to_0based_offset(self):
        self.assertEqual((0, 16, 8, 16), (A52.DI_OFFSET, A52.DO_OFFSET, A52.DI_COUNT, A51.DI_COUNT))
        self.assertEqual(23, coil_offset(24))
        for value in (0, -1, 65537, True, "00001"):
            with self.assertRaises(ValueError):
                coil_offset(value)

    def test_all_24_channels_are_read(self):
        transport = FakeCoils()
        self.assertEqual(8, len(Adam6052Driver(transport).read_di()))
        self.assertEqual(16, len(Adam6251Driver(transport).read_di()))
        self.assertEqual([("read", 0, 8), ("read", 0, 16)], transport.calls)

    def test_default_gate_prevents_every_write(self):
        transport = FakeCoils()
        driver = Adam6052Driver(transport)
        for action in (driver.all_off, lambda: driver.bench_on(0)):
            with self.assertRaises(PermissionError):
                action()
        self.assertEqual([], transport.calls)

    def test_partial_gate_never_enables_output(self):
        for field in GATE.__dataclass_fields__:
            with self.assertRaises(PermissionError):
                replace(GATE, **{field: False}).require()

    def test_pulse_cleans_up_success_and_exception(self):
        for exc in (None, RuntimeError("failure"), KeyboardInterrupt("signal")):
            transport = FakeCoils()
            def sleeper(duration):
                self.assertEqual(0.5, duration)
                if exc:
                    raise exc
            driver = Adam6052Driver(transport, GATE)
            if exc:
                with self.assertRaises(type(exc)):
                    pulse_once(driver, 3, 0.5, sleeper)
            else:
                pulse_once(driver, 3, 0.5, sleeper)
            self.assertEqual(("on", 19, True), transport.calls[3])
            self.assertFalse(any(transport.outputs))
            self.assertEqual(1, sum(call[0] == "on" for call in transport.calls))

    def test_invalid_pulse_is_rejected_before_io(self):
        for duration in (0, -1, 2, math.nan, math.inf):
            transport = FakeCoils()
            with self.assertRaises(ValueError):
                pulse_once(Adam6052Driver(transport, GATE), 0, duration)
            self.assertEqual([], transport.calls)

    def test_another_on_channel_blocks_new_output(self):
        transport = FakeCoils()
        transport.outputs[1] = True
        with self.assertRaises(ModbusError):
            Adam6052Driver(transport, GATE).bench_on(0)
        self.assertFalse(any(call[0] == "on" for call in transport.calls))

    def test_cleanup_failure_is_visible(self):
        class BrokenOff(FakeCoils):
            def write_all_off(self, offset, count):
                raise ModbusError("link unavailable; OFF unverified")
        with self.assertRaisesRegex(ModbusError, "OFF unverified"):
            pulse_once(Adam6052Driver(BrokenOff(), GATE), 0, 0.5, lambda _: None)


class FakeSocket:
    def __init__(self, mutate=None):
        self.response = bytearray()
        self.mutate = mutate
        self.sent = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def settimeout(self, timeout):
        pass

    def sendall(self, frame):
        self.sent = frame
        tid, protocol, length, unit = struct.unpack(">HHHB", frame[:7])
        pdu = b"\x01\x01\x81"
        self.response = bytearray(struct.pack(">HHHB", tid, protocol, len(pdu) + 1, unit) + pdu)
        if self.mutate:
            self.response = bytearray(self.mutate(bytes(self.response)))

    def recv(self, size):
        size = min(size, 2)  # Force fragmented MBAP/PDU receives.
        data = bytes(self.response[:size])
        del self.response[:size]
        return data


class TcpTransportTest(unittest.TestCase):
    def test_fragmented_response_and_lsb_bit_order(self):
        sock = FakeSocket()
        transport = ModbusTcpTransport("bench", 502, 1, connector=lambda *a, **k: sock)
        self.assertEqual((True, False, False, False, False, False, False, True),
                         transport.read_coils(0, 8))
        self.assertEqual(b"\x01\x00\x00\x00\x08", sock.sent[7:])

    def test_bad_transaction_is_rejected(self):
        sock = FakeSocket(lambda data: b"\xff\xff" + data[2:])
        with self.assertRaisesRegex(ModbusError, "MBAP"):
            ModbusTcpTransport("bench", 502, 1, connector=lambda *a, **k: sock).read_coils(0, 8)

    def test_bad_byte_count_is_rejected(self):
        with self.assertRaises(ModbusError):
            unpack_bits(b"\x01\x02\x01", 1, 8)

    def test_closed_connection_is_rejected(self):
        sock = FakeSocket(lambda data: data[:8])
        with self.assertRaisesRegex(ModbusError, "closed"):
            ModbusTcpTransport("bench", 502, 1, connector=lambda *a, **k: sock).read_coils(0, 8)

    def test_modbus_exception_is_not_bit_data(self):
        sock = FakeSocket(lambda data: data[:4] + b"\x00\x03" + data[6:7] + b"\x81\x02")
        with self.assertRaisesRegex(ModbusError, "exception"):
            ModbusTcpTransport("bench", 502, 1, connector=lambda *a, **k: sock).read_coils(0, 8)


class InputAndModeTest(unittest.TestCase):
    def test_per_device_polarity_and_all_channels(self):
        mapping = {
            "limit": {"device": "adam6052", "channel": 0, "invert": True},
            "remote": {"device": "adam6251", "channel": 15, "invert": False},
        }
        samples = {
            "adam6052": RawInput((False,) * 8, 10, 0, True),
            "adam6251": RawInput((True,) * 16, 10, 0, True),
        }
        self.assertEqual({"limit": True, "remote": True}, normalize(mapping, samples, 10.1, 1))

    def test_unknown_stale_or_failed_input_is_none(self):
        mapping = {"safety": {"device": "adam6251", "channel": 0, "invert": False}}
        for sample in (
            RawInput((True,) * 16, 10, 0, False),
            RawInput((True,) * 16, 10, 1, True),
            RawInput((True,) * 16, 11, 0, True),
            RawInput((True,) * 16, 10, math.nan, True),
            RawInput((True,) * 8, 10, 0, True),
        ):
            self.assertIsNone(normalize(mapping, {"adam6251": sample}, 10.2, 1)["safety"])
        mapping["safety"]["invert"] = "NOT_CONFIGURED"
        self.assertIsNone(normalize(mapping, {}, 10.2, 1)["safety"])

    def test_poll_failure_does_not_refresh_evidence(self):
        tracker = EvidenceTracker()
        tracker.accept((True,), now=10)
        tracker.failed("disconnected")
        self.assertEqual(10, tracker.sample.observed_monotonic)
        self.assertEqual("DEGRADED", tracker.validity(1, 10.5))
        self.assertEqual("STALE", tracker.validity(1, 12))

    def test_remote_auto_handover_always_discards_prior_commands(self):
        for args, mode in (
            ((False, True, False, True), "REMOTE"),
            ((True, True, True, True), "CONFLICT"),
            ((True, True, False, True), "AUTO_PENDING"),
            ((True, False, False, True), "BLOCKED"),
            ((True, True, False, False), "BLOCKED"),
        ):
            decision = evaluate_mode(*args)
            self.assertEqual(mode, decision.mode)
            self.assertTrue(decision.release_automatic_outputs)
            self.assertTrue(decision.discard_pending_commands)
        self.assertEqual("AUTO_READY", evaluate_mode(True, True, False, True, True, True).mode)


PROTOCOL = PullWireProtocol("/dev/serial/by-id/test", 9600, "N", 1, 1, 3, 0,
                            "uint16", "big", "big")
CALIBRATION = PullWireCalibration("test", 0.001, 0.0, 1, 0.0, 0.0, 10.0)


class PullWireTest(unittest.TestCase):
    def test_crc_standard_request(self):
        self.assertEqual(0x0A84, crc16(bytes.fromhex("010300000001")))

    def test_unknown_protocol_cannot_open_transport(self):
        for field in ("baudrate", "slave_id", "register_offset", "data_type", "byte_order", "word_order"):
            with self.assertRaises((ValueError, TypeError)):
                replace(PROTOCOL, **{field: "NOT_CONFIGURED"}).validate()

    def test_register_zero_is_allowed_only_when_explicit(self):
        PROTOCOL.validate()
        self.assertEqual(321, PROTOCOL.decode([321]))

    def test_byte_and_word_order(self):
        protocol = replace(PROTOCOL, data_type="uint32")
        self.assertEqual(0x12345678, protocol.decode([0x1234, 0x5678]))
        self.assertEqual(0x12345678, replace(protocol, word_order="little").decode([0x5678, 0x1234]))
        self.assertEqual(0x12345678, replace(protocol, byte_order="little").decode([0x3412, 0x7856]))

    def test_float_nan_and_wrong_count_rejected(self):
        with self.assertRaises(ValueError):
            replace(PROTOCOL, data_type="float32").decode([0x7FC0, 0])
        with self.assertRaises(ValueError):
            PROTOCOL.decode([])

    def test_calibration_direction_zero_and_range(self):
        calibration = replace(CALIBRATION, direction=-1, zero_raw=1000, offset_m=1)
        self.assertAlmostEqual(0.5, calibration.position(1500))
        with self.assertRaises(ValueError):
            calibration.position(100000)
        with self.assertRaises(ValueError):
            replace(calibration, scale_m_per_unit="NOT_CONFIGURED").validate()

    def test_driver_preserves_raw_and_calibrated_value(self):
        class Transport:
            def read_registers(self):
                return [2500]
        self.assertEqual((2500, 2.5), PullWireDriver(PROTOCOL, CALIBRATION, Transport()).read())


class FakeSerial:
    def __init__(self, payload, short_write=False):
        self.response = bytearray(payload)
        self.short_write = short_write

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def reset_input_buffer(self):
        pass

    def write(self, request):
        self.request = request
        return len(request) - int(self.short_write)

    def read(self, count):
        data = bytes(self.response[:min(count, 2)])
        del self.response[:len(data)]
        return data


def rtu_reply(payload):
    return payload + struct.pack("<H", crc16(payload))


class RtuFrameTest(unittest.TestCase):
    def test_fragmented_reply(self):
        port = FakeSerial(rtu_reply(bytes.fromhex("01030209c4")))
        result = ModbusRtuTransport(PROTOCOL, lambda **kwargs: port).read_registers()
        self.assertEqual((2500,), result)
        self.assertEqual(bytes.fromhex("010300000001840a"), port.request)

    def test_crc_slave_function_and_size_rejected(self):
        payloads = [
            bytes.fromhex("01030209c40000"),  # Bad CRC.
            rtu_reply(bytes.fromhex("02030209c4")),  # Wrong slave.
            rtu_reply(bytes.fromhex("01040209c4")),  # Wrong function.
            rtu_reply(bytes.fromhex("01030409c4")),  # Wrong byte count.
        ]
        for payload in payloads:
            port = FakeSerial(payload)
            with self.assertRaises(ValueError):
                ModbusRtuTransport(PROTOCOL, lambda **kwargs: port).read_registers()

    def test_exception_and_short_write_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "exception"):
            ModbusRtuTransport(PROTOCOL, lambda **kwargs: FakeSerial(
                rtu_reply(bytes.fromhex("018302")))).read_registers()
        with self.assertRaisesRegex(RuntimeError, "short"):
            ModbusRtuTransport(PROTOCOL, lambda **kwargs: FakeSerial(b"", True)).read_registers()

    def test_empty_reply_times_out(self):
        protocol = replace(PROTOCOL, timeout_sec=0.002)
        with self.assertRaises(TimeoutError):
            ModbusRtuTransport(protocol, lambda **kwargs: FakeSerial(b"")).read_registers()


class SessionLatchTest(unittest.TestCase):
    def test_success_clears_but_exception_preserves_latch(self):
        import tempfile
        from pathlib import Path
        from unittest.mock import patch
        from chili_crane_hardware.adam.session import output_recovery_latch
        with tempfile.TemporaryDirectory() as temporary:
            marker = Path(temporary) / "active"
            with patch("chili_crane_hardware.adam.session.session_paths",
                       return_value=(Path(temporary) / "lock", marker)):
                with output_recovery_latch("test", 502, 1):
                    self.assertTrue(marker.exists())
                self.assertFalse(marker.exists())
                with self.assertRaises(KeyboardInterrupt):
                    with output_recovery_latch("test", 502, 1):
                        raise KeyboardInterrupt()
                self.assertTrue(marker.exists())
                with output_recovery_latch("test", 502, 1):
                    pass
                self.assertFalse(marker.exists())

    @unittest.skipUnless(__import__("sys").platform.startswith("linux"), "Linux flock runtime only")
    def test_latch_blocks_reader_and_exclusive_blocks_shared(self):
        import tempfile
        from pathlib import Path
        from unittest.mock import patch
        from chili_crane_hardware.adam.session import device_lock, output_recovery_latch
        with tempfile.TemporaryDirectory() as temporary:
            paths = (Path(temporary) / "lock", Path(temporary) / "active")
            with patch("chili_crane_hardware.adam.session.session_paths", return_value=paths):
                with device_lock("test", 502, 1, exclusive=True):
                    with self.assertRaisesRegex(RuntimeError, "held"):
                        with device_lock("test", 502, 1):
                            pass
                with self.assertRaises(RuntimeError):
                    with output_recovery_latch("test", 502, 1):
                        raise RuntimeError("writer crashed")
                with self.assertRaisesRegex(RuntimeError, "unverified"):
                    with device_lock("test", 502, 1):
                        pass
                with device_lock("test", 502, 1, exclusive=True):
                    with output_recovery_latch("test", 502, 1):
                        pass
                with device_lock("test", 502, 1):
                    pass


if __name__ == "__main__":
    unittest.main()
