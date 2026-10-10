import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/chili_crane_hardware/src"))
from chili_crane_hardware.trolley.modulo import modular_delta, ModuloPosition
from chili_crane_hardware.trolley.calibration import PullWireCalibration
from chili_crane_hardware.trolley.pull_wire_driver import PullWireDriver
from chili_crane_hardware.trolley.pull_wire_protocol import PullWireProtocol


class ModuloTests(unittest.TestCase):
    def test_real_report_wrap_examples(self):
        self.assertEqual(10, modular_delta(2, 409592, 409600))
        self.assertEqual(-8, modular_delta(409599, 7, 409600))

    def test_zero_crossings_and_multi_turn_continuity(self):
        p = ModuloPosition(100, 95, 30, 1)
        values = [p.update(x, t) for t, x in enumerate([95, 5, 25, 45, 65, 85, 5])]
        self.assertEqual([0, 10, 30, 50, 70, 90, 110], values)
        self.assertEqual(104, p.update(99, 7))

    def test_discontinuity_latches_without_position_jump(self):
        for raw, now in [(30, 0.1), (96, 2), (96, -1)]:
            p = ModuloPosition(100, 95, 10, 1)
            p.update(95, 0)
            with self.assertRaises(ValueError):
                p.update(raw, now)
            self.assertEqual(0, p.relative)
            with self.assertRaises(ValueError):
                p.update(95, 3)

    def test_invalid_samples_and_half_turn(self):
        for x in [100, -1, True, float("nan"), 50]:
            with self.assertRaises(ValueError):
                modular_delta(x, 0, 100)

    def test_development_calibration_never_production_ready(self):
        c = PullWireCalibration("bench-dev-20261010", 0.001 / 18.162, 0, 1,
                                409585, -0.1, 7, 409600, "DEVELOPMENT_ONLY", False)
        self.assertAlmostEqual(10 * c.scale_m_per_unit, c.position(409595))
        self.assertAlmostEqual(17 * c.scale_m_per_unit, c.position(2))
        self.assertFalse(c.production_ready())
        self.assertFalse(replace(c, production_approved=True).production_ready())

    def test_failed_read_never_advances_anchor(self):
        class Transport:
            values = iter([[409592 >> 16, 409592 & 65535], TimeoutError(),
                           [0, 2], ValueError("RTU CRC mismatch")])
            def read_registers(self):
                value = next(self.values)
                if isinstance(value, Exception):
                    raise value
                return value
        c = PullWireCalibration("dev", 0.001, 0, 1, 409592, -1, 10, 409600)
        protocol = PullWireProtocol("/dev/serial/by-id/test", 9600, "N", 1, 1, 3, 0,
                                    "uint32", "big", "big")
        driver = PullWireDriver(protocol, c, Transport(), 100)
        self.assertEqual((409592, 0), driver.read(0))
        with self.assertRaises(TimeoutError):
            driver.read(0.1)
        self.assertEqual((2, 0.01), driver.read(0.2))
        with self.assertRaises(ValueError):
            driver.read(0.3)
        self.assertEqual(10, driver.position_tracker.relative)
