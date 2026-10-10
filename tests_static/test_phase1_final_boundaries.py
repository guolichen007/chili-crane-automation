import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for package in ("control", "hardware"):
    sys.path.insert(0, str(ROOT / "src" / ("chili_crane_" + package) / "src"))
from chili_crane_control.execution import ExecutionStrategy, BridgeServoStrategy, GrabLimitStrategy
from chili_crane_hardware.mapping import RawInput, auto_sources_physical
from chili_crane_control.evidence_policy import SourceType


class FinalBoundaryTests(unittest.TestCase):
    def test_reserved_strategies_fail_closed(self):
        for strategy, axis in ((BridgeServoStrategy(), "X"), (GrabLimitStrategy(), "G")):
            self.assertIsInstance(strategy, ExecutionStrategy)
            self.assertTrue(strategy.supports_axis(axis))
            self.assertFalse(strategy.supports_axis("Y"))
            self.assertFalse(strategy.configured())
            self.assertFalse(strategy.step(0, 1, permitted=True, feedback_fresh=True).enable)

    def test_optional_auto_input_may_not_hide_simulated_device(self):
        mapping = {"safety_ok": {"device": "adam6251"}, "power_ok": {"device": "adam6052"}}
        samples = {"adam6251": RawInput((True,) * 16, 1, 0, True, SourceType.PHYSICAL),
            "adam6052": RawInput((True,) * 8, 1, 0, True, SourceType.SIMULATED)}
        self.assertTrue(auto_sources_physical(mapping, samples, ["safety_ok"]))
        self.assertFalse(auto_sources_physical(mapping, samples, ["safety_ok", "power_ok"]))
        self.assertFalse(auto_sources_physical(mapping, samples, ["missing"]))
        self.assertFalse(auto_sources_physical(mapping, samples, []))
