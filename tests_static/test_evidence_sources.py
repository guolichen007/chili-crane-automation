import unittest
from chili_crane_hardware.mapping import requirement_status
from chili_crane_control.evidence_policy import RuntimePolicy, SourceType


class EvidenceTests(unittest.TestCase):
    def test_optional_unknown_not_false_and_not_algorithm_block(self):
        values = {"position": 1.0, "fault": None}
        result = requirement_status(values, ["position"], ["position", "fault"], ["fault"])
        self.assertTrue(result["algorithm_ready"])
        self.assertFalse(result["auto_ready"])
        self.assertEqual(["fault"], result["unknown"])
        self.assertIsNone(values["fault"])

    def test_physical_simulated_replay_and_synthetic_are_explicit(self):
        for mode in ("algorithm_dev", "shadow_control", "production"):
            policy = RuntimePolicy(mode)
            self.assertFalse(policy.physical_permission((SourceType.PHYSICAL,)))
            self.assertFalse(policy.sources_allowed((SourceType.UNKNOWN,)))
        self.assertFalse(RuntimePolicy("production").sources_allowed((SourceType.REPLAY,)))
        self.assertTrue(RuntimePolicy("algorithm_dev").sources_allowed((SourceType.SIMULATED,)))
