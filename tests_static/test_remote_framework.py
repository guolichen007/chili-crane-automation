"""The unconfigured remote interface is present even without physical DI."""
import ast
import unittest
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]


class RemoteFrameworkTest(unittest.TestCase):
    def test_24di_inventory_and_unknown_remote_mapping(self):
        config = yaml.safe_load((ROOT / "config/hardware/io_mapping.template.yaml").read_text(encoding="utf-8"))
        self.assertEqual(24, config["total_di_channels"])
        self.assertEqual(24, sum(d["di_count"] for d in config["devices"].values()))
        names = [name for name in config["digital_inputs"] if name.startswith("remote_")]
        self.assertEqual(10, len(names))
        for name in names:
            self.assertEqual({"NOT_CONFIGURED"}, set(config["digital_inputs"][name].values()))

    def test_mock_publishes_blocked_remote_contract_and_ci_observes_it(self):
        mock = (ROOT / "src/chili_crane_hardware/scripts/mock_hardware_adapter.py").read_text(encoding="utf-8")
        ast.parse(mock, feature_version=(3, 10))
        self.assertIn('"remote_control_state": RemoteControlState', mock)
        self.assertIn('message.control_mode = "BLOCKED"', mock)
        self.assertIn("message.release_automatic_outputs = True", mock)
        self.assertIn("message.discard_pending_commands = True", mock)
        collector = (ROOT / "tools/validate_ros2_mock.py").read_text(encoding="utf-8")
        self.assertIn('"remote_control_state": RemoteControlState', collector)
        self.assertIn('msg.control_mode != "BLOCKED"', collector)
