"""Public delivery and asset contracts."""
import importlib.util
import unittest
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("delivery", ROOT / "tools/check_delivery.py")
DELIVERY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DELIVERY)


class DeliveryContracts(unittest.TestCase):
    def test_conventional_titles_and_sensitive_paths(self):
        self.assertTrue(DELIVERY.title_valid("feat(control)!: 冻结执行接口"))
        self.assertFalse(DELIVERY.title_valid("update everything"))
        for path in ("LOCAL_PROJECT_CONTEXT.md", ".env", ".local-data/log.txt", "secret.pem", "bags/run.db3"):
            self.assertTrue(DELIVERY.forbidden_path(path))
        self.assertFalse(DELIVERY.forbidden_path("docs/api/PHASE05_CONTRACTS.md"))

    def test_reported_assets_do_not_guess_roles_or_camera_ip(self):
        sensors = yaml.safe_load((ROOT / "config/hardware/sensor_inventory.yaml").read_text(encoding="utf-8"))["sensors"]
        self.assertEqual([7799, 6688], sensors["er1_204"]["udp_ports"])
        self.assertEqual([6699, 7788], sensors["er1_205"]["udp_ports"])
        for identity in ("er1_204", "er1_205"):
            self.assertEqual("NOT_CONFIGURED", sensors[identity]["physical_side"])
            self.assertEqual("NOT_CONFIGURED", sensors[identity]["port_roles"])
            self.assertFalse(sensors[identity]["driver_enabled"])
        self.assertEqual("192.168.180", sensors["camera_a"]["reported_ip"])
        self.assertEqual("NEEDS_CONFIRMATION", sensors["camera_a"]["network_config_state"])
        self.assertFalse(sensors["camera_a"]["enabled_for_control"])

    def test_24_physical_channels_unassigned(self):
        mapping = yaml.safe_load((ROOT / "config/hardware/io_mapping.template.yaml").read_text(encoding="utf-8"))
        channels = mapping["physical_channels"]
        self.assertEqual(24, sum(map(len, channels.values())))
        for device, count in (("adam6052", 8), ("adam6251", 16)):
            self.assertEqual(list(range(count)), [item["channel"] for item in channels[device]])
            self.assertTrue(all(item["assignment"] == "NOT_CONFIGURED" for item in channels[device]))

    def test_default_capability_and_recording_are_disabled(self):
        config = yaml.safe_load((ROOT / "config/control/axis_capabilities.template.yaml").read_text(encoding="utf-8"))
        for axis in ("y", "z"):
            self.assertFalse(config[axis]["verified"])
            self.assertFalse(config[axis]["stop_distance_verified"])
            self.assertIsNone(config[axis]["tolerance_m"])
        recording = yaml.safe_load((ROOT / "config/recording/rosbag2_profile.yaml").read_text(encoding="utf-8"))
        self.assertFalse(recording["enabled"])
