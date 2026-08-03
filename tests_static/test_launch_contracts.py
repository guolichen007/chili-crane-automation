import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = ROOT / "src" / "chili_crane_bringup" / "launch"


class LaunchContractTest(unittest.TestCase):
    def test_public_entry_launches_accept_config_root(self):
        entry_points = {
            "mapping.launch",
            "localization.launch",
            "mock_system.launch",
            "bag_replay.launch",
        }
        for name in sorted(entry_points):
            with self.subTest(launch=name):
                root = ET.parse(LAUNCH_DIR / name).getroot()
                args = {
                    element.attrib.get("name")
                    for element in root.findall("arg")
                }
                self.assertIn("config_root", args)

    def test_source_default_is_explicitly_overridable(self):
        for path in LAUNCH_DIR.glob("*.launch"):
            if path.name == "base_system.launch":
                continue
            root = ET.parse(path).getroot()
            config_arg = next(
                element
                for element in root.findall("arg")
                if element.attrib.get("name") == "config_root"
            )
            self.assertIn("default", config_arg.attrib)


if __name__ == "__main__":
    unittest.main()
