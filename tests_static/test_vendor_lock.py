import importlib.util
import unittest
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("er1_render", ROOT / "scripts/sensors/render_er1_config.py")
RENDER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RENDER)


class VendorTests(unittest.TestCase):
    def test_three_vendor_commits_are_exact(self):
        lock = yaml.safe_load((ROOT / "third_party/rslidar_sdk.lock.yaml").read_text())
        for section in ("sdk", "rs_driver", "rslidar_msg"):
            self.assertRegex(lock[section]["commit"], r"^[0-9a-f]{40}$")
            self.assertEqual("BSD-3-Clause", lock[section]["license"])
        self.assertEqual("XYZIRT", lock["build"]["point_type"])

    def test_current_site_cannot_render_unverified_ports(self):
        with self.assertRaisesRegex(ValueError, "not confirmed"):
            RENDER.render(ROOT / "config/sites/crane_01")
