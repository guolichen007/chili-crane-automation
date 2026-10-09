import ast
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = ROOT / "src/chili_crane_bringup/launch"


class LaunchContractTest(unittest.TestCase):
    def test_installed_configuration_override(self):
        for name in ("mock_system", "mapping", "localization", "bag_replay", "production", "bench_io"):
            text = (LAUNCH_DIR / (name + ".launch.py")).read_text(encoding="utf-8")
            ast.parse(text)
            self.assertIn('DeclareLaunchArgument("config_root", default_value=share + "/config")', text)

    def test_launch_does_not_start_writer(self):
        for path in LAUNCH_DIR.glob("*.launch.py"):
            self.assertNotIn("adam_do_test", path.read_text(encoding="utf-8"))
        text = (LAUNCH_DIR / "base_system.launch.py").read_text(encoding="utf-8")
        self.assertIn('physical_output_enabled', text)
        self.assertIn('raise ValueError("launch is read-only', text)


if __name__ == "__main__":
    unittest.main()
