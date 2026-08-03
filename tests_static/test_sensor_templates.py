import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


class SensorTemplateContractTest(unittest.TestCase):
    def test_dual_camera_template_is_disabled_and_unconfigured(self):
        path = ROOT / "config" / "sensors" / "cameras.template.yaml"
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        self.assertEqual("NOT_CONFIGURED", data["config_state"])
        cameras = data["cameras"]
        self.assertEqual(
            {"camera_left", "camera_right"},
            {camera["sensor_id"] for camera in cameras},
        )
        for camera in cameras:
            self.assertFalse(camera["enabled"])
            self.assertEqual("NOT_CONFIGURED", camera["topic"])
            self.assertEqual("NOT_CONFIGURED", camera["frame_id"])
            self.assertEqual("NOT_CONFIGURED", camera["intrinsic_version"])
            self.assertEqual("NOT_CONFIGURED", camera["extrinsic_version"])


if __name__ == "__main__":
    unittest.main()
