import importlib.util
import struct
import unittest
import tempfile
import hashlib
import sys
import numpy as np
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("udp_probe", ROOT / "scripts/sensors/er1_udp_probe.py")
PROBE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROBE)
SPEC = importlib.util.spec_from_file_location("phase1_manifest", ROOT / "tools/phase1_manifest.py")
MANIFEST = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MANIFEST)
sys.path.insert(0, str(ROOT / "src/chili_crane_slam/src"))
from chili_crane_slam.rotation import quaternion


class ToolTests(unittest.TestCase):
    def test_udp_probe_only_metadata(self):
        ethernet = bytes(12) + b"\x08\x00"
        ip = bytearray(20)
        ip[0], ip[9] = 0x45, 17
        ip[2:4] = (31).to_bytes(2,"big")
        ip[12:16] = bytes([192,168,1,204])
        udp = struct.pack("!HHHH", 123, 7799, 11, 0)
        frame = ethernet + ip + udp + b"abc"
        self.assertEqual(("192.168.1.204", 7799, 3), PROBE.udp_metadata(frame))
        self.assertIsNone(PROBE.udp_metadata(frame[:20]))
        ip[6] = 0x20
        self.assertIsNone(PROBE.udp_metadata(ethernet + ip + udp + b"abc"))

    def test_all_site_runtime_outputs_remain_disabled(self):
        for path in (ROOT / "config/sites/crane_01/runtime").glob("*.yaml"):
            config = yaml.safe_load(path.read_text())
            self.assertFalse(config["physical_output_enabled"])
            self.assertFalse(config["automatic_control_enabled"])

    def test_no_network_modification_executor_exists(self):
        text = (ROOT / "scripts/network/create_lidar_profile.sh").read_text()
        self.assertNotIn("--apply", text)
        self.assertIn("Only --dry-run", text)
        self.assertNotIn("\nnmcli ", text)

    def test_calibration_placeholders_never_identity(self):
        for path in (ROOT / "config/sites/crane_01/calibration").glob("*.yaml"):
            config = yaml.safe_load(path.read_text())
            self.assertEqual("NOT_CONFIGURED", config["config_state"])
            self.assertIsNone(config["rotation_matrix"])

    def test_bag_manifest_hashes_without_claiming_field_acceptance(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "synthetic-capture"
            MANIFEST.create(ROOT / "config/sites/crane_01", output, "SYNTHETIC", "unit fixture")
            bag = output / "rosbag2"
            bag.mkdir()
            payload = b"synthetic-fixture-not-a-real-bag"
            (bag / "test.db3").write_bytes(payload)
            info = {"duration": {"nanoseconds": 2000000000}, "topics_with_message_count": [
                {"topic_metadata": {"name": topic}, "message_count": 3} for topic in
                ("/crane_01/lidar/er1_204/points", "/crane_01/lidar/er1_205/points", "/crane_01/system/run_manifest")]}
            (bag / "metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information": info}))
            self.assertTrue(MANIFEST.finalize(output))
            result = yaml.safe_load((output / "bag_manifest.yaml").read_text())
            self.assertEqual("NOT_RUN", result["field_acceptance"])
            self.assertEqual("SYNTHETIC", result["source_type"])
            self.assertEqual(hashlib.sha256(payload).hexdigest(), result["files"][0]["sha256"])
            info["topics_with_message_count"].pop()
            (bag / "metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information": info}))
            self.assertFalse(MANIFEST.finalize(output))

    def test_rotation_quaternion_identity_and_half_turn(self):
        np.testing.assert_allclose(quaternion(np.eye(3)), [0, 0, 0, 1])
        np.testing.assert_allclose(quaternion(np.diag([1, -1, -1])), [1, 0, 0, 0])
        np.testing.assert_allclose(quaternion(np.diag([-1, 1, -1])), [0, 1, 0, 0])
        np.testing.assert_allclose(quaternion(np.diag([-1, -1, 1])), [0, 0, 1, 0])
