import importlib.util
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


PTP = module("ptp", "tools/ptp_packet_probe.py")
PROBE = module("time_probe", "scripts/sensors/dual_er1_timing_probe.py")
MANIFEST = module("phase1b_manifest", "tools/phase1b_manifest.py")
RENDER = module("render_er1", "scripts/sensors/render_er1_config.py")
CAP = module("ptp_cap", "tools/ptp_capabilities.py")


class Phase1BToolTests(unittest.TestCase):
    def test_hardware_timestamp_capability_requires_full_evidence(self):
        text = "hardware-transmit hardware-receive hardware-raw-clock\nPTP Hardware Clock: 0"
        self.assertTrue(CAP.parse(text)["hardware_timestamp_supported"])
        self.assertFalse(CAP.parse(text)["ptp_verified"])
        self.assertFalse(CAP.parse("PTP Hardware Clock: 0")["hardware_timestamp_supported"])
        self.assertIsNone(CAP.parse("", False)["hardware_timestamp_supported"])

    def test_ptp_l2_and_types_domains(self):
        for kind, name in PTP.NAMES.items():
            payload = bytearray(34)
            payload[0:5] = bytes([kind, 2, 0, 34, 7])
            payload[20:28] = b"12345678"
            data = bytes(12) + b"\x88\xf7" + payload
            result = PTP.ptp_metadata(data)
            self.assertEqual(name, result["message_type"])
            self.assertEqual(7, result["domain"])
            self.assertEqual("L2", result["transport"])
            self.assertIsNone(PTP.ptp_metadata(data[:30]))

    def test_ptp_udp_and_fragment_reject(self):
        payload = bytes([8, 2, 0, 34, 0]) + bytes(29)
        ip = bytearray(20)
        ip[0], ip[9] = 0x45, 17
        ip[2:4] = (62).to_bytes(2, "big")
        data = bytes(12) + b"\x08\x00" + ip + struct.pack("!HHHH", 320, 320, 42, 0) + payload
        self.assertEqual("Follow_Up", PTP.ptp_metadata(data)["message_type"])
        ip[6] = 0x20
        self.assertIsNone(PTP.ptp_metadata(bytes(12) + b"\x08\x00" + ip + data[34:]))

    def test_probe_statistics_bounded_time_pairing(self):
        from tests_static.test_phase1b_timebase import cloud
        p = PROBE.TimingProbe(.01, .01, .5)
        p.receive("204", cloud(10, 10, 10.1), 10.2, 1)
        p.receive("205", cloud(10.04, 10.04, 10.06), 10.2, 1)
        result = p.summary()
        self.assertEqual(1, result["paired_count"])
        self.assertAlmostEqual(.04, result["pair_statistics"]["header_delta"]["max"])
        self.assertAlmostEqual(0, result["pair_statistics"]["mid_delta"]["max"])
        self.assertFalse(result["site_config_modified"])
        json.dumps(result, allow_nan=False)

    def test_manifest_unknown_mode_is_not_host_false(self):
        data = MANIFEST.timebase(ROOT / "config/sites/crane_01")
        self.assertIsNone(data["lidars"]["er1_204"]["use_lidar_clock"])
        self.assertFalse(data["ptp"]["ptp_verified"])
        self.assertIsNone(data["ptp"]["hardware_timestamp_supported"])
        self.assertFalse(data["camera"]["camera_control_authority"])

    def test_render_common_host_and_mixed_reject(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "sensors").mkdir()
            configs = []
            for i, s in enumerate(("204", "205")):
                cfg = {"driver_enabled": True, "port_roles": "VALID", "clock_mode": "HOST_DERIVED",
                    "clock_sync_state": "PROVISIONAL", "clock_domain": "host-one",
                    "msop_port": 10000 + 2 * i, "difop_port": 10001 + 2 * i,
                    "frame_id": "sensor_" + s, "raw_topic": "/vendor_" + s}
                configs.append(cfg)
                (root / "sensors" / ("er1_" + s + ".yaml")).write_text(yaml.safe_dump(cfg))
            result = RENDER.render(root)
            self.assertTrue(all(not l["driver"]["use_lidar_clock"] for l in result["lidar"]))
            configs[1]["clock_domain"] = "host-other"
            (root / "sensors/er1_205.yaml").write_text(yaml.safe_dump(configs[1]))
            with self.assertRaisesRegex(ValueError, "MIXED"):
                RENDER.render(root)

    def test_phase1b_topics_no_spatial_merge_and_require_camera(self):
        self.assertTrue(any("image_raw" in t for t in MANIFEST.TOPICS))
        self.assertTrue(any("dual_lidar/timing" in t for t in MANIFEST.TOPICS))
        self.assertFalse(any("merged" in t or t.startswith("/tf") for t in MANIFEST.TOPICS))

    def test_tools_do_not_change_host_or_call_sudo(self):
        for path in ("scripts/time/check_ptp_capabilities.sh", "scripts/time/ptp_packet_probe.sh",
                     "tools/ptp_packet_probe.py", "tools/sensor_network_benchmark.py"):
            text = (ROOT / path).read_text()
            for forbidden in ("sudo ", "systemctl enable", "systemctl stop", "timedatectl set", "ethtool -K", "ethtool -G"):
                self.assertNotIn(forbidden, text)
