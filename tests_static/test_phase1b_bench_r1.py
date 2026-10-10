"""Synthetic bench R1 regressions. Never connect to hardware or mutate networking."""
import importlib.util
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import yaml

ROOT = Path(__file__).resolve().parents[1]
for package in ("hardware", "slam"):
    sys.path.insert(0, str(ROOT / ("src/chili_crane_" + package + "/src")))
from chili_crane_hardware.er1_transport import packet_role, driver_transport
from chili_crane_slam.timebase import ClockContract, FrameTime


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


NETWORK = load("bench_network", "tools/sensor_network_contract.py")
RENDER = load("bench_render", "scripts/sensors/render_er1_config.py")
PROBE = load("bench_udp", "scripts/sensors/er1_udp_probe.py")
SITE = ROOT / "config/sites/crane_01"


def transport(**extra):
    return dict(transport_mode="UNICAST", host_address="192.168.1.102",
                destination_address="192.168.1.102", group_address=None, **extra)


def ptp(**extra):
    cfg = dict(clock_mode="SENSOR_PTP", clock_sync_state="VALID", clock_domain="synthetic-ptp-0",
               time_evidence_id="synthetic-time", host_clock_relation="VALID",
               host_clock_relation_evidence_id="synthetic-host",
               host_clock_domain="synthetic-ptp-0")
    cfg.update(extra)
    return cfg


def fake_network(*cmd):
    expected = {"connection.id": "SENSOR-NET", "connection.interface-name": "enp3s0",
                "ipv4.method": "manual", "ipv4.addresses": "192.168.1.102/24",
                "ipv4.never-default": "yes", "ipv4.gateway": "", "ipv4.dns": "",
                "ipv6.method": "disabled"}
    if cmd[0] == "nmcli":
        value = expected[cmd[2]]
    elif "address" in cmd:
        value = "3: enp3s0 inet 192.168.1.102/24 brd 192.168.1.255"
    elif "route" in cmd:
        value = cmd[-1] + " dev enp3s0 src 192.168.1.102"
    else:
        value = "4: enp4s0 <NO-CARRIER,BROADCAST,UP> state DOWN"
    return subprocess.CompletedProcess(cmd, 0, value, "")


class BenchR1Tests(unittest.TestCase):
    def test_site_uses_latest_not_s1(self):
        cfg, _ = NETWORK.settings(SITE)
        self.assertEqual("SENSOR-NET", cfg["profile"])
        self.assertEqual("192.168.1.102/24", cfg["address"])
        self.assertEqual("BENCH_VERIFIED", cfg["config_state"])
        self.assertFalse(cfg["modification_allowed"])

    def test_adam_disconnected_cannot_fail_sensor_check(self):
        cfg, control = NETWORK.settings(SITE)
        with patch.object(NETWORK, "run", side_effect=fake_network):
            failures, adam = NETWORK.check(cfg, control, True)
        self.assertEqual([], failures)
        self.assertEqual("DISCONNECTED", adam)

    def test_adam_default_not_checked(self):
        cfg, control = NETWORK.settings(SITE)
        with patch.object(NETWORK, "run", side_effect=fake_network) as runner:
            self.assertEqual(([], "NOT_CHECKED"), NETWORK.check(cfg, control))
            self.assertFalse(any("enp4s0" in c.args for c in runner.call_args_list))

    def test_existing_profile_no_duplicate_or_write(self):
        cfg, _ = NETWORK.settings(SITE)
        with patch.object(NETWORK, "run", side_effect=fake_network) as runner:
            self.assertEqual(([], ["NO_CHANGE_REQUIRED"]), NETWORK.dry_run(cfg))
            self.assertFalse(any("add" in c.args or "up" in c.args for c in runner.call_args_list))

    def test_existing_wrong_profile_is_not_replaced(self):
        cfg, _ = NETWORK.settings(SITE)
        def bad(*cmd):
            result = fake_network(*cmd)
            if cmd[0] == "nmcli" and cmd[2] == "ipv4.addresses":
                result.stdout = "192.168.1.10/24"
            return result
        with patch.object(NETWORK, "run", side_effect=bad):
            failures, lines = NETWORK.dry_run(cfg)
        self.assertTrue(failures)
        self.assertEqual(["PROFILE_REVIEW_REQUIRED"], lines)

    def test_missing_profile_commands_are_print_only(self):
        cfg, _ = NETWORK.settings(SITE)
        with patch.object(NETWORK, "run", return_value=subprocess.CompletedProcess([], 10, "", "")) as runner:
            failures, lines = NETWORK.dry_run(cfg)
        self.assertEqual([], failures)
        self.assertIn("SENSOR-NET", lines[1])
        self.assertEqual(1, runner.call_count)

    def test_sensor_route_mismatch_fails(self):
        cfg, control = NETWORK.settings(SITE)
        def wrong(*cmd):
            result = fake_network(*cmd)
            if "route" in cmd:
                result.stdout = "dev eno1"
            return result
        with patch.object(NETWORK, "run", side_effect=wrong):
            self.assertTrue(NETWORK.check(cfg, control)[0])

    def test_profile_override_validated(self):
        cfg, _ = NETWORK.settings(SITE, profile="REVIEWED-SENSOR")
        self.assertEqual("REVIEWED-SENSOR", cfg["profile"])
        with self.assertRaises(ValueError):
            NETWORK.settings(SITE, profile="bad;touch")

    def test_unicast_host_binding(self):
        self.assertEqual({"host_address": "192.168.1.102", "group_address": "0.0.0.0"},
                         driver_transport(transport()))

    def test_multicast_host_and_group(self):
        cfg = transport()
        cfg.update(transport_mode="MULTICAST", group_address="224.0.0.205", destination_address="224.0.0.205")
        self.assertEqual("224.0.0.205", driver_transport(cfg)["group_address"])
        self.assertEqual("192.168.1.102", driver_transport(cfg)["host_address"])

    def test_multicast_missing_group_rejected(self):
        cfg = transport()
        cfg.update(transport_mode="MULTICAST", destination_address="224.0.0.205")
        with self.assertRaisesRegex(ValueError, "GROUP_REQUIRED"):
            driver_transport(cfg)

    def test_transport_mismatch_unknown_and_invalid_rejected(self):
        for change in ({"transport_mode": "NOT_CONFIGURED"},
                       {"destination_address": "192.168.1.10"}, {"host_address": "0.0.0.0"},
                       {"group_address": "224.0.0.205"}, {"host_address": "invalid"}):
            cfg = transport()
            cfg.update(change)
            with self.assertRaises(ValueError):
                driver_transport(cfg)

    def test_broadcast_explicit_contract(self):
        cfg = transport()
        cfg.update(transport_mode="BROADCAST", destination_address="192.168.1.255")
        self.assertEqual("0.0.0.0", driver_transport(cfg)["group_address"])

    def test_msop_signature_not_port(self):
        self.assertEqual("CONFIRMED_MSOP", packet_role(bytes.fromhex("55aa5aa5") + bytes(1196)))

    def test_difop_signature_not_port(self):
        self.assertEqual("CONFIRMED_DIFOP", packet_role(bytes.fromhex("a5ff005a11115555") + bytes(248)))

    def test_unknown_length_or_signature(self):
        for payload in (bytes(1200), bytes(256), b"", bytes.fromhex("55aa5aa5") + bytes(252),
                        bytes.fromhex("a5ff005a11115555") + bytes(1192)):
            self.assertEqual("UNKNOWN", packet_role(payload))

    def test_role_metadata_destination_no_payload(self):
        payload = bytes.fromhex("a5ff005a11115555") + bytes(248)
        ip = bytearray(20)
        ip[0], ip[9] = 0x45, 17
        ip[2:4] = (20 + 8 + len(payload)).to_bytes(2, "big")
        ip[12:16], ip[16:20] = bytes([192, 168, 1, 205]), bytes([192, 168, 1, 102])
        frame = bytes(12) + b"\x08\x00" + ip + struct.pack("!HHHH", 123, 7799, 264, 0) + payload
        result = PROBE.udp_packet_metadata(frame)
        self.assertEqual("CONFIRMED_DIFOP", result["role"])  # Even at the reported MSOP port.
        self.assertEqual("192.168.1.102", result["destination_ip"])
        self.assertEqual(7799, result["destination_port"])
        self.assertFalse(any(isinstance(v, bytes) for v in result.values()))
        self.assertIsNone(PROBE.udp_packet_metadata(frame[:-1]))

    def test_ptp_probing_never_ready_even_with_evidence(self):
        for domain in ("NOT_CONFIGURED", "synthetic-ptp-0"):
            contract = ClockContract.from_config(ptp(clock_sync_state="PROBING", clock_domain=domain))
            self.assertFalse(contract.ptp_verified)
            self.assertFalse(contract.pairing_ready)

    def test_ptp_valid_requires_host_relation_domain_and_evidence(self):
        self.assertTrue(ClockContract.from_config(ptp()).ptp_verified)
        for change in ({"host_clock_relation": "NOT_CONFIGURED"}, {"host_clock_relation": "PROBING"},
                       {"host_clock_relation": "DEGRADED"}, {"host_clock_domain": "another-domain"},
                       {"host_clock_relation_evidence_id": "NOT_CONFIGURED"},
                       {"time_evidence_id": "NOT_CONFIGURED"}, {"clock_domain": "NOT_CONFIGURED"}):
            with self.assertRaises(ValueError):
                ClockContract.from_config(ptp(**change))

    def test_render_ptp_probing_and_unicast(self):
        with tempfile.TemporaryDirectory() as tmp:
            site = Path(tmp)
            (site / "sensors").mkdir()
            for i, sensor in enumerate(("204", "205")):
                cfg = transport()
                cfg.update(ptp(clock_sync_state="PROBING", clock_domain="NOT_CONFIGURED"))
                cfg.update(driver_enabled=True, port_roles="VALID", msop_port=11000 + i * 2,
                           difop_port=11001 + i * 2, frame_id="synthetic/" + sensor, raw_topic="/vendor_" + sensor)
                (site / "sensors" / ("er1_" + sensor + ".yaml")).write_text(yaml.safe_dump(cfg))
            result = RENDER.render(site)
            self.assertTrue(all(l["driver"]["use_lidar_clock"] for l in result["lidar"]))
            self.assertTrue(all(l["driver"]["host_address"] == "192.168.1.102" for l in result["lidar"]))

    def test_frame_span_optional_then_enforced(self):
        frame = FrameTime.from_points(10, [10, 10.2])
        frame.check(10.3, .01, .5, .001, None)
        frame.check(10.3, .01, .5, .001, .2)
        with self.assertRaisesRegex(ValueError, "FRAME_SPAN_EXCEEDED"):
            frame.check(10.3, .01, .5, .001, .1)
        for value in (0, -1, float("inf"), float("nan")):
            with self.assertRaises(ValueError):
                frame.check_span(value)

    def test_obsolete_readiness_field_removed_not_replaced_with_true(self):
        text = (ROOT / "src/chili_crane_slam/src/chili_crane_slam/dual_lidar_node.py").read_text()
        self.assertNotIn("point_timestamp_header_tolerance_sec", text)
        self.assertIn("all(c.pairing_ready", text)
        dual = yaml.safe_load((SITE / "sensors/dual_lidar.yaml").read_text())
        self.assertNotIn("point_timestamp_header_tolerance_sec", dual)
        self.assertIsNone(dual["maximum_frame_span_sec"])

    def test_camera_and_driver_assets_remain_locked(self):
        cfg = yaml.safe_load((SITE / "sensors/hik_01.yaml").read_text())
        self.assertEqual("192.168.1.180", cfg["expected_ip"])
        self.assertEqual("MV-CS060-10GC", cfg["model"])
        self.assertEqual("BENCH_REACHABLE", cfg["network_state"])
        self.assertEqual("NOT_CONFIGURED", cfg["expected_serial"])
        self.assertFalse(cfg["driver_enabled"])
        self.assertFalse(cfg["camera_control_authority"])
        for sensor in ("204", "205"):
            cfg = yaml.safe_load((SITE / ("sensors/er1_" + sensor + ".yaml")).read_text())
            self.assertEqual("UNICAST", cfg["transport_mode"])
            self.assertEqual("NOT_CONFIGURED", cfg["port_roles"])
            self.assertFalse(cfg["driver_enabled"])
