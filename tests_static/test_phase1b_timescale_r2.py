"""R2 synthetic time-scale/protocol regressions, never physical acceptance."""
from pathlib import Path
import sys
import unittest
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
for package in ("slam", "hardware"):
    sys.path.insert(0, str(ROOT / ("src/chili_crane_" + package + "/src")))
from chili_crane_slam.timescale import TimeScale, TimeScaleRelation, normalize_times
from chili_crane_slam.timebase import ClockContract, FrameTime
from chili_crane_slam.pointcloud import CANONICAL
from chili_crane_hardware.er1_time_status import clock_fields, RuntimeClockTracker


def relation(**extra):
    result = dict(source_scale="PTP_TIMESCALE", target_scale="HOST_UTC",
                  source_domain="ptp-domain-0", target_domain="host-utc",
                  utc_offset_sec=37, relation_state="VALID", evidence_id="synthetic-proof",
                  offset_source="synthetic-pmc-and-comparison",
                  current_utc_offset_valid=True, time_traceable=True)
    result.update(extra)
    return result


def clock(**extra):
    result = dict(clock_mode="SENSOR_PTP", clock_sync_state="VALID", clock_domain="ptp-domain-0",
                  time_evidence_id="synthetic-sync-proof", raw_time_scale="PTP_TIMESCALE",
                  time_scale_relation=relation())
    result.update(extra)
    return result


def difop(status=1, mode=2):
    payload = bytearray(256)
    payload[:8] = bytes.fromhex("a5ff005a11115555")
    payload[101], payload[102] = mode, status
    return bytes(payload)


def msop(frame=1, mode=2):
    payload = bytearray(1200)
    payload[:4] = bytes.fromhex("55aa5aa5")
    payload[9], payload[20] = mode, frame
    return bytes(payload)


def points(start):
    result = np.zeros(3, dtype=CANONICAL)
    result["timestamp"] = [start, start + .0625, start + .125]
    return result


class TimescaleR2Tests(unittest.TestCase):
    def test_four_scales_declared(self):
        self.assertEqual({"UNKNOWN", "HOST_UTC", "PTP_TIMESCALE", "BOOT_RELATIVE"}, {x.value for x in TimeScale})

    def test_header_and_all_points_subtract_same_37(self):
        raw = points(1700000037)
        original = raw.tobytes()
        header, normalized, offset = normalize_times(1700000037000000001, raw, "PTP_TIMESCALE",
                                                     TimeScaleRelation.from_config(relation()))
        self.assertEqual(1700000000000000001, header)
        self.assertEqual(37, offset)
        np.testing.assert_array_equal(raw["timestamp"] - 37, normalized["timestamp"])
        self.assertEqual(original, raw.tobytes())

    def test_span_mid_delta_and_overlap_exactly_invariant(self):
        a, b = points(1700000037), points(1700000037 + 2**-20)
        raw_a, raw_b = FrameTime.from_points(a["timestamp"][0], a["timestamp"]), FrameTime.from_points(b["timestamp"][0], b["timestamp"])
        converted = []
        for raw in (a, b):
            ns, normalized, _ = normalize_times(int(raw["timestamp"][0] * 1e9), raw,
                                                "PTP_TIMESCALE", TimeScaleRelation.from_config(relation()))
            converted.append(FrameTime.from_points(ns / 1e9, normalized["timestamp"]))
        na, nb = converted
        self.assertEqual(raw_a.span, na.span)
        self.assertEqual(raw_b.span, nb.span)
        self.assertEqual(raw_a.mid - raw_b.mid, na.mid - nb.mid)
        self.assertEqual(raw_a.overlap(raw_b), na.overlap(nb))

    def test_future_stale_only_after_normalization(self):
        raw = points(1700000037)
        with self.assertRaisesRegex(ValueError, "FUTURE"):
            FrameTime.from_points(1700000037, raw["timestamp"]).check(1700000000.2, .01, .5)
        ns, normalized, _ = normalize_times(1700000037000000000, raw, "PTP_TIMESCALE",
                                            TimeScaleRelation.from_config(relation()))
        FrameTime.from_points(ns / 1e9, normalized["timestamp"]).check(1700000000.2, .01, .5, .001)

    def test_missing_relation_fails_closed(self):
        for proof in (None, TimeScaleRelation(), TimeScaleRelation.from_config(relation(relation_state="PROBING"))):
            with self.assertRaisesRegex(ValueError, "RELATION_NOT_VERIFIED"):
                normalize_times(100000000000, points(100), "PTP_TIMESCALE", proof)

    def test_unknown_and_boot_relative_not_host_utc(self):
        for scale in ("UNKNOWN", "BOOT_RELATIVE"):
            with self.assertRaises(ValueError):
                normalize_times(100000000000, points(100), scale, TimeScaleRelation.from_config(relation()))

    def test_local_offset_invalid_flags_still_bench_allowed(self):
        proof = TimeScaleRelation.from_config(relation(relation_state="LOCAL_VERIFIED",
                                                       current_utc_offset_valid=False, time_traceable=False))
        self.assertTrue(proof.bench_allowed)
        self.assertFalse(proof.production_allowed)
        normalize_times(137000000000, points(137), "PTP_TIMESCALE", proof)
        cfg = clock(time_scale_relation=relation(relation_state="LOCAL_VERIFIED",
                                                 current_utc_offset_valid=False, time_traceable=False))
        contract = ClockContract.from_config(cfg)
        self.assertTrue(contract.pairing_ready)
        self.assertFalse(contract.production_ready)

    def test_valid_traceable_relation_production_timing_allowed(self):
        self.assertTrue(ClockContract.from_config(clock()).production_ready)
        self.assertTrue(TimeScaleRelation.from_config(relation()).production_allowed)

    def test_valid_cannot_hide_false_traceability(self):
        for key in ("current_utc_offset_valid", "time_traceable"):
            for flag in (False, None):
                with self.assertRaisesRegex(ValueError, "TRACEABILITY"):
                    TimeScaleRelation.from_config(relation(**{key: flag}))

    def test_domain_mapping_not_domain_equality(self):
        contract = ClockContract.from_config(clock(host_clock_domain="unrelated-legacy-host-label"))
        self.assertNotEqual(contract.domain, contract.relation.target_domain)
        self.assertTrue(contract.pairing_ready)
        with self.assertRaisesRegex(ValueError, "SOURCE_DOMAIN"):
            ClockContract.from_config(clock(time_scale_relation=relation(source_domain="other-ptp-domain")))

    def test_legacy_domain_equality_not_sufficient(self):
        cfg = clock(time_scale_relation=None, host_clock_domain="ptp-domain-0",
                    host_clock_relation="VALID", host_clock_relation_evidence_id="legacy")
        self.assertFalse(ClockContract.from_config(cfg).pairing_ready)

    def test_ptp_cannot_be_declared_host_utc(self):
        with self.assertRaisesRegex(ValueError, "SENSOR_PTP_SCALE_MISMATCH"):
            ClockContract.from_config(clock(raw_time_scale="HOST_UTC"))

    def test_direct_relation_cannot_truncate_fractional_compensation(self):
        proof = TimeScaleRelation(**relation(utc_offset_sec=37.12))
        self.assertFalse(proof.bench_allowed)
        with self.assertRaises(ValueError):
            normalize_times(137000000000, points(137), "PTP_TIMESCALE", proof)

    def test_invalid_offset_and_missing_evidence(self):
        for extra in ({"utc_offset_sec": None}, {"utc_offset_sec": 37.12},
                      {"utc_offset_sec": float("nan")}, {"utc_offset_sec": True},
                      {"utc_offset_sec": 999}, {"evidence_id": "NOT_CONFIGURED"},
                      {"offset_source": "UNKNOWN"}, {"target_scale": "UNKNOWN"}):
            with self.assertRaises(ValueError):
                TimeScaleRelation.from_config(relation(**extra))

    def test_host_derived_never_subtracts_37(self):
        cfg = dict(clock_mode="HOST_DERIVED", clock_sync_state="PROVISIONAL", clock_domain="host-utc",
                   raw_time_scale="HOST_UTC")
        self.assertTrue(ClockContract.from_config(cfg).pairing_ready)
        header, normalized, offset = normalize_times(100000000001, points(100), "HOST_UTC",
                                                     TimeScaleRelation.from_config(relation()))
        self.assertEqual(100000000001, header)
        self.assertEqual(0, offset)
        np.testing.assert_array_equal(points(100)["timestamp"], normalized["timestamp"])

    def test_camera_host_utc_never_ptp_converted(self):
        proof = TimeScaleRelation.from_config(relation())
        header, _, offset = normalize_times(1700000000000000000, points(1700000000), TimeScale.HOST_UTC, proof)
        self.assertEqual(1700000000000000000, header)
        self.assertEqual(0, offset)
        camera = yaml.safe_load((ROOT / "config/sites/crane_01/sensors/hik_01.yaml").read_text())
        self.assertFalse(camera["driver_enabled"])
        self.assertFalse(camera["camera_control_authority"])

    def test_receive_after_first_point_not_vendor_bug(self):
        f = FrameTime.from_points(100, [100, 100.125])
        f.check(100.145, .01, .5, .001)
        self.assertEqual(.125, f.span)
        self.assertAlmostEqual(.145, 100.145 - f.start)
        self.assertAlmostEqual(.0825, 100.145 - f.mid)
        self.assertAlmostEqual(.020, 100.145 - f.end)

    def test_fixed_offsets_difop_and_msop(self):
        self.assertEqual({"kind": "DIFOP", "timesync_mode": 2, "timesync_status": 1}, clock_fields(difop()))
        self.assertEqual({"kind": "MSOP", "time_mode": 2, "frame_sync": 1}, clock_fields(msop()))
        for payload in (b"", bytes(256), difop()[:-1], bytes(1200), msop()[:-1]):
            self.assertIsNone(clock_fields(payload))

    def test_success_failure_timeout_immediate(self):
        tracker = RuntimeClockTracker(1)
        tracker.observe(msop(), 10)
        for status, label in ((1, "PTP_SYNC_SUCCESS"), (0, "PTP_SYNC_FAILED"), (2, "PTP_SYNC_TIMEOUT"), (255, "UNKNOWN")):
            tracker.observe(difop(status), 10)
            state = tracker.state(10.01)
            self.assertEqual(label, state["clock_state"])
            self.assertEqual(status == 1, state["clock_sync_valid"])

    def test_frame_zero_is_not_clock_failure(self):
        tracker = RuntimeClockTracker(1)
        tracker.observe(difop(), 10)
        tracker.observe(msop(0), 10)
        state = tracker.state(10.1)
        self.assertTrue(state["clock_sync_valid"])
        self.assertFalse(state["scan_phase_sync_valid"])
        self.assertEqual("FRAME_NOT_SYNCED", state["scan_phase_state"])

    def test_clock_success_does_not_infer_frame_sync(self):
        tracker = RuntimeClockTracker(1)
        tracker.observe(difop(), 10)
        self.assertFalse(tracker.state(10.1)["scan_phase_sync_valid"])
        tracker.observe(msop(1), 10.1)
        self.assertTrue(tracker.state(10.1)["scan_phase_sync_valid"])
        tracker.observe(difop(0), 10.2)
        self.assertTrue(tracker.state(10.2)["scan_phase_sync_valid"])
        self.assertFalse(tracker.state(10.2)["clock_sync_valid"])

    def test_runtime_expires_despite_continuing_msop(self):
        tracker = RuntimeClockTracker(.5)
        tracker.observe(difop(), 10)
        tracker.observe(msop(), 10)
        self.assertTrue(tracker.state(10)["clock_sync_valid"])
        tracker.observe(msop(), 11)
        self.assertFalse(tracker.state(11)["clock_sync_valid"])
        self.assertTrue(tracker.state(11)["scan_phase_sync_valid"])
        self.assertEqual("PTP_SYNC_TIMEOUT", tracker.state(11)["clock_state"])

    def test_unknown_timeout_or_mode_never_ready(self):
        for timeout, mode in ((None, 2), (1, 0), (1, 3), (1, 255)):
            tracker = RuntimeClockTracker(timeout)
            tracker.observe(difop(mode=mode), 10)
            tracker.observe(msop(mode=mode), 10)
            self.assertFalse(tracker.state(10)["clock_sync_valid"])

    def test_dual_relation_offsets_must_agree(self):
        a = ClockContract.from_config(clock())
        b = ClockContract.from_config(clock(time_scale_relation=relation(utc_offset_sec=36)))
        self.assertFalse(a.compatible(b))

    def test_site_s14_is_local_only_and_outputs_remain_off(self):
        for sensor in ("204", "205"):
            cfg = yaml.safe_load((ROOT / ("config/sites/crane_01/sensors/er1_" + sensor + ".yaml")).read_text())
            contract = ClockContract.from_config(cfg)
            self.assertEqual(37, contract.relation.utc_offset_sec)
            self.assertEqual("LOCAL_VERIFIED", contract.relation.relation_state)
            self.assertFalse(contract.production_ready)
            self.assertFalse(cfg["driver_enabled"])
            self.assertIsNone(cfg["runtime_status_timeout_sec"])
