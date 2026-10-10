"""Unit/epoch evidence gates for MVS raw timestamps; no control or spatial authority."""
import math
import time


def proven(cfg, kind):
    evidence = cfg.get(kind + "_time_evidence_id")
    reference = cfg.get("sdk_document_reference")
    return bool(evidence and evidence != "NOT_CONFIGURED" and reference and reference != "NOT_CONFIGURED")


def scaled(raw, scale, offset, verified):
    if (not verified or type(raw) is not int or raw <= 0 or type(scale) not in (int, float)
            or not math.isfinite(scale) or not 0 < scale <= 1 or type(offset) not in (int, float)
            or not math.isfinite(offset)):
        return None
    value = raw * scale + offset
    return value if math.isfinite(value) and value > 0 else None


class CameraTimeModel:
    def __init__(self, cfg):
        if cfg.get("camera_control_authority") is not False:
            raise ValueError("camera_control_authority must remain false")
        self.cfg = cfg
        self.last_frame = None
        self.previous = {}
        self.gaps = self.regressions = self.jumps = 0
        self.last_mono = None

    def observe(self, info, receive_sec, ptp_status, mono=None):
        if not math.isfinite(receive_sec) or receive_sec <= 0:
            raise ValueError("invalid receive timestamp")
        mono = time.monotonic() if mono is None else mono
        cfg = self.cfg
        raw_device = (int(info["nDevTimeStampHigh"]) << 32) | int(info["nDevTimeStampLow"])
        raw_host = int(info["nHostTimeStamp"])
        ticks = cfg.get("device_tick_frequency_hz")
        scale = 1 / ticks if type(ticks) in (float, int) and math.isfinite(ticks) and ticks > 0 else None
        device = scaled(raw_device, scale, cfg.get("device_epoch_offset_sec"), proven(cfg, "device"))
        host = scaled(raw_host, cfg.get("host_seconds_per_tick"), cfg.get("host_epoch_offset_sec"), proven(cfg, "host"))
        regression = jump = False
        for kind, value in (("device", raw_device), ("host", raw_host), ("receive", receive_sec)):
            previous = self.previous.get(kind)
            if previous is not None and value < previous:
                regression = True
            self.previous[kind] = value
        jump_limit = cfg.get("maximum_timestamp_jump_sec")
        for kind, value in (("device_sec", device), ("host_sec", host)):
            previous = self.previous.get(kind)
            if value is not None:
                if (previous is not None and type(jump_limit) in (float, int)
                        and math.isfinite(jump_limit) and 0 < jump_limit <= 10
                        and value - previous > jump_limit):
                    jump = True
                self.previous[kind] = value
        self.regressions += int(regression)
        self.jumps += int(jump)
        number = int(info["nFrameNum"])
        counter_reset = self.last_frame is not None and number <= self.last_frame
        if self.last_frame is not None and number > self.last_frame + 1:
            self.gaps += number - self.last_frame - 1
        self.last_frame = number
        tolerance = cfg.get("maximum_time_offset_sec")
        bounded = type(tolerance) in (float, int) and math.isfinite(tolerance) and 0 < tolerance <= 1
        device_ok = device is not None and bounded and abs(receive_sec - device) <= tolerance
        host_ok = host is not None and bounded and abs(receive_sec - host) <= tolerance
        mode, stamp, precision = "CAMERA_RECEIVE_ESTIMATE", receive_sec, False
        if device_ok and ptp_status.lower() == "slave":
            mode, stamp, precision = "CAMERA_PTP_DEVICE", device, True
        elif host_ok:
            mode, stamp = "CAMERA_HOST_TIMESTAMP", host
        fault = regression or jump or counter_reset
        if fault:
            mode, stamp, precision = "CAMERA_RECEIVE_ESTIMATE", receive_sec, False
        rate = 1 / (mono - self.last_mono) if self.last_mono is not None and mono > self.last_mono else None
        self.last_mono = mono
        return {"timestamp_mode": mode, "stamp": stamp,
            "validity": "DEGRADED" if fault or mode == "CAMERA_RECEIVE_ESTIMATE" else "VALID",
            "reason": "TIMESTAMP_REGRESSION_OR_JUMP" if fault else
                      ("TIMESTAMP_UNIT_OR_EPOCH_NOT_VERIFIED" if mode == "CAMERA_RECEIVE_ESTIMATE" else "TIMESTAMP_EVIDENCE_VALID"),
            "device_timestamp_raw": raw_device, "host_timestamp_raw": raw_host,
            "device_timestamp_sec": device, "host_timestamp_sec": host, "ros_receive_timestamp": receive_sec,
            "device_time_valid": bool(device_ok and not fault), "host_time_valid": bool(host_ok and not fault),
            "device_host_offset_sec": device - host if device is not None and host is not None else None,
            "host_receive_latency_sec": receive_sec - host if host is not None else None,
            "frame_number": number, "frame_gaps": self.gaps, "lost_packets": int(info["nLostPacket"]),
            "frame_rate_hz": rate, "exposure_time_raw": float(info["fExposureTime"]),
            "exposure_unit": cfg.get("exposure_unit", "NOT_CONFIGURED"),
            "timestamp_regression_count": self.regressions, "timestamp_jump_count": self.jumps,
            "ptp_status": ptp_status, "high_precision_time_valid": precision,
            "camera_control_authority": False, "rolling_shutter_compensated": False}
