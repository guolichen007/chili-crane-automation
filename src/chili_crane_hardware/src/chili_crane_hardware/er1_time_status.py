"""Read-only E1R/RSE1 status fields, separate clock and scan phase facts."""
import math
import socket
import struct
from .er1_transport import packet_role

# Official E1R protocol 4.3 + pinned rs_driver 897b14d packed RSE1 layouts.
PROTOCOL_REFERENCE = "Official E1R User Guide 4.3.1.1 and 4.3.2"


def clock_fields(payload):
    role = packet_role(payload)
    if role == "CONFIRMED_DIFOP":
        return {"kind": "DIFOP", "timesync_mode": payload[101], "timesync_status": payload[102]}
    if role == "CONFIRMED_MSOP":
        return {"kind": "MSOP", "time_mode": payload[9], "frame_sync": payload[20]}
    return None


def udp_datagram(frame):
    """Ephemeral payload for local parsing only; never persist/export it."""
    if len(frame) < 14:
        return None
    offset, ethertype = 14, int.from_bytes(frame[12:14], "big")
    while ethertype in (0x8100, 0x88a8):
        if len(frame) < offset + 4:
            return None
        ethertype = int.from_bytes(frame[offset + 2:offset + 4], "big")
        offset += 4
    if ethertype != 0x0800 or len(frame) < offset + 20:
        return None
    ip = frame[offset:]
    ihl, total = (ip[0] & 15) * 4, int.from_bytes(ip[2:4], "big")
    if (ip[0] >> 4 != 4 or ihl < 20 or len(ip) < total or total < ihl + 8
            or ip[9] != 17 or int.from_bytes(ip[6:8], "big") & 0x3fff):
        return None
    _, port, length, _ = struct.unpack("!HHHH", ip[ihl:ihl + 8])
    if length < 8 or ihl + length > total:
        return None
    return socket.inet_ntoa(ip[12:16]), socket.inet_ntoa(ip[16:20]), port, ip[ihl + 8:ihl + length]


class RuntimeClockTracker:
    def __init__(self, timeout=None):
        if timeout is not None and (type(timeout) not in (int, float)
                                    or not math.isfinite(timeout) or not 0 < timeout <= 10):
            raise ValueError("RUNTIME_STATUS_TIMEOUT_INVALID")
        self.timeout = timeout
        self.fields = {}
        self.received = {}
        self.sequence = 0

    def observe(self, payload, monotonic_now):
        fields = clock_fields(payload)
        if fields is None or not math.isfinite(monotonic_now):
            return False
        kind = fields.pop("kind")
        self.fields.update(fields)
        self.received[kind] = monotonic_now
        self.sequence += 1
        return True

    def state(self, monotonic_now):
        def fresh(kind):
            age = monotonic_now - self.received.get(kind, float("-inf"))
            return self.timeout is not None and 0 <= age <= self.timeout
        mode, status = self.fields.get("timesync_mode"), self.fields.get("timesync_status")
        time_mode, frame = self.fields.get("time_mode"), self.fields.get("frame_sync")
        clock = {0: "PTP_SYNC_FAILED", 1: "PTP_SYNC_SUCCESS", 2: "PTP_SYNC_TIMEOUT"}.get(status, "UNKNOWN")
        if status is not None and self.timeout is not None and not fresh("DIFOP"):
            clock = "PTP_SYNC_TIMEOUT"
        phase = {0: "FRAME_NOT_SYNCED", 1: "FRAME_SYNCED"}.get(frame, "UNKNOWN")
        return {**self.fields, "mode_state": "PTP_MODE" if mode == 2 else "NON_PTP_OR_UNKNOWN",
                "clock_state": clock, "scan_phase_state": phase,
                "clock_sync_valid": mode == 2 and time_mode == 2 and status == 1 and fresh("DIFOP") and fresh("MSOP"),
                "scan_phase_sync_valid": frame == 1 and fresh("MSOP"),
                "observation_sequence": self.sequence}
