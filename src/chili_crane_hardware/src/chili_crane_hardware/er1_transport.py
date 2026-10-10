"""Pinned RSE1 transport/signature adapter; no device writes or payload retention."""
from ipaddress import IPv4Address


def packet_role(payload):
    """Length plus minimal decoder ID, never an inference from UDP port."""
    if len(payload) == 1200 and payload[:4] == bytes.fromhex("55 aa 5a a5"):
        return "CONFIRMED_MSOP"
    if len(payload) == 256 and payload[:8] == bytes.fromhex("a5 ff 00 5a 11 11 55 55"):
        return "CONFIRMED_DIFOP"
    return "UNKNOWN"


def driver_transport(cfg):
    """Return only parameters supported by the pinned RoboSense SDK."""
    mode = cfg.get("transport_mode")
    if mode not in {"UNICAST", "MULTICAST", "BROADCAST"}:
        raise ValueError("TRANSPORT_NOT_CONFIGURED")
    host = IPv4Address(cfg.get("host_address"))
    destination = IPv4Address(cfg.get("destination_address"))
    if host.is_multicast or host.is_unspecified or host.is_loopback or int(host) == 0xffffffff:
        raise ValueError("HOST_ADDRESS_INVALID")
    group = cfg.get("group_address")
    if mode == "MULTICAST":
        if not group:
            raise ValueError("MULTICAST_GROUP_REQUIRED")
        if not IPv4Address(group).is_multicast or destination != IPv4Address(group):
            raise ValueError("MULTICAST_DESTINATION_MISMATCH")
        return {"host_address": str(host), "group_address": str(IPv4Address(group))}
    if group:
        raise ValueError("NON_MULTICAST_GROUP_FORBIDDEN")
    if mode == "UNICAST" and destination != host:
        raise ValueError("UNICAST_DESTINATION_HOST_MISMATCH")
    if mode == "BROADCAST" and not str(destination).endswith(".255"):
        raise ValueError("BROADCAST_DESTINATION_INVALID")
    return {"host_address": str(host), "group_address": "0.0.0.0"}
