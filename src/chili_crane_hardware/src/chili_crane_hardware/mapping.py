"""Map device bits to semantic observations; unknown polarity never becomes true."""
import math
from dataclasses import dataclass

DEVICE_CHANNELS = {"adam6052": 8, "adam6251": 16}
from chili_crane_control.evidence_policy import SourceType


@dataclass(frozen=True)
class RawInput:
    bits: tuple
    received_monotonic: float
    evidence_age_sec: float
    valid: bool
    source_type: SourceType = SourceType.UNKNOWN


def requirement_status(values, required_for_algorithm, required_for_auto, optional_observation):
    inventories = (required_for_algorithm, required_for_auto, optional_observation)
    if any(not isinstance(items, (list, tuple)) or any(not isinstance(n, str) for n in items)
           for items in inventories):
        raise ValueError("invalid evidence requirement inventory")
    return {
        "algorithm_ready": all(values.get(n) is not None for n in required_for_algorithm),
        "auto_ready": bool(required_for_auto) and all(values.get(n) is not None for n in required_for_auto),
        "known": sorted(n for n in set(sum((list(x) for x in inventories), [])) if values.get(n) is not None),
        "unknown": sorted(n for n in set(sum((list(x) for x in inventories), [])) if values.get(n) is None),
    }


def semantic_input(name, mapping, samples, now, stale_timeout_sec):
    item = mapping.get(name, {})
    device, channel, invert = item.get("device"), item.get("channel"), item.get("invert")
    if device not in DEVICE_CHANNELS or type(channel) is not int or not (
            0 <= channel < DEVICE_CHANNELS[device]) or type(invert) is not bool:
        return None
    sample = samples.get(device)
    if sample is None or not sample.valid or len(sample.bits) != DEVICE_CHANNELS[device]:
        return None
    elapsed = now - sample.received_monotonic
    age = sample.evidence_age_sec + elapsed
    if elapsed < 0 or not math.isfinite(age) or not 0 <= age <= stale_timeout_sec:
        return None
    raw = sample.bits[channel]
    if type(raw) is not bool:
        return None
    return raw != invert


def normalize(mapping, samples, now, stale_timeout_sec):
    return {name: semantic_input(name, mapping, samples, now, stale_timeout_sec)
            for name in mapping}

def derive_logical_inputs(config):
    """Build the only logical view from the complete 24-channel physical table."""
    if not config:
        return {}
    if "digital_inputs" in config or config.get("contract_version") != 4:
        raise ValueError("legacy/parallel DI mapping is not accepted")
    essential = config.get("essential_signals", [])
    optional = config.get("optional_capabilities", {})
    if (not isinstance(essential, list) or not isinstance(optional, dict)
            or any(not isinstance(n, str) or not n or n == "NOT_CONFIGURED" for n in essential + list(optional))
            or len(set(essential)) != len(essential) or set(essential) & set(optional)):
        raise ValueError("invalid semantic inventory")
    names = set(essential) | set(optional)
    mapping = {name: {"device": "NOT_CONFIGURED", "channel": "NOT_CONFIGURED",
                      "invert": "NOT_CONFIGURED"} for name in names}
    physical = config.get("physical_channels", {})
    if not isinstance(physical, dict) or set(physical) != set(DEVICE_CHANNELS):
        raise ValueError("physical inventory must include exactly two ADAM devices")
    assigned = set()
    for device, count in DEVICE_CHANNELS.items():
        rows = physical[device]
        if (not isinstance(rows, list) or len(rows) != count
                or any(not isinstance(row, dict) or type(row.get("channel")) is not int for row in rows)
                or {row["channel"] for row in rows} != set(range(count))):
            raise ValueError("physical inventory must contain each channel exactly once")
        for row in rows:
            state, signal, invert = row.get("assignment"), row.get("signal"), row.get("invert")
            if state not in {"ASSIGNED", "RESERVED", "NOT_CONFIGURED"}:
                raise ValueError("invalid assignment state")
            if state == "ASSIGNED":
                if signal not in names or signal in assigned or type(invert) is not bool:
                    raise ValueError("unknown/duplicate signal or polarity")
                assigned.add(signal)
                mapping[signal] = {"device": device, "channel": row["channel"], "invert": invert}
            elif signal != "NOT_CONFIGURED" or invert != "NOT_CONFIGURED":
                raise ValueError("unassigned channel must not carry a binding")
    return mapping
