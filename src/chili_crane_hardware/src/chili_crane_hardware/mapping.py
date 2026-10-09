"""Map device bits to semantic observations; unknown polarity never becomes true."""
import math
from dataclasses import dataclass

DEVICE_CHANNELS = {"adam6052": 8, "adam6251": 16}


@dataclass(frozen=True)
class RawInput:
    bits: tuple
    received_monotonic: float
    evidence_age_sec: float
    valid: bool


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
