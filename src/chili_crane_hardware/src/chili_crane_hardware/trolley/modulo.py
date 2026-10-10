"""Continuous modulo coordinates; rejected samples never advance the anchor."""
import math


def modular_delta(raw, reference, modulus):
    if (type(modulus) is not int or modulus < 2 or
            any(type(v) not in (int, float) or not math.isfinite(v)
                or not 0 <= v < modulus for v in (raw, reference))):
        raise ValueError("invalid modulo sample/reference")
    delta = raw - reference
    if abs(delta) == modulus / 2:
        raise ValueError("ambiguous half-modulus displacement")
    if delta > modulus / 2:
        delta -= modulus
    if delta < -modulus / 2:
        delta += modulus
    return delta


class ModuloPosition:
    def __init__(self, modulus, zero_raw, max_delta_per_sample, max_gap_sec):
        modular_delta(zero_raw, zero_raw, modulus)
        if (type(max_delta_per_sample) not in (int, float)
                or not math.isfinite(max_delta_per_sample)
                or not 0 < max_delta_per_sample < modulus / 2
                or type(max_gap_sec) not in (int, float)
                or not math.isfinite(max_gap_sec) or max_gap_sec <= 0):
            raise ValueError("modulo continuity limits must be configured")
        self.modulus, self.zero_raw = modulus, zero_raw
        self.max_delta, self.max_gap = max_delta_per_sample, max_gap_sec
        self.previous = self.previous_time = None
        self.relative = 0.0
        self.discontinuous = False

    def update(self, raw, observed, validate=lambda _: None):
        if self.discontinuous:
            raise ValueError("modulo continuity lost; explicit restart/re-anchor required")
        if type(observed) not in (int, float) or not math.isfinite(observed):
            raise ValueError("invalid observation clock")
        delta = modular_delta(raw, self.zero_raw if self.previous is None else self.previous,
                              self.modulus)
        if self.previous is not None and (abs(delta) > self.max_delta
                or not 0 < observed - self.previous_time <= self.max_gap):
            self.discontinuous = True
            raise ValueError("modulo discontinuity/gap; position not advanced")
        relative = delta if self.previous is None else self.relative + delta
        validate(relative)
        self.previous, self.previous_time, self.relative = raw, observed, relative
        return relative
