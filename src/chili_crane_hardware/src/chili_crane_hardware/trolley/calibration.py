"""Explicit Y calibration; no raw units, zero or scale is inferred."""
import math
from dataclasses import dataclass
from .modulo import modular_delta


@dataclass(frozen=True)
class PullWireCalibration:
    calibration_id: str
    scale_m_per_unit: float
    offset_m: float
    direction: int
    zero_raw: float
    min_m: float
    max_m: float
    raw_modulus: int | None = None
    calibration_class: str = "NOT_CONFIGURED"
    production_approved: bool = False

    def validate(self):
        if not isinstance(self.calibration_id, str) or self.calibration_id in {"", "NOT_CONFIGURED"}:
            raise ValueError("calibration_id is NOT_CONFIGURED")
        values = (self.scale_m_per_unit, self.offset_m, self.zero_raw, self.min_m, self.max_m)
        if not all(type(x) in (float, int) and math.isfinite(x) for x in values):
            raise ValueError("calibration is NOT_CONFIGURED")
        if self.scale_m_per_unit <= 0 or type(self.direction) is not int or self.direction not in (-1, 1):
            raise ValueError("invalid calibration scale/direction")
        if self.min_m >= self.max_m:
            raise ValueError("invalid calibrated range")
        if self.raw_modulus is not None:
            modular_delta(self.zero_raw, self.zero_raw, self.raw_modulus)

    def production_ready(self):
        return self.calibration_class == "PRODUCTION" and self.production_approved is True

    def position(self, raw):
        self.validate()
        if not math.isfinite(raw):
            raise ValueError("raw must be finite")
        relative = (raw - self.zero_raw if self.raw_modulus is None else
                    modular_delta(raw, self.zero_raw, self.raw_modulus))
        return self.position_relative(relative)

    def position_relative(self, relative):
        self.validate()
        if not math.isfinite(relative):
            raise ValueError("relative counts must be finite")
        position = self.direction * self.scale_m_per_unit * relative + self.offset_m
        if not self.min_m <= position <= self.max_m:
            raise ValueError("position outside calibrated range")
        return position
