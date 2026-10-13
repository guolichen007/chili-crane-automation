"""State time/validity contracts independent of ROS publication time."""
import math


def validity_for_reason(reason):
    if any(k in reason for k in ("TIMEOUT", "STALE", "FRESHNESS", "DEADLINE")):
        return "STALE"
    if any(k in reason for k in ("NOT_CONFIGURED", "MISSING", "CALIBRATION", "FRAME_MISMATCH", "MAP_MISMATCH")):
        return "NOT_CONFIGURED"
    if any(k in reason for k in ("SHADOW", "PREDICTED", "CONFLICT", "SUPPORT", "OBSERVABILITY", "GATE", "LOST")):
        return "DEGRADED"
    return "UNKNOWN"


def epoch_diagnostic(measurement_epoch, publication_time):
    if not all(math.isfinite(v) for v in (measurement_epoch, publication_time)):
        raise ValueError("TIME_DOMAIN_UNKNOWN")
    return measurement_epoch, publication_time-measurement_epoch


def field_extrinsic_eligible(state, calibration_id, model_source, source_type):
    return (state == "FIELD_VALIDATED" and isinstance(calibration_id, str)
            and calibration_id not in ("", "NOT_CONFIGURED", "UNKNOWN")
            and model_source == source_type == "PHYSICAL")
