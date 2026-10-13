"""Independent spatial SHADOW policy; no ROS publisher, device protocol or permit.

Inputs reuse normalized ServoState/TrolleyState attributes (or mapping snapshots).
The configuration REFERENCES SemanticMap V1; it never owns another pit polygon.
Directional booleans are diagnostics, not permission to energize a motor.
"""
import hashlib
import json
import math
import numpy as np
from shapely.geometry import Polygon, Point, LineString


def field(value, key, default=None):
    return value.get(key, default) if isinstance(value, dict) else getattr(value, key, default)


def identity(value):
    return isinstance(value, str) and value not in ("", "NOT_CONFIGURED", "UNKNOWN")


def finite(value, nonnegative=False):
    return (type(value) in (int, float) and math.isfinite(value)
            and (not nonnegative or value >= 0))


def validity_for_reason(reason):
    if any(k in reason for k in ("TIMEOUT", "STALE", "FRESHNESS", "DEADLINE")):
        return "STALE"
    if any(k in reason for k in ("NOT_CONFIGURED", "MISSING", "CALIBRATION", "FRAME_MISMATCH", "MAP_MISMATCH")):
        return "NOT_CONFIGURED"
    if any(k in reason for k in ("SHADOW", "PREDICTED", "CONFLICT", "SUPPORT", "OBSERVABILITY", "GATE", "LOST")):
        return "DEGRADED"
    return "UNKNOWN"


class OperationalGeofence:
    def __init__(self, config, semantic):
        # Snapshot, and hash actual canonical content, not a fabricated identity.
        self.config = json.loads(json.dumps(config, allow_nan=False))
        self.semantic = json.loads(json.dumps(semantic, allow_nan=False))
        c, s = self.config, self.semantic
        self.hash = hashlib.sha256(json.dumps(c, sort_keys=True, separators=(",", ":"),
                                              allow_nan=False).encode("utf-8")).hexdigest()
        if c.get("config_state") != "VALID" or s.get("config_state") != "VALID":
            raise ValueError("GEOFENCE_NOT_CONFIGURED")
        if s.get("schema_version") != 1:
            raise ValueError("SEMANTIC_MAP_VERSION_UNSUPPORTED")
        for key in ("geofence_id", "geofence_version", "config_version", "pit_id"):
            if not identity(c.get(key)):
                raise ValueError("GEOFENCE_ID_NOT_CONFIGURED")
        if (c.get("semantic_map_id") != s.get("map_id") or c.get("semantic_version") != s.get("semantic_version")
                or c.get("frame_id") != s.get("frame_id") or not identity(s.get("map_id"))
                or not identity(s.get("semantic_version"))):
            raise ValueError("SEMANTIC_MAP_MISMATCH")
        if c.get("reference_point") != "SUSPENSION_POINT":
            raise ValueError("REFERENCE_POINT_NOT_CONFIGURED")
        if any(c.get(k) is not False for k in ("physical_output_enabled", "automatic_control_enabled", "camera_control_authority")):
            raise ValueError("SHADOW_FLAGS_REQUIRED")
        if c.get("source_type") not in ("SYNTHETIC", "SIMULATED", "REPLAY", "PHYSICAL"):
            raise ValueError("SOURCE_NOT_CONFIGURED")
        a = c["axis_to_map"]
        expected_state = "FIELD_VALIDATED" if c["source_type"] == "PHYSICAL" else "SYNTHETIC_VALIDATED"
        if a.get("state") != expected_state or not identity(a.get("calibration_id")):
            raise ValueError("AXIS_TO_MAP_CALIBRATION_NOT_CONFIGURED")
        for key in ("map_id", "semantic_version"):
            if a.get(key) != s.get(key):
                raise ValueError("AXIS_TO_MAP_MAP_MISMATCH")
        if a.get("pit_id") != c["pit_id"]:
            raise ValueError("AXIS_TO_MAP_PIT_MISMATCH")
        for axis in ("x", "y"):
            if not identity(c[axis].get("calibration_id")) or a.get(axis+"_calibration_id") != c[axis]["calibration_id"]:
                raise ValueError("AXIS_CALIBRATION_MISMATCH")
            if not finite(a.get(axis+"_scale")) or a[axis+"_scale"] == 0 or not finite(a.get(axis+"_offset_m")):
                raise ValueError("AXIS_TO_MAP_CALIBRATION_NOT_CONFIGURED")
        if c["x"].get("source") != "SERVO_POSITION" or c["y"].get("source") != "RS485_PULL_WIRE":
            raise ValueError("AXIS_SOURCE_NOT_CONFIGURED")
        for key in ("origin_map_xy_m", "x_direction_map_xy", "y_direction_map_xy"):
            p = np.asarray(a.get(key), dtype=float)
            if p.shape != (2,) or not np.isfinite(p).all():
                raise ValueError("AXIS_TO_MAP_CALIBRATION_NOT_CONFIGURED")
        x, y = np.array(a["x_direction_map_xy"]), np.array(a["y_direction_map_xy"])
        if not np.allclose([x@x, y@y, x@y], [1, 1, 0], atol=1e-8):
            raise ValueError("AXIS_TO_MAP_BASIS_INVALID")
        for key in ("evidence_timeout_sec", "direction_probe_m"):
            if not finite(c.get(key)) or c[key] <= 0:
                raise ValueError("POLICY_NOT_CONFIGURED")
        y = c["y"]
        if (y.get("limit_semantics") != "PIT_DIRECTION_LIMIT"
                or not identity(y.get("negative_limit_signal")) or not identity(y.get("positive_limit_signal"))
                or y["negative_limit_signal"] == y["positive_limit_signal"]):
            raise ValueError("Y_LIMIT_BINDING_NOT_CONFIGURED")
        windows = []
        for key in ("negative_expected_window_m", "positive_expected_window_m"):
            w = y.get(key)
            if not isinstance(w, list) or len(w) != 2 or not all(finite(v) for v in w) or w[0] > w[1]:
                raise ValueError("Y_LIMIT_WINDOW_NOT_CONFIGURED")
            windows.append(w)
        if windows[0][1] >= windows[1][0]:
            raise ValueError("Y_LIMIT_WINDOWS_CONFLICT")
        matches = [p for p in s["pits"] if p["pit_id"] == c["pit_id"]]
        if len(matches) != 1:
            raise ValueError("PIT_ID_NOT_CONFIGURED")
        pit = matches[0]
        self.polygon = Polygon(pit["polygon_map_m"])
        if not self.polygon.is_valid or self.polygon.is_empty or self.polygon.area <= 0:
            raise ValueError("PIT_GEOMETRY_NOT_CONFIGURED")
        margin_keys = ("grab_horizontal_radius_m", "maximum_swing_m", "position_uncertainty_m",
                       "calibration_uncertainty_m", "stop_margin_m", "additional_margin_m")
        if any(not finite(c["static_margin"].get(k), True) for k in margin_keys) or not finite(pit.get("safety_inset_m"), True):
            raise ValueError("MARGIN_NOT_CONFIGURED")
        self.margin = sum(c["static_margin"][k] for k in margin_keys) + pit["safety_inset_m"]
        track = s.get("track", {})
        self.track_origin = np.asarray(track.get("origin_map_m"), dtype=float)
        self.track_direction = np.asarray(track.get("direction_map_xy"), dtype=float)
        self.track_range = np.asarray(track.get("valid_x_range_m"), dtype=float)
        if (not identity(track.get("track_id")) or self.track_origin.shape != (2,)
                or self.track_direction.shape != (2,) or self.track_range.shape != (2,)
                or not np.isfinite(np.r_[self.track_origin, self.track_direction, self.track_range]).all()
                or not np.isclose(self.track_direction @ self.track_direction, 1, atol=1e-8)
                or self.track_range[0] >= self.track_range[1]):
            raise ValueError("TRACK_NOT_CONFIGURED")
        # Round inset polygon uses arcs approximated by chords. Inflate radius so
        # each chord remains outside the true forbidden disk (resolution=16).
        self.safe_region = self.polygon.buffer(-self.margin/math.cos(math.pi/64), resolution=16)
        if self.safe_region.is_empty:
            raise ValueError("EMPTY_ADMISSIBLE_REGION")

    def point(self, x, y):
        a = self.config["axis_to_map"]
        return (np.array(a["origin_map_xy_m"])
                + np.array(a["x_direction_map_xy"])*(x*a["x_scale"]+a["x_offset_m"])
                + np.array(a["y_direction_map_xy"])*(y*a["y_scale"]+a["y_offset_m"]))

    def track_inside(self, p):
        position = float((p-self.track_origin) @ self.track_direction)
        return self.track_range[0] <= position <= self.track_range[1]

    def provenance(self):
        c = self.config
        return dict(geofence_id=c["geofence_id"], geofence_version=c["geofence_version"],
                    map_id=c["semantic_map_id"], semantic_version=c["semantic_version"], pit_id=c["pit_id"],
                    x_calibration_id=c["x"]["calibration_id"], y_calibration_id=c["y"]["calibration_id"],
                    axis_to_map_calibration_id=c["axis_to_map"]["calibration_id"],
                    config_version=c["config_version"], config_sha256=self.hash)

    def evaluate(self, servo, trolley, dynamic, age_x=0., age_y=0.):
        c = self.config
        result = dict(provenance=self.provenance(), valid_for_control=False,
                      production_control_eligible=False, automatic_eligibility=False,
                      x_geometry_eligible=False, y_geometry_eligible=False, inside=False,
                      stop_deenergize=True, x_positive=False, x_negative=False,
                      y_positive=False, y_negative=False, reasons=[], validity="DEGRADED")
        reasons = result["reasons"]
        for name, state, received_age in (("x", servo, age_x), ("y", trolley, age_y)):
            axis_reasons = []
            age = field(state, "evidence_age_sec")
            if (not finite(age, True) or not finite(received_age, True)
                    or age+received_age > c["evidence_timeout_sec"]):
                axis_reasons.append(name.upper()+"_STALE")
            if field(state, "validity") not in ("VALID", 4) or field(state, "fault") is not False:
                axis_reasons.append(name.upper()+"_STATE_INVALID")
            if field(state, "calibration_id") != c[name]["calibration_id"]:
                axis_reasons.append(name.upper()+"_CALIBRATION_MISMATCH")
            source = field(field(state, "evidence", {}), "source_type")
            source = {1:"PHYSICAL", 2:"SIMULATED", 3:"REPLAY", 4:"SYNTHETIC"}.get(source, source)
            if source != c["source_type"]:
                axis_reasons.append(name.upper()+"_SOURCE_MISMATCH")
            if not finite(field(state, "position_m")):
                axis_reasons.append(name.upper()+"_POSITION_UNKNOWN")
            if name == "x" and (field(state, "homed") is not True or field(state, "communication_ok") is not True):
                axis_reasons.append("X_UNHOMED_OR_COMMUNICATION_FAILURE")
            if name == "y":
                left, right = field(state, "left_limit"), field(state, "right_limit")
                known = field(state, "known_signals", [])
                if (type(left) is not bool or type(right) is not bool
                        or any(c["y"][k] not in known for k in ("negative_limit_signal", "positive_limit_signal"))):
                    axis_reasons.append("Y_LIMIT_UNKNOWN")
                elif left and right:
                    axis_reasons.append("Y_LIMIT_CONFLICT")
                elif finite(field(state, "position_m")):
                    for asserted, key in ((left, "negative_expected_window_m"), (right, "positive_expected_window_m")):
                        low, high = c["y"][key]
                        inside = low <= field(state, "position_m") <= high
                        if asserted and not inside:
                            axis_reasons.append("Y_POSITION_LIMIT_MISMATCH")
                        if inside and not asserted:
                            axis_reasons.append("Y_EXPECTED_LIMIT_NOT_ASSERTED")
            if c["source_type"] == "PHYSICAL" and c[name].get("production_approved") is not True:
                axis_reasons.append(name.upper()+"_PRODUCTION_CALIBRATION_NOT_CONFIGURED")
            result[name+"_geometry_eligible"] = not axis_reasons
            reasons.extend(axis_reasons)
        # No field stopping proof => not even shadow motion eligibility for physical inputs.
        if c["source_type"] == "PHYSICAL" and any(c[a].get(k) is not True for a in ("x", "y")
                for k in ("stop_distance_verified", "reaction_time_verified")):
            reasons.append("STOP_MODEL_NOT_CONFIGURED")
        if c["source_type"] == "PHYSICAL" and any(not identity(c[a].get("stop_model_evidence_id")) for a in ("x", "y")):
            reasons.append("STOP_MODEL_EVIDENCE_NOT_CONFIGURED")
        if reasons:
            result["validity"] = validity_for_reason("|".join(reasons))
            return result
        p = self.point(field(servo, "position_m"), field(trolley, "position_m"))
        result["reference_xy_m"] = p.tolist()
        result["inside"] = (self.safe_region.covers(Point(p))
                            and self.polygon.boundary.distance(Point(p)) >= self.margin
                            and self.track_inside(p))
        # Dynamic evidence is independent. NEVER clamp a measured Grab into the region.
        if (field(dynamic, "wall_collision_risk") is not False or field(dynamic, "clearance_defined") is not True
                or field(dynamic, "fresh") is not True or field(dynamic, "measurement_accepted") is not True
                or field(dynamic, "tracking_state") != "TRACKED"
                or any(field(dynamic, k) != self.provenance()[k] for k in ("map_id", "semantic_version", "pit_id"))):
            reasons.append("DYNAMIC_GRAB_GEOMETRY_GATE")
        if not result["inside"]:
            reasons.append("REFERENCE_OUTSIDE_GEOFENCE")
        if reasons:
            return result
        a = c["axis_to_map"]
        for axis in ("x", "y"):
            for sign, suffix in ((1, "positive"), (-1, "negative")):
                direction = np.array(a[axis+"_direction_map_xy"])*a[axis+"_scale"]*sign
                target = p + direction/np.linalg.norm(direction)*c["direction_probe_m"]
                safe = self.safe_region.covers(LineString([p, target])) and self.track_inside(target)
                limit = (field(servo, suffix+"_limit", None) if axis == "x"
                         else field(trolley, "right_limit" if sign == 1 else "left_limit"))
                # Unknown optional servo limits must be explicitly observed as false.
                result[axis+"_"+suffix] = bool(safe and limit is False)
        result["automatic_eligibility"] = any(result[k] for k in ("x_positive", "x_negative", "y_positive", "y_negative"))
        return result
