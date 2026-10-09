"""Load explicit axis capabilities and provenance, without enabling hardware."""
import hashlib
import json
import re
from .contracts import AxisCapability, DriveProfile


def capability_from_config(config, axis):
    data = config.get(axis.lower(), {})
    profile = data.get("drive_profile", "NOT_CONFIGURED")
    try:
        profile = DriveProfile(profile)
    except ValueError:
        profile = DriveProfile.NOT_CONFIGURED
    return AxisCapability(
        profile, data.get("verified") is True, data.get("speed_command_supported") is True,
        data.get("stop_distance_verified") is True, data.get("reaction_time_verified") is True,
        data.get("fixed_speed_verified_safe") is True, data.get("pulse_jog_allowed") is True,
        data.get("minimum_on_time"), data.get("minimum_off_time"))


def configuration_hash(config):
    canonical = json.dumps(config, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def manifest_ready(manifest):
    required = ("run_id", "task_id", "cycle_id", "map_id", "grab_geometry_version")
    if not all(isinstance(manifest.get(k), str) and manifest[k] not in ("", "NOT_CONFIGURED")
               for k in required):
        return False
    if (not isinstance(manifest.get("git_sha"), str)
            or re.fullmatch("[0-9a-f]{40}", manifest["git_sha"]) is None
            or not isinstance(manifest.get("config_hash"), str)
            or re.fullmatch("[0-9a-f]{64}", manifest["config_hash"]) is None):
        return False
    required_sensors = manifest.get("required_sensor_ids")
    refs = manifest.get("calibrations")
    if not isinstance(required_sensors, list) or not required_sensors or not isinstance(refs, list):
        return False
    if any(not isinstance(v, str) or v in ("", "NOT_CONFIGURED") for v in required_sensors):
        return False
    if len(set(required_sensors)) != len(required_sensors):
        return False
    seen = set()
    for ref in refs:
        if not isinstance(ref, dict) or any(
                not isinstance(ref.get(k), str) or ref[k] in ("", "NOT_CONFIGURED")
                for k in ("sensor_id", "calibration_id", "extrinsic_version", "intrinsic_version")):
            return False
        if ref["sensor_id"] in seen:
            return False
        seen.add(ref["sensor_id"])
    return set(required_sensors).issubset(seen)
