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
    return (all(isinstance(manifest.get(k), str) and manifest[k] not in ("", "NOT_CONFIGURED")
                for k in required)
            and re.fullmatch("[0-9a-f]{40}", manifest.get("git_sha", "")) is not None
            and re.fullmatch("[0-9a-f]{64}", manifest.get("config_hash", "")) is not None
            and bool(manifest.get("sensor_calibration_ids"))
            and all(isinstance(v, str) and v not in ("", "NOT_CONFIGURED")
                    for v in manifest["sensor_calibration_ids"]))
