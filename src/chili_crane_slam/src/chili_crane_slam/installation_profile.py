"""Explicit temporary sensor-only overlay; no changes to default site or time core."""
from pathlib import Path
import shutil
import tempfile
import yaml


def prepare(config_root, name):
    root = Path(config_root)
    if name not in {"installation_preview", "calibration_capture"}:
        raise ValueError("EXPLICIT_INSTALLATION_PROFILE_REQUIRED")
    profile = yaml.safe_load((root / ("runtime/" + name + ".yaml")).read_text())
    for key in ("physical_output_enabled", "automatic_control_enabled", "camera_control_authority", "spatial_production_merger_enabled"):
        if profile.get(key) is not False:
            raise ValueError("UNSAFE_INSTALLATION_PROFILE:" + key)
    result = Path(tempfile.mkdtemp(prefix="chili-sensor-only-"))
    shutil.copytree(root, result / "config")
    for sensor in ("204", "205"):
        path = result / ("config/sites/crane_01/sensors/er1_" + sensor + ".yaml")
        cfg = yaml.safe_load(path.read_text())
        if cfg.get("port_roles") != "VALID":
            raise ValueError("PORT_ROLE_EVIDENCE_REQUIRED")
        cfg["driver_enabled"] = True  # Only the explicitly selected temporary copy.
        path.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")
    dual = result / "config/sites/crane_01/sensors/dual_lidar.yaml"
    cfg = yaml.safe_load(dual.read_text())
    cfg["timing_only"] = True
    dual.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")
    return result / "config"
