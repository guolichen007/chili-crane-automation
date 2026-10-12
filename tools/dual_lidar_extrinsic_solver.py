#!/usr/bin/env python3
"""Offline scene solver/report CLI; only candidates, explicit review is separate."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile
import numpy as np
import yaml
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/chili_crane_slam/src"))
from chili_crane_slam.calibration_geometry import relative_seed
from chili_crane_slam.extrinsic_solver import solve_scenes


def load_scene(directory):
    directory = Path(directory).resolve()
    manifest = json.loads((directory / "scene_manifest.json").read_text(encoding="utf-8"))
    arrays = {}
    for name, digest in manifest["artifacts"].items():
        path = (directory / name).resolve()
        if path.parent != directory or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError("ARTIFACT_HASH_OR_PATH_INVALID")
    for sensor in ("204", "205"):
        name = "er1_" + sensor + "_canonical.npz"
        path = (directory / name).resolve()
        if path.parent != directory or not path.is_file():
            raise ValueError("ARTIFACT_PATH_INVALID")
        if hashlib.sha256(path.read_bytes()).hexdigest() != manifest["artifacts"][name]:
            raise ValueError("ARTIFACT_HASH_MISMATCH")
        with zipfile.ZipFile(path) as archive:
            if sum(i.file_size for i in archive.infolist()) > 512000000:
                raise ValueError("SCENE_DECOMPRESSION_TOO_LARGE")
        with np.load(path, allow_pickle=False) as data:
            p = data["points"]
            if not len(p) <= 2000000:
                raise ValueError("SCENE_POINTS_TOO_LARGE")
            arrays[sensor] = np.column_stack([p[k] for k in ("x", "y", "z")])
    return {**manifest, "target": arrays["204"], "source": arrays["205"],
            "artifact_sha256": manifest["artifacts"]}


def report(output, artifact):
    (output / "metrics.json").write_text(json.dumps(artifact, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    text = "# 双 ER1 外参标定报告\n\n"
    text += "状态：" + artifact.get("status", "REJECTED") + "；非生产外参，所有控制输出关闭。\n\n"
    text += "变换约定：T_A_B 将 B frame 的点变换到 A frame；目标为 ER1-204，源为 ER1-205。\n\n"
    text += "独立验证与定量验收：" + str(artifact.get("quantitative_pass", False)) + "\n\n"
    text += "\n".join("    " + line for line in json.dumps(artifact, ensure_ascii=False, indent=2, allow_nan=False).splitlines()) + "\n"
    (output / "外参标定报告.md").write_text(text, encoding="utf-8")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--site", type=Path, required=True)
    p.add_argument("--scene", type=Path, action="append", required=True)
    p.add_argument("--parameters", type=Path, required=True)
    p.add_argument("--geometry", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    output = a.output.resolve()
    if output.exists() or output == ROOT or ROOT in output.parents:
        p.error("new output outside repository required")
    rig = yaml.safe_load((a.site / "calibration/sensor_rig.yaml").read_text())
    seeds = [yaml.safe_load((a.site / ("calibration/mechanical_seed_er1_" + s + ".yaml")).read_text()) for s in ("204", "205")]
    seed = relative_seed(*seeds, rig["mount_revision"])
    scenes = [load_scene(d) for d in a.scene]
    cfg, geometry = yaml.safe_load(a.parameters.read_text()), yaml.safe_load(a.geometry.read_text())
    sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    output.mkdir(parents=True)
    try:
        artifact = solve_scenes(scenes, seed, cfg, geometry, rig["mount_revision"], sha)
        (output / "candidate.yaml").write_text(yaml.safe_dump(artifact, allow_unicode=True, sort_keys=False), encoding="utf-8")
    except ValueError as exc:
        artifact = {"status": "REJECTED", "reason": str(exc), "quantitative_pass": False, "git_sha": sha}
        artifact["calibration_scenes"] = getattr(exc, "results", [])
        report(output, artifact)
        raise SystemExit(1) from exc
    report(output, artifact)


if __name__ == "__main__":
    main()
