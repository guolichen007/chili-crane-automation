#!/usr/bin/env python3
"""Capture provenance and hash recordings; never commit raw bags or infer field PASS."""
import argparse
import hashlib
import json
import subprocess
import time
import uuid
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]


def config_fingerprint(site):
    parts = [(p.relative_to(site).as_posix(), hashlib.sha256(p.read_bytes()).hexdigest())
             for p in sorted(site.rglob("*.yaml"))]
    return hashlib.sha256(json.dumps(parts, separators=(",", ":")).encode()).hexdigest()


def create(site, output, source, scene):
    output = output.resolve()
    if output == ROOT or ROOT in output.parents or output.exists():
        raise ValueError("new artifact directory outside repository required")
    if source not in {"PHYSICAL", "SIMULATED", "REPLAY", "SYNTHETIC"}:
        raise ValueError("explicit evidence source required")
    sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    data = {"run_id": str(uuid.uuid4()), "git_sha": sha, "config_hash": config_fingerprint(site),
        "source_type": source, "started_at_unix": time.time(), "scene_description": scene,
        "calibrations": {sensor: yaml.safe_load((site / "calibration" / (sensor + "_to_base.yaml")).read_text())
                          for sensor in ("er1_204", "er1_205")}}
    data["calibrations"]["pull_wire_y"] = yaml.safe_load(
        (site / "hardware/pull_wire_y.development.yaml").read_text())["calibration"]
    output.mkdir(parents=True)
    (output / "run_manifest.yaml").write_text(yaml.safe_dump(data, allow_unicode=True), encoding="utf-8")


def finalize(output):
    manifest_path = output / "run_manifest.yaml"
    data = yaml.safe_load(manifest_path.read_text())
    files = list((output / "rosbag2").rglob("*.db3")) + list((output / "rosbag2").rglob("*.mcap"))
    if not files:
        raise ValueError("no bag files; capture NOT_RUN/FAILED")
    hashes = []
    for path in sorted(files):
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        hashes.append({"path": path.relative_to(output).as_posix(), "sha256": digest.hexdigest(),
                       "size_bytes": path.stat().st_size})
    info = yaml.safe_load((output / "rosbag2/metadata.yaml").read_text())["rosbag2_bagfile_information"]
    counts = {x["topic_metadata"]["name"]: x["message_count"] for x in info["topics_with_message_count"]}
    required = ["/crane_01/lidar/er1_204/points", "/crane_01/lidar/er1_205/points", "/crane_01/system/run_manifest"]
    data.update({"duration_sec": info["duration"]["nanoseconds"] / 1e9,
        "files": hashes, "topic_message_counts": counts,
        "capture_complete": all(counts.get(name, 0) > 0 for name in required),
        "field_acceptance": "NOT_RUN"})
    (output / "bag_manifest.yaml").write_text(yaml.safe_dump(data, allow_unicode=True), encoding="utf-8")
    return data["capture_complete"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=["create", "finalize"])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--site", type=Path, default=ROOT / "config/sites/crane_01")
    parser.add_argument("--source", choices=["PHYSICAL", "SIMULATED", "REPLAY", "SYNTHETIC"])
    parser.add_argument("--scene", default="NOT_REPORTED")
    args = parser.parse_args()
    if args.mode == "create":
        create(args.site, args.output, args.source, args.scene)
    elif not finalize(args.output):
        raise SystemExit("capture incomplete; see manifest")
