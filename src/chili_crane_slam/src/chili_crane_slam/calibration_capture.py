"""Bounded, fresh, stationary scene gate. No rig-stopped inference from LiDAR alone."""
import hashlib
import json
import math
from pathlib import Path
from datetime import datetime, timezone
import numpy as np


def capture_gate(now, evidence, revision, parent_mode, ttl, source_type):
    if parent_mode not in {"STATIC_PARENT", "DYNAMIC_PARENT"} or not revision or revision == "NOT_CONFIGURED":
        return False
    if type(ttl) not in (int, float) or not math.isfinite(ttl) or not 0 < ttl <= 10:
        return False
    if set(evidence) != {"204", "205", "dual", "rig"}:
        return False
    for key, item in evidence.items():
        age = now - item.get("received_at", float("-inf"))
        source_age = now - item.get("measurement_at", float("-inf"))
        if not 0 <= age <= ttl or not 0 <= source_age <= ttl or item.get("source_type") != source_type:
            return False
        if item.get("valid") is not True:
            return False
        if key in {"204", "205"} and not all(item.get(k) is True for k in ("normalization_valid", "clock_valid", "phase_valid")):
            return False
        if key == "dual" and not all(item.get(k) is True for k in ("timing_ready", "clock_valid", "phase_valid")):
            return False
        if key == "rig" and (item.get("stationary") is not True or item.get("mount_revision") != revision
                             or not item.get("evidence_id") or item.get("verification_method") not in
                             {"MECHANICALLY_LOCKED_VERIFIED", "MEASURED_RIG_MOTION", "SYNTHETIC_FIXTURE"}):
            return False
        if key == "rig" and source_type == 1 and item.get("verification_method") == "SYNTHETIC_FIXTURE":
            return False
    return True


class SceneAccumulator:
    def __init__(self, count, max_delta, stable_frames):
        if (type(count) is not int or not 20 <= count <= 50 or type(stable_frames) is not int
                or not 2 <= stable_frames <= 100 or type(max_delta) not in (int, float)
                or not math.isfinite(max_delta) or not 0 < max_delta <= 1):
            raise ValueError("CAPTURE_PARAMETERS_NOT_CONFIGURED")
        self.count, self.delta, self.stable_frames = count, max_delta, stable_frames
        self.stable = 0
        self.rows = []
        self.last_ns = {"204": 0, "205": 0}

    def add(self, frames, gate):
        if not gate:
            self.stable = 0
            self.rows.clear()
            return False
        if set(frames) != {"204", "205"}:
            return False
        mids = []
        for sensor, row in frames.items():
            ns = row["canonical_header_ns"]
            offset = row["applied_offset_sec"]
            if ns <= self.last_ns[sensor] or row["raw_header_ns"] - ns != round(offset * 1e9):
                return False
            raw, canonical = row["raw_points"], row["canonical_points"]
            if len(raw) != len(canonical) or not np.array_equal(raw["timestamp"] - offset, canonical["timestamp"]):
                raise ValueError("RAW_CANONICAL_PAIR_MISMATCH")
            mids.append((float(canonical["timestamp"].min()) + float(canonical["timestamp"].max())) / 2)
        if abs(mids[0] - mids[1]) > self.delta:
            return False
        self.last_ns = {s: row["canonical_header_ns"] for s, row in frames.items()}
        self.stable += 1
        if self.stable >= self.stable_frames and len(self.rows) < self.count:
            self.rows.append(frames)
        return len(self.rows) == self.count

    def save(self, output, manifest, repository):
        output = Path(output).resolve()
        repository = Path(repository).resolve()
        if output == repository or repository in output.parents or output.exists() or len(self.rows) != self.count:
            raise ValueError("NEW_EXTERNAL_COMPLETE_SCENE_REQUIRED")
        if manifest.get("split") not in {"CALIBRATION", "VALIDATION"}:
            raise ValueError("EXPLICIT_SCENE_SPLIT_REQUIRED")
        output.mkdir(parents=True)
        files = {}
        for sensor in ("204", "205"):
            for kind in ("raw", "canonical"):
                arrays = [r[sensor][kind + "_points"] for r in self.rows]
                name = "er1_" + sensor + "_" + kind + ".npz"
                path = output / name
                np.savez_compressed(path, points=np.concatenate(arrays),
                    header_ns=np.array([r[sensor][kind + "_header_ns"] for r in self.rows], dtype=np.int64),
                    counts=np.array([len(a) for a in arrays], dtype=np.int64))
                files[name] = hashlib.sha256(path.read_bytes()).hexdigest()
        manifest.update(frame_count=self.count, capture_gate_passed=True, artifacts=files,
                        created_at=datetime.now(timezone.utc).isoformat())
        (output / "scene_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2,
                                                             allow_nan=False) + "\n", encoding="utf-8")
        return manifest
