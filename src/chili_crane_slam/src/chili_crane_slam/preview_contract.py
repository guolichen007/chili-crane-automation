"""Preview-only clock diagnostics and explicit T_target_source frame graph."""
import math
import numpy as np
from .calibration_geometry import rigid, mechanical_seed


def receive_fresh(now, arrival, timeout):
    return (all(type(v) in (float, int) and math.isfinite(v) for v in (now, arrival, timeout))
            and timeout > 0 and 0 <= now - arrival <= timeout)


def preview_transform(source, target, rig, seeds, candidate=None):
    if not source or not target or "NOT_CONFIGURED" in (source, target):
        raise ValueError("NOT_AVAILABLE_FRAME_MISMATCH")
    edges = {}

    def edge(a, b, t):
        edges.setdefault(b, []).append((a, rigid(t)))
        edges.setdefault(a, []).append((b, np.linalg.inv(rigid(t))))

    if candidate is not None:
        if (candidate.get("status") not in {"CANDIDATE", "REVIEWED", "FIELD_VALIDATED"}
                or candidate.get("mount_revision") != rig.get("mount_revision")):
            raise ValueError("CANDIDATE_INVALIDATED")
        edge(candidate["target_frame"], candidate["source_frame"], candidate["T_target_source"])
    for seed in seeds:
        try:
            if seed["target_frame"] != rig["rig_frame"]:
                continue
            edge(seed["target_frame"], seed["source_frame"],
                 mechanical_seed(seed, rig.get("mount_revision")))
        except (ValueError, KeyError):
            continue
    if (rig.get("parent_mode") == "STATIC_PARENT" and rig.get("parent_transform") is not None
            and rig.get("parent_evidence_id") not in (None, "", "NOT_CONFIGURED")):
        edge(rig["base_frame"], rig["rig_frame"], rig["parent_transform"])
    pending, visited = [(source, np.eye(4))], set()
    while pending:
        frame, t = pending.pop(0)
        if frame == target:
            return t
        if frame in visited:
            continue
        visited.add(frame)
        pending.extend((following, step @ t) for following, step in edges.get(frame, [])
                       if following not in visited)
    raise ValueError("NOT_AVAILABLE_FRAME_MISMATCH")
