#!/usr/bin/env python3
"""Deterministic Monte Carlo counterexamples; outputs synthetic-only JSON evidence."""
import argparse
import itertools
import json
from pathlib import Path
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/chili_crane_slam/src"))
sys.path.insert(0, str(ROOT / "src/chili_crane_perception/src"))
from chili_crane_perception.synthetic import fixture, scene, pit
from chili_crane_perception.pipeline import GrabPerceptionPipeline
from chili_crane_perception.geometry import PitBoundaryModel, convex_xy
from chili_crane_perception.registration import equivalent_pose
from chili_crane_slam.calibration_geometry import transform


def run():
    config = fixture()
    boundary = PitBoundaryModel(pit(), "map")
    rows, timings, stages = [], [], {}
    unsafe, bottom_violations, false_tracked = 0, 0, 0
    for index, (occlusion, noise, single, span) in enumerate(itertools.product(
            (0, .25, .6), (0, .003, .012), (False, True), (.05, .15))):
        pipeline = GrabPerceptionPipeline(config)
        truth = np.eye(4)
        truth[:3,3] = [0.1 if index%3 == 0 else 3.05 if index%3 == 1 else 4.1, -.1, 2]
        truth[:3,:3] = Rotation.from_euler("xyz", [.03, -.04, .12]).as_matrix()
        result = None
        for frame in range(4):
            stamp = 10 + frame * .1
            points = scene(config["model"], truth, stamp, noise=noise, occlusion=occlusion,
                           outliers=20, one_sensor=single, span=span, seed=index*10+frame)
            result = pipeline.process(points, stamp, "CLOSED", "SYNTHETIC", np.eye(6)*1e-5)
            if result["envelope"] is not None:
                start = time.perf_counter()
                clearance = boundary.clearance(result["envelope"])
                elapsed = time.perf_counter() - start
                result["total_seconds"] += elapsed
                result["stage_seconds"]["clearance"] = elapsed
            timings.append(result["total_seconds"])
            for name, elapsed in result["stage_seconds"].items():
                stages.setdefault(name, []).append(elapsed)
        row = dict(index=index, occlusion=occlusion, noise=noise, one_sensor=single,
                   span=span, tracking=result["tracking"], rejected_reasons=result["reasons"])
        if result["tracking"] == "TRACKED":
            nominal = pipeline.model.safety("CLOSED")
            true_vertices = transform(nominal, truth)
            true_clearance = boundary.signed_clearance(convex_xy(true_vertices))
            estimate = boundary.clearance(result["envelope"])["conservative_clearance_m"]
            canonical = equivalent_pose(pipeline.tracker.pose, truth, pipeline.model.period)
            row.update(translation_error_m=float(np.linalg.norm(canonical[:3,3]-truth[:3,3])),
                rotation_error_rad=float(np.linalg.norm(Rotation.from_matrix(truth[:3,:3].T@canonical[:3,:3]).as_rotvec())),
                bottom_error_m=abs(result["envelope"].bottom_nominal-(truth[:3,:3]@np.array([0,0,-.6])+truth[:3,3])[2]),
                true_clearance_m=true_clearance, conservative_clearance_m=estimate)
            unsafe += int(estimate > true_clearance + 1e-6)
            bottom_violations += int(result["envelope"].bottom_lower_bound > true_vertices[:,2].min() + 1e-6)
        rows.append(row)
    # A wall-shaped false object must not become TRACKED simply because ICP converges.
    pipeline = GrabPerceptionPipeline(config)
    for i in range(4):
        points = scene(config["model"], np.eye(4), 20+i*.1)
        points["z"] = 2
        result = pipeline.process(points, 20+i*.1, "CLOSED", "SYNTHETIC", np.eye(6)*1e-5)
        false_tracked += int(result["tracking"] == "TRACKED")
    percentiles = lambda values: dict(P50=float(np.percentile(values,50)), P95=float(np.percentile(values,95)), MAX=float(max(values)))
    return dict(scope="SYNTHETIC_ONLY_NOT_FIELD", scenarios=rows,
        tracked_scenarios=sum(r["tracking"]=="TRACKED" for r in rows),
        false_TRACKED_count=false_tracked, unsafe_optimistic_clearance_count=unsafe,
        bottom_conservative_violation_count=bottom_violations,
        runtime_seconds=percentiles(timings), stages_seconds={k:percentiles(v) for k,v in stages.items()},
        FIELD_EXTRINSIC_STATUS="NOT_RUN", GRAB_FIELD_VALIDATION_STATUS="NOT_RUN",
        PHYSICAL_ACTUATION_STATUS="NOT_RUN", FULL_AUTO_CYCLE_STATUS="NOT_RUN")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = run()
    if args.output:
        if args.output.resolve().is_relative_to(ROOT) and ".local-data" not in args.output.parts:
            raise ValueError("Evidence must be outside versioned source")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    summary = {k:v for k,v in result.items() if k not in ("scenarios","stages_seconds")}
    print(json.dumps(summary, ensure_ascii=False))
    return int(result["false_TRACKED_count"] or result["unsafe_optimistic_clearance_count"]
               or result["bottom_conservative_violation_count"] or result["tracked_scenarios"] == 0)


if __name__ == "__main__":
    raise SystemExit(main())
