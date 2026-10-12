#!/usr/bin/env python3
"""Explicit human lifecycle review; never copies a candidate into production config."""
import argparse
from pathlib import Path
import sys
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/chili_crane_slam/src"))
from chili_crane_slam.calibration_geometry import lifecycle


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--candidate", type=Path, required=True)
    p.add_argument("--state", choices=["REVIEWED", "FIELD_VALIDATED", "INVALIDATED"], required=True)
    p.add_argument("--operator", required=True)
    p.add_argument("--mount-revision", required=True)
    p.add_argument("--confirm-report", action="store_true")
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    if a.output.exists() or a.output.resolve() == a.candidate.resolve():
        p.error("new reviewed artifact required; do not overwrite provenance")
    candidate = yaml.safe_load(a.candidate.read_text())
    reviewed = lifecycle(candidate, a.state, a.operator, a.mount_revision,
        a.confirm_report and candidate.get("quantitative_pass") is True, candidate.get("independent_validation") is True)
    a.output.write_text(yaml.safe_dump(reviewed, allow_unicode=True, sort_keys=False), encoding="utf-8")


if __name__ == "__main__":
    main()
