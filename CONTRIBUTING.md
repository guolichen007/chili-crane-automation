# Contributing

## Branches

Use small reviewable branches such as:

```text
codex/bootstrap-architecture-v1
codex/dual-lidar-core-v1
codex/servo-slam-fusion-v1
ubuntu/validate-<short-sha>
fix/<short-description>
```

## Before commit

```text
git diff --check
python tools/check_repo_contracts.py
python -m unittest discover -s tests_static -p "test_*.py"
```

Do not report ROS/catkin/bag/field status as passed without evidence from the
matching Ubuntu/site SHA.

## Reused NDT code

Every extracted/adapted module must record upstream repository, file, SHA,
adaptation summary, and validation status. Warehouse public cargo/safety
semantics are not part of the chili API.

## Hardware changes

Add or change vendor protocols only inside adapters. Do not hard-code unknown
registers, paths, units, calibration, or safety thresholds in core logic.
