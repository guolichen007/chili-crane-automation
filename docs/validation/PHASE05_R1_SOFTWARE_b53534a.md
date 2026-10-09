# Phase 0.5-R1 exact-SHA software evidence

EVIDENCE_SHA: b53534a79b79d4b3af01921fdee5f31ac0aff13c
BASE_SHA: fce3fa0aa49d4278b72a65b350741cd73f85cf4b
BRANCH: codex/architecture-freeze-r1
EVIDENCE_SCOPE: UBUNTU22_HUMBLE_CONTAINER_SOFTWARE_AND_SYNTHETIC_MOCK_ONLY
DATE_UTC: 2026-10-09

## Observed results

WINDOWS_STATIC_STATUS: PASS (121 tests: 120 passed, Linux flock test skipped)
GITHUB_STATIC_STATUS: PASS (121 tests, no skips)
ROS2_BUILD_STATUS: PASS (7 packages)
ROS2_TEST_STATUS: PASS (85 tests, 0 errors, 0 failures, 0 skips)
SYNTHETIC_SCENARIO_STATUS: PASS (46 tests, synthetic-only values)
ROS2_MOCK_STATUS: PASS (14 evidence topics)
PRE_PUSH_STATIC_STATUS: PASS (explicit base, approved SSH review branch)

Observed R1 regressions cover consecutive Y/Z fixed-slow commands, accept-once
multi-tick execution, bounded leases, stale/mode/epoch/clock abort, cancellation,
wrong-route/action/target rejection before sequence consumption, all directional
permit mappings, STOP replay barrier, mode/reset graph, independent Z readiness,
canonical physical DI validation, keyed calibration and fail_cycle return semantics.
These are software policy tests, not timing or safety certification.

## Source evidence

- [Static workflow](https://github.com/guolichen007/chili-crane-automation/actions/runs/37908473904)
- [Humble build/test/mock workflow](https://github.com/guolichen007/chili-crane-automation/actions/runs/37908473786)
- Humble job113747600793 reports the exact EVIDENCE_SHA, package/test counts and
  MOCK_FAIL_CLOSED_STATUS. The software-evidence artifact includes validation log,
  build/test results, software-environment and apt-package-versions.
- Image digest observed in container initialization log:
  sha256:860e53793ccf575e5369b5e9a19939de2bfcfb46a299954d06f54f2af54fed19
  (reported for this run, not pinned in workflow; future reproducibility remains OPEN).

This document archives a completed run, not its own commit or an unspecified
future HEAD. Reviewers must check the branch's final exact SHA against Actions.
Previous Phase0.5 software evidence remains historical and is not replaced.

## Not tested or enabled

NATIVE_UBUNTU_STATUS: NOT_RUN
LIVE_ADAM_STATUS: NOT_RUN
LIVE_PULL_WIRE_STATUS: NOT_RUN
DUAL_LIDAR_ALGORITHM_STATUS: NOT_RUN
PHYSICAL_DO_STATUS: NOT_RUN
BAG_STATUS: NOT_RUN
FIELD_STATUS: NOT_RUN

ROS Executor publishes only OFF; the mock rejects every ON request.
Lease expiry yields logical OFF only, not verified physical relay release.
Real polling deadlines, trusted adapter session setup, WDT/FSV, stopping dynamics,
point assignments, calibration and perception algorithms remain open.
No main merge, force-push, production release or repository-setting change occurred.
