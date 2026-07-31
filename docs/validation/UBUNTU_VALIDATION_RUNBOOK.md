# Ubuntu Validation Runbook

Run only against an exact pushed SHA.

## Handoff record

```text
INPUT_SHA:
BRANCH:
EXPECTED_RUNTIME_BASELINE:
SOURCE_NDT_SHA:
WINDOWS_STATIC_STATUS:
UBUNTU_BUILD_STATUS: NOT_RUN
ROSLAUNCH_STATUS: NOT_RUN
BAG_STATUS: NOT_RUN
FIELD_STATUS: NOT_RUN
EVIDENCE_DIRECTORY:
```

## Validation sequence

1. Fetch the branch and verify `git rev-parse HEAD == INPUT_SHA`.
2. Confirm a clean working tree.
3. Record OS, ROS distribution, compiler, PCL, Eigen, Sophus, and driver
   versions.
4. Configure a clean catkin workspace.
5. Run a clean build.
6. Run C++ unit tests and rostests.
7. Run launch/XML/config smoke checks.
8. Replay a known bag with simulated time.
9. Collect topic rates, dual-LiDAR pair timing, drops/fallbacks, CPU/memory,
   localization diagnostics, and failures.
10. Record every command, exit code, and artifact path.
11. Apply only small evidence-driven fixes on an Ubuntu validation branch.
12. Commit fixes without rewriting the original validation SHA.
13. Return `INPUT_SHA`, `OUTPUT_SHA`, changed modules, evidence, remaining
    failures, and next action.

## Phase 0 note

Until ADR 0001 is resolved, the clean build command and base image remain
`NOT_CONFIGURED`. Do not choose an OS silently.
