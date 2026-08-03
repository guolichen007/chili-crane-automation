# Ubuntu Validation Runbook

Run only against an exact pushed SHA.

Accepted Phase 1 baseline:

```text
OS: Ubuntu 20.04.6 LTS
ROS: ROS Noetic
BUILD_SYSTEM: catkin_tools
PYTHON: 3.8
CXX: C++17
COMPILER: GCC 9.x
```

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

ADR 0001 fixes the target baseline but does not constitute build evidence.
Use `scripts/validation/ubuntu20_phase0_validate.sh` from an exact pushed SHA.
Until that script and the runbook are actually executed on Ubuntu, build,
roslaunch, bag, and field results remain `NOT_RUN`.

Example invocation from the repository root:

```bash
scripts/validation/ubuntu20_phase0_validate.sh \
  --workspace "$PWD" \
  --evidence-dir "$PWD/../chili-crane-evidence" \
  --expected-sha "$(git rev-parse HEAD)"
```

Use an evidence directory outside the tracked repository. The script verifies
the OS/ROS/Python/GCC baseline, exact SHA, clean tree, dependencies, catkin
build/tests, launch entry, false-default permit, and `NOT_CONFIGURED` mock
hardware topics before writing any PASS status.
