# Chili Crane Automation

Software and algorithm baseline for a dual-LiDAR automatic crane serving
indoor chili storage pits.

This repository currently contains the **Phase 0 architecture/bootstrap**. It
defines stable contracts, fail-safe mock adapters, semantic-map templates,
mapping/localization profiles, and the package boundaries needed before field
hardware protocols are frozen. It is not yet a field-ready automatic control
system.

## Target workflow

The system will receive a pit task, localize the bridge crane on its rail,
scan the selected pit, build a 2.5D chili-surface model, choose a safe grasp
region, track the known grab mechanism, lower/close/raise it, move to the fixed
unloading station, unload, return to a safe wait position, and report evidence
and task state.

The intended localization relationship is:

```text
servo X prior + frozen static LiDAR map + semantic rail constraints
    -> fused crane pose and localization quality
```

Changing chili surfaces are task-perception inputs, not localization-map
evidence.

## Repository layout

```text
docs/                         Requirements, decisions, APIs, and validation
src/chili_crane_msgs/         ROS message contracts only
src/chili_crane_core/         Hardware-neutral domain and geometry contracts
src/chili_crane_slam/         Dual-LiDAR, mapping, localization, and fusion
src/chili_crane_perception/   Pit surface, grasp planning, and grab tracking
src/chili_crane_control/      Safety, task orchestration, and adapters
src/chili_crane_bringup/      Namespaced launch composition
config/                       Site-independent templates and profiles
maps/                         Versioned map artifact conventions
tools/                        Repository contract checks
tests_static/                 Windows-safe static contract tests
```

For a fresh Desktop Codex task, begin with
`docs/CODEX_START_PROMPT.md`. The canonical requirements/reuse/architecture
handoff is linked there.

## Current validation status

| Check | Status |
|---|---|
| Windows repository/static contracts | Run locally before each commit |
| Ubuntu catkin build | `NOT_RUN` |
| ROS launch smoke test | `NOT_RUN` |
| ROS bag replay | `NOT_RUN` |
| Sensor/control-board validation | `NOT_RUN` |
| Chili field acceptance | `NOT_RUN` |

Windows checks do not replace Ubuntu or field evidence.
The initial pre-commit result is recorded in
`docs/validation/WINDOWS_STATIC_20260731.md`.

## Windows static checks

```text
git diff --check
python tools/check_repo_contracts.py
python -m unittest discover -s tests_static -p "test_*.py"
```

## Ubuntu handoff

Ubuntu validation must start from an exact Git SHA and follow
`docs/validation/UBUNTU_VALIDATION_RUNBOOK.md`. The intended runtime baseline
is still a proposed decision because the historical hardware sheet says Ubuntu
22.04 while the reusable upstream is ROS1 Noetic/catkin. See
`docs/decisions/0001-runtime_baseline.md`.

## Reference upstream

Selective reuse is based on `guolichen007/NDT-SLAM-Warehouse`. The inspected
local reference SHA and adaptation matrix are recorded in
`docs/NDT_REUSE_PLAN.md`.
