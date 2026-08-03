# ADR 0001: Runtime Baseline

- Status: `ACCEPTED`
- Decision owner: project technical lead

## Context

The reusable NDT upstream is a ROS1 Noetic/catkin workspace. A historical
procurement sheet specifies Ubuntu 22.04 for edge servers, while the current
Phase 1 priority is minimizing algorithm-extraction and field-validation risk.

## Decision

Phase 1 freezes the runtime and validation baseline as:

```text
Operating system: Ubuntu 20.04.6 LTS
ROS: ROS Noetic
Build system: catkin_tools
Python: 3.8
C++: C++17
Compiler: GCC 9.x
Runtime: native Ubuntu
```

Algorithm development, ROS bag replay, hardware integration, and field
validation use this baseline. Windows is limited to editing, static checks,
interface tests that do not require ROS, documentation, and Git operations.
Windows evidence never substitutes for an Ubuntu build or runtime result.

The repository does not maintain parallel ROS1 and ROS2 implementations. If a
future purchased server is required to run Ubuntu 22.04, containerization or a
system/ROS migration must be evaluated in a separate ADR and branch.

## Consequences

- the ROS1/catkin package skeleton remains the only active implementation;
- C++ public contracts compile as C++17 targets on the Ubuntu baseline;
- Python runtime code and tooling remain compatible with Python 3.8;
- Ubuntu build, roslaunch, bag, and field status remain `NOT_RUN` until executed
  against an exact pushed SHA;
- the historical Ubuntu 22.04 procurement preference is recorded but does not
  silently alter Phase 1.
