# ADR 0001: Runtime Baseline

- Status: `PROPOSED`
- Decision owner: project technical lead

## Context

The reusable NDT upstream is a ROS1 Noetic/catkin workspace. A historical
procurement sheet specifies Ubuntu 22.04 for edge servers. The project has not
yet confirmed whether to:

1. use Ubuntu 20.04 + ROS1 Noetic for direct upstream compatibility;
2. use Ubuntu 22.04 with a controlled container/source-built ROS1 stack;
3. port the new project to ROS2 while selectively extracting algorithms.

This decision changes build tooling, message/runtime integration, dependency
versions, deployment, and validation.

## Phase 0 position

The package skeleton follows ROS1/catkin interfaces because it minimizes the
first NDT extraction risk. This is not a final OS approval. No Ubuntu build is
claimed.

## Required evidence before acceptance

- confirmed edge-server OS policy;
- support/maintenance requirements;
- dependency availability for PCL, Sophus, ndt_omp, and drivers;
- build proof on the selected clean image;
- deployment and recovery strategy;
- expected project lifetime and migration cost.
