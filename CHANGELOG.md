# Changelog

## Unreleased

### Added

- Phase 0 repository governance and architecture review.
- Six-package ROS1/catkin skeleton.
- Hardware-neutral messages and contracts.
- Fail-closed safety and mock adapter nodes.
- Semantic-map, map-manifest, hardware, sensor, and perception templates.
- Mapping/localization/replay/mock profile separation.
- Windows static repository contract checks and Ubuntu handoff runbook.
- Ubuntu 20.04/ROS Noetic hardening contracts, Python 3.8 static CI, C++ public
  header compile tests, and an exact-SHA Ubuntu validation script.
- One-time authorized-command and execution-state interfaces.
- Dual-camera and canonical task-state configuration contracts.

### Changed

- Froze the Phase 1 runtime baseline at Ubuntu 20.04.6, ROS Noetic,
  `catkin_tools`, Python 3.8, C++17, and GCC 9.x.
- Split X/Y safety permissions by direction and removed raw intent consumption
  from the hardware-adapter boundary.
- Decoupled servo CSV replay scheduling from ROS time.
