# System Architecture and Roadmap

## 1. Design goal

The framework must survive hardware and protocol changes without rewriting
perception, localization, safety, or orchestration.

## 2. Package boundaries

### `chili_crane_msgs`

ROS interfaces only. Device registers and vendor protocols are forbidden.

### `chili_crane_core`

Hardware-neutral validity, semantic map, geometry, task-state, and policy
contracts. Keep ROS/PCL coupling minimal.

### `chili_crane_slam`

Dual-LiDAR synchronization, extrinsics, static registration cloud, servo prior,
NDT, observability, fitness gating, fusion, rail constraints, relocalization,
and map lifecycle.

### `chili_crane_perception`

Pit surface grid, grasp planning, known-grab detection/tracking/bottom/envelope,
wall clearance inputs, and unloading observations. It never actuates hardware.

### `chili_crane_control`

Normalized I/O, safety supervisor, command arbitration, task state machine,
timeouts/watchdogs, mock adapters, and future device adapters.

### `chili_crane_bringup`

Per-crane namespace, profile composition, mapping/localization/replay/mock
launch, health gates, and later RViz configuration.

Do not create more packages until a clear ownership boundary exists.

## 3. Namespace and frames

Topics are relative inside a crane namespace:

```text
/crane_01/lidar/merged_points
/crane_01/localization/pose
/crane_01/perception/pit_surface
/crane_01/perception/grasp_target
/crane_01/perception/grab_state
/crane_01/safety/permit
/crane_01/control/intent
/crane_01/task/status
```

Frames:

```text
map
track_01
crane_01/base
crane_01/trolley
crane_01/grab
crane_01/lidar_left
crane_01/lidar_right
```

## 4. Data flow

```text
LiDAR left ----\
                -> DualLidarMerger -> merged cloud
LiDAR right ---/                         |
                                           +-> static registration cloud
Servo X -> normalized prior ---------------+-> NDT/fusion -> CranePoseStatus
Semantic map -> rail/pit masks ------------+

merged cloud + selected pit + fused pose
  -> PitSurfaceBuilder -> GraspPlanner -> GraspTarget

merged cloud + expected ROI + known grab model + limit signals
  -> GrabTracker -> GrabState

CranePoseStatus + GrabState + semantic map + normalized hardware health
  -> SafetySupervisor -> SafetyPermit

TaskStateMachine + GraspTarget + SafetyPermit
  -> ControlIntent -> Mock/real adapter -> electrical actuation layer
```

Registration, pit-surface, and grab-tracking outputs are separate products.

## 5. Explicit runtime modes

### Mapping

- semantic map loaded;
- pit interiors and current grab masked;
- static building structure prioritized;
- servo prior used when valid;
- map writes allowed only when localization quality and motion policy allow;
- output is a versioned, reviewable map snapshot.

### Localization

- frozen map loaded;
- production map mutation disabled by default;
- servo provides longitudinal prediction/initial guess;
- NDT provides spatial correction;
- semantic track constraints reject nonphysical drift;
- degraded/invalid quality is published explicitly.

## 6. Servo and NDT fusion contract

Normalized servo input:

```text
stamp
raw_position
position_m
velocity_mps
validity
homed
fault
calibration_id
```

Calibration remains explicit:

```text
map_x = direction * scale * raw_position + offset
```

V1 fusion sequence:

1. validate servo freshness/calibration;
2. derive map-X/delta prior;
3. seed NDT from the prior and previous pose;
4. evaluate convergence, observability, and fitness;
5. accept/reject the NDT correction;
6. fuse accepted evidence;
7. apply rail constraints;
8. publish innovation, covariance, source, and quality.

## 7. Semantic-map and map-artifact lifecycle

DWG/CAD conversion is offline. Runtime loads a versioned YAML/JSON semantic
map. A frozen localization snapshot contains PCD layers plus semantic,
calibration, and manifest files.

Snapshot publication is atomic at the directory/version level. A production
localization process does not overwrite the loaded snapshot.

## 8. Pit surface and grasp planning

Each 2.5D grid cell reserves robust height, point count, coverage, roughness,
slope, and validity. The planner scores a grab footprint using material
height, footprint coverage, wall clearance, roughness, reachability,
visibility, and recent-grab history.

The result always includes confidence and a rejection reason.

## 9. Grab tracking

Tracking states:

```text
UNKNOWN
CANDIDATE
TRACKED
LOST_HOLD
LOST
```

Opening states:

```text
UNKNOWN
OPEN
CLOSED
TRANSITION
CONFLICT
```

Open/closed geometry profiles are distinct. Tracking publishes evidence age,
measurement/prediction source, uncertainty, bottom height, and an outer
envelope used by safety.

## 10. Safety and actuation

Permissions include X/Y movement, lowering, raising, grab open, grab close,
unload, and full automatic task. Every permission is false by default.

Examples:

- localization invalid -> block geometry-dependent automatic movement;
- grab tracking stale -> block lowering;
- wall clearance insufficient -> block lowering/approach;
- upper limit -> block raise;
- lower limit -> block lower;
- manual mode/e-stop/heartbeat loss -> block autonomous outputs;
- conflicting grab limits -> block grab motion.

The real-time board is responsible for deterministic direction interlocks,
limits, action timeouts, communication safe state, and output release. The
edge server sends semantic intents and permissions, not raw uncontrolled
strong-current commands.

## 11. Phase roadmap

### Phase 0 - bootstrap

Governance, messages, six packages, semantic schema, profile separation,
fail-safe mocks, static contracts, architecture decisions, and validation
handoff.

### Phase 1 - dual-LiDAR core

Adapt the consume-once merger, extrinsics, typed timing/health diagnostics, and
explicit fallback policy.

Ubuntu evidence: catkin build, topic smoke, bag pairing/timing diagnostics.

### Phase 2 - semantic map and single-track mapping

Implement offline semantic import, masks, structure cloud, mapping mode, map
manifest, and immutable save/load.

### Phase 3 - servo + NDT localization

Implement calibration, normalized prior, NDT correction, observability,
fitness breaker, EKF/fusion, track constraints, and rail-aware relocalization.

### Phase 4 - pit surface and grasp target

Implement semantic crop, robust height grid, wall inset, footprint candidates,
and scoring. Real chili bags are required for useful tuning.

### Phase 5 - grab tracking

Implement open/closed geometry, LiDAR tracking, bottom/envelope, uncertainty,
and discrete-signal conflict handling.

### Phase 6 - safety and control integration

Implement permission policy, command arbitration, task transitions, board
protocol adapter, watchdogs, and evidence logging after interfaces are known.

### Phase 7 - site closed loop

Validate navigation, scan, plan, lower, contact, close, load verify, raise,
unload, and return on real chili under normal production conditions.

## 12. Validation split

Windows:

- LF/UTF-8/path/static-contract checks;
- message/config/schema checks;
- pure policy tests that do not require ROS/PCL.

Ubuntu:

- clean catkin build;
- C++ gtest/rostest;
- roslaunch smoke;
- ROS bag replay and timing/performance evidence.

Site:

- calibration;
- real chili surfaces and repeated grabs;
- wall-adjacent material, high/low fill, post-grab holes;
- failure injection and complete closed-loop acceptance.
