# NDT-SLAM-Warehouse Reuse Plan

## 1. Inspected upstream

- Repository: `guolichen007/NDT-SLAM-Warehouse`
- Local branch: `master`
- Inspected SHA: `42f921e91574ff0f29b91fc08cbade67c976aa36`
- Role: read-only engineering reference, not chili source of truth

The inspected upstream is a mature ROS1 Noetic crane system with dual-LiDAR
merging, NDT localization, observability-aware EKF, stationary-motion policy,
relocalization, map snapshots/layers, warehouse cargo tracking, safety
protocols, diagnostics, and operational tooling.

Its current Windows static status and historical field tag do not imply that
all current master changes have Ubuntu/bag/field evidence.

## 2. Strategy

1. Keep upstream unchanged.
2. Record the upstream file/SHA for every adapted module.
3. Extract generic low-level mechanisms.
4. Remove warehouse cargo, topic, path, and safety-code coupling.
5. Add chili-specific contracts and tests.
6. Require Ubuntu build/bag evidence for the exact chili commit.

Do not copy the whole upstream workspace.

## 3. Reuse matrix

| Upstream module | Decision | Chili adaptation |
|---|---|---|
| `PointCloudMerger.cpp` | High structural reuse | Generic names/topics, `crane_01/base`, typed diagnostics, real quantiles, explicit fallback policy |
| `registration_cloud_builder.*` | Reuse after decoupling | Replace `CargoObbFootprint` with generic semantic/dynamic masks; pit interiors always excluded |
| `ndt_observability.*` | Near-direct reuse | Preserve directional covariance; add provenance and chili tests |
| `ndt_fitness_circuit_breaker.*` | High reuse | Reset on map/source identity changes; expose typed quality |
| `crane_pose_constraint.*` | High reuse | Constrain crane body to surveyed track; never constrain grab pose with it |
| `crane_motion_ekf.*` | Reuse concept/core | Add normalized servo delta/prior without device protocol coupling |
| `stationary_motion_policy.*` | Reuse concept | Include servo delta/velocity evidence and keep map-write gates |
| `ndt_relocalizer.*` | Later selective reuse | Track/servo-gated candidates and frozen static map; bounded rail-aware search |
| `map_session_snapshot.*` | Partial reuse | Immutable versioned artifacts and atomic publication; no default online mutation |
| point-cloud utilities | Selective reuse | Filters/grids only; replace warehouse ground/object semantics |
| cargo OBB/bottom/temporal code | Algorithm ideas only | New `Grab*` names and known open/closed geometry |
| hook load state/evidence | Pattern reuse | Normalized load/tension states; never use load as Z |
| warehouse safety codes/payload lifecycle | Do not port | Chili uses typed permissions and reasons |

## 4. Findings from direct code inspection

### 4.1 Dual-LiDAR merger

Valuable upstream behavior:

- one- or two-LiDAR configuration;
- immutable extrinsics;
- conversion/transform outside the queue lock;
- bounded timestamp-sorted queues;
- closest-pair search within a bounded queue;
- consume-once semantics;
- stale-timeout single-LiDAR output rather than fake reuse;
- pairing/drop/fallback counters.

Required fixes before extraction:

- output/topic defaults still use warehouse/global semantics;
- console health counters identify sensors by names containing `201`/`203`;
- reported p50/p95/max values currently mirror the latest pair rather than a
  real time window;
- diagnostics are an untyped string;
- `transform_to_base` defaults false;
- fallback state is not yet connected to the chili safety contract.

Phase 1 should preserve the synchronizer structure while replacing these
couplings.

### 4.2 Registration cloud builder

The builder correctly refuses full-ground fallback, requires static structure
point/coverage thresholds, caps ground contribution, and reports
`STRUCTURE_RICH`, `STRUCTURE_RECOVERY`, `GROUND_AUGMENTED`, or
`INSUFFICIENT_STRUCTURE`.

The current cargo-exclusion API directly depends on `CargoObbFootprint`.
Chili needs a generic mask contract supporting:

- semantic pit polygons;
- current grab envelope;
- people/temporary objects;
- movable unloading cart;
- other dynamic/no-registration regions.

The chili surface is excluded even when dense.

### 4.3 Observability, fitness, EKF, and constraints

`NdtObservability` estimates a 2D normal-information proxy, identifies
strong/weak translation directions, rotates them into the measurement frame,
and inflates weak-direction covariance. This is valuable on repetitive tracks.

The fitness circuit breaker uses a median/MAD target-relative baseline,
confirmation streaks, a hard limit, and recovery confirmations. Bad matches
must not enter the filter or map.

The EKF state is `[x, y, vx, vy]` with prediction, NDT update, anisotropic
covariance, NIS/innovation gating, physical step limiting, prediction-only, and
stationary constraints. Chili should first add a separate
`ServoMotionPrior`:

```text
raw servo -> calibration -> map-X/delta prior
                               |
static cloud -> NDT ------------+-> fusion -> track constraint
```

Do not put raw registers or drive brands in the EKF. Do not adapt servo
calibration silently every frame.

`CranePoseConstraint` is appropriate for bridge-body Z/roll/pitch/yaw and rail
lateral constraints, but angular wraparound and surveyed rail-frame behavior
must be reviewed during the port.

### 4.4 Map lifecycle and relocalization

Reuse immutable snapshot/generation/identity concepts and same-filesystem
atomic publication. Start with:

- `mapping`: build and freeze a static track map;
- `localization`: load the frozen map and disable map mutation.

Full long-term online tile mutation is out of Phase 0.

Initial relocalization should be local and rail-aware. If broader recovery is
later enabled, gate candidates by track ID, servo X window, semantic range,
map generation, result age, and multi-frame confirmation.

ScanContext remains disabled by default because repeated pits/columns can cause
false place matches.

### 4.5 Grab and load

Warehouse cargo code offers useful OBB, percentile, temporal continuity,
bottom-height, physical-step, and load-evidence patterns. Public cargo names,
formal/degraded payload authorization, and warehouse safety zones do not match
the known chili grab mechanism.

Chili creates `GrabGeometryModel`, `GrabDetector`, `GrabTracker`,
`GrabBottomEstimator`, and `GrabEnvelope`. Open/closed limit signals select or
validate the geometry state. Conflicting geometry and hardware limits produce
`CONFLICT` and block the affected action.

## 5. Map pipeline

```text
dual LiDAR
  -> consume-once pair + extrinsics
  -> semantic/dynamic masks
  -> structure-preserving registration cloud
  -> servo-seeded NDT
  -> observability + fitness gate
  -> EKF/fusion + track constraint
  -> fused crane pose
```

Mapping outputs:

```text
maps/track_01/
  localization_map.pcd
  display_map.pcd
  semantic_map.yaml
  calibration.yaml
  manifest.json
```

The manifest records map version, source run/bag, source Git SHA, sensor
extrinsic version, semantic-map version, servo calibration version, and
creation identity.

## 6. Provenance record required for every port

```text
UPSTREAM_REPOSITORY:
UPSTREAM_FILE:
UPSTREAM_SHA:
ADAPTATION_SUMMARY:
WINDOWS_STATIC_STATUS:
UBUNTU_BUILD_STATUS:
BAG_STATUS:
FIELD_STATUS:
```

No reused algorithm is “validated for chili” until the relevant evidence is
attached to the exact chili SHA.

## 当前 ROS2 迁移边界

上游仍是 ROS1，只读 SHA 42f921e91574ff0f29b91fc08cbade67c976aa36。
本仓库已改 Humble/ament；本次仅迁移接口与台架框架，不声称完成 NDT/PCL 算法迁移。
后续先抽取无 ROS 的同步、观测性、fitness、轨道约束机制，再编写 ROS2 薄节点、
QoS/时钟/rosbag2 测试。不要复制 catkin、rospy、旧消息/launch或旧CRLF例外。
