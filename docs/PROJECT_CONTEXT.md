# Project Context

## 当前版本确认（2026-10-09）

运行基线已切换 Ubuntu 22.04.x / ROS2 Humble / Python 3.10 / GCC 11 / ament/colcon，
见 ADR 0003。当前是框架及隔离台架准备，不是自动抓取上线。
初始 I/O 为 ADAM-6052 的 8DI 加 ADAM-6251 的 16DI，**合计 24DI，包含遥控器控制输入**。
设备/通道/有效电平的逐点接线仍 OPEN；旧交接 V1.1 的最低 8DI 不是当前容量。
遥控状态必须经硬件标准化接口进入控制仲裁，不能由感知直接驱动输出。


## 1. Objective and current phase

Build an automatic crane system for indoor chili pits. Phase 1 focuses on one
crane and one dedicated edge server. Multi-crane scheduling is reserved at the
namespace and interface level but is not part of the initial closed loop.

Phase 0 creates a durable software framework while LiDAR models, device
protocols, calibration values, board I/O, and final safety parameters remain
unconfirmed.

## 2. Target operating cycle

1. Receive `task_id`, `crane_id`, `pit_id`, unloading station, and action.
2. Validate the task against the semantic map and system readiness.
3. Localize the bridge crane along its fixed track.
4. Move X to the selected pit region and position trolley Y.
5. Synchronize and merge the two 3D LiDAR streams in the crane-base frame.
6. Build a 2.5D surface model inside the selected pit safety inset.
7. Select a graspable region using coverage, height, wall clearance,
   roughness/slope, reachability, and confidence.
8. Track the grab pose, bottom height, and outer envelope.
9. Permit lowering only when localization, grab tracking, target, wall
   clearance, limits, mode, and communication evidence are valid.
10. Lower the grab; use LiDAR height as primary position evidence and
    load/tension changes only as auxiliary contact evidence.
11. Close the grab and confirm the hardware closed-limit signal.
12. Verify load evidence, raise to a safe height, and move to the unloading
    station.
13. Confirm unloading conditions, open the grab, and confirm the hardware
    open-limit signal.
14. Return to a safe wait position and report the final state/evidence.

## 3. Field scene

Current planning assumptions:

- indoor storage/processing building with repeated beams, columns, and pits;
- rectangular pits approximately `7 m x 7 m`;
- pit depth approximately `5 m`;
- crane/sensors approximately `5 m` above ground;
- maximum sensor-to-pit-bottom distance may approach `10 m`;
- chili surfaces contain peaks, slopes, collapse regions, and post-grab holes;
- smooth pit walls must not be struck or scraped;
- a relatively fixed unloading cart/station is in the passage/work area;
- one rail group may later contain two cranes;
- Phase 1 uses one crane, currently planned to serve one side/about seven pits,
  pending survey confirmation.

Site photos show strong fixed structural geometry for registration, but also
highly repetitive column/pit patterns. This supports structure-prioritized NDT
with servo/track gating and argues against unrestricted place recognition.

## 4. Confirmed current equipment direction

### 4.1 Per crane

- one dedicated edge server;
- two side-mounted 3D LiDARs;
- two side-mounted industrial cameras as auxiliary evidence;
- X: bridge travel, servo/encoder position available;
- Y: trolley travel, ordinary motor, draw-wire displacement as primary
  position feedback;
- Z: hoist, ordinary motor, no draw-wire displacement sensor in the current
  baseline;
- G: grab open/close actuation;
- ADAM-6052/6251 remote I/O between the edge server and intermediate relays;
- deterministic safety/interlocks remain in the crane manufacturer's electrical layer;
- no PLC in the current baseline;
- original e-stop, limits, brake, contactors, and electrical safety remain
  authoritative.

### 4.2 X bridge axis

The normalized servo/encoder position is a strong longitudinal prior. NDT
matches fixed building structure, while the semantic map constrains the rail
direction and maps physical position to pit/station identity.

Unknown: brand, protocol, raw units, absolute/relative nature, homing, update
rate, scale, direction, offset, and fault semantics.

### 4.3 Y trolley axis

The current confirmed baseline uses one draw-wire displacement sensor as the
primary Y position feedback. The architecture supports direction/stop and
later speed/jog control if a VFD is available.

Time-only position estimation is not acceptable as the primary Y measurement.

Unknown: sensor model, signal type, range, calibration, update rate, cable
routing, motor control mode, VFD availability, braking/stopping behavior, and
limit/fault wiring.

### 4.4 Z hoist axis

The current confirmed baseline does not use a draw-wire sensor for Z. The
actual grab bottom height is obtained primarily from real-time LiDAR grab
tracking. Load/tension is auxiliary contact/load evidence, not a position
measurement.

Original upper/lower limits, brake state, motor faults, and electrical safety
remain mandatory.

Unknown: motor/brake circuit, speed control, safe-height reference, hard-limit
wiring, fallback behavior when the grab is occluded, and achievable LiDAR
refresh/accuracy through the full Z range.

### 4.5 Grab

The grab is a known double-shell mechanism. It is confirmed to provide:

- `OPEN_LIMIT`;
- `CLOSED_LIMIT`.

The normalized interface reserves open, close, stop/output release, motor
fault, running feedback, current feedback, mode, and e-stop state. Open/closed
completion must not be inferred from elapsed command time.

No grab IMU or opening-angle sensor is assumed in the Phase 1 baseline.
Cameras may support anomaly review but are not the sole state source.

### 4.6 Load/tension

A load/tension module provides a calibrated measurement and change evidence.
It may help classify empty/contact-candidate/loaded/fault states. Missing load
evidence is never success, and load alone never defines Z.

### 4.7 Cameras and unloading station

Cameras are auxiliary for grab condition, anomalies, unloading review, and
acceptance evidence. The unloading detector remains hardware-neutral and
publishes present/aligned/full/overflow state with validity and age.

## 5. Coordinate frames

Reserved frames:

```text
map
track_01
crane_01/base
crane_01/trolley
crane_01/grab
crane_01/lidar_left
crane_01/lidar_right
```

Map, servo, trolley, grab, and sensor-local coordinates must never be mixed.

## 6. Semantic map

DWG/CAD conversion is offline. The real-time system consumes a versioned
semantic map containing:

- map and track identifiers;
- rail origin/direction and valid X range;
- pit IDs and polygons;
- wall/top and bottom elevations;
- safety inset and optional preferred grasp regions;
- unloading ROI/target;
- safe wait pose;
- restricted/no-go regions;
- calibration anchors/home references.

The semantic map assigns pit identity. SLAM does not invent pit numbering.

## 7. Perception requirements

### 7.1 Dual-LiDAR input

- configurable topics, frames, and extrinsics;
- bounded queues and consume-once timestamp pairing;
- pair-time, age, drop, and fallback diagnostics;
- no reuse of an old frame to fake a dual-LiDAR pair;
- explicit single-LiDAR fallback policy;
- transform both clouds into `crane_01/base` before fusion.

### 7.2 Separate point-cloud products

Registration uses fixed beams, columns, walls, and stable edges. It masks pit
interiors, chili, grab, people, temporary equipment, and movable carts.

Task perception separately produces a pit surface and a grab observation.
These products must not be conflated.

### 7.3 Pit surface and grasp target

For the selected pit:

- crop by semantic polygon;
- exclude a configurable wall band;
- remove the tracked grab envelope when valid;
- build a 2.5D grid with robust height percentiles;
- publish point count, coverage, roughness/slope, validity, and age;
- score an area that fits the grab footprint, not a single maximum point;
- publish rejection reasons when no safe target exists.

### 7.4 Grab tracking

Combine the expected ROI, known open/closed geometry, LiDAR observations,
discrete limits, task motion constraints, and optional camera evidence.
Publish center/pose, bottom height, outer envelope, source, confidence,
validity, and evidence time.

### 7.5 Wall clearance

Combine semantic wall geometry, grab envelope, localization covariance, grab
tracking uncertainty, and configured safety margin. The result is a safety
permission, not merely a marker.

## 8. Control and safety

The control boundary separates:

1. normalized device state;
2. requested high-level `ControlIntent`;
3. `SafetyPermit`;
4. a one-time `AuthorizedCommand` bound to intent, permit generation, and
   expiry;
5. ActionExecutor -> ActuationRequest -> a device-specific adapter and CommandExecutionState.

All autonomous outputs fail closed. Invalid localization blocks
geometry-dependent X/Y motion. Stale grab tracking blocks lowering. Limit,
manual mode, e-stop, communication loss, and conflicting open/closed signals
block the affected actions.

Hardware adapters never consume raw requested intents.

Software supplements but never replaces certified electrical safety.

## 9. Task states

The Phase 0 interface reserves:

```text
IDLE
TASK_ACCEPTED
LOCALIZATION_READY
NAVIGATING_TO_PIT
PIT_POSITIONED
SCANNING_PIT
GRASP_TARGET_READY
POSITIONING_TROLLEY
WAITING_STABLE
LOWERING
CONTACT_DETECTED
CLOSING_GRAB
LOAD_VERIFY
RAISING_TO_SAFE_HEIGHT
NAVIGATING_TO_UNLOAD
UNLOAD_READY
OPENING_GRAB
UNLOAD_VERIFY
RETURNING_TO_SAFE_WAIT
DONE
FAULT
ABORTED
```

Phase 0 defines the interface and skeleton only.

## 10. Evidence and source conflicts

The latest V2 field plan is the current equipment baseline. Two older artifacts
conflict with it:

- an old procurement sheet lists draw-wire sensors for both Y and Z;
- an older V1 plan proposes Z encoders/draw-wire and a grab IMU/angle sensors.

Those assumptions are superseded for the current baseline. The V2 plan itself
contains one contradictory sentence saying Phase 1 does not depend on a
draw-wire sensor, while its tables and detailed sections repeatedly confirm Y
draw-wire feedback. This repository interprets it as “Y uses draw-wire; Z does
not” and keeps final field confirmation open.

The upstream remains ROS1 Noetic/catkin and read-only. The current explicit
migration replaces ADR 0001 with ADR 0003: Ubuntu 22.04.x / ROS2 Humble.
Do not retain a second ROS1 implementation or claim the NDT algorithm has been ported.

V1.1 (2026-09-22) confirms the relay boundary, six automatic direction outputs,
hardware mode isolation and separate safety_ok. Its minimum-eight-input table
is SUPERSEDED by the user's current 24DI requirement, including remote inputs.
The PDF's generic real-time-board safety allocation is SUPERSEDED: ADAM handles
I/O; the original crane electrical layer owns hardwired safety and interlocks.
Full 24-channel assignment, unused channels and absent semantic inputs are OPEN.

AUTO to REMOTE releases outputs and discards queued commands. REMOTE to AUTO
requires stopped state, all six outputs OFF readback, valid mode_auto/safety_ok,
no remote conflict and a new task. Software cannot substitute for physical
break-before-make mode isolation or the always-effective original e-stop.

ADAM DI polarity differs by model: do not assume a shared active-high mapping.
Missing e-stop evidence remains unknown and cannot be inferred from safety_ok.
See docs/hardware/HARDWARE_BENCH_RUNBOOK.md for vendor sources and bench gates.

## 11. Known unknowns

Do not guess production values for:

- LiDAR/camera models, topics, rates, timing source, or extrinsics;
- full-range sensor coverage and chili reflectivity;
- servo, control-board, draw-wire, load, VFD, or network protocols;
- coordinate survey accuracy and DWG calibration;
- grab dimensions/CAD and open/closed envelopes;
- safety margins, speed limits, stop distances, and contact thresholds;
- unloading detection source;
- multi-crane collision/scheduling policy.

Every unknown must remain a configuration placeholder, adapter boundary, or
open field question.

## Phase 0.5 当前补充（2026-10-09）

按用户确认实施完整架构冻结，ADR0004及PHASE05_CONTRACTS为当前契约。
双ER1报告地址/端口已进入资产清单，但左右、端口角色、外参和时钟未知；
Camera A报告192.168.180不完整，保持NEEDS_CONFIRMATION且不进控制闭环。
Y/Z调速能力未确认，分别选择VARIABLE_SPEED或FIXED_SLOW；停车/响应/安全速度未经
验收禁止相应自动运动。控制链新增Executor/ActuationRequest和session/epoch/sequence。
任务与系统模式分离，重复Cycle每斗重扫；24物理DI与可选逻辑能力分离。
未实现驱动、NDT、料面、抓斗算法、MES或真实闭环，实机项继续NOT_RUN。
