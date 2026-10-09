# AGENTS.md

## 1. Repository purpose

This repository is the software and algorithm baseline for the chili-pit
automatic crane project.

The repository is being built before all field hardware and I/O contracts are
frozen. Therefore:

- establish stable module boundaries first;
- isolate hardware-specific code behind adapters;
- keep uncertain field parameters configurable;
- do not hard-code guessed hardware interfaces;
- prefer fail-safe `UNKNOWN`, `NOT_CONFIGURED`, `STALE`, or `DEGRADED` states;
- preserve ROS bag replay and replacement of mock adapters with real adapters.

The NDT-SLAM-Warehouse repository is a read-only reference/upstream. It is not
the chili project's source of truth.

## 2. Source-of-truth order

When project materials disagree, use this order:

1. the user's current explicit instruction;
2. the latest confirmed V2 field plan summarized in `docs/PROJECT_CONTEXT.md`;
3. repository decisions under `docs/decisions/`;
4. the handoff documents in `docs/`;
5. historical V1 plans and procurement spreadsheets.

Do not silently merge conflicting assumptions. Record the conflict and mark it
`OPEN` or `SUPERSEDED`.

## 3. Codex role and validation boundary

Desktop Codex on Windows is responsible for:

- repository architecture and large code changes;
- extracting or adapting reusable components from NDT-SLAM-Warehouse;
- implementation, test scaffolding, configuration, and documentation;
- Windows static checks;
- committing and pushing when credentials and permissions allow.

Desktop Codex must not claim:

- ROS 2 Humble build success on Windows;
- PCL, Sophus, or ndt_omp runtime correctness on Windows;
- colcon or ROS2 launch success unless actually run on Ubuntu;
- ROS bag acceptance;
- physical sensor, control-board, or field validation.

Ubuntu validation is responsible for clean colcon builds, C++ tests, ROS2 launch,
bag replay, runtime evidence, and field validation against an exact Git SHA.

## 4. Cross-platform contract

The runtime target is Linux/Ubuntu/ROS even when files are edited on Windows.

The accepted Phase 1 runtime baseline is:

- Ubuntu 22.04.x LTS;
- ROS 2 Humble;
- ament/colcon;
- Python 3.10;
- C++17 with GCC 11.x;
- native Ubuntu runtime.

Windows is limited to editing, static/interface checks, documentation, and Git
operations. ADR 0003 supersedes the ROS1 baseline; the active branch maintains
only ROS2. The previous ROS1 branch is historical evidence.

- all first-party text files use LF;
- text is UTF-8 without BOM;
- every text file ends with one newline;
- shell scripts use POSIX syntax;
- repository/runtime paths use `/`;
- runtime code must not assume a Windows drive letter;
- do not commit generated Visual Studio or colcon build files;
- do not copy legacy CRLF exceptions from the NDT repository.

Before every commit run:

```text
git diff --check
python tools/check_repo_contracts.py
python -m unittest discover -s tests_static -p "test_*.py"
```

Report unavailable commands as `NOT_RUN` with a reason.

## 5. Architecture rules

### 5.1 Hardware independence

Device protocols belong only in hardware adapters. Core logic consumes typed,
normalized state and must not know device registers.

### 5.2 Configuration before constants

Unknown LiDAR topics/extrinsics, servo calibration, pit geometry, grab
dimensions, sensor calibration, safety margins, and speed limits remain
explicitly `NOT_CONFIGURED` until confirmed.

### 5.3 Fail-safe state

No module may infer `SAFE` because a warning is absent. Missing, stale,
conflicting, or unconfigured evidence blocks the affected automatic action.

### 5.4 Mapping and localization are different modes

- `mapping` may construct versioned static track artifacts.
- `localization` loads a frozen map and does not mutate it by default.

The changing chili surface must never become the primary localization map.

### 5.5 Perception, safety, and actuation are separate

Perception publishes observations. Planning publishes targets. Safety publishes
permissions. Control adapters execute only permitted intents. A perception node
must never toggle a hardware output directly.

## 6. NDT reuse policy

Prefer selective extraction over copying the warehouse application. Reuse
proven low-level mechanisms only with provenance and chili-specific tests.

Likely reusable:

- consume-once dual-LiDAR synchronization and extrinsic transformation;
- structure-preserving registration cloud construction;
- NDT observability and fitness circuit breaking;
- crane-body pose constraints and EKF concepts;
- stationary-motion and relocalization confirmation policies;
- immutable map snapshot/lifecycle concepts;
- selected rigid-geometry and load-state algorithms.

Do not transplant warehouse cargo names, safety codes, payload lifecycle, or
online-map mutation policy into the chili public API.

See `docs/NDT_REUSE_PLAN.md`.

## 7. First implementation goal

Phase 0 establishes:

- repository and package boundaries;
- messages and normalized interfaces;
- mock hardware adapters;
- dual-LiDAR and servo-prior contracts;
- semantic-map schema and templates;
- mapping/localization profile separation;
- pit-surface, grasp-target, grab-tracking, safety, and task interfaces;
- static checks and Ubuntu validation handoff.

Stubs are acceptable while protocols are unknown, but they must remain visibly
`NOT_CONFIGURED` or `NOT_READY`.

## 8. 当前硬件框架约束

24DI = ADAM-6052 8DI + ADAM-6251 16DI，包含遥控器输入；逐通道来源和
极性经配置确认。遥控器状态不得直接变成自动授权。厂家协议只进入
chili_crane_hardware。bench 默认只读，DO 仅允许显式门禁和有限脉冲；
production 不加载 bench writer。所有未确认设备参数继续 NOT_CONFIGURED。

## 9. Phase 0.5 / Public delivery

Follow ADR0004 and docs/api/PHASE05_CONTRACTS.md. Hardware consumes ActuationRequest,
not AuthorizedCommand. Separate SystemMode and Task/Cycle; epoch/session/sequence must
invalidate stale commands. STOP may only de-energize, never bypass permission to energize.
Y/Z profiles are independent; unverified stopping/capability blocks automatic actions.
Use CONTRIBUTING.md and Conventional Commit titles. Before SSH push run check_delivery.py
with the exact base SHA. Do not force-push, rewrite public history, touch main, upload local
memory/secrets, or claim production readiness. CI/mock and field evidence stay separate.
