# Project Context

## Phase2C-R1.1 窄修（2026-10-14，当前授权）

BASE_SHA=1c14ac89d6429a31fa1cfcd3253b9eb7feabe56a；新分支 codex/phase2c-r1-1-closeout。
当前指令 SUPERSEDES 下方历史 skip CI 要求：本轮正常 push，不跳 CI、不改原 C 分支、不 merge main、不进入 D。
R1.1 载荷确认只依赖冻结 bound surface 与原 load/motion 门禁；新料面可含 UNKNOWN，下一候选必须过原 B 局部门禁。
updated clear 保守要求受扰区全部有可靠新证据，独立于 payload。session restart active 要 abort，terminal 后 full rescan；
generation 只在同 session 比较。ledger 按 run 有界，terminal 新 run 可重建。默认记录 STOP_ONLY，全根目录预算未确认保持 null。
完整 shadow stack 组合现有 B/C 节点，commissioning=false；所有控制 flags 保持 false。
R1_1_CODE_STATUS=WINDOWS_STATIC_PASS_UBUNTU_PENDING，最终 exact SHA 的八包 normal/symlink、installed C/console、DDS 均待 Ubuntu。
历史 stable-lift timeout 的现存归档没有逐门载荷/运动采样，根因 OPEN；原生耦合复现不冒充历史 ROS 复现或修复。
详见 [R1.1接口](api/PHASE2C_R11_CONTRACTS.md) 和 [软件审查/工控机待验](validation/Phase2C-R1.1软件审查与工控机待验.md)。

## Phase2C 抓取交互证据与调试台（2026-10-14，当前授权）

当前代码4af2bb0378c134800d34de6178b004b6fa4d8f6e：C0-C4与加固已完成，C5交付归档。
最新用户指令：无需本轮Ubuntu安装/测试，交工控机Ubuntu22.04；最终提交skip CI防自动发起作业，保留全部验证入口/门禁。
Windows425项=419PASS/6缺ROS跳过，pytest同数；diff/合同/compileall/46架构场景通过。
状态WINDOWS_STATIC_PASS_UBUNTU_ACCEPTANCE_PENDING，不能称整体软件封板。历史0f8正常八包/372包测试通过，
installedC稳定提升payload/updated-clear等待timeout未定位，最终代码补诊断/反证，由工控机按最终SHA确认。
详见[交付与待验记录](validation/Phase2C软件交付与工控机待验.md)及[手册](deployment/Phase2C抓取交互与调试监控台手册.md)。

BASE_SHA=077d0bfe44d8fe34fc7572f52cff65362cbc83d8；分支codex/phase2c-grasp-interaction-evidence-v1。
Phase2A、Phase2B已软件shadow封板；本轮新增证据层，不重写其核心，也不另造TaskCycles。
项目处理生产豆瓣酱的辣椒酱/辣椒物料，不按刚性固体料堆假定。抓斗侵入、闭合、取料会导致塌陷、重排或局部流动。
单抓链：SCAN → GraspProposal → TargetBinding → 下降 → 接触/侵入 → 闭斗 → 提升 → 新料面更新 → 下一SCAN。
TargetBinding建立后，本cycle的proposal/candidate、目标XYZ、capture footprint、surface snapshot及
map/calibration/geofence/kernel lineage全部冻结；下降/接触/闭斗中的新proposal不能替换目标。
换点必须ABORT当前cycle → 新SCAN → 新cycle_id → 新TargetBinding；lineage改变只输出STALE/CONFLICT/ABORT_REQUIRED。

料面生命周期：抓前generation=N可规划，TARGET_BOUND后N作为当前cycle不可变证据保留。
第一次可靠interaction即局部失效受扰区域，独立于最终是否抓到料；未受扰坑区可保留。
抓斗未脱离原料面/遮挡区时，受扰cell不得用旧点恢复。CLEAR_OF_BOUND_SURFACE只开启重新观察，非最终安全事实。
脱离后继续Phase2A抓斗mask，只接收post-interaction新点；达到既有覆盖率/新鲜度/稳定性/lineage质量后产生N+1或更高新世代。
再用更新料面证明CLEAR_OF_UPDATED_SURFACE；发生interaction的下一cycle必须使用generation > bound generation。
无新点保持UNKNOWN/DEGRADED，不以插值或抓前数据伪造刷新。

长期工具需求：Crane Commissioning Console / 辣椒行车调试与运行监控台，独立ROS工具包。
覆盖SIL、遥控器模拟、shadow证据监控、事件时间线、运行数据/ROS bag保存、隔离台架DO、
人工万用表电压记录、现场故障复盘及后续参数优化。默认仅127.0.0.1；production不反向依赖/默认启动工具包。
模拟遥控是SOURCE_SIMULATED observation，不是物理授权；X DO未确认，不猜造映射。
台架输出显式门禁、独占会话、FSV/WDT/OFF证据与有限单通道pulse；软件readback不等于万用表或电机方向证据。
FIELD极性、接触/载荷绝对门槛、坑高程、机械能力未确认继续null/NOT_CONFIGURED，FIELD/BENCH实机验收NOT_RUN。
Phase2C最多PASS_SOFTWARE_SHADOW_NOT_FIELD；SafetyPermit=false，AuthorizedCommand=0，ActuationRequest=0，
physical_output_enabled=false，automatic_control_enabled=false，camera_control_authority=false，valid_for_control=false。

## Phase2B-R1.1 最终软件封板（2026-10-14）

用户授权基于8ee41886dfb8d084b2b01d505b770c98bf2fefde建立codex/phase2b-final-seal-r1-1。
范围仅两个P1：真正点时间MID_SCAN的Grab遮罩共时，以及surface producer进程session/restart身份。
HEADER继续FIRST_POINT身份；MID_SCAN从合并点timestamp的min/max计算，不改dual LiDAR/PTP/vendor。
surface_session_id进入status/snapshot/proposal/cache；新session先清空旧状态，再验证新鲜空间证明。
所有冻结算法、Phase2A门限、SemanticMap V1、Geofence v2保持不变；不扩搜索/ESDF/payload/performance算法。
9f497a20664cd2146ef4865bd54e9d3f260b6743已通过Static/Ubuntu normal+symlink七包、各326测试、
A/B/R1.1 installed反证及所有旧入口/vendor，三控制topics各0；新进程独立probe及12份日志核验通过。
PHASE2B_SOFTWARE_STATUS=SEALED_SOFTWARE_SHADOW_NOT_FIELD；证据见[软件封板验收](validation/Phase2B-R1.1软件封板验收.md)。
同runner精确base前后配对无明显性能回归，非现场SLA；归档后的最终HEAD仍按精确SHA完整验CI。
不继续Phase2B-R1.2。
剩余任务仅FIELD gate或另行授权的Phase2C。PHYSICAL disturbance仍不接入，四控制flags保持false。

## Phase2B-R1 FIELD-readiness（2026-10-13）

用户授权从ac0dec1d589d92ab8215d1a0f7b5207762bddbd4建立codex/phase2b-field-readiness-r1。
原Phase2B保持PASS_SOFTWARE_SHADOW_NOT_FIELD；深审发现FIELD就绪契约需要窄加固，历史验收不改写为FAIL。
本轮仅稳定融合/遮罩共时与身份、候选搜索/料量硬门禁、稀疏传输/世代缓存/QoS及分阶段软件证据。
Phase2A核心和门限、PTP/vendor、SemanticMap V1、Geofence v2均冻结；不做Phase2C/NDT/X协议。
接口V2需全体消费者重建；FIELD参数继续null/NOT_CONFIGURED，四控制flags继续false。
LOST+dense+wall+multiple与TRANSITION core+real moving jaws+continuous material两个现场组合缺口仍POST_R1_FIELD_VALIDATION_REQUIRED。
本轮验收单独记录于[Phase2B-R1软件验收](validation/Phase2B-R1软件验收.md)。
R1当前代码cabd809b8ed98c8cb926b9b0f9fb5429feb05278的Static/ROS/vendor均通过，
normal/symlink七包及各314测试、A/B installed shadow、稀疏CDR/缓存/QoS/发布龄和旧入口通过。
PHASE2B_R1_STATUS=PASS_SOFTWARE_SHADOW_NOT_FIELD。归档后的最终HEAD仍按精确SHA独立验CI，现场状态不升级。

## Phase2B 新阶段（2026-10-13）

用户已授权基于d6f846f1f98e621f18e2f6a124253e083ee68bae建立独立Phase2B分支。
PHASE2A_FINAL_SOFTWARE_STATUS=PASS_SOFTWARE_SHADOW_NOT_FIELD；PHASE2A_FURTHER_SYNTHETIC_EXPANSION=STOP。
七包保持；perception料面/mask/扰动，control纯提议；Geofence v2/PTP/vendor/V1及Grab主算法冻结。
explicit spatial chain+observed-only格网+footprint hard gates+proxy/ranking；所有输出valid_for_control=false。
PIT_SURFACE_FIELD_STATUS/GRASP_PROPOSAL_FIELD_STATUS/MATERIAL_VOLUME_PROXY_FIELD_STATUS=NOT_RUN。
实测kernel/CAD/进入深度/生产门槛仍NOT_CONFIGURED；payload留Phase2C。接口与验收见ADR0011和算法文档。

## Phase2A-R1.1 合同收口（历史封板，2026-10-13）

基线5a121f8；分支codex/phase2a-r1-1-contract-closeout。保持既有R1感知算法。
Y negative/positive统一按owner canonical配置解析（含反向绑定和负map比例），X物理限位职责独立。
Geofence模板contract_version=2，删除X死字段；directional_geometry_available不代表自动控制许可。
Clearance内外age同步并保持epoch/counter，超时/异常fail-closed。所有现场状态和四控制flags不升级。
两类组合缺口标记POST_R1_FIELD_VALIDATION_REQUIRED，参见现场OPEN项；该历史收口轮不做Phase2B。
新的Phase2B授权与范围见本页上方最新阶段，不覆盖或改写历史验收。

## Phase2A-R1 当前补充（2026-10-13）

dc42fb16原版软件shadow已交付。本轮按ADR0010增加固定作业区OperationalGeofence，
引用SemanticMap V1，采用X servo+Y pull-wire+坑区方向限位+FIELD axis-to-map独立强先验；
不替代动态Grab包络。当前坑限位安装/硬接、X/Y FIELD标定、停车与地图现场验证均未执行。
原y_left_limit/y_right_limit仅可经owner确认后绑定坑区语义，模板不猜DI或窗口。
R1加固TRACKING_CORE、模型门控、IO几何冲突、MID_SCAN状态epoch、可观信息归一化与clearance身份。
valid_for_control/physical_output_enabled/automatic_control_enabled/camera_control_authority仍false。
Windows仅静态/合成；CI Ubuntu容器与现场独立。主链PTP/vendor/main不改，不进入Phase2B。

## Phase1C 当前补充（2026-10-12）

基线 72019e0d 的 S15 时间链已由 owner 回传 FROZEN；本轮不修改 PTP 主链或生产 UTC 级别。
只增加双 ER1 sensor-only 安装/机械 seed/静态采集/离线 GICP+VGICP、
多场景和独立验证/候选 overlay、相机只读安装预览；不做生产空间合并、NDT、Pit、Grab或运动。
未知 rig parent、机械测量、ROI、验收门限保持未配置；动态 rig 禁止静态 base TF。
所有控制输出保持 false，现场四项状态保持 NOT_RUN。
此前 Phase1B 不做外参是阶段历史边界，现由 ADR0008 仅扩展离线外参工具，不扩展自动授权。

## Phase 1B 当前补充（2026-10-10）

双 ER1 台架接入与统一时间基准；单台 MV-CS060-10GC 只做基础图像及时间证据。
时钟显式 SENSOR_PTP / HOST_DERIVED；每点 start/end/mid，MID_SCAN consume-once 配对。
HOST_DERIVED 只 PROVISIONAL，PTP 与硬件时间戳能力/服务运行分开验证。
相机 raw device/host/ROS receive 三份证据，单位和 epoch 未证明时 RECEIVE_ESTIMATE。
current phase 不做空间 merged、外参、NDT、视觉 AI 或运动；camera_control_authority=false。
既有物理输出/自动控制仍 false。详见 ADR0006 和 Phase1B Ubuntu 交接。

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

## Phase 0.5-R1 评审修补（2026-10-09）

依据fce3fa0精确SHA的用户评审，只修七项契约/生命周期问题，不扩大框架。
ADR0005补充ADR0004：固定慢速新命令复位、accept-once/tick与短输出租约、
轴/动作/方向先验证再消费命令序号、模式转换矩阵、Z上下独立证据、
CANCELLED/ABORTED与单一SafetyEvent、canonical24DI和带身份标定出处。
配置未知时仍不接收动作；SAFE_IDLE需物理停止/OFF证据，故障reset不直达自动。
租约只证明逻辑输出应OFF，不能证明原厂回路/实际继电器已OFF。
当前ROS运行仍全OFF/mock，真实deadline、WDT/FSV、硬件/bag/现场均未验收。

## 当前Phase1C/Phase2A（2026-10-12，取代旧阶段的“尚无驱动/时间事实”描述）

Phase1A/B/R2与Phase1C已交付，基线72234843d5850947c5a2fd5e230516bc8fa64839。
用户S15回传PTP台架事实见hardware/20261012_S15时间链封板事实.md；
S16A ER1 GICP/VGICP smoke及约3.2mm/0.05deg差异仅USER_REPORTED。
BENCH_FLOOR_001不是FIELD外参，正式5+1外参到现场再做。
docs/项目进展.md作为长期事实源；旧阶段记录保留。
Phase2A按ADR0009实现模型驱动Grab shadow及保守几何，不做Pit/planner/contact/NDT。
FIELD外参、CAD、pit/bag仍未验收，control authority禁用；TRACKED不代表控制有效。
详情见algorithms/Phase2A抓斗状态估计与几何安全.md。
