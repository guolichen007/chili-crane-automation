# State and Safety Contract

## Validity

Public state messages use:

```text
UNKNOWN
NOT_CONFIGURED
STALE
DEGRADED
VALID
```

Every message also carries a reason. `VALID` means the message-specific
contract is satisfied; it does not mean all automatic motion is safe.

## Normalized state

Adapters translate vendor data into SI units, explicit timestamps, validity,
faults, limits, calibration identity, and reason strings. Core modules never
read vendor registers.

`GrabIoState` is the raw electrical boundary for grab limits, drive state and
mode. The perception stack may consume it but exclusively owns the fused
`GrabState` pose, geometry, height and opening classification.
`ControlBoardState` separately carries E-stop, power, mode, heartbeat and board
fault evidence.

## Control intent

`ControlIntent` expresses a semantic request such as move X, move Y, raise,
lower, open, close, or stop on `control/requested_intent`. It is not permission
and must never be consumed by a hardware adapter.

## One-time command authorization

The safety/authorization boundary creates an `AuthorizedCommand` only by
binding one requested `intent_id` to a specific `permit_id`, permit generation,
issue time, and expiry time. ActionExecutor checks permit identity, generation,
intent, session, epoch, sequence and expiry before producing an ActuationRequest.
The hardware adapter consumes only control/actuation_request and rejects
unverified energization. De-energize/STOP is not blocked by an expired permit;
without device evidence a logical STOP is not physical OFF completion.

```text
Planner / task -> control/requested_intent
Safety gate    -> control/authorized_command
Executor       -> control/actuation_request
Adapter        -> control/execution_state
```

A prior permit cannot authorize a later intent by topic freshness alone. Phase
0 publishes only invalid/expired authorization records for integration testing
and never produces physical output.

## Safety permit

`SafetyPermit` contains false-by-default permissions:

- X positive and X negative movement independently;
- Y positive and Y negative movement independently;
- lower;
- raise;
- grab open;
- grab close;
- unload;
- automatic task.

Directional permissions allow a separately implemented safety policy to block
motion toward a hazard while preserving an explicitly evaluated escape
direction. A permit heartbeat does not make stale input evidence fresh.

## Conflict examples

- both grab limits active -> `CONFLICT`, block grab motion;
- LiDAR grab state disagrees with a limit -> `DEGRADED`/`CONFLICT`;
- invalid localization -> block geometry-dependent automatic X/Y;
- stale grab track -> block lowering;
- load missing -> never classify contact/load success;
- manual mode or e-stop -> block autonomous output;
- communication timeout -> release/stop outputs according to board contract.

## 遥控器与24DI（当前确认）

6052 8DI + 6251 16DI，共24DI；遥控器输入包含在总容量内。
ControlBoardState.safety_ok/safety_ok_known 与 e_stop/e_stop_known 独立。
RemoteControlState 预留接收器就绪、模式、安全、X/Y/Z/G方向、停止及冲突状态。
缺少点位、未知极性、陈旧采样或反向同时有效均阻止自动接管。
mode_policy 返回释放自动输出、作废待执行命令、要求新任务的决策；
它不是 SafetyPermit。正常框架未配置静止/OFF证据，所以 AUTO_PENDING 也不能运动。
24物理输入与可选语义名称不是一一等同；缺失语义必须保持未知，不能猜配线。
台架DO工具是独立隔离测试入口，不连接规划/感知/自动授权链。

## Phase 0.5 接口升级

完整定义见PHASE05_CONTRACTS。硬件仅接ActuationRequest，不接原始授权；
Stop/de-energize不因许可过期而拒绝，但没有physical OFF反馈不能宣称动作完成。
自动energize需要session/epoch/sequence、授权与permit绑定及动作readiness有效。
新增EvidenceMetadata未知时保持阻断；TaskState不再表示遥控状态。
R1规定命令只accept一次、tick持续复核证据；输出command_sequence与actuation_sequence
分开，有限租约到期的逻辑DO必须OFF。状态机禁止启动/故障恢复直跳AUTO_READY。
取消为CANCELLED，安全/模式中止为ABORTED；仅SafetyEvent按domain表示故障。
详见ADR0005和PHASE05_CONTRACTS第9节，不能把纯租约测试当作物理看门狗验收。

## Phase2A-R1：方向限位与固定作业区gate

OperationalGeofence引用SemanticMap V1，不复制pit坐标。SUSPENSION_POINT经X servo、
Y pull-wire和axis-to-map标定得到；map/version/pit/两轴标定/geofence ID/hash必须可追溯。
旧轨道机械限位不得默认为坑区方向限位。owner确认绑定后，Y negative坑限位禁止negative，
positive方向需单独几何评估；双限位冲突、连续位置与限位窗口矛盾或缺证据禁止自动资格。
STOP只能去励磁；反向逃离仍是励磁，绝不能绕过其它安全门禁。

物理方向禁止链+ADAM DI监督是推荐层级，但硬接/安装/极性/断线未确认，状态NOT_RUN。
Y生产拉绳标定、X homing/通信/标定、FIELD axis-to-map和停车/反应能力未验证时阻断。
shadow方向结果仅有限direction_probe路径的必要条件，既非SafetyPermit也不代表真实自动readiness。
固定强先验不替代动态Grab envelope/clearance，不clamp感知、不掩盖摆动越界。

消息迁移：GrabState/GrabGeometry header=MID_SCAN状态epoch；clearance继承，不用publish-now。
GrabGeometry新增model/version/extrinsic ID/state与时延；GrabClearance另带map/version/pit。
内部exact positive minimum，接触0；cross/outside=0哨兵、clearance_defined=false、geometry_violation=true。
消费者须重建消息，读取defined/inside/risk，不得沿用“负数=穿透深度”的判断。
NOT_CONFIGURED/STALE/DEGRADED/UNKNOWN分开，始终fail-closed且valid_for_control=false。

## R1.1 合同更正

Geofence配置v2删除X limit signal死字段，拒绝旧版本/旧字段；X只输出方向几何诊断。
物理/驱动X限位由独立轴安全层核验，不能把默认false当已观测；ServoState消息不升级。
Y通过统一canonical解析绑定negative/positive，known_signals不够或bool未知即Y_LIMIT_BINDING_UNKNOWN。
允许左右信号反向绑定；y_scale负号不改变绑定语义，窗口和方向仍属轴位置坐标。
directional_geometry_available替换automatic_eligibility，只有shadow必要条件，不是自动任务/控制许可。
Clearance外层age与嵌套EvidenceMetadata age同时采样，异常也同步；重复发布保持epoch/counter，最终STALE。
