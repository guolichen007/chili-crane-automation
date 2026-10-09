# Changelog

## Unreleased — Phase 0.5-R1 Architecture Repair

- 修复固定慢速跨命令复位；一次接收的ActiveExecutionContext持续tick及短输出租约。
- 独立command/actuation序号，拒绝重放、模式接管和过期续跑，STOP序号阻断迟到ON。
- 严格模式矩阵、Z上下独立证据、CANCELLED/ABORTED、集中轴/动作/方向权限验证。
- 24DI canonical物理点表派生逻辑视图；CalibrationRef绑定传感器和版本。
- BREAKING CHANGE：ActuationRequest序号拆分、移除FaultEvent、manifest v2、DI v4；
  全消费者clean rebuild，不能使用旧消息或双映射配置。
- CI事件范围替代永久基线SHA，环境包版本进入证据；镜像digest锁定仍OPEN。
- 实际ROS executor只发OFF，mock拒绝所有ON；租约仅纯软件边界，不冒充物理看门狗。

## Phase 0.5 original freeze (unreleased)

- 新增reported设备资产、24DI物理/逻辑分层、Y/Z独立能力与停止模型。
- 新增Executor、ActuationRequest、readiness、SystemMode、重复Cycle和typed events。
- 新增session/epoch/sequence、STOP dominance与27项纯策略/故障情景。
- 新增证据元数据/RunManifest/禁用录包骨架、工程推送门控和外部审查入口。
- BREAKING CHANGE：硬件订阅ActuationRequest，TaskState21改ABORTED，新增消息字段；
  全消费者clean rebuild，不能混用旧生成消息或推断实机能力。
- 仅接口与mock基线，无生产release或hardware/bag/field验收。

## 0.2.0 - ROS2 Humble / 硬件台架框架

- ADR 0003 切换 Ubuntu22.04/Humble/Python3.10/GCC11，保留旧ADR/验证脚本为历史。
- 七包 ament/rosidl/rclpy、安装配置和 launch.py、有限 VOLATILE QoS。
- 初始24DI包含遥控器输入，逐点极性/接线未知；遥控/自动仲裁 fail-closed。
- ADAM TCP、Y拉绳 RTU/校准、标准化状态、只读台架与门控单路短脉冲。
- WDT/FSV OFF证据、独占会话及崩溃锁存阻止读循环掩盖通信看门狗。
- Windows纯协议检查、Python3.10静态CI、Humble容器 build/test/mock与原生Ubuntu交接。
- 本次未迁NDT算法、接真实设备或启用无人控制；所有现场项 NOT_RUN。

## Historical bootstrap (0.1)

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
