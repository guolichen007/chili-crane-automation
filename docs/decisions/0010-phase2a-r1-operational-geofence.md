# ADR0010：固定作业区强先验与 Phase2A-R1

状态：ACCEPTED_SOFTWARE_DESIGN。日期：2026-10-13。
基线：dc42fb16ad0ca14a38d919a9e26acb8836fef16d。
分支：codex/phase2a-grab-hardening-r1。现场实施与控制验收未执行。

## 决策

采用独立的物理方向限位、轴连续位置、固定 SemanticMap 和动态抓斗包络四层证据。
OperationalGeofence 引用 SemanticMap V1 的 map_id、semantic_version、pit_id；
不复制 pit polygon，不升级 V1 schema。新增 core 类型与 control 纯 Python 策略；
不新增硬件协议、控制节点或 SafetyPermit 发布者。

参考点是 SUSPENSION_POINT，经显式 axis-to-map 标定映射，不是 servo 数字直接当 map.x，
也不是会摆动的 grab center。标定关联 X/Y calibration、地图版本和 pit 身份；
物理输入要求 FIELD_VALIDATED 标定和生产批准。来源不可混用。

可达域由坑多边形保守内缩推导。圆形半径包含抓斗最大水平半径、最大摆幅、位置误差、
标定误差、已验证停车/滑行距离和附加余量，以及既有 pit safety inset。
非矩形、凹多边形、分裂的可达域都支持；半径增大域只能缩小。
配置半径必须包住所有开度的实际水平包络；它不是抓斗姿态修正项。

Y 轨道机械限位与坑区方向限位不是同一概念。现有 y_left_limit/y_right_limit
只有经 owner 确认后才能绑定 PIT_DIRECTION_LIMIT；模板仍未绑定。
连续 Y 位置与两限位的预期触发窗口交叉验证；冲突、缺观测、错配全部阻断。
单侧触发禁止危险方向；逃离方向必须另行证明。STOP 只表示去励磁，不授权反向励磁。

## 安全边界

推荐 Y 坑边限位接物理/驱动方向禁止链，并以 ADAM DI 只读观察，不只依赖 DI→ROS→STOP。
安装停止线须早于碰壁位置，包含惯性停车距离与全链路延迟，由电控/厂家现场确认。
X 推荐物理行程极限→驱动内部软限位→ROS Geofence→LiDAR 动态包络。
驱动协议、能力、配置、硬接链、极性与断线诊断当前均未现场验证。

Geofence 可以拒绝不可能目标/方向、限制未来规划搜索域，不能 clamp/吸附抓斗位姿，
不能将摆动/开度/外参/协方差导致的真实越界判为安全。
本轮 direction_probe 只是配置步长内路径的必要条件诊断，不是轨迹、制动模型或持续运动许可。
真实命令仍需全链路 SafetyPermit/执行器复核；本轮不接入自动链。

## R1 感知修复

可选 TRACKING_CORE 描述开闭不变刚性机构；TRANSITION 可用其测量更新 ESKF，
安全包络仍 OPEN/CLOSED 并集，不插值斗瓣。没有 core/弱可观则预测与失锁。
TRACKED 用预测模型表面走廊、法线一致性、法向残差上限和每模型样本有界支持，
而不是整个 Grab/material 欧氏连通簇；LOST_HOLD 扩大门控并局部聚类；LOST 全局有界搜索。
OPEN/CLOSED 同时有有限另一开度诊断；主假设失败且另一高质量时记录 IO 几何冲突，不更新。

state header 是 canonical MID_SCAN，不是发布时刻；clearance 继承几何 epoch。
时延单独报告。信息门禁用模型 bbox 中心与特征长度归一化；真实协方差保留物理量纲。
跨界/外部 clearance=0 为无定义哨兵，clearance_defined=false、geometry_violation=true；
不使用巨大负包围盒直径冒充穿透深度。消息增加地图/模型/外参身份。

## 验证与影响

新消息字段和 clearance/time 语义要求全部 ROS 消费者重新构建；不能混用旧生成消息。
保留 dc42fb16 历史验收；R1 证据按精确 SHA 单独归档。
有限合成序列、连通料面与密集云测试不是 FIELD 概率保证或工业安全认证。
PTP 五文件、vendor lock/patch、main 不改；不进入 PitSurface、GraspPlanner 或物理动作。

## R1.1 合同收口（2026-10-13）

基线5a121f8b9ce5f6615b672bb896b570b7e2cd4bcc；不重做R1算法。
Geofence配置contract_version升级为2，旧配置需移除x.negative_limit_signal/positive_limit_signal，
旧版或保留无效X字段的配置明确拒绝，不静默解释。ServoState消息不改。

Y只支持canonical y_left_limit→TrolleyState.left_limit、y_right_limit→right_limit。
统一resolve_y_limit_state解析，要求known_signals及实际bool；未知/重复/未配置绑定fail-closed。
negative/positive由owner配置，不再left=negative。窗口/方向是标定后的轴位置坐标，
axis_to_map.y_scale=-1仅改变到map的几何投影，不再次交换信号绑定。
冲突、触发窗口、未触发诊断、方向和反证计数都使用同一解析结果。

X Geofence只负责位置/标定/track/pit-derived几何必要条件，不解释ServoState限位bool的knownness。
X物理限位及驱动内部soft-limit属于独立轴安全层，必须与几何结果、完整SafetyPermit相交后才能考虑动作。
默认false、甚至没有X限位字段，不代表限位已观测或轴已安全。

纯策略结果automatic_eligibility删除，替换为directional_geometry_available：
它只表示至少一个shadow方向满足几何必要条件，不等于allow_auto_task，不是SafetyPermit。
没有旧名兼容别名，消费者须显式迁移；valid_for_control、production_control_eligible永远false。

Clearance每次发布从upstream age+monotonic residence重新采样，同步外层与EvidenceMetadata年龄。
正常/异常/STALE均不刷新header、measurement_stamp或source_counter；未知年龄使用双层一致的失效哨兵。
超过既有TTL即STALE/risk=true，嵌套validity同步失效；不改变TTL/deadline或几何计算。
