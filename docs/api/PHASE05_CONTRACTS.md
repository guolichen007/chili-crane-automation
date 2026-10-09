# Phase 0.5 软件架构契约

Status: 接口及纯策略/mock基线；实际算法、执行闭环和现场参数未实现/未验收。
Decision: ADR 0004。运行基线仍 Ubuntu22.04/Humble，七包边界保持。

## 1. 冻结的职责链

Task/Cycle -> ControlIntent -> SafetyPermit -> Command Authorizer ->
AuthorizedCommand -> ActionExecutor -> ActuationRequest -> HardwareAdapter -> Physical I/O。

本轮安全节点只发无效授权；Executor只发de-energize，mock硬件拒绝所有energize。
STOP可以逻辑接收，但physical_off_not_verified；没有实物反馈不能宣称设备已停。
感知只能发布观察，绝不直接调用设备输出。ADAM是远程I/O，不承担原厂硬安全。

## 2. 资产与四类雷达产品

sensor_inventory.yaml记录ER1资产A/B、报告的IP/端口；左右安装、端口角色、
驱动/外参/时钟/覆盖仍未知，driver_enabled=false。
Camera A不完整reported_ip=192.168.180保持NEEDS_CONFIRMATION，B暂不接；
两相机enabled_for_control=false，不作为SafetyPermit主证据。

双雷达输出四种逻辑产品：静态定位registration、料面pit、grab ROI、wall geometry。
每雷达online/fresh/frame/calibration有效且pairing/extrinsic/coverage有效才dual-ready。
single_lidar_fallback默认false；单雷达不允许危险自动下降。
Y主反馈TrolleyState.position_m；Z主反馈GrabState.bottom_z_m及置信/跟踪/时间证据。
MEASURED/FILTERED也需验证质量；PREDICTED/LOST只能导致保持/停止，不继续下降。
模型估计可见抓斗结构与已知几何，不要求必须看见最低斗刃；算法后续实现。

## 3. 每轴能力与停止模型

Y/Z各自选择VARIABLE_SPEED或FIXED_SLOW，不建两个工程。
VARIABLE_SPEED使用显式阈值/速度band（Y FAST/SLOW/JOG、Z NORMAL_DOWN/SLOW_DOWN/CREEP），
未知band不产生运动。当前没有真实调速接口。
FIXED_SLOW只有方向/enable，无速度给定；以反馈、停车提前量、settling和容差停止。
停稳后误差超限报告REPLAN_REQUIRED，不自动反复点动。
pulse_jog_allowed以及最小通/断时间都需厂家确认；当前策略未启用pulse jog。

d_stop = abs(v) * (sensor_latency + processing_latency + communication_latency +
output_release_latency) + mechanical_coast_distance + safety_margin。
每项须实测/标定；未知模型返回不可用，不填示例作为现场值。
FIXED_SLOW必须verified/stop_distance_verified/reaction_time_verified/fixed_speed_verified_safe。
Z下降还需双雷达、GrabTracker、安全、定位/场景等对应有效证据；固定速度过快必须禁用。
VARIABLE_SPEED Z也需要停车距离验收。Readiness只陈述能力，不能替代动作特定安全许可。

## 4. 执行与命令有效性

BridgeServoExecutor、TrolleyExecutor、HoistExecutor、GrabExecutor是纯策略边界。
X真实厂家move_to/stop/ready/at_target/fault/state协议仍NOT_CONFIGURED。
ActuationRequest含axis/direction/enable、可选speed、source_command_id、会话、
epoch、sequence、issue/expiry和run/task/cycle；不是6个裸DO bool。
实际方向必须与授权方向一致，有效期不超过授权或permit。
direction+enable在hardware转换DO0..5；VARIABLE_SPEED不能被direction-only ADAM静默忽略。
保留DO6/7。此转换函数不是物理写入授权。

STOP dominance：missing/stale permit、mode切换、安全丢失、过期、fault、掉线均允许向OFF退化，
不能通过STOP旁路energize。实际掉线OFF仍依赖已验证原厂回路/WDT/FSV。

## 5. SystemMode、epoch与Task/Cycle

SystemMode单独表达BOOT/SELF_CHECK/SAFE_IDLE/AUTO_PENDING/AUTO_READY/AUTO_ACTIVE/
REMOTE/PROTECTIVE_STOP/FAULT_LATCHED/EMERGENCY_STOP/MAINTENANCE。
重启生成新session_id；控制权切换及新任务递增epoch；序号单调，旧/重复包拒绝。
AUTO->REMOTE作废旧授权、停止自动输出、abort cycle。
REMOTE->AUTO要求静止/OFF/新鲜安全/感知定位重新确认；必须接受新任务/明确恢复决策，
不能续跑旧LOWER。故障锁存/急停恢复需显式reset及停止/OFF/安全证据。

Run包含Task，Task包含多个Cycle。每Cycle：
SCAN -> PLAN_PICK -> POSITION_Y -> WAIT_STABLE -> ENSURE_GRAB_OPEN -> LOWER ->
CLOSE -> VERIFY_LOAD -> RAISE -> MOVE_TO_UNLOAD -> VERIFY_CART -> POSITION_UNLOAD ->
OPEN_GRAB -> VERIFY_EMPTY -> COMPLETE。
CONTINUE重新SCAN；FINISH先RETURN_SAFE后DONE。重试须新扫描计数，不复用旧料面。
空抓/无抓点为cycle failure；授权重试不自动升级system FAULT，不默认无限重试。
任务接口移除MANUAL_OVERRIDE（改ABORTED）；遥控归SystemMode，消费方必须重新构建。

## 6. DI、readiness、卸料与事件

物理DI固定6052 0..7、6251 0..15共24；每点ASSIGNED/RESERVED/NOT_CONFIGURED。
逻辑能力允许更多名称，未知fault/brake/overload/remote_x/remote_stop不分配实物通道。
已要求遥控Y/Z/G六方向及模式观察；不经软件转发遥控DI到DO。
SystemReadiness发布配置/硬件/双雷达/定位/抓斗/料面/控制/安全和每轴能力；
任一必需项未知，整机system_ready=false。

UnloadSafeZone保留XYZ区间、frame及几何版本；需抓斗包络/料流投影包含于接料区，
不只追单个坐标。本轮仅几何包含判断，不代替真实接料车检测或开斗许可。

SafetyEvent/FaultEvent带severity/domain/code/source、active/latched/recoverable、
run/task/cycle/command、first_seen/last_seen/reason；区分cycle failure、protective stop、
recoverable fault、latched fault、emergency stop。故障来源不以字符串日志替代类型化接口。

## 7. 证据与运行出处

EvidenceMetadata统一measurement_stamp/receive_stamp/source_counter/evidence_age_sec/
calibration_id/config_version。未知元数据不能产生READY；receive time和timer不能伪造measurement。
旧header/evidence_age字段暂保留以迁移消费者；消费方必须验证新metadata和消息validity，
不能只因为header更新就继续动作。
RunManifest记录run/task/cycle、git SHA、config hash、map、标定IDs及抓斗版本；
配置hash为确定性排序JSON的SHA256。录包profile是默认禁用骨架，未开始真实记录。

## 8. 不兼容变更与验收

消息增加会话/epoch/证据字段，硬件输入由AuthorizedCommand变为ActuationRequest，
TaskState值21变ABORTED。全工作区/订阅者需要clean rebuild；不可混用旧生成消息。
旧ROS1 bag不能直接当rosbag2；转换和对照证据需单独审查。
见phase05_acceptance.yaml的实现文件与验证入口。
纯策略27项情景验证与ROS2运行mock是不同证据；实际硬件/BAG/FIELD仍NOT_RUN。
