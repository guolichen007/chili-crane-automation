# Phase 1A 双 ER1 数据与证据合同

## 层次与来源

官方 rslidar_sdk → vendor PointCloud2 → 本项目规范化 → consume-once 配对 →
已标定外参 → merged → 四种 ROI 点云产品。perception 不依赖厂商字节布局。
无 UDP 解码器移植，无 NDT/抓斗/料面算法混入融合器。

`EvidenceMetadata.source_type`：UNKNOWN=0、PHYSICAL=1、SIMULATED=2、REPLAY=3、SYNTHETIC=4。
来源不等于有效性；物理输入仍可能 STALE/INVALID。生产许可仅接受每项 physical
证据，development Y 标定、模拟 X、模拟 DI 不授权真实运动。

raw PointCloud2 没有自定义 evidence 字段，来源在配置、readiness、diagnostics、
RunManifest 和隔离运行入口中显式记录。bag 回放必须设置 `use_sim_time=true` 并使用
`ros2 bag play --clock`，节点将来源改为 REPLAY。禁止在 physical 会话中回放同名 topic。
操作者给 manifest 的 source 是声明，不是设备认证或现场验收证明。

## Topic / frame / units

| Topic | 类型 | frame / 说明 |
| --- | --- | --- |
| /crane_01/lidar/er1_204/vendor_points | sensor_msgs/PointCloud2 | crane_01/er1_204，官方输出 |
| /crane_01/lidar/er1_205/vendor_points | sensor_msgs/PointCloud2 | crane_01/er1_205，官方输出 |
| /crane_01/lidar/er1_204/points | sensor_msgs/PointCloud2 | 204 规范化原坐标系 |
| /crane_01/lidar/er1_205/points | sensor_msgs/PointCloud2 | 205 规范化原坐标系 |
| /crane_01/lidar/merged_points | sensor_msgs/PointCloud2 | crane_01/base，仅已标定配对 |
| /crane_01/lidar/dual_lidar/readiness | chili_crane_msgs/DualLidarReadiness | 不等于自动运动许可 |
| /crane_01/lidar/dual_lidar/diagnostics | std_msgs/String JSON | 频率、年龄、队列、丢弃、来源 |
| /crane_01/system/run_manifest | chili_crane_msgs/RunManifest | 当前采集 provenance |

点云使用 ROS sensor-data QoS（best-effort、volatile）。readiness/diagnostics 使用
本项目 state QoS；TF static 为 transient-local。x/y/z 与 translation 均为米；
rotation 为 proper 3×3 正交矩阵；timestamp 为秒，与 ROS header stamp 同时间域。

规范化 PointCloud2 little-endian、point_step=32：

| 字段 | 类型 | offset | 含义 |
| --- | --- | --- | --- |
| x / y / z | float32 | 0 / 4 / 8 | 空间坐标 m |
| intensity | float32 | 12 | 厂商强度值原样保留，不当作温度 |
| ring | uint16 | 16 | 无 ring 时明确 sentinel 65535 |
| sensor_id | uint8 | 18 | 204=0，205=1 |
| timestamp | float64 | 24 | 原每点时间，禁止伪造 |

允许厂商 timestamp 未对齐/大小端/行 padding，通过字段表读取，不假设厂商 point_step。
必须存在 XYZ、intensity、timestamp；非有限值、空点云、非法布局/时间/frame 被拒绝。
point timestamp 与 header 的允许差由现场确认的配置决定，缺配置不得 dual_lidar_ready。

## 配对与外参

`consume_once=true`、`allow_old_frame_reuse=false`、`single_lidar_fallback=false`。
双有界队列（最多 128，当前 16）；每路 stamp 严格递增，重复/乱序丢弃。
源时间和接收 monotonic 时间同时满足新鲜度；仅 abs(delta)≤maximum_pair_delta 配对。
配对后两帧立即消费；失去任一雷达只发布降级诊断，不以旧帧补齐。
ROS 源时钟倒退锁存故障，必须重新建立运行会话，不能悄悄续用旧配对。

外参配置要包含 VALID、calibration_id、source_frame、target_frame、rotation_matrix、translation_m。
缺外参时 raw/health 仍可用；merged 和 static TF 均不发布，不使用 identity 冒充标定。
只有双新鲜有效 frame、合法同步配置、配对、外参、覆盖及点时钟合同齐备才 ready。
诊断中的 1 秒默认仅用于未知配置时显示在线状态，不授予 ready。

## 产品边界

以下仅在各自 ROI `config_state=VALID` 时发布；未知 ROI 不裁剪出虚假有效产品：

- /crane_01/localization/registration_cloud
- /crane_01/perception/pit_cloud
- /crane_01/perception/grab_cloud
- /crane_01/perception/wall_geometry

wall_geometry 是选定 ROI 的几何点云，不是已经识别的墙面或安全距离。
PitSurfaceBuilder、GrabTracker、WallClearance 是下一阶段消费者边界；本轮不实现检测结论。

## 控制证据约束

physical_output_enabled=false、automatic_control_enabled=false；Z lowering false。
缺 DI 用 unknown_signals/known_signals 表达，不把 bool 默认 false 当作已知“未触发”。
auto_evidence_ready 要求所有 auto-required 输入 physical、有效且生产标定成立。
ADAM direction-only adapter 只支持 Y/Z/G 固定方向，X 或 speed request 在租约接受前拒绝。
输出有效期由本地 monotonic deadline 执行；wall/ROS timestamp 只用于准入及审计。
G 开斗区分抓料 ENSURE_GRAB_OPEN 与卸料 OPEN_GRAB；后者还要 allow_unload。
X BridgeServoStrategy 与 G GrabLimitStrategy 为显式未配置接口，不能借用 Y/Z 位置策略。
