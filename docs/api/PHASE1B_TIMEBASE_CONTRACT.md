# Phase 1B：时间证据接口与字段字典

## 时间与状态

所有时间为秒或 ROS `builtin_interfaces/Time`；raw 相机 tick 明确例外。
clock_mode：NOT_CONFIGURED / SENSOR_PTP / HOST_DERIVED。
clock_sync_state：NOT_CONFIGURED / PROBING / PROVISIONAL / VALID / DEGRADED。
stamp_basis：FIRST_POINT / LAST_POINT / MID_SCAN / HOST_RECEIVE / CAMERA_DEVICE / CAMERA_HOST。
当前 ER1 adapter 使用 FIRST_POINT header，MID_SCAN 配对。
VALID 不能由系统服务、收到包或设备在线推断；SENSOR_PTP 需明确同一 domain、
已验证时间证据 ID、host_clock_relation=VALID、host_clock_relation_evidence_id，
且 host_clock_domain 与两雷达 clock_domain 一致。
HOST_DERIVED 只允许 PROVISIONAL，不标 PTP_VERIFIED。
SENSOR_PTP/PROBING 可 render use_lidar_clock=true 采集诊断；domain 尚未确认可保留
NOT_CONFIGURED，但永不 timing_ready/ptp_verified/full_ready。
PROBING 不拿未证实同域的 Ubuntu now 拒绝全部源时间；原始时间及 receive 对照保留，
帧结构/有限正时间/单调性和配置后的 span 上限仍校验。VALID 才使用已确认 host relation
执行 source future/stale 检查。两雷达模式不得混用。
参数 null 保持未配置；实验 CLI 参数不自动成为生产配置。

## 每点时间

Canonical XYZIRT：x/y/z/intensity float32、ring uint16、sensor_id uint8、timestamp float64；
32 字节 point_step，timestamp offset=24。保留原始 header 和每点时间。
帧 start=min(timestamp)，end=max(timestamp)，mid=start+(end-start)/2，span=end-start。
FIRST_POINT 校验 abs(header-start)，不校验整帧所有点距 header。
maximum_future_skew_sec 同时约束 header、end；正偏差可接受但上限有限，默认 null。
stale 同时检查最早帧时间及服务器 monotonic receive。理论 RSE1 0.1s 仅参考。
maximum_frame_span_sec 默认 null，先统计；设置后超过上限报 FRAME_SPAN_EXCEEDED。
废弃 point_timestamp_header_tolerance_sec 和 clocks_synchronized；
完整 readiness 只依赖实际 FIRST_POINT/future/stale 配对验证、ClockContract 及空间证据，
不再受废弃字段影响。null 门限不自动转成生产常量。
默认 MID_SCAN 差小于明确 maximum_pair_delta_sec 才配对；每帧只消费一次，队列有界，
不用旧帧补单雷达。区间 overlap 为交集时长，ratio=交集/并集，零跨度同点 ratio=1。
零 overlap 仍作为诊断事实记录；中点门限不等价于空间/运动补偿。

Pinned rs_driver `897b14d3bdb6186a75df27ba51b65b5bd5557723` 的
`decoder_RSE1.hpp` 使用 host getTimeHost()-packet duration 建立 packet 时间，
再加 block time offset 生成 point 时间。use_lidar_clock=false 属 HOST_DERIVED，
不应称为 HOST_RECEIVE header，更不等于硬件 PTP。

## typed topics

| Topic（/crane_01 前缀） | 类型 | 主要事实 |
| --- | --- | --- |
| lidar/er1_204/points、er1_205/points | sensor_msgs/PointCloud2 | 传感器自身 frame，原 header |
| lidar/er1_204/timing、er1_205/timing | SensorTimingState | header/receive/start/end/mid、span、basis、时钟合同、period/jitter、计数 |
| lidar/dual_lidar/timing | DualLidarTimingState | header/mid delta、overlap、pair/drop、timing_ready |
| lidar/dual_lidar/readiness | DualLidarReadiness | 兼容既有完整 readiness，Phase1B 恒不 full ready |
| lidar/dual_lidar/diagnostics | std_msgs/String JSON | offline/fresh、丢帧、invalid、时钟和空间原因 |
| camera/hik_01/image_raw | sensor_msgs/Image | 只发布支持的 Mono8/Bayer8 原始图像 |
| camera/hik_01/camera_info | sensor_msgs/CameraInfo | K[0]=0：未标定，不用 identity 冒充 |
| camera/hik_01/timing | CameraTimingState | 三份原始/换算时间、有效标志、frame gap、lost packet、exposure |
| camera/hik_01/diagnostics | std_msgs/String JSON | 缺失数值为 null，真实参数与状态 |
| system/run_manifest | RunManifest | Git/config SHA、timebase_json、source_type |

新消息均有 header/validity/reason/EvidenceMetadata；接口增加后七包及消费方需一起重建。
时间诊断不授予 SafetyPermit。timing_ready 可以是台架 HOST_DERIVED 配对，
extrinsic_valid/spatial_merge_ready/dual_lidar_full_ready 在 Phase1B 恒 false。
TEMPORAL_PAIR_VALID 和 SPATIAL_FUSION_NOT_CONFIGURED 是两个独立事实。

## 相机时间证据

设备 raw=(nDevTimeStampHigh<<32)|nDevTimeStampLow，host raw=nHostTimeStamp，
receive 为真实 ROS 接收时间，不用 publish 前 now 替代有效采样时间。
raw 字段单位不得猜。需官方 SDK 文档出处、GevTimestampTickFrequency 实测、
明确单位/epoch 的证据 ID 与 CLOCK_REALTIME 比较；门限未配置不提升有效状态。
CAMERA_PTP_DEVICE 要求实际符号 Slave；数字枚举不猜意义。无法解析则回退。
CAMERA_HOST_TIMESTAMP 也要求单位/epoch 证据；否则 CAMERA_RECEIVE_ESTIMATE，
只能基础辅助图像，不作高精度融合。device_time_valid/host_time_valid 描述转换和新鲜度，
high_precision_time_valid 另描述 Slave 时间证据；三者都不能授权动作。
未转换 typed float 字段使用 ROS 默认值，但有效 flag=false，JSON 为 null，消费方不得当真值。
rolling shutter 未补偿。实际曝光单位默认 NOT_CONFIGURED，raw 保留不擅自换算。
SlaveOnly 强制失败仅记 SLAVE_ONLY_FORCE_UNSUPPORTED；不推断 CAMERA_CANNOT_BE_PTP_SLAVE。

官方 MVS 分发入口：[海康机器人下载中心](https://www.hikrobotics.com/cn/machinevision/service/download/?module=0)。
本仓库不分发厂商 SDK，不保证尚未安装版本的 API 兼容；现场探测失败保持显式失败。

## 证据边界

PHYSICAL/SYNTHETIC/REPLAY 由 EvidenceMetadata 明确区分。CI synthetic 时间测试
不是 ER1 或相机实测。PTP Capabilities 只证明网卡能力；packet probe 只证明报文存在；
两者都不能单独证明 sensor 与系统时钟同源。生产验收 DEFERRED_TO_FIELD。

## ER1 网络合同（R1）

transport_mode：NOT_CONFIGURED / UNICAST / MULTICAST / BROADCAST。
host_address 为本机接口 IPv4；destination_address 为设备实际发送目的地。
UNICAST 要求两者一致；MULTICAST 必须指定 group_address，且等于组播目的地；
renderer 和 launch 均验证，禁止手写 vendor config 绕过。
按 pinned SDK 输出 host_address、group_address；非组播 group 为 0.0.0.0。
当前 S3-FINAL 为双单播 .102，不再采用 S1 .10/组播建议。
网络 observation 不提升 port_roles；只读 probe 必须以 length+最小 ID 判定角色：
1200B + 55 AA 5A A5 为 CONFIRMED_MSOP；
256B + A5 FF 00 5A 11 11 55 55 为 CONFIRMED_DIFOP，其他 UNKNOWN。
两设备两角色的目的地址/端口需人工复核，才可在现场副本设置 port_roles=VALID。
不保存 payload、不改设备或 site config。
