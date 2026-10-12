# Phase1B R2 时间尺度合同与字段字典

## 分层

vendor_points = ER1 原始 PTP_TIMESCALE，不修改 SDK。
canonical /points = 显式映射后的 HOST_UTC，header 和每点必须同时变换。
UNKNOWN / BOOT_RELATIVE 不可当 HOST_UTC；HOST_DERIVED 与相机 HOST_UTC 不减 37。

TimeScale：UNKNOWN、HOST_UTC、PTP_TIMESCALE、BOOT_RELATIVE。
TimeScaleRelation：source_scale、target_scale、source_domain、target_domain、
utc_offset_sec、relation_state、evidence_id、offset_source、
current_utc_offset_valid、time_traceable。
relation_state：NOT_CONFIGURED / PROBING / LOCAL_VERIFIED / VALID / DEGRADED。

PTP→HOST_UTC：normalized=raw-utc_offset_sec，offset 必须为有证据的整数秒，
不能当成扫描延迟常量。只改时间，不改 x/y/z/intensity/ring/sensor_id 或几何。
header 使用整数纳秒减 offset，保留纳秒精度；每点保持 float64。
当前纪元的同整数 offset 平移不改变 span、双帧 mid delta 和 overlap，测试覆盖。
future/stale 与 receive latency 只在正规化之后计算。
关系未验证：vendor/raw evidence 仍可采集，canonical 不发布；
SensorTimingState normalization_valid=false，normalized_time_scale=UNKNOWN。

## 台架与生产

LOCAL_VERIFIED：要求已测 offset、来源、证据 ID 和明确两域关系，
允许 currentUtcOffsetValid=false / timeTraceable=false；算法台架可用。
VALID：除上述外要求两个 flag=true，并由 owner 明确确认，
不能只把字符串 LOCAL_VERIFIED 改成 VALID 绕过可追溯性。
ClockContract 不比较 host domain 与 PTP domain 相等；仅校验映射 source_domain
对应该传感器的 PTP domain，以及双雷达共同 target/offset。
clock_sync_state=PROBING 不进入 timing_ready；HOST_DERIVED=PROVISIONAL。
production_timing_ready 还要求时钟合同 VALID、正式关系 VALID、新鲜运行状态和有效配对。
LOCAL_VERIFIED 不可提升 DualLidarReadiness 的生产 Safety/full readiness。
本轮没有外参，timing-only 一直禁止 TF/空间 merged 输出。

## raw provenance

SensorTimingState 新增：

| 字段 | 含义 |
| --- | --- |
| raw_time_scale / normalized_time_scale | 明确两个尺度，不凭 clock 名称猜 |
| raw_header_stamp / normalized_header_stamp | 原始整数纳秒与正规化整数纳秒 |
| applied_offset_sec | 本次实际减去的秒数；HOST_UTC 为 0 |
| relation_state / relation_evidence_id | 明确关系等级及出处 |
| normalization_valid | 转换有效；false 时 normalized 时间字段不可使用 |
| clock_sync_valid / scan_phase_sync_valid | 两种独立的新鲜事实 |
| production_timing_ready | 生产时间门禁，不等同空间/控制许可 |

原有 header_stamp/start/end/mid/offset 字段描述正规化时间；
未转换时这些 ROS 时间默认值无效，必须读 normalization_valid，不当 0 秒事实。
raw vendor topic 是原始每点时间的证据源，不复制完整 payload 到状态消息。

## RSE1 只读运行解析

出处：[RoboSense 官方 E1R 协议](https://robosense-robotics.github.io/product-manual/E1R/)；
与固定 rs_driver 897b14d3bdb6186a75df27ba51b65b5bd5557723 的 packed layout 核对。
1200B MSOP 最小 ID 55 AA 5A A5：TimeMode 偏移 9，FrameSync 偏移 20。
256B DIFOP 最小 ID A5 FF 00 5A 11 11 55 55：TimesyncMode/Status 偏移 101/102。
只解析这些字段，不替换点云 decoder、不读取或泄露序列号。
Mode=0x02 是当前 E2E PTP，Status 0/1/2 分别 FAILED/SUCCESS/TIMEOUT。
FrameSync 0/1 分别 FRAME_NOT_SYNCED/FRAME_SYNCED，与 clock 独立。
unknown/missing/过期/模式矛盾不能变成有效同步。

独立 AF_PACKET observer：显式启动、需要 owner CAP_NET_RAW，不抢 vendor UDP bind，
不写设备、不保存 payload，状态变化立即发布，常态最高 10Hz 状态流。
Rse1ClockStatus 携带 source_type、boot_id、sequence、DIFOP/MSOP 原始 receive 时间。
DualLidarNode 同时检查源、机器人 frame、序号和原始 receive 新鲜度；
重复 timer 发布不能续活旧证据，重启/失败/过期清空未消费配对。
runtime_status_timeout_sec 默认 null，owner 按实际输出周期确认；未配置仍不 ready。
节点重启、消息恢复都不能用过去 S13/S14 的静态事实代替当前观测。

## 冻结与现场输入

驱动仍 disabled、端口角色仍未解锁、所有运动输出 false。
现场需确认持续状态流、新鲜度门限和正式 UTC 可追溯来源；
当前 37 秒是数据配置，不是硬编码常量，关系变更需重新采证，不自动猜测。
消息变更后必须整套七包重建；旧消息类型不可混装。
