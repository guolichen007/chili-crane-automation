# ADR 0006：Phase 1B 台架传感器与统一时间基准

- 状态：ACCEPTED，2026-10-10。
- 基线：`fbe823bb4794d7c7d9cb502312e657ae361614c6`。
- 分支：`codex/phase1b-sensor-timebase-v1`；不修改 main。

## 本轮决策

双 ER1 用 canonical XYZIRT 每点时间计算帧起止及中点；原 header 原样保留，
consume-once 配对以 MID_SCAN 为默认基准。时间配对不需要外参，但绝不证明空间融合。
Phase 1B 强制 timing-only，不发布空间 merged cloud 或 identity TF。

时钟模式为 NOT_CONFIGURED、SENSOR_PTP、HOST_DERIVED；同步状态为
NOT_CONFIGURED、PROBING、PROVISIONAL、VALID、DEGRADED。双传感器必须同模式、
同一明确 clock domain。HOST_DERIVED 只能 PROVISIONAL，PTP_VERIFIED=false。
SENSOR_PTP 的 VALID 要求独立实测证据，不能由服务运行推断。

FIRST_POINT 校验 header 与帧起点，帧跨度单独统计；不把所有点到 header 的
差当成同步误差。未来偏差、帧起点偏差、配对差及 stale 门限保持 null，需现场测量。
RSE1 理论 0.1 秒仅作参考，不作为生产门限。

PTP 工具只探测；不更改系统时间、不启停服务、不修改网卡或全局 DDS。
PTP 是 IEEE1588，E2E/P2P 是 delay mechanism，现场报文决定选择。

一台 MV-CS060-10GC 通过独立厂家适配接入，仅图像及时间证据；
camera_control_authority=false。设备 tick、host 原始时间及 ROS receive 分别记录，
未确认单位和 epoch 不提升 VALID。SlaveOnly 写入失败不等于不能成为 Slave。
rolling shutter 只记录，不补偿。SDK 不随仓库分发，不猜测 ABI 或 timestamp 单位。

## 安全与验收

physical_output_enabled=false；automatic_control_enabled=false。
不做 DO、运动、X 伺服、NDT、料面、自动抓斗、外参及视觉 AI。
既有 Phase 1A 空间融合工具仅保留历史软件测试能力，Phase 1B 启动强制禁用它。
Windows 静态、Ubuntu CI build/test、实机时间、网络、相机及真实录包分别报告。
实机未知项 NOT_RUN；PRODUCTION_TIME_SYNC_ACCEPTANCE=DEFERRED_TO_FIELD。
