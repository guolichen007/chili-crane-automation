# Phase 1B 实施与验收跟踪

BASE_SHA: fbe823bb4794d7c7d9cb502312e657ae361614c6

| 工作项 | 状态 | 验证层级 |
| --- | --- | --- |
| 时间合同及安全边界 | 完成 | ADR 0006 |
| typed 时间证据接口 | 完成 | 静态及 ROS2 七包 build 通过 |
| 每点帧时间与中点配对 | 完成 | synthetic 静态及 ROS2 transport 通过 |
| PTP、网络及时间统计工具 | 已实现 | 单元与 shell syntax；现场 NOT_RUN |
| MVS 相机适配与时间证据 | 已实现 | synthetic 通过；官方 SDK 动态绑定及现场 NOT_RUN |
| Phase 1B 录包与 manifest | 已实现 | 静态；真实录包 NOT_RUN |
| Ubuntu 交接与 GitHub CI | 完成 | [cc9501d 精确 SHA 软件证据](PHASE1B_SOFTWARE_cc9501d.md) |

ER1_204_LIVE_STATUS=NOT_RUN
ER1_205_LIVE_STATUS=NOT_RUN
ER1_204_TIME_STATUS=NOT_RUN
ER1_205_TIME_STATUS=NOT_RUN
DUAL_LIDAR_COMMON_CLOCK_STATUS=NOT_RUN
DUAL_LIDAR_TEMPORAL_PAIRING_STATUS=NOT_RUN
PTP_STATUS=NOT_RUN
CAMERA_LIVE_STATUS=NOT_RUN
CAMERA_TIMESTAMP_STATUS=NOT_RUN
SENSOR_NETWORK_STATUS=NOT_RUN
REAL_SENSOR_BAG_STATUS=NOT_RUN
EXTRINSIC_CALIBRATION_STATUS=NOT_RUN
SPATIAL_MERGE_STATUS=NOT_RUN
PRODUCTION_TIME_SYNC_ACCEPTANCE=DEFERRED_TO_FIELD

physical_output_enabled=false
automatic_control_enabled=false
camera_control_authority=false
