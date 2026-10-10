# Phase1B Bench R1：台架接入前收口跟踪

BASE_SHA: 8118f993b3b4a69e2933ded5b5730d4b48f5a68f
EVIDENCE_SHA: 857d6bbc13f6680df7111b9c725310675222d470

分支 codex/phase1b-bench-r1；main 不修改。本轮代码软件验证已经通过，
详见[精确 SHA 验收](Phase1B台架R1软件验收_857d6bb.md)。
当前文档归档后的最终 HEAD 远端检查独立复核，不把旧 Phase1B CI 复用为本轮证据。

| 项目 | 状态 | 落点 |
| --- | --- | --- |
| 最新 S3-FINAL 与 S1 取代关系 | IMPLEMENTED | network/interfaces、两份网络中文文档 |
| site 参数网络检查、ADAM 独立、已有 profile 不重复创建 | IMPLEMENTED | sensor_network_contract、network shell |
| UNICAST/MULTICAST/BROADCAST 合同及 launch 防绕过 | IMPLEMENTED | hardware/er1_transport、renderer、launch |
| RSE1 最小签名角色只读 probe | IMPLEMENTED | hardware/er1_transport、er1_udp_probe |
| PTP PROBING 请求与已验证证据分开 | IMPLEMENTED | ClockContract、DualLidarNode |
| host relation/domain/evidence 必须满足 | IMPLEMENTED | ClockContract、sensor yaml、manifest |
| 废弃 readiness 字段移除 | IMPLEMENTED | DualLidarNode、真实 ROS synthetic 回归 |
| 帧 span 可选门限及超限失败 | IMPLEMENTED | FrameTime、node、timing probe |
| 相机最新地址、网络状态、SDK 身份未确认 | IMPLEMENTED | hik_01、sensor_inventory |
| Windows 静态及负例 | PASS | 215 项：214 通过、1 ROS 不可用跳过 |
| Ubuntu22/Humble 构建、colcon、ROS transport | PASS | 7 包、149 项 colcon、六场景 |
| 远端代码精确 SHA 全绿 | PASS | Static 38039834244、ROS/vendor 38039834249 |

## 冻结边界

physical_output_enabled=false；automatic_control_enabled=false；camera_control_authority=false。
生产时钟验收仍 DEFERRED_TO_FIELD；role/time/外参未知不自动升级有效状态。
LIVE_ER1_204_STATUS=NOT_RUN，LIVE_ER1_205_STATUS=NOT_RUN，
CAMERA_LIVE_STATUS=NOT_RUN，PTP_LIVE_STATUS=NOT_RUN，
REAL_SENSOR_BAG_STATUS=NOT_RUN，EXTRINSIC_CALIBRATION_STATUS=NOT_RUN，
SPATIAL_MERGE_STATUS=NOT_RUN。

R1 只修改代码、配置、文档与测试；未安装现场系统、修改网络/设备、
启停服务、移动车辆或触碰稳定服务器。
