# Phase 1B 精确 SHA 软件验收

BASE_SHA: fbe823bb4794d7c7d9cb502312e657ae361614c6
EVIDENCE_SHA: cc9501df4513889c784a7ca03d2cf5238a53d6aa
日期：2026-10-10。仅软件；不得替代原生 Ubuntu 现场或真实传感器验收。

## 已执行

| 层级 | 命令/任务 | 结果 |
| --- | --- | --- |
| Windows | git diff --check；check_repo_contracts.py | 通过 |
| Windows | unittest discover tests_static | 192：191 通过，1 ROS2 不可用跳过 |
| Windows | validate_architecture_scenarios.py | 46 通过（与静态存在重叠，不相加统计） |
| Windows | check_public_secrets.py；check_delivery.py --base-sha 指定基线 | 通过 |
| Windows | Git Bash -n；hardware/software PTP dry-run | 语法通过、实际只打印 |
| GitHub Ubuntu22.04 | unittest discover tests_static | 192 全通过 |
| GitHub Ubuntu22.04 / ROS2 Humble | 七包 colcon build | 7 成功 |
| GitHub Ubuntu22.04 / ROS2 Humble | colcon test/result | 125，0 errors / 0 failures / 0 skipped |
| GitHub ROS2 transport | validate_dual_lidar_ros.py | 四场景通过 |
| GitHub ROS2 transport | validate_ros2_mock.py | fail-closed 通过 |
| GitHub official vendor | 固定 SHA SDK + rs_driver + rslidar_msg XYZIRT wrapper | 两包编译通过，不启动硬件 |

ROS2 场景保留历史 calibrated synthetic 软件回归（37 merged），不代表本轮允许空间输出；
无外参场景零 merged/TF，独立 timing-only 即使有 synthetic VALID 外参也零 merged/TF，
MID_SCAN 可以配对 header 差 >30ms 的帧，混合 domain 不配对，单流丢失不复用旧帧。
原始 header 的整数纳秒需与发送事实一致。以上 source_type=SYNTHETIC。

## 远端证据

- [Static contracts 成功](https://github.com/guolichen007/chili-crane-automation/actions/runs/38033757118)。
- [ROS2 + vendor 成功](https://github.com/guolichen007/chili-crane-automation/actions/runs/38033757109)。
- ROS2 job：114159836218；vendor job：114159836124。
- ros2-humble-software-evidence artifact 11662674148：
  sha256 `29512388fc08118e6c918a1099e4cad966df6194306b1b5ffd0e513b98f627e1`。
- rslidar-vendor-compile-evidence artifact 11663367738：
  sha256 `8f90179269400a772cb36959dc6045e8fbc97d3d644450af65d292da3e21c648`。
- CI 容器仍是 `ros:humble-ros-base-jammy` tag，未冻结 digest；日志保存依赖版本。
  不将可变镜像称为完整供应链可复现认证。

## 未执行与边界

LIVE_ER1_204_STATUS=NOT_RUN
LIVE_ER1_205_STATUS=NOT_RUN
PTP_LIVE_STATUS=NOT_RUN
CAMERA_LIVE_STATUS=NOT_RUN
REAL_SENSOR_BAG_STATUS=NOT_RUN
EXTRINSIC_CALIBRATION_STATUS=NOT_RUN
SPATIAL_MERGE_STATUS=NOT_RUN
MVS_SDK_DYNAMIC_BINDING_STATUS=NOT_RUN
PRODUCTION_TIME_SYNC_ACCEPTANCE=DEFERRED_TO_FIELD

本机未找到官方 MVS SDK；相机 adapter 已实现并通过静态/纯时间模型测试，
不把接口源码或 ROS2 安装成功宣称为厂商 SDK 动态运行成功。
现场以 [Ubuntu 交接](../deployment/PHASE1B_UBUNTU_BENCH.md)补实际证据。
没有启停 PTP/chrony/timesyncd，没有改时钟、网卡、DDS；没有移动或物理 DO。

physical_output_enabled=false
automatic_control_enabled=false
camera_control_authority=false
