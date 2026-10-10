# Phase1B Bench R1 精确 SHA 软件验收

BASE_SHA: 8118f993b3b4a69e2933ded5b5730d4b48f5a68f
EVIDENCE_SHA: 857d6bbc13f6680df7111b9c725310675222d470

日期：2026-10-10；分支 codex/phase1b-bench-r1。
本文归档上述代码提交，归档文档产生的新 HEAD 另在 GitHub 复核，最终输出其 SHA。
main 保持 01ce545bd064d7f80470f76d3e2b0658530c3ae6。
只报告软件结果，不代表原生现场主机、真实设备或生产验收。

## 修改收口

- S3-FINAL .102 / SENSOR-NET / 双 UNICAST 为当前配置；S1 .10/组播/ADAM
  NO-CARRIER 单独保存并标 SUPERSEDED。
- 相机 .180 网络 BENCH_REACHABLE，requested 型号与 SDK 实测身份分开；
  serial 未配、驱动和控制权限仍 false。
- sensor-only site/显式参数检查；ADAM link 独立，断线不污染传感器检查；
  已有 profile 不重复创建，所有 nmcli 执行只读，创建命令仅打印。
- transport renderer 与 launch 双重校验 host/group/destination；
  UDP 角色以 pinned RSE1 长度和最小 signature 判定，不保存 payload。
- SENSOR_PTP PROBING 可 render/采集不同 epoch 诊断，不能提升任何 timing/full readiness；
  VALID 必须时间证据、双雷达同域、host relation/domain/evidence。
- 删除旧 readiness 字段，实际门限保留 null；新增 maximum_frame_span_sec
  可选界限，超限明确 FRAME_SPAN_EXCEEDED。
- requested timebase 与 measured snapshot 分开，manifest 补齐网络和 host relation 字段。

## 实际命令与数量

| 环境 | 命令/检查 | 结果 |
| --- | --- | --- |
| Windows | git diff --check；python tools/check_repo_contracts.py | 通过 |
| Windows | python -m unittest discover -s tests_static -p "test_*.py" | 215：214 通过，1 ROS 不可用跳过 |
| Windows | python tools/validate_architecture_scenarios.py | 46 通过，与静态有重叠，不相加统计 |
| Windows | python tools/check_public_secrets.py | 通过 |
| Windows | python tools/check_delivery.py --base-sha 8118f993b3b4a69e2933ded5b5730d4b48f5a68f | 干净工作区通过 |
| Windows | Git Bash -n 两个 network shell | 通过 |
| GitHub Ubuntu22 / Python3.10 | 同一 tests_static 全套 | 215 全通过 |
| GitHub Ubuntu22 / Humble / GCC11 | colcon build 七包 | 7 成功 |
| GitHub Ubuntu22 / Humble | colcon test/result | 149，0 errors / 0 failures / 0 skipped |
| GitHub ROS2 transport | python3 tools/validate_dual_lidar_ros.py | 六场景通过 |
| GitHub ROS2 mock | validate_ros2_mock.py | fail-closed 通过 |
| GitHub vendor | pinned rslidar_msg + rslidar_sdk XYZIRT wrapper | 两包编译通过，没有设备运行 |

R1 增加 23 个静态测试。六个真实 ROS2 transport synthetic 场景包含：
历史 calibrated 软件回归、未标定、强制 timing-only、混合域拒绝、
PTP PROBING（人为 +3600s epoch）可采诊断但零配对、
配置后 span 超限零 raw/merged 有效消息。
历史 calibrated synthetic 只在临时夹具中测试既有代码；未修改实机外参、
未赋 identity、未向现场发布空间点云。本轮入口继续强制 timing-only。

## 精确 SHA 远端证据

- [Static contracts](https://github.com/guolichen007/chili-crane-automation/actions/runs/38039834244)：success。
- [ROS2 与 vendor](https://github.com/guolichen007/chili-crane-automation/actions/runs/38039834249)：success。
- ROS job：114177689541；vendor job：114177689312。
- ros2-humble-software-evidence artifact：11665770931，
  SHA256：c3bbc47093d3e525d3ea085b03dd63658e6c941a26ecade217ec5c0b65d3b304。
- rslidar-vendor-compile-evidence artifact：11664834013，
  SHA256：2f2edb596b7a02b67ab719acc0cad3637e2f5f90a3cfb55184fd58d4ae79e4dc。

容器 ros:humble-ros-base-jammy 仍是 tag；构建日志保存依赖版本，
不宣称完整供应链冻结认证。网络配置证据来自用户报告，代码分支的 CI
不重新证明 SENSOR_NETWORK_FINAL_ACCEPTANCE。

## 尚需现场采证

LIVE_ER1_204_STATUS=NOT_RUN
LIVE_ER1_205_STATUS=NOT_RUN
CAMERA_LIVE_STATUS=NOT_RUN
PTP_LIVE_STATUS=NOT_RUN
REAL_SENSOR_BAG_STATUS=NOT_RUN
EXTRINSIC_CALIBRATION_STATUS=NOT_RUN
SPATIAL_MERGE_STATUS=NOT_RUN
PRODUCTION_TIME_SYNC_ACCEPTANCE=DEFERRED_TO_FIELD

现场需 role signature/正式端口绑定、真实双雷达 clock domain 与 host relation、
采样统计和门限、MVS SDK 型号/serial/图像、PHYSICAL bag；
不因网络可达填充这些事实。driver_enabled=false；
physical_output_enabled=false；automatic_control_enabled=false；
camera_control_authority=false。
本轮未安装现场系统、未改网卡/设备、未启动 PTP 或操作物理输出。
