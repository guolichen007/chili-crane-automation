# Phase1B PTP 时间尺度 R2 精确 SHA 软件验收

BASE_SHA: 48674999d07c5df24e8b3e36ded55b8b779794a2
EVIDENCE_SHA: 64d00a3a3dc4e434fe449d214bbe67c5ef335e92

日期：2026-10-12。分支 codex/phase1b-ptp-timescale-r2。
归档对象是上述最终代码 SHA；归档文档产生的 HEAD 另执行 exact-SHA CI，
最终报告输出该 HEAD。不修改 main、不合并、不打 tag、不提交 bag。
main 保持 01ce545bd064d7f80470f76d3e2b0658530c3ae6。

## 收口内容

- S13/S14 仅用户回传，正式保存相对同步 PASS 和非生产可追溯 UTC 关系；
  currentUtcOffsetValid=0 不视为 PTP 失败。
- TimeScale / TimeScaleRelation 替代 host_domain=ptp_domain 错误条件。
- vendor_points 保持 raw PTP；canonical header/每点一起减明确整数 offset，
  无关系、未知尺度、矛盾尺度或小数补偿均 fail closed。
- raw/normalized provenance、LOCAL 与 production VALID、clock 与 scan phase 分开。
- 只读 RSE1 DIFOP/MSOP observer、状态文字、TTL、sequence/boot、降级和配对清空。
- future/stale 在正规化之后；不修改 vendor，不减 raw 37、不补 0.12 秒。
- 只扩展时间链路，没有新增空间、NDT、Grab/Pit、相机 PTP 或控制功能。

## 实际测试

| 环境 | 命令/任务 | 结果 |
| --- | --- | --- |
| Windows | git diff --check；python tools/check_repo_contracts.py | 通过 |
| Windows | python -m unittest discover -s tests_static -p "test_*.py" | 240：239 通过，1 ROS 不可用跳过 |
| Windows | python tools/validate_architecture_scenarios.py | 46 通过，和静态重叠，不累加 |
| Windows | python tools/check_public_secrets.py | 通过 |
| Windows | python tools/check_delivery.py --base-sha 48674999d07c5df24e8b3e36ded55b8b779794a2 | 干净工作区通过 |
| GitHub Ubuntu22 / Python3.10 | 全套 tests_static | 240 全通过 |
| GitHub Ubuntu22 / ROS2 Humble / GCC11 | 七包 clean colcon build | 7 成功 |
| GitHub Ubuntu22 / ROS2 Humble | colcon test/result | 175，0 errors / 0 failures / 0 skipped |
| GitHub ROS2 | validate_dual_lidar_ros.py | 6 个既有/调整后的 synthetic 场景通过 |
| GitHub ROS2 | validate_ptp_timescale_ros.py | 8 个新增场景通过，全部零 TF/merged |
| GitHub ROS2 | validate_ros2_mock.py | fail-closed mock 通过 |
| GitHub vendor | pinned rslidar_sdk / rs_driver / rslidar_msg wrapper | 2 包编译通过，没有设备执行 |

新增 25 项纯软件回归；新增 ROS2 场景：
LOCAL 可配对但不 production、正式 VALID 时间可用、missing relation raw-only、
unknown scale raw-only、运行失败、运行 timeout、FrameSync0 不否定 clock 成功、
重复序号不能续活旧证据。header 整数纳秒与每点数组实际校验，future 错误数为 0。
frame span / mid delta / overlap 同整数 offset 转换不变由纯模型严格测试。
HOST_DERIVED 和 camera HOST_UTC 不转换由独立测试覆盖，相机实机未启动。

旧 calibrated synthetic 空间回归只是已有隔离临时夹具，未增加或解锁实机空间功能。
新增 ROS2 的 120ms 仅模拟接收晚于 frame start，不是 runtime 或 SDK 补偿。
代码中没有硬编码 37 秒修 driver；37 是 site relation 配置。

## 远端可查证据

- [Static](https://github.com/guolichen007/chili-crane-automation/actions/runs/38206321552)：success。
- [ROS2/vendor](https://github.com/guolichen007/chili-crane-automation/actions/runs/38206321572)：success。
- ROS job：114670490813；vendor job：114670490968。
- ros2-humble-software-evidence：artifact 11727093191，
  SHA256 fb52124b6906bb08b826d8509b90338fbc778402829b383831f9e1756ada7c74。
- rslidar-vendor-compile-evidence：artifact 11727766196，
  SHA256 646f2764b78bb7cb3306fe329c33df18867f5ae1418cfe80847956b1717a0b78。

CI 镜像仍为 ros:humble-ros-base-jammy tag，依赖版本保存于 artifact，
不宣称完整供应链 digest 冻结认证。软件证据不能升级任何现场结论。
vendor lock/setup、实机 calibration、hik_01 和 runtime 输出配置相对 BASE_SHA 未变。

## 真实状态与边界

DUAL_ER1_RELATIVE_TIME_SYNC=PASS（用户 S13/S14 回传）
HOST_UTC_RELATION=LOCAL_VERIFIED_NOT_PRODUCTION_TRACEABLE
CURRENT_UTC_OFFSET_VALID=false
TIME_TRACEABLE=false
RSE1_CURRENT_LIVE_RUNTIME_OBSERVATION=NOT_RUN_BY_CODEX
PRODUCTION_TIME_RELATION=NOT_VERIFIED
REAL_SENSOR_BAG_STATUS=NOT_RUN
CAMERA_PTP_STATUS=NOT_RUN
EXTRINSIC_CALIBRATION_STATUS=NOT_RUN
SPATIAL_MERGE_STATUS=NOT_RUN
REAL_CONTROL_STATUS=NOT_RUN

现场 driver_enabled=false；port_roles 尚未解锁；runtime_status_timeout_sec=null，
没有持续状态证据时，运行 timing readiness 仍不通过。
physical_output_enabled=false
automatic_control_enabled=false
camera_control_authority=false

本轮未连接工控机/设备、启停 PTP、改系统时间/网络、安装现场驱动或产生物理输出。
下一步由 owner 配置已验证端口及新鲜度门限，显式启动只读 observer 并采集持续状态；
生产 VALID 还需正式 UTC 可追溯证据，不能仅修改枚举字符串。
