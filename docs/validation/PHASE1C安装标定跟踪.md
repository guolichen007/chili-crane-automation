# Phase1C 安装与标定工具跟踪

BASE_SHA: 72019e0d464d4de6b38df74bc8c2097ae1829846
分支：codex/phase1c-lidar-install-calibration-v1，main 不修改。

| 项目 | 状态 | 证据/剩余动作 |
| --- | --- | --- |
| S15 脱敏事实、port roles 与 BENCH TTL | IMPLEMENTED | owner 回传事实，不冒充本轮采集 |
| sensor-only profile、rig parent、机械 seed | IMPLEMENTED | unknown fail-closed；不发布未知 TF |
| raw/canonical 配对、多帧、新鲜静止采集 | IMPLEMENTED | ROS transport 待精确 SHA 远端验收 |
| 原生 GICP/VGICP、可观测性与多场景 | IMPLEMENTED | Windows native 合成正负例 |
| 独立 holdout、中文指标报告、审核生命周期 | IMPLEMENTED | CANDIDATE 不产生生产许可 |
| RViz 单雷达/粗 overlay/candidate 与相机入口 | IMPLEMENTED | Ubuntu headless 加载待验证；真图像 NOT_RUN |
| Windows 静态完整检查 | IN_PROGRESS | 提交前检查 |
| Ubuntu22/Humble clean build/test | IN_PROGRESS | 精确 SHA CI，不复用 R2 结果 |
| GitHub Actions | IN_PROGRESS | 本分支全部 job 通过才归档 |

INSTALLATION_ON_CRANE=NOT_RUN
EXTRINSIC_FIELD_STATUS=NOT_RUN
CAMERA_EXTRINSIC_STATUS=NOT_RUN
PHYSICAL_ACTUATION_STATUS=NOT_RUN

physical_output_enabled=false；automatic_control_enabled=false；camera_control_authority=false。
现场未知项见[手册](../deployment/双ER1现场安装与外参标定手册.md)第8节。
