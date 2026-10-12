# Phase1C 安装与标定工具跟踪

BASE_SHA: 72019e0d464d4de6b38df74bc8c2097ae1829846
EVIDENCE_SHA: bd4a32593edc71926a9635e8668ed44862268b2e
分支：codex/phase1c-lidar-install-calibration-v1，main 不修改。

| 项目 | 状态 | 证据/剩余动作 |
| --- | --- | --- |
| S15 脱敏事实、port roles 与 BENCH TTL | IMPLEMENTED | owner 回传事实，不冒充本轮采集 |
| sensor-only profile、rig parent、机械 seed | IMPLEMENTED | unknown fail-closed；不发布未知 TF |
| raw/canonical 配对、多帧、新鲜静止采集 | PASS_SOFTWARE | 精确 SHA synthetic ROS transport；现场 NOT_RUN |
| 原生 GICP/VGICP、可观测性与多场景 | IMPLEMENTED | Windows native 合成正负例 |
| 独立 holdout、中文指标报告、审核生命周期 | IMPLEMENTED | CANDIDATE 不产生生产许可 |
| RViz 单雷达/粗 overlay/candidate 与相机入口 | PASS_SOFTWARE | 四套 Ubuntu headless 加载通过；相机真图像 NOT_RUN |
| Windows 静态完整检查 | PASS | 266：265通过、1 ROS skip；原生 GICP/VGICP 与完整 CLI |
| Ubuntu22/Humble clean build/test | PASS | 七包构建，202项 colcon 全通过 |
| GitHub Actions | PASS | Static 38215139034 / ROS/vendor 38215139032 |

详细证据：[Phase1C 软件验收](Phase1C安装标定软件验收_bd4a325.md)。
归档后的最终 HEAD 仍须复核远端 exact-SHA CI，不挪用上述证据冒充新 HEAD。

INSTALLATION_ON_CRANE=NOT_RUN
EXTRINSIC_FIELD_STATUS=NOT_RUN
CAMERA_EXTRINSIC_STATUS=NOT_RUN
PHYSICAL_ACTUATION_STATUS=NOT_RUN

physical_output_enabled=false；automatic_control_enabled=false；camera_control_authority=false。
现场未知项见[手册](../deployment/双ER1现场安装与外参标定手册.md)第8节。
