# Phase1C 安装标定软件验收

日期：2026-10-12。
BASE_SHA: 72019e0d464d4de6b38df74bc8c2097ae1829846
EVIDENCE_SHA: bd4a32593edc71926a9635e8668ed44862268b2e
BRANCH: codex/phase1c-lidar-install-calibration-v1
范围：软件、合成几何、ROS2 transport、headless RViz 配置加载；不是现场安装验收。

## 实际执行结果

| 检查 | 实际结果 |
| --- | --- |
| Windows native venv | Python3.11、numpy1.26.4/scipy1.15.3/small_gicp1.0.1；266 项，265 通过、1 ROS 缺失跳过 |
| 仓库合同/格式/secret/pre-push | 全部通过；无录包/点云/编译产物入 Git |
| 既有架构回归 | 46 场景通过，纯软件，现场状态 NOT_RUN |
| Ubuntu22/Humble static | 266 项通过，无 skip |
| clean colcon build | 七包成功；Ubuntu22/Python3.10/GCC11/ROS2 Humble |
| colcon test | 202 项，0 errors、0 failures、0 skipped |
| 冻结双雷达/PTP ROS 回归 | 6 个双雷达场景 +8 个 TimeScale 场景通过 |
| Phase1C ROS transport | unknown seed 无 identity overlay；动态父无静态 TF；缺静止证据阻断；20 帧配对/四 artifact hash；失步/phase失效/rig运动阻断 |
| ROS 观测数 | rough=48、tf=0、merged=0，SYNTHETIC_ONLY |
| RViz | 204-only、205-only、MECHANICAL_SEED rough、CANDIDATE 四套实际启动加载成功 |
| mock fail-closed | 实际启动/检查通过，未产生物理运动 |
| 官方 ER1 wrapper | 固定 SHA/XYZIRT vendor 包装单独编译成功；未执行设备 |

原生合成测试包括：已知 SE3、机械初值扰动、平面退化、plane+wall 弱可观测、corner GOOD、
outlier/partial overlap、移动对象 mask、SE3 共识、五场景加独立验证集、重复 artifact 拒绝、
粗体素“收敛但错误”拒绝、完整 capture artifact→CLI→中文报告、raw artifact 篡改拒绝、
时间新鲜度、source_type、seed 不授权、支架失效、BENCH 参数不升 FIELD_VALIDATED。

## 精确 SHA GitHub 证据

- [Static contracts 38215139034](https://github.com/guolichen007/chili-crane-automation/actions/runs/38215139034)：success。
- [ROS2/vendor 38215139032](https://github.com/guolichen007/chili-crane-automation/actions/runs/38215139032)：两个 job success。
- ros2-humble job：114698020070；rslidar-vendor-wrapper job：114698019844。
- 软件 artifact ID：11731941890；ZIP SHA256：`92e5c874982d04b4500d32fde96506040a727ebe1107a540267da8e25789c832`。
- vendor artifact ID：11732166272；ZIP SHA256：`1967a9e14e994017de5819585de80d30a82dc17e74a889c2ef26a21691037abb`。

软件证据含实际测试日志、版本清单、构建/安装输出、synthetic scene manifest/hash、RViz 日志。
归档期限由 GitHub artifact retention 管理；部署 owner 可按上述 SHA256 另行保存。
ROS CI image 仍为基线继承的 `ros:humble-ros-base-jammy` tag，image digest 未捕获；
不能把 exact Git SHA 软件通过说成完整供应链锁定或生产可追溯认证。
后续仅文档归档提交仍须重新跑 exact-HEAD CI；不能把本页旧 SHA 冒充新 HEAD 全绿。

## 冻结与现场边界

Phase1B 五个时间主链文件受 SHA256 manifest 测试保护；official vendor lock/patch 相对基线零差异。
main 仍 `01ce545bd064d7f80470f76d3e2b0658530c3ae6`，本轮只推送新 Phase1C 分支。
无安装现场、原始 S15 目录或相机 SDK 访问；S15 真机结论仅作为 USER_REPORTED 保存。
没有执行 PTP 模式/服务配置、相机 PTP、DO、NDT、Pit、Grab 或自动任务。

INSTALLATION_ON_CRANE=NOT_RUN
EXTRINSIC_FIELD_STATUS=NOT_RUN
CAMERA_EXTRINSIC_STATUS=NOT_RUN
PHYSICAL_ACTUATION_STATUS=NOT_RUN

physical_output_enabled=false
automatic_control_enabled=false
camera_control_authority=false

## 下一步现场输入

仅由 owner 确认：rig 最终机械安装/parent、mount_revision、防松标记、机械测量与不确定度、
经过 S15 验证的外部运行配置、独立静止证据来源、ROI/动态 masks、现场 solver/验收门限、
camera serial/model 和参数读回；在此之前保持未知，不生成 FIELD_VALIDATED 外参。
按[安装与标定手册](../deployment/双ER1现场安装与外参标定手册.md)逐步现场执行。
