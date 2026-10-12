# Phase2A 抓斗shadow精确SHA软件验收

日期：2026-10-12。
BASE_SHA: 72234843d5850947c5a2fd5e230516bc8fa64839
EVIDENCE_SHA: bfbb2ed89540793d42e6e48a36091f23808d3efd
BRANCH: codex/phase2a-grab-state-estimation-v1
SCOPE: SYNTHETIC_NATIVE_AND_UBUNTU_ROS2_SHADOW_ONLY

## 实际结果

WINDOWS_STATIC_STATUS: PASS (293项：292通过，1项无ROS环境跳过)
ROS2_BUILD_STATUS: PASS (七包normal clean build)
ROS2_TEST_STATUS: PASS (227 tests，0 errors/failures/skips)
SYMLINK_INSTALL_STATUS: PASS (独立目录七包build --symlink-install)
SHADOW_RUNTIME_STATUS: PASS_SYNTHETIC_ROS (normal和symlink安装的实际节点运行)
GITHUB_ACTIONS_STATUS: PASS

Ubuntu静态293项全通过；既有46个架构场景仍通过。
installed preview启动且观察diagnostics；capture入口--help实际执行；
pkg executables列出新旧节点；shadow launch实际启动，合成流完成TRACKED、
bottom/clearance检查、input expiry，控制topic消息0。
symlink目录再次执行20帧Capture及installed shadow，结果存symlink-executables-and-transport.log。
原双雷达6场景、TimeScale8场景、Capture/RViz/mock及vendor包装继续通过。

## 模块与反证状态

PROJECT_PROGRESS_DOC_STATUS: PASS_VERSIONED
PHASE1C_P1_STATUS: PASS_SOFTWARE_ROS
GRAB_GEOMETRY_MODEL_STATUS: PASS_SYNTHETIC
GRAB_CANDIDATE_STATUS: PASS_SYNTHETIC
GRAB_REGISTRATION_STATUS: PASS_NATIVE_SYNTHETIC_GICP_AND_HUBER_REFINEMENT
GRAB_VALIDATOR_STATUS: PASS_SYNTHETIC
GRAB_TRACKER_STATUS: PASS_SYNTHETIC_ESKF
GRAB_BOTTOM_STATUS: PASS_SYNTHETIC
GRAB_SWING_STATUS: PASS_SYNTHETIC
GRAB_ENVELOPE_STATUS: PASS_SYNTHETIC
GRAB_CLEARANCE_STATUS: PASS_SYNTHETIC_GEOMETRY
SYMMETRY_TEST_STATUS: PASS_SYNTHETIC
OCCLUSION_TEST_STATUS: PASS_SYNTHETIC_WITH_REJECTIONS
REACQUISITION_TEST_STATUS: PASS_SYNTHETIC_FULL_PIPELINE
MOTION_SMEAR_TEST_STATUS: PASS_SYNTHETIC_BOUND_INFLATION_NOT_DESKEW
CONSERVATIVE_CLEARANCE_TEST_STATUS: PASS_FINITE_SYNTHETIC_COUNTEREXAMPLES
BOTTOM_BOUND_TEST_STATUS: PASS_FINITE_SYNTHETIC_COUNTEREXAMPLES

24项Grab核心用例含完整全局重捕获/重新确认及cross covariance reset。
36组合扫描24组TRACKED，12组被拒绝；不把拒绝组删除后只报成功率。
该有限合成数据集false TRACKED、unsafe optimistic clearance、bottom violation均0。
不构成真实CAD/噪声/遮挡分布下的安全概率证明。

## 性能与证据

本次Ubuntu容器合成扫描（含clearance）：
PERFORMANCE_P50=26.07ms
PERFORMANCE_P95=57.59ms
PERFORMANCE_MAX=74.39ms
生产frame period/实时阈值未冻结，不把此虚拟机结果当现场SLA。
per-stage P50/P95/MAX和全部失败场景保存在phase2a-monte-carlo.json。

- [静态CI 38239355507](https://github.com/guolichen007/chili-crane-automation/actions/runs/38239355507)
- [ROS2/vendor CI 38239355519](https://github.com/guolichen007/chili-crane-automation/actions/runs/38239355519)
- ros2-humble job114777839797、vendor job114777839494均success。
- 软件artifact11743120911含normal/symlink构建日志、两套shadow结果、版本、
  Monte Carlo和旧传输/RViz/PTP证据。
- 该次container log报告digest：
  sha256:860e53793ccf575e5369b5e9a19939de2bfcfb46a299954d06f54f2af54fed19。
  workflow仍继承floating tag，不声称供应链全部锁定。

本页归档上述已完成精确SHA，不自引用文档提交；最终HEAD需再次对照Actions。

## 冻结与未执行

时间链五文件及vendor lock/patch相对BASE_SHA Git内容/属性零差异，main未动。
observer仅通过独立安装薄入口解决可执行性，不改冻结实现。
所有新runtime仅shadow，physical_output_enabled/automatic_control_enabled/
camera_control_authority均false，valid_for_control始终false。

FIELD_EXTRINSIC_STATUS: NOT_RUN
GRAB_FIELD_VALIDATION_STATUS: NOT_RUN
PIT_FIELD_VALIDATION_STATUS: NOT_RUN
PHYSICAL_ACTUATION_STATUS: NOT_RUN
FULL_AUTO_CYCLE_STATUS: NOT_RUN
NATIVE_FIELD_MACHINE_STATUS: NOT_RUN

CAD mesh导入、deskew、AI proposal、TEASER++、真实料面/planner/contact、
X servo/NDT和控制闭环未实现，本轮没有声称这些完成。
已有CAD采样/模型包络和kσ界须owner现场确认；未知配置保持NOT_CONFIGURED。
S16A及BENCH_FLOOR_001仅用户回传事实，不是本机或本次FIELD外参验收。
