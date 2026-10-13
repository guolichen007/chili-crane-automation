# Phase2B 料面与抓取提议软件验收

BASE_SHA=d6f846f1f98e621f18e2f6a124253e083ee68bae。
分支codex/phase2b-pit-surface-grasp-planning-v1；Phase2A-R1.1软件封板，不扩展Grab侧synthetic算法。
验收状态：PENDING_UBUNTU_INSTALLED_VALIDATION。不能把Windows通过或前一阶段CI当作本阶段通过。

## 实现与复验入口

保持七包和SemanticMap V1；B1→B5分笔实现observed-only grid、独立mask、footprint hard gates、
proxy/ranking、扰动tombstone与确认、有界typed snapshot/proposal、shadow/RViz/sequence。
ROS适配器均安装于perception，纯规划策略在control；不反转既有package依赖方向。
消息扩展需全量重建；旧GraspTarget不发布，禁止把GraspProposal当控制意图。
详细合同见[算法文档](../algorithms/Phase2B料面建模与抓取候选.md)和[ADR0011](../decisions/0011-phase2b-observed-surface-grasp-proposal.md)。

```text
git diff --check
python tools/check_repo_contracts.py
python -m unittest discover -s tests_static -p "test_*.py"
python tools/validate_architecture_scenarios.py
python tools/validate_phase2b_synthetic.py --output <external-evidence>/phase2b.json
```

Ubuntu22/Humble按scripts/validation/ubuntu22_ros2_phase0_validate.sh绑定精确SHA。
normal/symlink均执行原Phase2A/R1/R1.1全部测试、新Phase2B installed反证及Capture；
原dual LiDAR/PTP/Phase1C/RViz/mock/vendor不删减。不放宽旧TTL/deadline/注册/stop门禁。
Phase2B自己的生产阈值仍null/NOT_CONFIGURED，synthetic参数只在独立NON_PRODUCTION fixture。

## 已确认Windows证据

354项：351通过、3跳过（既有ROS环境依赖、新generated-message roundtrip、新ROS节点bootstrap）。
ROS build/launch/colcon在Windows均NOT_RUN；typed roundtrip另由Ubuntu package tests执行。
21场景与64确定性序列×7帧，seed=20261013，全部八计数0；保留全部场景和失败样例数组。
P50/P95/MAX=26.32/142.93/195.72ms，含场景提议与序列融合/提议/快速拒绝的混合采样，不是生产SLA。
运动Grab64帧、单雷达64帧、dropout64帧、恢复128帧、扰动64次、旧generation核验256次；每个序列候选独立审计。
静态/native入口证明有限合成合同，不证明真实CAD、空间标定、运动补偿、物料动力学或载荷。

## 计数和软件边界

```ini
UNKNOWN_AS_OBSERVED_COUNT=0
INTERPOLATED_CELL_CONTROL_ELIGIBLE_COUNT=0
FALSE_SAFE_WALL_CANDIDATE_COUNT=0
GEOFENCE_ESCAPE_COUNT=0
STALE_SURFACE_PROPOSAL_COUNT=0
PROVENANCE_MISMATCH_PROPOSAL_COUNT=0
GRAB_POINT_AS_MATERIAL_COUNT=0
DISTURBED_OLD_SURFACE_REUSED_COUNT=0
```

上述计数目前绑定Windows的有限synthetic证据；Ubuntu与最终HEAD另行归档。
没有planning interpolation；无真实sample/frame/sensor/邻域support不能作为覆盖或proxy。
ESDF只有reject路径，最乐观错误场仍需exact polygon；四项控制flags恒false。
扰动ID/counter确认避免普通新generation冒充失效；重观测需事件后测量并满足原support。

## 保留失败过程

- 30ceaff：Static38354266880通过；ROS38354266843在package拓扑排序失败。
  新control→perception与既有perception→slam→control成环；纯规划保留control，ROS适配器移至perception修复。
- 5bc271e：Static38355123106通过；ROS38355123119七包build成功但CTest失败。
  artifact11793066546 JUnit显示新测试fixture的ModuleNotFoundError；独立CTest cwd需显式tests_static路径。
  修复导入，原断言全部保留；增强受扰格真实恢复和停止前最后输入drain，不把候选来自别处算恢复。
- 3d83d26：Static38356263042通过；ROS38356263171七包build、295包测试全部通过，旧Phase2A installed也通过。
  新提议节点启动赋值self.context触及rclpy.Node只读属性，实际launch日志显示AttributeError。
  改用spatial_context_input并新增真实Node bootstrap反证；同步处理新节点SIGINT关闭，不改冻结Phase2A。
- 3f7ebdd：Static38357790669通过；ROS38357790733七包build、296包测试通过，
  但旧R1.1单次Geometry反证收到样本不足，clearance进程日志无启动异常。
  原harness只检查输入订阅图，未确认输出数据链；新增实际初始输出握手后再发唯一Geometry。
  六秒发现期限、一秒观察窗、原epoch/age/STALE断言和生产TTL均不改；不能标该SHA为全部PASS。

这些SHA不标全部PASS。没有删除失败测试、缩减入口或通过重复重跑掩盖实际缺陷。

## 现场边界（不由CI升级）

```ini
FIELD_EXTRINSIC_STATUS=NOT_RUN
GRAB_FIELD_VALIDATION_STATUS=NOT_RUN
PIT_FIELD_VALIDATION_STATUS=NOT_RUN
Y_PIT_LIMIT_INSTALLATION_STATUS=NOT_RUN
Y_PIT_LIMIT_HARDWIRE_STATUS=NOT_RUN
Y_PULLWIRE_FIELD_CALIBRATION_STATUS=NOT_RUN
X_SERVO_PROTOCOL_STATUS=NOT_CONFIGURED
X_SERVO_FIELD_CALIBRATION_STATUS=NOT_RUN
X_SERVO_DRIVE_SOFT_LIMIT_STATUS=NOT_RUN
AXIS_TO_MAP_CALIBRATION_STATUS=NOT_RUN
OPERATIONAL_GEOFENCE_FIELD_VALIDATION_STATUS=NOT_RUN
PIT_SURFACE_FIELD_STATUS=NOT_RUN
GRASP_PROPOSAL_FIELD_STATUS=NOT_RUN
MATERIAL_VOLUME_PROXY_FIELD_STATUS=NOT_RUN
PHYSICAL_ACTUATION_STATUS=NOT_RUN
FULL_AUTO_CYCLE_STATUS=NOT_RUN
physical_output_enabled=false
automatic_control_enabled=false
camera_control_authority=false
valid_for_control=false
```

真实payload留Phase2C标定；既有POST_R1_FIELD_VALIDATION_REQUIRED两项组合仍OPEN，不用新料面场景冒充已覆盖。
未知FIELD transform、Grab CAD/kernel、进入深度、pit实测、停车和生产误差界未确认。
