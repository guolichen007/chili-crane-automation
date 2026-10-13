# Phase2A-R1 软件验收与现场边界

基线：dc42fb16ad0ca14a38d919a9e26acb8836fef16d。
分支：codex/phase2a-grab-hardening-r1。
本记录归档已完成验收的代码SHA fb0999d4a4d970cbaac6129a14d338ad51cea45a。
记录提交的HEAD不是其自身验收SHA；后续HEAD另核对Actions，不能沿用旧绿灯。

## 复验入口

Windows使用已安装numpy/scipy/small_gicp1.0.1/Shapely2.0.7的Python运行：

```text
git diff --check
python tools/check_repo_contracts.py
python -m unittest discover -s tests_static -p "test_*.py"
python tools/validate_architecture_scenarios.py
python tools/validate_phase2a_synthetic.py --output <outside-repository>/matrix.json
python tools/validate_phase2a_r1.py --output <outside-repository>/r1.json
```

Ubuntu22/Humble用scripts/validation/ubuntu22_ros2_phase0_validate.sh绑定精确SHA。
normal与独立symlink均build/test；启动真实installed preview/capture/shadow入口，
额外10ms合成处理延迟验证state epoch不变；输入停止后STALE/LOST，无控制消息。
旧双雷达、PTP、Capture、RViz、mock回归及官方driver wrapper编译保持。
Windows不执行/宣称ROS2 build、launch、colcon或实机测试。

## 软件验收项目

|项目|证据入口|验收边界|
|---|---|---|
|TRANSITION core|test_phase2a_r1 + sequence fuzz|core可观时测量；缺失/弱可观预测并失锁|
|模型门控与料面接触|flat/rough、above/touch/25%/50%、OPEN/CLOSED/TRANSITION|连续欧氏连通料面，不仅孤立离群点|
|IO几何冲突|相反开度假设|不接受冲突测量|
|state epoch/validity|纯函数+installed ROS延迟/过期|MID_SCAN继承，不心跳刷新|
|空间可观性|model frame平移反证|bbox中心sector、特征长度normalized H；物理R保留量纲|
|clearance/provenance|内部/接触/凹坑/跨界/外部|undefined哨兵0，不伪造signed penetration；绑定map/model/extrinsic|
|FIELD外参资格|pure gate反证|BENCH/CANDIDATE不具空间控制资格|
|OperationalGeofence|凹坑内缩/单调性/track range|引用V1，不复制polygon，不改Grab pose|
|Y方向限位与交叉验证|known_signals、双触发、窗口错配、应触发未触发|只shadow；危险方向禁止，逃离另评估|
|X servo/axis-to-map|stale/unhomed/fault/source/标定/地图身份|缺现场生产标定/停车模型不具资格|
|200序列|seed=20261013，结果JSON保留coverage与失败样例|有限合成样本，不是FIELD概率保证|
|dense/latest-wins|接近200000点、料面/Grab连通、墙/多个簇|Python retained allocation检查，不等于native heap长期验收|
|控制隔离|shadow launch三控制topic观察|0 SafetyPermit/AuthorizedCommand/ActuationRequest；不加载DO writer|

八类计数：false_TRACKED、wrong_reacquisition、transition_tracking_loss、optimistic_clearance、
bottom_bound_violation、state_timestamp_violation、geofence_false_safe、directional_limit_violation。
任何非零导致验收失败；脚本不隐藏失败序列。性能分sequence与dense两类、独立平台记录，
不冻结生产P50/P95/MAX或最坏时延；每帧deadline继续fail-closed。

## 当前结果归档

SOFTWARE_STATUS=PASS_SOFTWARE_SHADOW_NOT_FIELD。
验证日期：2026-10-13。代码SHA：fb0999d4a4d970cbaac6129a14d338ad51cea45a。
[静态CI](https://github.com/guolichen007/chili-crane-automation/actions/runs/38340866383)与
[Ubuntu/Humble和vendor CI](https://github.com/guolichen007/chili-crane-automation/actions/runs/38340866364)
均SUCCESS；ROS job115111300434、vendor job115111300160。
[完整软件证据](https://github.com/guolichen007/chili-crane-automation/actions/runs/38340866364/artifacts/11786887113)。

- Windows Python3.11 native/static：309项，308通过、1项因无ROS环境跳过；46架构案例通过。
- Ubuntu22/Humble CI容器：309静态全部通过；normal七包build+245 tests，symlink七包build+245 tests，均零错误/失败/跳过。
- 两套installed shadow/preview/capture入口均通过，MID_SCAN epoch继承与10ms额外处理延迟反证、过期失效通过。
- 旧dual-LiDAR六场景、PTP八场景、20帧采集、四套RViz与mock均通过；官方驱动包装编译通过。
- 两套shadow均0 SafetyPermit/AuthorizedCommand/ActuationRequest；不加载硬件DO writer。
- 原36组合：25个最终TRACKED、11个未TRACKED；false TRACKED/乐观clearance/bottom界违规0。
- 新200序列：320料面接触帧、160长TRANSITION帧、200一致外参扰动、10全局重捕获、16标定错配、600Geofence状态样本；八类违规均0。
- dense：198737输入点、8帧，latest-wins与Python retained allocation通过；峰值32283230bytes、稳定期增长4078bytes。

|平台/场景|P50 ms|P95 ms|MAX ms|
|---|---:|---:|---:|
|Windows 200序列|16.37|46.82|63.77|
|Windows dense（tracemalloc开启）|142.45|145.08|145.71|
|Ubuntu CI 200序列|32.57|56.89|75.60|
|Ubuntu CI dense（tracemalloc开启）|314.75|321.02|321.21|
|Ubuntu CI 原36组合|38.54|111.55|157.93|

这是合成基线，不是生产帧周期/最坏时延保证；dense较慢的数字不隐藏，超时门禁仍保留。
Native heap长期增长、真实CPU现场资源和实机性能均NOT_RUN。
PTP五文件、全部third_party（含lock/patch）、SemanticMap V1 schema相对基线零变更；
远程main仍01ce545bd064d7f80470f76d3e2b0658530c3ae6；本机上下文与生成证据不提交。

### 完整软件状态表（仅上述代码SHA）

```ini
TRANSITION_CORE_STATUS=PASS_NATIVE_SYNTHETIC
MODEL_GATED_TRACKING_STATUS=PASS_NATIVE_SYNTHETIC
MATERIAL_CONTACT_STATUS=PASS_SYNTHETIC
IO_GEOMETRY_CONFLICT_STATUS=PASS_SYNTHETIC
STATE_EPOCH_STATUS=PASS_SYNTHETIC_ROS
VALIDITY_SEMANTICS_STATUS=PASS_SOFTWARE
SUPPORT_FRAME_INVARIANCE_STATUS=PASS_SYNTHETIC
OBSERVABILITY_NORMALIZATION_STATUS=PASS_SYNTHETIC
CLEARANCE_SEMANTICS_STATUS=PASS_SYNTHETIC
SEMANTIC_MAP_PROVENANCE_STATUS=PASS_SYNTHETIC_ROS
FIELD_EXTRINSIC_GATE_STATUS=PASS_PURE_POLICY
OPERATIONAL_GEOFENCE_STATUS=PASS_PURE_POLICY
Y_DIRECTION_LIMIT_POLICY_STATUS=PASS_PURE_POLICY
Y_POSITION_LIMIT_CROSSCHECK_STATUS=PASS_PURE_POLICY
X_SERVO_GEOFENCE_POLICY_STATUS=PASS_PURE_POLICY
AXIS_TO_MAP_GATE_STATUS=PASS_PURE_POLICY
DIRECTIONAL_ESCAPE_STATUS=PASS_PURE_POLICY
SEQUENCE_FUZZ_STATUS=PASS_SYNTHETIC_200_SEQUENCES
DENSE_CONTACT_STRESS_STATUS=PASS_SYNTHETIC_198737_POINTS
FALSE_TRACKED_COUNT=0
WRONG_REACQUISITION_COUNT=0
TRANSITION_TRACKING_LOSS_COUNT=0
OPTIMISTIC_CLEARANCE_COUNT=0
BOTTOM_BOUND_VIOLATION_COUNT=0
STATE_TIMESTAMP_VIOLATION_COUNT=0
GEOFENCE_FALSE_SAFE_COUNT=0
DIRECTIONAL_LIMIT_VIOLATION_COUNT=0
PERFORMANCE_P50=32.57ms_UBUNTU_CI_SEQUENCE
PERFORMANCE_P95=56.89ms_UBUNTU_CI_SEQUENCE
PERFORMANCE_MAX=75.60ms_UBUNTU_CI_SEQUENCE
WINDOWS_STATUS=PASS_STATIC_NATIVE_308_PASS_1_ROS_SKIP
UBUNTU_STATIC_STATUS=PASS_CI_309_TESTS
ROS2_NORMAL_STATUS=PASS_7_PACKAGES_245_TESTS
ROS2_SYMLINK_STATUS=PASS_7_PACKAGES_245_TESTS
ROS2_INSTALLED_SHADOW_STATUS=PASS_SYNTHETIC_ROS
VENDOR_STATUS=PASS_SOFTWARE_COMPILE
PTP_FROZEN_STATUS=UNCHANGED
GITHUB_ACTIONS_STATUS=PASS_EXACT_CODE_SHA
NATIVE_UBUNTU_FIELD_MACHINE_STATUS=NOT_RUN
```

### 深审修复过程（保留失败证据，不用重试掩盖缺陷）

0045538/ddbfcad的installed shadow因processing deadline正确失效；独立限制另开度诊断预算，
复用已知pose seed后normal通过，evidence deadline和生产参数未放宽。
8fb7349的normal shadow和normal/symlink build/test通过，但旧Capture合成测试在异步topic
半代送达时反复读取最新帧、错过同代配对；fb0999d仅调整测试batch投送/采集节拍并公开symlink日志。
生产Capture、20帧要求、1ms配对条件、0.2s证据TTL及所有冻结PTP文件未修改。
最终完整流程重新执行并通过，不将上述失败SHA标为全部PASS。

## 现场状态（全部不由合成PASS升级）

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
PHYSICAL_ACTUATION_STATUS=NOT_RUN
FULL_AUTO_CYCLE_STATUS=NOT_RUN
physical_output_enabled=false
automatic_control_enabled=false
camera_control_authority=false
valid_for_control=false
```

## 不兼容迁移与回退

R1增加GrabGeometry/GrabClearance字段并修正header、clearance定义；全部ROS消费者须重建，
不能混用旧消息或将undefined的0哨兵视作真实触壁距离。
需要回退时另建评审分支指向历史dc42fb16，保留公开历史，不强推，不改main。
