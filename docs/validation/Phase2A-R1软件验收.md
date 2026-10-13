# Phase2A-R1 软件验收与现场边界

基线：dc42fb16ad0ca14a38d919a9e26acb8836fef16d。
分支：codex/phase2a-grab-hardening-r1。
软件输出SHA/Actions结果须在实际CI结束后归档，当前不能沿用Phase2A旧绿灯。

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

软件CI精确SHA验收：PENDING，待实际完成后更新本节。Windows测试不代替UbuntuCI。
PTP五文件、vendor lock/patch与main须最终核对零变更；本机上下文与生成证据不提交。

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
