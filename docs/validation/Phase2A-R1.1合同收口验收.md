# Phase2A-R1.1 合同收口验收

BASE_SHA=5a121f8b9ce5f6615b672bb896b570b7e2cd4bcc。
分支codex/phase2a-r1-1-contract-closeout。
代码验收SHA=8533722c1ea8062fed0f0bf355b913818639b29b。
本报告归档该精确代码SHA；包含本报告的最终提交仍须独立通过CI，不能沿用旧R1绿灯。

## 修复与迁移

- Y统一resolve_y_limit_state，显式支持canonical左右字段，依赖known_signals与确定bool。
  negative/positive可反向绑定；窗口/冲突/逃离/违规计数都基于解析值。y_scale=-1不交换信号。
- X只评估位置/axis-to-map/track/pit几何，不把ServoState默认限位false视为已观察安全。
  独立轴安全层仍需核验物理limit/servo soft-limit，不是Geofence geometry truth。
- Geofence config contract_version=2，删除X两项limit signal，拒绝旧版本/旧字段以避免死配置。
  automatic_eligibility无兼容别名，改为directional_geometry_available；不等于allow_auto_task或SafetyPermit。
- Clearance在发布时统一采样upstream age+monotonic residence，正常与异常双层age一致。
  保持header/measurement stamp/source_counter；未知年龄双层同样失效，超过原TTL则STALE/risk=true。

没有修改ServoState或其它ROS消息。消费者应迁移配置/结果名；所有控制flags仍false。
没有重构registration、ESKF、Grab数学、clearance几何；没有扩大Phase2B或新增硬件协议。

## 反证入口

Windows静态使用tests_static/test_operational_geofence.py和test_phase2a_r11.py；
所有旧309案例继续运行，新总计316项（315通过、1因无ROS跳过）。
旧36组合25最终TRACKED、0违规；200序列与198737点dense复验八类违规0。

tools/validate_phase2a_ros.py增加独立namespace下的installed clearance node：
只发布一次synthetic Geometry，20Hz计时发布Clearance，观察内外age严格增加且一致，
epoch/counter恒定，最终STALE与嵌套STALE，valid_for_control恒false。
normal和symlink流程均执行；未放宽TTL、deadline、registration阈值、候选预算、margin或stop model。

## 精确SHA软件证据

2026-10-13核验上述代码SHA的实际GitHub Actions日志：

- [Static CI 38348485814](https://github.com/guolichen007/chili-crane-automation/actions/runs/38348485814)：SUCCESS。
- [ROS2/vendor CI 38348485812](https://github.com/guolichen007/chili-crane-automation/actions/runs/38348485812)：SUCCESS。
  ros2-humble job 115135269667与vendor job 115135269883均SUCCESS。
- Ubuntu22/Humble CI容器：316项静态测试通过；normal与symlink各七包构建、253项colcon测试通过，0失败/错误/跳过。
- 两种安装方式各一次Geometry、20份Clearance：内外age一致、严格递增，header/measurement/counter不变，最终STALE。
  两套installed shadow/preview及过期阻断通过，各0控制topic消息。
- 原36组合：25最终TRACKED，false_TRACKED/乐观clearance/bottom违规均0。
- 原200序列，seed=20261013：八类违规计数均0；没有新增或重写序列生成算法。
- dense：198737点、8帧；Ubuntu CI P50/P95/MAX=324.96/328.63/329.31ms；
  Python tracemalloc峰值32283237bytes、稳定期增长4414bytes。不是native heap或现场容量保证。
- dual LiDAR、PTP、20帧Capture及失步/运动拒绝、RViz加载、mock fail-closed、官方vendor wrapper编译通过。

```ini
Y_BINDING_NORMAL=PASS
Y_BINDING_REVERSED=PASS
Y_NEGATIVE_SCALE_BINDING=PASS
Y_LIMIT_CONFLICT_REVERSED=PASS
Y_POSITION_WINDOW_REVERSED=PASS
X_GEOFENCE_NO_UNKNOWN_LIMIT_ASSUMPTION=PASS
CLEARANCE_OUTER_INNER_AGE_MATCH=PASS
CLEARANCE_AGE_MONOTONIC=PASS
CLEARANCE_MEASUREMENT_EPOCH_UNCHANGED=PASS
CLEARANCE_EVENTUALLY_STALE=PASS
Y_LIMIT_BINDING_STATUS=PASS
Y_REVERSED_BINDING_STATUS=PASS
Y_NEGATIVE_SCALE_STATUS=PASS
X_GEOFENCE_LIMIT_RESPONSIBILITY_STATUS=PASS_CONTRACT
DEAD_CONFIG_STATUS=PASS
CLEARANCE_EVIDENCE_AGE_STATUS=PASS_INSTALLED_SYNTHETIC_ROS
CLEARANCE_EPOCH_PRESERVATION_STATUS=PASS_INSTALLED_SYNTHETIC_ROS
CLEARANCE_STALE_STATUS=PASS_INSTALLED_SYNTHETIC_ROS
OLD_PHASE2A_TEST_STATUS=PASS_SYNTHETIC
SEQUENCE_FUZZ_STATUS=PASS_SYNTHETIC
DENSE_STATUS=PASS_SYNTHETIC
ROS2_NORMAL_STATUS=PASS_CI
ROS2_SYMLINK_STATUS=PASS_CI
ROS2_INSTALLED_SHADOW_STATUS=PASS_SYNTHETIC_ROS
VENDOR_STATUS=PASS_COMPILE_ONLY
GITHUB_ACTIONS_STATUS=PASS_CODE_SHA
```

冻结PTP五文件、全部third_party、SemanticMap V1 schema相对BASE_SHA零差异；
registration/tracker/geometry/pipeline/synthetic与Phase2A perception参数未修改。
新增ROS夹具只在首次构造synthetic输入时同步receive stamp，不刷新已发布测量身份。
Windows静态/native合成不代表ROS运行、bag、硬件或FIELD成功。

## 后续组合缺口

POST_R1_FIELD_VALIDATION_REQUIRED：

1. LOST+dense chili+wall clutter+multiple candidates全局重捕获。
2. TRANSITION core+真实moving jaws+continuous material contact。

当前dense从TRACKED开始；global重捕获较干净；transition可见几何主要为core。
不宣称当前已覆盖，也不把未做的组合标为当前FAIL。本轮不为它们增加新算法。
软件收口后停止扩展Phase2A算法，准备真实外参/CAD/pit/X-Y/停车证据；Phase2B另行推进。

## 现场与控制边界

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
