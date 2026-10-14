# Phase2B-R1.1 最终软件封板验收

日期：2026-10-14。BASE_SHA=8ee41886dfb8d084b2b01d505b770c98bf2fefde。
分支：codex/phase2b-final-seal-r1-1。范围仅Grab真正MID_SCAN共时与surface producer session/restart两个P1。
历史R1验收不覆盖；main、PTP/vendor、dual LiDAR clock、V1、Geofence v2、Phase2A核心/门限与B算法冻结。

## 当前状态

CODE_SHA=9ed69a3ecc112ce95f6bfebf5d24f789b7b5e6c6；PHASE2B_SOFTWARE_STATUS=PENDING_RESTART_ORIGIN_AND_PAIRED_CI。
WINDOWS_STATUS=PASS_STATIC_NATIVE；382项测试，377通过、5项缺ROS环境跳过。
该CODE_SHA的[Static 38499327447](https://github.com/guolichen007/chili-crane-automation/actions/runs/38499327447)与
[ROS/vendor 38499327449](https://github.com/guolichen007/chili-crane-automation/actions/runs/38499327449)全部通过。
normal/symlink各七包、各326 tests无错误/失败/跳过；构建前382项，378通过、4项缺生成消息跳过。
两安装树A/B和R1.1 installed全部通过，三种重启、TL无新snapshot恢复、旧session/世代重放反证通过，三控制topic各0。
以上是脚本历史回传，不作为最终planner-only证据：归档normal planner-2.log显示在Python初始化时就被中断。
审计发现旧节点排队proposal可能在received清空后到达，导致planner-only predicate假阳性；软件状态不封板。
修复验收callback保存Humble DDS source_timestamp，仅用于证明publication发生于新进程启动之后，
不替换任何measurement epoch/arrival/freshness。planner-only先等旧publisher退出图，再接纳新DDS publication。
最终SHA必须重新跑完整CI与检查节点日志；历史全绿不隐藏验收漏洞。
PTP/dual LiDAR/Phase1C/Capture/全部RViz/mock/vendor旧入口全部通过。
ROS2_NORMAL_STATUS/ROS2_SYMLINK_STATUS/ROS2_PHASE2B_SHADOW_STATUS/VENDOR_STATUS=PASS_FOR_CODE_SHA。
GITHUB_ACTIONS_STATUS=PASS_FOR_CODE_SHA_ONLY；新publication-origin反证/同runner性能及最终HEAD仍需完整精确CI。
不能用Windows或历史8ee CI代替本轮ROS构建、installed重启与最终HEAD CI。
只有全部反证与旧回归通过才标SEALED_SOFTWARE_SHADOW_NOT_FIELD；不继续Phase2B-R1.2。

## 时间锚点

header_epoch继续FIRST_POINT证据身份，merged header=max(两个FIRST_POINT)，上游未修改。
frame_mid_epoch由canonical point timestamp min/max计算；frame.epoch不暗改，显式frame.mid_epoch用于Grab skew。
80/100ms扫描与true MID Grab通过；FIRST_POINT Grab（半跨度超skew）、旧但TTL fresh Grab拒绝。
双scan 4ms phase，mid不是任一header；单sensor内部点时间regression/nonfinite拒绝。
header落在范围内的检查更名HEADER_OUTSIDE_POINT_TIME_RANGE，不称其MID_SCAN证明。
latest-only真实测量，不插值/预测/arrival伪造。

## session与重启

一次进程UUID进入status/snapshot/proposal/cache，snapshot_id=session:config前12位:generation:lineage前12位。
新session先清latest/cache/floors/pending与旧事件，再验证freshness/provenance/空间context建立基线。
同session严格代际单调；更高无效世代也阻断旧状态；137→1在新session允许。
退役session有界拒绝集合128项，容量耗尽fail-closed，不LRU复活。session不是签名/认证。
installed独立测试必须证明surface-only、planner-only TL无新snapshot、both restart恢复，
旧session重放/同世代不同session/同session倒序不复用旧proposal/cache。三控制topic仍0。
消息扩展需全体重建，禁止新旧消息混跑；PHYSICAL disturbance仍拒绝，真实执行证据链留Phase2C。

## 回归与性能

保留原21场景、64序列、旧八类/新增五类计数、A/B installed与所有旧入口。
冻结方法AST与精确base比较；normal/symlink各七包和完整测试必须实际通过。
FAST_REJECT/FULL_VALID_PROPOSAL/DENSE_SURFACE原instrumentation不扩优化，P95与实际CDR待Ubuntu归档。
search completeness != safety eligibility；SEARCH_TRUNCATED/GLOBAL_OPTIMUM_NOT_PROVEN保持。
truncated candidate控制准入留未来policy，本轮不决定。

9ed69a3 normal P95 FAST/FULL/DENSE=23.46/368.13/1222.52ms；symlink=35.45/389.99/1252.41ms。
最大实际CDR=109374bytes；partial64/256/1024格传30/120/482 observed格，4766/14846/55390bytes。
相对旧CDR增加112bytes属于session身份字段；不是扩展格网或候选算法。
Windows同机P95=.580/176.834/583.464ms；旧R1Windows约.598/175.942/586.364ms，基本一致。
但共享Ubuntu runner的跨run dense数字高于8ee的856.19ms，不能直接宣称没有性能回归。
增加只读精确base archive的同runner base-before/current/base-after八样本对比，等待实际配对结果。
旧版本使用同一已安装ROS类型支持，新增session字段为空；该对比不是新旧wire混跑兼容性验证。
artifact11865238700，SHA256=e05cb8a8425f241c95aa4f00ec254e6f61b0c35cb61cc6b0aac870f7c1a65b57；
vendor artifact11865256450。临时下载凭证/原始ZIP只存忽略目录，不能提交。

## FIELD与执行边界

FIELD_EXTRINSIC_STATUS=NOT_RUN
GRAB_FIELD_VALIDATION_STATUS=NOT_RUN
PIT_SURFACE_FIELD_STATUS=NOT_RUN
GRASP_PROPOSAL_FIELD_STATUS=NOT_RUN
MATERIAL_VOLUME_PROXY_FIELD_STATUS=NOT_RUN
AXIS_TO_MAP_CALIBRATION_STATUS=NOT_RUN
OPERATIONAL_GEOFENCE_FIELD_VALIDATION_STATUS=NOT_RUN
FIELD_PERFORMANCE_STATUS=NOT_RUN
PHYSICAL_ACTUATION_STATUS=NOT_RUN
FULL_AUTO_CYCLE_STATUS=NOT_RUN

Phase2A两个组合缺口保持POST_R1_FIELD_VALIDATION_REQUIRED，不扩Grab synthetic算法。
physical_output_enabled=false，automatic_control_enabled=false，camera_control_authority=false，valid_for_control=false。
SafetyPermit=0、AuthorizedCommand=0、ActuationRequest=0必须由实际installed订阅验证，非配置推断。
