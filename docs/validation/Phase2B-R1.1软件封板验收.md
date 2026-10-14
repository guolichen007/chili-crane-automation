# Phase2B-R1.1 最终软件封板验收

日期：2026-10-14。BASE_SHA=8ee41886dfb8d084b2b01d505b770c98bf2fefde。
分支：codex/phase2b-final-seal-r1-1。范围仅Grab真正MID_SCAN共时与surface producer session/restart两个P1。
历史R1验收不覆盖；main、PTP/vendor、dual LiDAR clock、V1、Geofence v2、Phase2A核心/门限与B算法冻结。

## 当前状态

CODE_SHA=9f497a20664cd2146ef4865bd54e9d3f260b6743。
PHASE2B_SOFTWARE_STATUS=SEALED_SOFTWARE_SHADOW_NOT_FIELD。
WINDOWS_STATUS=PASS_STATIC_NATIVE；382项测试，377通过、5项缺ROS环境跳过。
该CODE_SHA的[Static 38502926155](https://github.com/guolichen007/chili-crane-automation/actions/runs/38502926155)与
[ROS/vendor 38502926358](https://github.com/guolichen007/chili-crane-automation/actions/runs/38502926358)全部通过。
normal/symlink各七包、各326 tests无错误/失败/跳过；构建前382项，378通过、4项缺生成消息跳过。
两安装树A/B和R1.1 installed全部通过：三种重启、TL无新snapshot恢复、旧session/同session倒序反证、
同generation不同session身份与cache隔离；SafetyPermit/AuthorizedCommand/ActuationRequest均0。
12份restart节点日志无Fatal Python/Traceback/KeyboardInterrupt/ERROR；实际ZIP digest核对通过。
PTP/dual LiDAR/Phase1C/Capture/全部RViz/mock/vendor旧入口全部通过，旧八类/新增五类计数均0。

MID_SCAN_TIME_ANCHOR_STATUS=PASS_UNCHANGED_MERGER
GRAB_MASK_MID_SCAN_ALIGNMENT_STATUS=PASS_INSTALLED
SURFACE_SESSION_ID_STATUS=PASS
SURFACE_RESTART_RECOVERY_STATUS=PASS_INSTALLED_ALL_THREE
OLD_SESSION_REPLAY_REJECTION_STATUS=PASS
SNAPSHOT_ID_UNIQUENESS_STATUS=PASS
RESTART_PUBLICATION_ORIGIN_STATUS=PASS_UNIQUE_OUTPUT_TOPIC_PER_PROCESS
P0_FINDINGS=0
P1_OPEN_FINDINGS=0

ROS2_NORMAL_STATUS/ROS2_SYMLINK_STATUS/ROS2_PHASE2B_SHADOW_STATUS/VENDOR_STATUS=PASS_FOR_CODE_SHA。
GITHUB_ACTIONS_STATUS=PASS_FOR_CODE_SHA；本文归档提交仅文档，最终HEAD仍需独立完整CI，不能以旧CODE_SHA代验。
只封软件shadow，不封现场/生产。剩余全部转FIELD gate或另行授权Phase2C，不继续Phase2B-R1.2。

## 验收审计与修正历史

首代码9ed69a3的[Static 38499327447](https://github.com/guolichen007/chili-crane-automation/actions/runs/38499327447)与
[ROS/vendor 38499327449](https://github.com/guolichen007/chili-crane-automation/actions/runs/38499327449)全绿。
但normal planner-2.log显示在Python初始化时就被中断：旧节点排队proposal可能在received清空后到达，
planner-only predicate有假阳性。历史全绿不作为最终重启证据，最终CODE_SHA已重新实跑加强反证。
验收修复采用每次planner启动独立volatile proposal输出topic/订阅的ROS测试remap，
只允许当前进程的probe回调，不可能接收旧writer排队proposal。snapshot INPUT topic/QoS不变，仍真实TL恢复。
planner-only先等旧publisher退出图；不替换任何measurement epoch/arrival/freshness，生产节点接口不改。
中间1220465的[ROS 38501704940](https://github.com/guolichen007/chili-crane-automation/actions/runs/38501704940)与
cd68dbb的[ROS 38501981513](https://github.com/guolichen007/chili-crane-automation/actions/runs/38501981513)失败：
normal七包/326 tests与旧入口通过，但Humble公开executor仅传单message参数，两参callback缺info引发TypeError。
虽然底层take_message含source_timestamp，不能据此假设公开订阅支持metadata callback；移除该不可用API。
9f497a2移除该API，完整CI重新通过，日志核验无该初始化/回调错误；不重跑失败SHA掩盖历史。
旧session/同session倒序重放改变generation时同步重建合法snapshot identity，保留真实旧epoch；
保证反证不是被不匹配ID偶然拒绝，而是真正检验retired session或代际门禁。

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
installed独立测试实际证明surface-only、planner-only TL无新snapshot、both restart恢复，
旧session重放/同世代不同session/同session倒序不复用旧proposal/cache。三控制topic仍0。
消息扩展需全体重建，禁止新旧消息混跑；PHYSICAL disturbance仍拒绝，真实执行证据链留Phase2C。

## 回归与性能

保留原21场景、64序列、旧八类/新增五类计数、A/B installed与所有旧入口。
冻结方法AST与精确base比较通过；normal/symlink各七包和完整测试实跑通过。
FAST_REJECT/FULL_VALID_PROPOSAL/DENSE_SURFACE原instrumentation不扩优化，实际CDR与性能如下。
search completeness != safety eligibility；SEARCH_TRUNCATED/GLOBAL_OPTIMUM_NOT_PROVEN保持。
truncated candidate控制准入留未来policy，本轮不决定。

每workload八样本；实际CDR，非FIELD实时SLA。normal/symlink与配对测量不混合P95。

| workload | normal P50/P95/MAX ms | symlink P50/P95/MAX ms |
| --- | --- | --- |
| FAST_REJECT | 1.93 / 12.46 / 18.05 | 9.01 / 20.12 / 25.94 |
| FULL_VALID_PROPOSAL | 140.33 / 141.90 / 141.95 | 141.94 / 144.25 / 144.89 |
| DENSE_SURFACE | 468.19 / 484.79 / 490.30 | 460.61 / 464.92 / 466.07 |

最大实际CDR=109374bytes；partial64/256/1024格传30/120/482 observed格，4766/14846/55390bytes。
相对旧CDR增加112bytes属于session身份字段；不是扩展格网或候选算法。
Windows同机P95=.580/176.834/583.464ms；旧R1Windows约.598/175.942/586.364ms，基本一致。
9ed69a3跨run dense曾为1222.52ms，高于8ee的856.19ms；因此增加精确base archive的同runner配对，
并非优化算法。源码仅增加身份/共时，不能把新runner更快包装为优化收益。

| workload | base-before P95 ms | current-paired P95 ms | base-after P95 ms |
| --- | --- | --- | --- |
| FAST_REJECT | 12.45 | 12.39 | 12.01 |
| FULL_VALID_PROPOSAL | 134.97 | 139.38 | 143.66 |
| DENSE_SURFACE | 474.89 | 455.89 | 457.88 |

同机配对三路径未见明显回归；FULL current位于两次base之间，DENSE没有变慢。不冻结生产P95或新性能门限。
旧版本使用同一已安装ROS类型支持，新增session字段为空；该对比不是新旧wire混跑兼容性验证。
配对base CDR=109270bytes（旧字段+新类型支持空字段），不能冒称原8ee wire=109270；原8ee归档为109262。
normal/symlink完整13阶段P50/P95/MAX与每样本容量在phase2b-r1-normal/symlink.json；
配对原始记录在phase2b-r11-base-before/current-paired/base-after.json。
artifact11866329941，SHA256=eead52bceecdcd4c40ed36868064da119adfcd584fef1d3b8bcfd8b5ea302ec7；
vendor artifact11866588366。临时下载凭证/原始ZIP只存忽略目录，不能提交；CI image digest仍NOT_CAPTURED。

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
