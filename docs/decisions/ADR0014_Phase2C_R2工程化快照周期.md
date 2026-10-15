# ADR0014 — Phase2C-R2 Engineering Snapshot Cycle

日期：2026-10-15。状态：ACCEPTED_FOR_IMPLEMENTATION；软件/工控机/FIELD 验收分别记录。
基线：`7f8378d9e670c4145836264e268ce3a500a438a7`。
本裁决取代 ADR0012 与 R1/R1.1/R1.2 关于连续料面刷新主链的规定，历史 PASS/FAIL 不改写。

## 裁决与理由

辣椒物料在一次抓取后会重排。第一版按抓取周期重新停车扫描，比抓取期间维护 affected cells、
局部 invalidation、observation_after、partial refresh 和 UNKNOWN 恢复更容易验证和排故。
提前计算下一抓节省的少量时间，不足以抵消跨模块等待与旧证据复用的复杂性。
必要的 identity、时间顺序、geofence、冻结 TargetBinding 和 fail-closed 保留。

主链固定为 SCAN -> snapshot -> B proposal -> PLAN_PICK/binding -> 冻结 -> 抓取/离坑/卸料 -> COMPLETE。
下一 cycle 重新 SCAN。保留原 TaskCycles 枚举、状态所有权与全部安全授权链。
Surface 的四个观测状态为 SCAN_REQUIRED、SCANNING、SNAPSHOT_READY、FROZEN_ACTIVE_CYCLE。
这些是编排状态，不是新增 TaskCycles 状态，也不授予控制权限。

SCAN 允许融合；PLAN_PICK 保留最后一个扫描窗口且停止融合；POSITION_Y 到 COMPLETE/ABORTED
禁止融合和新规划。Binding 建立也立即冻结。缺失、过期或冲突的 CycleStatus 阻止扫描/规划。
新 cycle 首次 SCAN 清空整个旧窗口，记录 scan_start_epoch，只接纳该时刻之后的新帧。
同 session 的 generation 单调增加；跨 producer session 不比较 generation。

下一绑定必须由本 cycle 的新扫描、有效 snapshot 和真正通过 B hard gates 的 proposal 建立。
ROS 状态/提议携带 cycle identity、scan_start、window_start/end；新 generation 本身不构成新扫描证明。
Node 核对 proposal history、源身份、digest，以及当前冻结 snapshot；engine 独立核对 ScanProof。
ACTIVE cycle 的 surface restart 仍 abort；terminal restart 只能在下一 cycle 完整扫描后恢复。

## 抓取证据边界

LOWER/CLOSE/RAISE 使用冻结 bound surface、当前 Grab/geometry/Hoist/IO/Load/CycleStatus。
PIT_EXIT 仍要求严格 bound clear、有效 pit rim geometry、safe height、tracking/provenance 与时序。
UNLOAD 使用合法离坑历史和当前卸料证据；COMPLETE 不依赖下一抓规划。
R2 的 surface_invalidation_required=false，updated_clear/refresh/next_plan=false，下一抓明确 DEFERRED_TO_NEXT_SCAN。
历史 SurfaceInteractionLifecycle/SurfaceRefreshEvidence 保留兼容与法证代码，退出 R2 full-stack 主路径。
Standalone Phase2B 的默认行为不变；cycle gate 仅在 Phase2C full stack 显式启用。

Contact 只保留已有诊断/penetration 逻辑，本轮不增加分类器或接触模型。
Phase2A 继续整个周期跟踪抓斗，保留 OPEN/CLOSED CAD 外包络、TRANSITION 保守并集与 invariant tracking core。
不实现连杆动力学、连续关节角或逐度网格模型。

## 帧束、进程与验证

cloud/meta/context 是同一 LiDAR frame bundle，必须同 stamp/epoch/counter 一起发布。
grab/geometry/hoist/load/io/cycle 可独立高频；SCAN 内严格 epoch equality 与 MID_SCAN mask 合同保持。
抓取期间直接拒绝 Surface cloud，不通过放松 pairing 掩盖问题。
ROS validator 只清理自己创建的独立 PGID，SIGINT/有界等待/SIGTERM/有界等待，并证明无 owned process 遗留。
normal 与 symlink 分别从干净的 `/opt/ros/humble` 环境构建和加载，记录 localhost/domain 与包解析路径。
两种安装各先执行 10/10 critical-path probe，再执行完整 R2 installed suite；不延长 9s/10s/300s 成功等待。
旧连续刷新 acceptance 明确标为 LEGACY_R1_ARCHITECTURE，不作为 R2 主验收。

## OPEN：真实下降目标 Z

`GraspCandidate.target_suspension_map` 是 3D，但 `surface_transport.proposal_to_message()`
目前只写 x/y，z 默认值不构成经过机械标定的下降目标。
GRAB_DESCENT_TARGET_Z_FIELD_CALIBRATION=NOT_RUN。不得用 synthetic 值或猜测尺寸填生产 Z。
后续必须确认真实 CAD/尺寸、suspension/bottom reference、允许入料深度、坑底余量及 Z 停止误差，
再裁决 target_grab_bottom_z_m 或 target_suspension_z_m 的正式合同。

全部物理/自动/相机/valid_for_control flags=false；不启 ADAM DO，不 merge main，不进入 Phase2D。
最终最多 PASS_CI_SOFTWARE_SHADOW_NOT_FIELD；本轮不封板，IPC/FIELD 待代码审查后另行验收。
