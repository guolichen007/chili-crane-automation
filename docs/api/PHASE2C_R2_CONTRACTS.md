# Phase2C-R2 Engineering Snapshot Cycle

基线 `7f8378d9e670c4145836264e268ce3a500a438a7`。
架构裁决见 [ADR0014](../decisions/ADR0014_Phase2C_R2工程化快照周期.md)。

## 扫描和规划

Phase2C full stack 对 Surface、Planner、Interaction 显式使用 cycle_snapshot_mode=true。
Standalone Phase2B 默认为 false，保留其历史独立合同。
SnapshotCycleGate 只观察原 CycleStatus，不产生状态转换、动作或安全权限。
状态/源 identity、epoch/counter 回退、active run/cycle 切换冲突会阻止工作；缺失/过期也阻止。

只有 SCAN 可以融合；PLAN_PICK 停止融合并保留最后窗口；binding 建立立即冻结。
POSITION_Y 到 COMPLETE/ABORTED 禁止 Surface fusion，禁止有效 B proposal。
Planner 输出 selected_candidate_id=""、PLANNING_FROZEN_ACTIVE_CYCLE、valid_for_control=false。
ACTIVE 周期不执行 grid.snapshot/expire，不因料面 TTL 变化修改冻结 material artifact。
Surface status 的证据年龄仍按原 measurement epoch 增长，不以 heartbeat 伪装 fresh。
当前 grab/geometry/IO/load/hoist/cycle 仍须各自通过 freshness/provenance/chronology。

新 cycle 首次 SCAN 清空整个 builder window、cache 与规划资格，记录 scan_start。
只接纳 scan_start 之后的完整新帧。同 producer session 的 generation 跨窗口单调增加。
新 cycle_id/index 需要上一 cycle 的已观察 COMPLETE/ABORTED；不接受 active 偷换 binding。
DDS 节点可在不同 SCAN heartbeat 加入。Surface 的 scan_start 是扫描来源，proposal 原样保留；
每个消费者要求 window_start 同时晚于该来源起点与自身首次观察本 SCAN 的时刻，不比较两个接收时刻相等。
晚加入时若窗口含更早帧，须等待这些帧退出窗口；不得改写起点或伪造 fresh。
Surface 适配层保留最多 32 对原始 grab geometry/clearance 观测，按 MID_SCAN 选择同 epoch/counter/frame 的一对。
其原 stamp、arrival、provenance、freshness 和原 maximum_grab_mask_time_skew_sec 检查均保留；
不能把最新 20Hz 几何错用于较早的 5Hz cloud，也不能外推、改 stamp 或增加时差阈值。

## ScanProof 与 binding

PitSurfaceStatus / GraspProposal 新增 scan_run_id、scan_task_id、scan_cycle_id、scan_cycle_index、
scan_start_stamp、surface_cycle_state、observed_cycle_state；proposal 还携带 surface_window_start/end。
该消息扩展要求八包在同 SHA 重建，normal/symlink 不混用。
TargetBinding 原 immutable identity/digest 不变；独立 ScanProof 用其 run/task/cycle/index 绑定扫描出处。

绑定入口必须同时具备：

- 当前 owner 为 PLAN_PICK；Surface status 已观察 PLAN_PICK。
- 当前 cycle 的 scan_start < window_start <= window_end，且 window_end 是绑定证据的真实 measurement epoch。
- 实际当前 frozen snapshot 的 session/generation/snapshot_id 与 proposal 一致。
- fresh proposal_history 中的真实 B proposal、selected candidate 的原 hard gates、source identity 和 binding digest。
- 同 session 的下一 generation 严格大于前一 bound generation；新 cycle 不得重用前一窗口。

只更改 generation、cycle_id 或 receive timestamp 不能把旧 material window 变成新扫描。
R2 engine.bind 不要求 last_refresh，也不接受缺失 ScanProof。
ACTIVE producer restart 仍 ABORT_REQUIRED；terminal 新 producer 只能通过下一 cycle 新 scan 恢复，跨 session 不比 generation。

## 当前抓证据

PAYLOAD 保留原 closure/interaction/rise/bound-clear/load/settling 门禁。
PIT_EXIT 保留严格 bound/rim/safe-height/tracking/provenance/chronology 和历史锁存。
UNLOAD 使用该历史及当前 cart/zone/geometry/IO/load；VERIFY_EMPTY 完成不要求下一 plan。
R2 surface_invalidation_required 始终 false，不发 SURFACE_INVALIDATION_REQUIRED 事件。
updated_clear、surface_refresh_ready、next_cycle_surface_ready=false；next_plan_reason=DEFERRED_TO_NEXT_SCAN。
SurfaceInteractionLifecycle / refresh / partial UNKNOWN 的历史实现仅兼容/法证保留，full stack 不构造或消费它们。

## 验证与边界

合成 LiDAR frame bundle 只在 due 时构造并一起发布 cloud/meta/context，相同 stamp、epoch、source counter；
独立 grab/IO/load/cycle 高频循环不覆写 frame metadata。产品严格 epoch equality 不变。
normal 与 symlink 使用独立 env -i 子进程，只从 /opt/ros/humble 加载各自 local_setup.sh；
验证构建前 prefix、安装后八包解析、ROS_LOCALHOST_ONLY=1 和显式 domain。
默认 domain 42；外部显式 0..232 可保留并记录，平台端口注意事项见
[ROS Humble domain 文档](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Concepts/Intermediate/About-Domain-ID.rst)。

validator 仅清理自己 start_new_session 创建的 PGID 与有自身 ownership marker 的后代 PGID。
使用 [Linux subreaper](https://man7.org/linux/man-pages/man2/PR_SET_CHILD_SUBREAPER.2const.html)
回收 wrapper 先退出后的孤儿子进程；SIGINT/有界等待/SIGTERM/有界等待后仍有成员即 FAIL。
分别完成 normal 10/10、symlink 10/10 critical probes，再完成完整 R2 suite。
没有延长 payload 9s、unload 10s 或 runner 300s。

## R2.1 工程边界收口

- **snapshot refresh 参数合同**：`GraspInteractionEngine` 将配置必需参数拆为
  `COMMON_REQUIRED` 与 `LEGACY_REFRESH_REQUIRED`。snapshot 模式只要求当前 cycle 安全参数；
  `collapse_margin_m`/`updated_clearance_margin_m` 仅 legacy 模式必需，快照模式下缺失不再
  返回 `INTERACTION_PARAMETERS_NOT_CONFIGURED`。
- **新 cycle GrabMask 时间边界**：`GrabMaskPairs.reset()` 在真正的新 cycle SCAN 时清空全部样本；
  `closest()` 增加 `minimum_epoch_ns`，surface snapshot 路径传入当前 `scan_start_ns`，凡
  `key_timestamp_ns <= scan_start_ns` 的 grab/clearance 一律不进入本轮扫描 mask（fail closed
  `GRAB_MASK_TIME_ALIGNMENT_REJECTED`）。
- **architecture_mode 一致性**：正式 R2 配置（surface/interaction）声明统一
  `architecture_mode: SNAPSHOT_PER_CYCLE`（`surface_mode` 为 legacy 别名）。节点启动时
  `cycle_snapshot_mode` 参数与配置 `architecture_mode` 必须一致，否则
  `PHASE2C_ARCHITECTURE_MODE_MISMATCH` fail closed，不发布可用于 planning/grasp 的有效结果。

GRAB_DESCENT_TARGET_Z_FIELD_CALIBRATION=NOT_RUN；proposal 的默认 z 不是有效生产下降目标。
physical_output_enabled/automatic_control_enabled/camera_control_authority/valid_for_control 全为 false。
GitHub 最多 PASS_CI_SOFTWARE_SHADOW_NOT_FIELD；本轮不封板、不执行 IPC/FIELD/ADAM DO/Phase2D。
