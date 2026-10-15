# Phase2C-R2 快照周期验证与待审

日期：2026-10-15。BASE_SHA=7f8378d9e670c4145836264e268ce3a500a438a7。
审查分支：codex/phase2c-r2-engineering-snapshot-cycle。
架构：SNAPSHOT_PER_CYCLE。见 [ADR0014](../decisions/ADR0014_Phase2C_R2工程化快照周期.md)
与 [接口合同](../api/PHASE2C_R2_CONTRACTS.md)。

## 变更边界

原 TaskCycles、Phase2A pipeline/OPEN/CLOSED/TRANSITION 模型、B grid/planner 硬门禁、
geofence、authority/executor、硬件驱动保持原实现；native R2 测试核验冻结文件的 Git blob。
SCAN 建窗、PLAN_PICK 停止融合，binding 立即冻结。下一 cycle 必须使用 scan_start 之后的新窗口和真实 B proposal。
旧窗口、旧 proposal、active 身份替换、同 session generation 回退均不能取得新 binding。
当前抓不构造或消费局部 SurfaceInteractionLifecycle/refresh/next-plan。
历史类型保留；legacy 测试显式标记，不能替代 R2 acceptance。

## Windows 本地证据

Python 3.11 纯算法/静态测试：unittest 477 项，451 通过、26 跳过；pytest 同为 451 通过、26 跳过。
跳过含 ROS 安装态、Linux 进程 ownership、明确历史 R1 生命周期测试，不计作通过。
架构 46 场景；Phase2A 25 场景与 R1 200 序列；Phase2B 21 场景/64 序列与 R1 workload 回归通过，安全违例计数均为 0。
R2 14 项 native 测试覆盖冻结、TTL 后不可变、新 scan 真观测、异步节点加入、旧料面/旧 proposal 拒绝、
producer restart、pit rim 缺失、provenance 冲突及原硬门禁。
repo contracts、公开秘密扫描、compileall、shell 语法和 diff 检查通过。
这些结果不代表 Windows 上运行过 ROS/colcon。

## Ubuntu CI 接受条件

authoritative script 在两个 env -i 子进程中分别从 /opt/ros/humble 构建 normal、symlink 八包。
每段记录 domain、prefix、ament/ros2 package resolution、Python module resolution；禁止另一段的 install/build 路径。
默认 ROS_LOCALHOST_ONLY=1、ROS_DOMAIN_ID=42；显式合法测试 domain 保留并记录。
LiDAR cloud/meta/context 仅在 200ms due 点一起生成和发布，同 stamp/epoch/counter；其余证据独立 50ms。
故意错配帧的独立反证必须仍由产品严格 pairing 拒绝。

每种安装方式先执行 10/10 critical path 探针，再执行完整 R2 安装态 suite。
保留 payload 9s、unload 10s、单次 runner 300s 上限。
断言 active surface mutation=0、active valid proposal=0、旧料面跨 cycle 复用=0、control topic messages=0。
所有 validator 创建的 ROS 会话按 owned PGID 执行 SIGINT、有界等待、SIGTERM、有界等待；
Linux 子进程反证涵盖 wrapper 提前退出和子进程自建 session，不能只检查 wrapper returncode。
最终每个场景必须确认 VALIDATOR_OWNED_PROCESS_LEAK_COUNT=0。

最终交付以同一最终 HEAD 的 GitHub static、ROS2 normal/symlink 和 vendor wrapper jobs、
下载 artifact 的 SHA256、result、xUnit、安装态 JSON 和环境来源共同核验。
本页描述接受条件，不预先将尚未结束的 CI 标记 PASS；最终精确 SHA、run 链接和结果随交付回传。
CI 最大状态为 PASS_CI_SOFTWARE_SHADOW_NOT_FIELD，不封板，不合并 main，不进入 Phase2D。

## OPEN 与后续边界

GRAB_DESCENT_TARGET_Z_FIELD_CALIBRATION=NOT_RUN：proposal_to_message 当前仅写 XY。
真实抓斗 CAD/尺寸、suspension/bottom reference、入料深度、坑底余量、Z 停止误差确认后，
再确定 target_grab_bottom_z_m 或 target_suspension_z_m 正式合同。
真实接触/载荷阈值与生产设备参数仍按已有 NOT_CONFIGURED/FIELD gate 管理。
physical_output_enabled=false、automatic_control_enabled=false、camera_control_authority=false、valid_for_control=false。
BENCH_DO_STATUS=NOT_RUN；FIELD_STATUS=NOT_RUN；FULL_AUTO=NOT_RUN；IPC 在人工代码审查后再执行。
