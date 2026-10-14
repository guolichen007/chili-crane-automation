# Phase2C-R1.2 窄修与 exact-SHA 验收

2026-10-14；BASE_SHA=`7389e88a4f45268b302320e0f805ca5da953c166`。
分支 `codex/phase2c-r1-2-final-closeout`。本轮不修改 R1.1 分支、main 或 Phase2D。
接口与边界见 [R1.2 contracts](../api/PHASE2C_R12_CONTRACTS.md)。

## 基线 CI 更正

R1.1 最终 SHA 的 [Static CI](https://github.com/guolichen007/chili-crane-automation/actions/runs/38547579632)
SUCCESS；[ROS2 CI](https://github.com/guolichen007/chili-crane-automation/actions/runs/38547579625) FAILURE。
实际 job 115797337027 日志确认依赖安装成功、normal 八包 build 成功；
phase2c_delivery 在默认 60s CTest 预算被杀，没有生成 pytest junit。
此前同 runner 的六场景 native validation 已 PASS。vendor job 115797337371 SUCCESS。
installed C、symlink 未到达，IPC/FIELD 从未验收。不能将该代码/测试失败写成网络故障或笼统 pending。

## 实现与本机证据

当前 cycle 的 RAISE admission 使用 PAYLOAD_CONFIRMED + PIT_EXIT_CONFIRMED；
VERIFY_EMPTY 使用 UNLOAD_CONFIRMED。下一 cycle 的扫描、B 候选和 binding 门禁独立。
PIT_EXIT 在 RAISE 内由 binding/interaction/raise chronology、有效 tracking/provenance、bound clear、
源语义地图的严格 rim margin 和 safe height 形成历史事件。refresh/context 缺失或过期不取消它；
新的有效来源/时序/静态几何冲突、非法 phase 回退、active run/session 切换、abort 会清理并 fail closed。
旧 terminal cycle 的 refresh 允许在下一绑定入口核验，保留 authoritative generation floor 和真实 B proposal。

diagnostics 同时报告单纯 payload_delta_gate_pass 和完整 payload_gate_pass，
以及 pit_rim_clear/safe_lift_height/pit_exit_confirmed/refresh/updated_clear/next_plan。
storage interval 是显式有限 [1,5]s，FIELD 保持 null；200 次 fast backend tick 仅 start + 1 次 full scan。
边界计数包括 symlink，OSError/100000 entries/quota/free space 门禁继续停止 owned bag，不删除证据。
bringup 声明可选 commissioning 的 exec dependency，production launch 仍不启动它。
phase2c_delivery 保留六场景，CTest TIMEOUT 180；仅改 runner 预算，旧内部等待与成功条件不变。

Windows / Python 3.11.9 原生与静态测试：451 项，445 PASS / 6 ROS unavailable SKIP；
pytest 同数量。新 R1.2 文件含 13 项反证，注册到 colcon CTest（TIMEOUT 180）。
compileall、Python 3.10 syntax/repository contracts、secret scan、46 architecture 场景执行。
Phase2A 25 场景 + 200 R1 sequences、Phase2B 21 场景 + 64 sequences / R1 门禁、
Phase2C 原六场景与 localhost commissioning headless 保留，违规计数为 0，物理/自动 flags=false。
本机记录是提交工作树的软件证据；最终 exact SHA 的 CI 与 IPC 证据独立核验。

核心联合反证由原 TaskCycles 驱动每个 phase：

```text
affected=9, refreshed=2, UNKNOWN=7
payload=CONFIRMED, updated_clear=false, next_plan=false
pit/rim clear + safe height -> PIT_EXIT_CONFIRMED
RAISE -> MOVE_TO_UNLOAD
pit refresh/context missing or stale
fresh unload geometry/IO/load/cart/zone -> UNLOAD_CONFIRMED
VERIFY_EMPTY -> COMPLETE without next plan
next_cycle -> SCAN
no fresh post-interaction B proposal -> binding REJECT
real local recovery -> new generation / actual B hard-gate candidate -> new binding
```

安装态工具保留原场景和原 payload/refresh 9s、unload 10s 条件，额外运行上述 owner-driven DDS 链。
不依据没有 warning 推断 SAFE，不执行真实 ADAM DO，不修改 A/B 算法或阈值。

## 最终 GitHub 与工控机分层

本提交必须正常 push，然后等待相同 exact SHA 的 Static 与 ROS2 workflows 完成，
读取 jobs 和 artifacts，最终回传分开列 normal build、colcon、installed C、symlink、vendor。
新提交的最终结果以同 SHA 的 GitHub 记录和回传为准，不用本机 PASS 替代 ROS。
任何代码/测试失败须定位具体阶段并记录；只有外部网络/基础设施失败记 INFRA_FAILURE。
历史 0f8 installed stable-lift 根因仍不可从旧缺采样日志唯一确定；native updated-clear 阻塞已有精确复现。
新 exact-SHA installed 条件未实际通过前，不允许 HISTORIC_TIMEOUT_FIXED=YES。

CI 全绿最多 PASS_CI_SOFTWARE_SHADOW_NOT_FIELD。之后工控机在同一个 final SHA 执行：

```bash
bash scripts/validation/ubuntu22_ros2_phase0_validate.sh \
  --workspace "$PWD" --expected-sha "$(git rev-parse HEAD)" \
  --evidence-dir /path/outside/repository
```

保留全部历史 Phase1/A/B/R1/R1.1、normal/symlink 八包、installed C/commissioning、DDS/vendor/RViz/mock。
IPC_NORMAL、IPC_SYMLINK、IPC_INSTALLED_PHASE2C、IPC_DDS 由工控机对该 exact SHA 实际报告。
IPC_UBUNTU_EXACT_SHA、BENCH_DO、FIELD_CONTACT/LOAD/SURFACE_REFRESH/GRASP、FULL_AUTO 本机均 NOT_RUN。
完成 CI 结果回传后停止，等待评审，不进入 Phase2D。
