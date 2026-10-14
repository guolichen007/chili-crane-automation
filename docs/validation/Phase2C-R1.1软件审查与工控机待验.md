# Phase2C-R1.1 软件审查与工控机待验

## 事后 CI 更正（2026-10-14，R1.2 审查）

最终 SHA `7389e88a4f45268b302320e0f805ca5da953c166` push 后实际自动运行了 CI。
[Static 38547579632](https://github.com/guolichen007/chili-crane-automation/actions/runs/38547579632) SUCCESS。
[ROS2 38547579625](https://github.com/guolichen007/chili-crane-automation/actions/runs/38547579625) FAILURE：
依赖安装 PASS、normal 八包 build PASS、colcon test FAIL_PHASE2C_DELIVERY_CTEST_60S_TIMEOUT；
installed Phase2C 和 symlink NOT_REACHED；vendor wrapper PASS。不是网络失败。
artifact 11885623187，SHA256 `4351acd5950ef19616511276d1a6f46bfc9efab7eff51272c11c13a71f0a2841`。
同 runner 的此前 native 六场景已经通过，失败是测试 runner 总预算，非 installed stable-lift 根因证明。
上轮回传遗漏了这些自动 CI 事实；下方提交时 Windows/IPC 待验记录保留历史，不代表该 SHA CI 未运行。
R1.1 当前 SOFTWARE_SEAL=NOT_APPROVED；先做 R1.2 并验最终 exact SHA CI，再交工控机。

日期：2026-10-14。BASE_SHA=`1c14ac89d6429a31fa1cfcd3253b9eb7feabe56a`。
分支 `codex/phase2c-r1-1-closeout`，在 Windows 原 checkout 建立；原 C 分支未修改。
最终 OUTPUT_SHA 以交付回传及 `git rev-parse HEAD` 为准，避免报告自引用 SHA。

```text
PHASE2C_REVIEW_STATUS=HOLD_FOR_R1_1
R1_1_CODE_STATUS=WINDOWS_STATIC_PASS_UBUNTU_PENDING
UBUNTU_EXACT_SHA_ACCEPTANCE=NOT_RUN
BENCH_DO_STATUS=NOT_RUN
FIELD_CONTACT_CALIBRATION=NOT_RUN
FIELD_LOAD_CALIBRATION=NOT_RUN
FIELD_SURFACE_REFRESH_VALIDATION=NOT_RUN
FIELD_GRASP_VALIDATION=NOT_RUN
FULL_AUTO=NOT_RUN
```

## 修复与反证

接口迁移见 [R1.1 contracts](../api/PHASE2C_R11_CONTRACTS.md)，操作见
[运行手册](../deployment/Phase2C抓取交互与调试监控台手册.md)。全部八包消费者须重建。

| 项目 | Windows 原生/静态证据 | 原生边界 |
| --- | --- | --- |
| payload / refresh 解耦 | 相同基线轨迹：旧 CANDIDATE，新 CONFIRMED；refresh pending、updated clear UNKNOWN | 非 ROS timeout 修复证明 |
| 部分料面 / 局部门禁 | 9 affected，2 refreshed、7 UNKNOWN；无候选等待；另一侧新观测局部恢复后原 B 候选合格 | 未改 B 门槛/评分 |
| 旧世代 / 旧点 | 新 generation，受扰旧点复用与旧世代绑定接受计数 0 | 非真实 bag 验收 |
| surface restart | active 拒绝并锁定 abort；COMPLETE/ABORTED 后实际 B 重扫、新 proposal/new binding 成功 | 不比较跨 session generation |
| ledger | 配置真实上限 1024 次绑定，第 1025 次同 run 拒绝；terminal 后新 run 成功，RAM 只留新账本 | 无 eviction、未做现场长期运行 |
| 记录治理 | 全根目录预算/最低空闲空间阻止新 bag，并停止 owned fake bag；历史文件保留 | 不宣称真实 rosbag 运行 |
| manifest | JSONL-only、已启动、进程退出分别验证 rosbag 状态 | FIELD 容量继续 null |
| shadow stack | 原 B surface + C config、原 B planner、C evidence；commissioning=false；production 无 console | ROS launch 待验 |

运行环境 Windows / Python 3.11.9；独立本地依赖使用仓库锁定的 NumPy 1.26.4、Shapely 2.0.7、
small-gicp 1.0.1。初始默认 Python 缺依赖的失败已记录并补齐后重跑，未削减测试或修改冻结算法。
最终完整 unittest：438 项，432 PASS / 6 SKIP；pytest 同为 432 PASS / 6 SKIP。
13 项新增 R1.1 反证全部通过；历史测试数量未减少。6 项 generated ROS/rclpy 依赖检查在 Windows 跳过。
compileall、Python 3.10 静态合同、secret scan、46 architecture 场景均 PASS。
Phase2A：25 tracked 场景、200 R1 sequences，所有违规计数 0。
Phase2B：21 scenes、64 sequences、R1 性能/容量门禁，所有违规计数 0。
Phase2C：保留四场景，追加 PARTIAL_POST_GRAB_VISIBILITY、POST_GRAB_OCCLUSION_THEN_RECOVERY。
commissioning：保留 HTTP/headless、mock pulse/OFF 与生产隔离检查；无 ADAM 物理执行。
实际 localhost headless CLI：PASS；journal_enabled=true、rosbag_enabled=false、
rosbag_process_alive=false、bench_enabled=false、physical/automatic=false。
本机证据保存在被忽略目录，不提交 logs、ZIP、bag、截图或个人路径。

## 历史 stable-lift timeout：OPEN

只读检查 [历史作业 38532376741](https://github.com/guolichen007/chili-crane-automation/actions/runs/38532376741)
及 artifact 11879415905，对应 `0f8db3d95a78f39782185cda5fe66fd61ae6c58e`。
ZIP SHA256=`31ee88ff257f3d6cb850132dc4f63ef3e158acd744b16dfc8132cfd3177103ec`。
日志证实：08:16:16 UTC 联合 payload/updated-clear 等待 timeout。
events.jsonl 有 CLOSURE_CONFIRMED、CLEAR_OF_BOUND_SURFACE 和 SURFACE_REFRESH_READY，
但没有失败时的 load baseline/freshness/settling、velocity/acceleration 数值或完整状态采样。
不能从这些一次性事件证明超时瞬间所有门均通过，也无法据此唯一确定失败门。

`tools/reproduce_phase2c_r11_baseline.py` 从精确 BASE_SHA 只读加载旧 engine，
以同一轨迹证明 native 阻塞门为 updated_clear：closure、interaction、bound_clear、rise、
vertical_settled、baseline/fresh/calibrated load、settling 与 delta 门全部通过，
delta=1、normalized/robust z-score=20，旧 payload=CANDIDATE，新 payload=CONFIRMED。
这有明确原生根因，但不是历史 ROS/DDS 时序复现。

```text
BASELINE_NATIVE_GATE_REPRODUCED=YES
BASELINE_NATIVE_BLOCKING_GATE=updated_clear
HISTORIC_TIMEOUT_REPRODUCED=NO
HISTORIC_TIMEOUT_FIXED=NO
HISTORIC_STABLE_LIFT_TIMEOUT_ROOT_CAUSE=UNRESOLVED_MISSING_GATE_SAMPLES_UBUNTU_PENDING
```

R5 的历史安装态根因与修复确认仍为必需待验项；不得把本报告当成该项已收口。
没有改生产门槛、增大既有 payload/updated-clear 等待阈值，或伪造 ready。
新 gate_diagnostics_json 含逐门数值与完整 binding/session/generation/count/source 身份；
早期失败明确 NOT_EVALUATED/null。工具保存 timeout-diagnostics.json，旧有效门值附独立时间戳，
不能替代当前失败状态。先等待 payload（刻意暂停 post cloud），再逐条等待 refresh、updated clear、候选。

## 工控机：同一个精确 SHA

检出最终回传 SHA，验证 clean tree 与 Ubuntu 22.04 / Humble / Python 3.10 / GCC 11。
运行现有完整入口，保留历史 Phase1/A/B/R1/R1.1、vendor、RViz、DDS 与 mock 项：

```bash
git diff --check
python3 tools/check_repo_contracts.py
python3 tools/check_public_secrets.py
python3 tools/reproduce_phase2c_r11_baseline.py --output /tmp/r11-baseline-native.json
bash scripts/validation/ubuntu22_ros2_phase0_validate.sh \
  --workspace "$PWD" --expected-sha "$(git rev-parse HEAD)" \
  --evidence-dir /path/outside/repository
```

必须完成 normal 与 symlink-install 的全部八包 build/test，installed Phase2C 和 commissioning，
完整 shadow-stack launch；日志与安装路径须对应同 SHA，不混用旧消息生成目录。
安装工具分别报告 PAYLOAD_CHAIN、SURFACE_REFRESH_CHAIN、NEXT_PLAN_CHAIN；实际统计旧点复用，
candidate 必须来自新 session/generation，三控制 topics 消息计数 0。
历史 timeout 必须消失，或通过本 SHA gate-level diagnostics 确定成具体真实缺陷。
任何超时不得以延长等待或省略其中一条链转 PASS。CI 容器结果与工控机原生/现场证据分开。
外部网络错误记 INFRA_FAILURE，不改写为代码 PASS/FAIL。

## OPEN 与停止边界

OPEN：最终 exact SHA Ubuntu normal/symlink、DDS/launch、installed C/console；历史 timeout 根因；
真实 bag/长期磁盘容量、物理 sensor/load/contact/料面/抓取标定与可停能力。
SafetyPermit=false、AuthorizedCommand=0、ActuationRequest=0 与四控制 flags=false 持续保持。
本轮正常提交/push；不合并 main、不改原 C 分支、不进入 Phase2D，推送后停止等待评审。
