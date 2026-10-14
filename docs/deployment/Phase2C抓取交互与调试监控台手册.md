# Phase2C 抓取交互证据与调试监控台使用手册

交付日期：2026-10-14。验收范围以[Windows交付与工控机待验记录](../validation/Phase2C软件交付与工控机待验.md)为准。

范围：Windows 静态/纯算法交付，Ubuntu 安装态验收待工控机；不是生产自动控制操作规程。运行环境 Ubuntu 22.04 / ROS2 Humble /
Python 3.10 / GCC 11 / C++17。Windows 仅编辑、静态/纯算法测试及 Git。

## 架构与接入

辣椒酱/辣椒料面会塌陷、重排、流动。当前抓取引用冻结的 N，下一抓必须依据新的合格观测。
不把料面作为定位主地图，不把发出闭斗指令当作闭斗，不把加速时载荷尖峰当作抓到料。

| 模块 | 负责 | 明确不负责 |
| --- | --- | --- |
| control/interaction_contracts | 不可变 TargetBinding、canonical digest、有界绑定账本 | 替换当前 cycle 目标 |
| control/load_features | 原始单位 median/MAD、空斗 baseline、delta/z-score、斜率、持续/迟滞、TTL/故障 | 未标定 kg 推算 |
| control/grasp_interaction | 接触、交互、闭斗、载荷、提升与卸料证据 | 第二套 TaskCycles、物理权限 |
| perception/surface_interaction | 局部受扰 cells、冻结 N、clear 后真实点刷新 | 改写原 SurfaceDisturbance 的来源语义 |
| commissioning | localhost Web、隔离模拟遥控、记录、显式台架与有界 bag 工具 | production 的反向依赖/默认启动 |
| hardware | 既有 ADAM 驱动、独占会话、OFF/WDT/latch 与脉冲门禁 | 接受网页提供的 gate 或任意寄存器写入 |

只有 TaskCycles 的既有 owner 可以推进状态。`evidence_allows_advance()` 是 shadow admission：
VERIFY_LOAD 仅检查已经交互、闭合且无矛盾；RAISE 出口才要求稳定最终 payload、更新料面 clear 和安全提升。
其他定位/限位/姿态/任务条件仍由原 owner 决定。没有引入新的周期状态机，也未接通真实 actuation。

新增消息：TargetBinding、ContactEvidence、SurfaceInteractionEvidence、ClosureEvidence、PayloadEvidence、
LiftEvidence、UnloadEvidence、GraspInteractionState、SurfaceRefreshEvidence、CommissioningShadowOutput、BenchOutputEvidence。
LoadState、CycleStatus、UnloadSafeZone、UnloadingCartState 追加统一 EvidenceMetadata；RunManifest 保留原字段并扩展 V2 记录字段。
这是 wire 接口扩展，所有 producer/consumer 必须按同一 SHA 重建；不支持混用旧生成消息。
旧消息缺来源、测量 epoch、counter、标定/配置身份的，保持 UNKNOWN/NOT_CONFIGURED，不猜来源或续期。
同周期 CycleStatus 拒绝 epoch/counter 倒序、同源同序号变更及来源身份变更；cart/zone 采用同样的观测顺序门禁。
权威 refresh generation 单调下限阻止旧合格结果在更新后的 UNKNOWN 结果之后复活。不是另一套状态机。

## 目标绑定与局部刷新

绑定 owner 将合格的 `planning/grasp_proposal_shadow` 转成 TargetBinding，发往 `phase2c/binding_request`。
`binding_from_proposal()` 检查候选唯一、hard gates、无 rejection、非控制用途；绑定事件时间与原测量时间分开。
同一 cycle 的重复绑定/换点拒绝。新 proposal 是下一次规划材料，不更新当前 XYZ、footprint、snapshot 或 lineage。
当前 live spatial context 缺失/失效/变更时只输出 UNKNOWN/STALE/CONFLICT/ABORT_REQUIRED；不得自动重绑。

摘要为 `PHASE2C_BINDING_CANONICAL_V1`：UTF-8、有序无空白 JSON、float64 hex、signed zero 归一、无 NaN/Inf；
摘要排除自身。Point32 footprint 先按实际 wire 精度归一，避免 ROS 往返改变 digest。不是密码学来源认证。

```text
SCAN -> proposal -> immutable binding(N)
                      |
                      v
                LOWER / interaction
                      |
                      v
           local invalidation (empty grasp also counts)
                      |
                      v
                CLOSE -> RAISE
                      |
                      v
           CLEAR_OF_BOUND_SURFACE (observation only)
                      |
                      v
        real post-interaction points + existing B mask/quality
                      |
                      v
         refreshed generation > N -> CLEAR_OF_UPDATED_SURFACE
                      |
                      v
          stable payload/lift -> next cycle uses new snapshot
```

mask 为绑定 footprint、实测 swept footprint、显式 collapse margin 的局部并集，只处理坑内交叠格。
可靠交互第一次立即幂等 invalidation；重复通知不重复递增 generation，新 swept cells 才扩展失效范围。
冻结 N 对象仍可供当前 cycle 查证。受扰 cell 未 clear 或点时间不晚于交互/clear 时不得恢复。
未受扰 cells 不因启用交互层的正常抓斗运动被整坑清除，仍按原观测 TTL 过期；移动期间不融合新料面。
丢失、无效 provenance、空间变更等原拒绝仍 fail-closed。只有显式 interaction_config_file 启用此行为。
默认 Phase2B launch、统计、mask、扰动、规划/ESDF、时基、session、TaskCycles 与 vendor patch 保持冻结。

新局部 surface 必须全部受扰 cells 满足原 B sample/frame/support/coverage、坡度/粗糙度、uncertainty、TTL、稳定平台与 lineage。
没有新点或被 Grab mask 遮挡时不插值、不填高程，不得将仅 bound-clear 宣布为 updated-clear。
pit-clear 优先使用 SemanticMap V1 的 pit.wall_top_z_m 并核对 map/version/frame/pit 与源标定状态。
safe lift height 是独立检查，不等于 surface clear，也不等于真实 MOVE_TO_UNLOAD 授权。

## 默认安全状态与 source 配置

`config/algorithms/phase2c_interaction.yaml` 为 FIELD 待确认模板；数值 null 不可用于生产。
独立 `.synthetic.yaml` 为 NON_PRODUCTION 软件测试值，不能重命名成 FIELD 配置或拿它解释现场载荷。
每类观测验证来源、统一 inner/outer envelope、measurement stamp、counter、共时、TTL、fault 与标定身份。
心跳、到达时间或 timer 不更新 measurement epoch；预测 Grab 不作为实测上升证据。

Contact 必须同时满足下降、保守区间 overlap、标定载荷变化与持续时序；仅几何最多 POSSIBLE。
Closure 要闭限位与实测 LiDAR CLOSED 一致；双限位、闭限位+OPEN 等冲突不做多数投票。
载荷只在空斗已知、停止稳定、非接触且未交互时更新 empty baseline；运动/抓取期间冻结 baseline。
最终 payload 要已交互/闭合/真实上升、updated clear、低速度/低加速度、load settling 与有效标定门槛。
卸料还要求匹配的安全区与车状态、not full/overflow、真实 OPEN 一致和 loaded baseline 相对下降。
所有这些均只是 shadow 证据，不是许可。

```text
physical_output_enabled=false
automatic_control_enabled=false
camera_control_authority=false
valid_for_control=false
SafetyPermit=false
AuthorizedCommand=0
ActuationRequest=0
```

## Ubuntu 安装态验证

先安装依赖、按精确 SHA 构建并 source。同一 SHA 的八包 normal 与 symlink 都需通过。
完整入口保留全部旧 Phase1/A/B/R1/R1.1、dual/PTP、RViz、mock/vendor 检查并追加 C：

```bash
git diff --check
python3 tools/check_repo_contracts.py
python3 -m compileall -q src tools tests_static
python3 -m unittest discover -s tests_static -p 'test_*.py'
python3 -m pytest -q tests_static
python3 tools/validate_phase2c_synthetic.py --output /tmp/phase2c-native.json
# 仅在 source 本 SHA 的 Humble install 后执行：
python3 tools/validate_phase2c_ros.py --evidence-dir /tmp/phase2c-installed-unique
```

最后一个工具只在自己的 UUID namespace 建立四个 owned process：surface、planner、interaction、console。
本轮按用户新指令不执行本段 Ubuntu 命令。最终 SHA 的构建、colcon test、DDS/launch/bag 都须工控机实际确认。
历史 0f8db3d 的 installed case 在稳定提升等待时未满足 payload/updated-clear 条件而 timeout，
不能写成已修复或通过。最终工具新增 timeout-diagnostics.json 与事件 evidence_summary，供工控机定位
baseline、sample gap、closure、velocity/acceleration、settling、refresh 与 clear 各门槛；不得先猜阈值改成 PASS。
使用未修改双雷达 merger 的 FIRST_POINT header、实际 point-derived MID_SCAN Grab 和完整标准化 synthetic 状态；
它没有 ADAM transport、writer、safety/authorized/executor 节点，不应拿 synthetic motion 输入当现场传感器结果。
覆盖成功抓取、空抓仍失效刷新、加速误报、同 cycle 换点，真实 DDS 局部 ACK/新世代/载荷/卸料链，
以及 HTTP -> SOURCE_SIMULATED -> 既有仲裁策略 -> shadow DO 期望、释放/租约 STOP、未 armed bench 拒绝。
三控制 topic 订阅计数为零，所有 valid_for_control 为 false；脚本退出只清理自己的进程，不删除证据。

## 启动与记录

默认 Phase2C launch 只启证据节点，配置未确认时 NOT_CONFIGURED，不启动控制或 console：

```bash
ros2 launch chili_crane_bringup phase2c_interaction_shadow.launch.py namespace:=crane_01
```

该节点依赖标准化 Grab/geometry、live PlanningSpatialContext、Hoist/IO/Load、原 owner CycleStatus 和明确绑定；
启用局部生命周期需另对 surface node 显式传 `interaction_config_file`，不是默认加载 production。
cart/zone/refresh 缺失分别阻断相关 evidence，不把缺消息解释成安全。

本地 ROS console 示例（namespace/run_id 必须与该次受监控 owner 运行相符；记录根目录由 operator 确认）：

```bash
ros2 run chili_crane_commissioning commissioning_console.py \
  --config config/commissioning/console.yaml \
  --git-sha "$(git rev-parse HEAD)" --run-id CONFIRMED_RUN_ID \
  --crane-namespace crane_01 --runs-directory /path/to/owned-runs \
  --host 127.0.0.1 --port 8080
```

只在 Windows 想检查空界面时用源码包路径启动 `--headless-preview`（此模式禁止 physical bench/bag）：

```text
PYTHONPATH 指向 commissioning/src、control/src、hardware/src
python src/chili_crane_commissioning/scripts/commissioning_console.py --config config/commissioning/console.yaml
  --git-sha EXACT_40_HEX_SHA --runs-directory .local-data/console-preview-runs
  --host 127.0.0.1 --port 8080 --headless-preview
```

这只是同一命令的参数说明，Windows 应按 PowerShell 语法设 task-specific PYTHONPATH；不得称为 ROS 启动成功。
访问输出的 LOCAL_URL。只有显式 `--allow-nonlocal` 与 allowed-host 才可非 loopback，armed bench 必须 loopback。
Web API 有 Host/Origin、session-token、body/path 边界，无网页 gate 开启入口，不提供任意 command/register/address。
页面包括总览、设备、cycle、A/B/C证据、模拟遥控、台架、事件和记录。UNKNOWN 不等于 SAFE。

模拟按钮只有 Y左/右、Z上/下、G开/关与 STOP；X 不猜造。SOURCE_SIMULATED 发布到独立 commissioning/simulation topics。
pointer release/cancel/lost capture、失焦、隐藏、离开与后端短租约都释放；不能转发成真实 DO。
载荷图使用原始单位/diagnostic delta，缺数据留空；不显示未标定 kg。

`runs/<run_id>/` 包含 manifest.json、append-only events/operator_actions/bench_measurements JSONL、rosbag/、reports/。
git_sha 和 composite config_hash 绑定 console/bench/recording/CLI gates；记录 wall、ROS、monotonic 和 run/task/cycle/binding/source。
headless 的 ROS timestamp 为 null，不假装接上 ROS 时钟。导出受大小/路径限制，不可覆盖既有 run。
local runs、bag、截图、下载 ZIP、下载凭证与 LOCAL_PROJECT_CONTEXT.md 不提交。公开报告仅放脱敏精确 SHA 结论。

## 台架与 bag 门禁

ISOLATED BENCH ONLY / 隔离台架专用。默认 bench.yaml physical=false、host/unit 未配置，六按钮禁用。
只有物理输出显式 CLI、隔离确认、独占会话确认、FSV/WDT/OFF 实证齐全、automatic=false 和明确配置同时满足才可脉冲。
不在本文提供虚构 gate 证据、设备地址或现场接线。config 与相关启动 flags 由现场 owner 经真实验收后确认。
复用 BenchOutputGate/Adam6052Driver/pulse_once/request_to_do；1ms < duration <=1000ms，默认 UI 500ms。
真实 transport 必须显式 PHYSICAL+VALID 配置；只有 explicit SYNTHETIC+mock factory 才能记软件 mock，通过双向检查禁止混标。
写前 OFF、active readback、finally OFF/readback，方向串行；DO0..5 六语义固定、DO6/7 reserved，无 X 输出。
异常 OFF 是 best-effort：断网/掉电/进程强杀无法靠软件保证 OFF，设备已验证 WDT/FSV 和独占 crash latch 不可省略。
不能以“软件 mock OFF 通过”替代真实输出、测电压、电机方向、硬限位链或自动端到端验收。

万用表值只允许关联已完成的 owned bench_test_id；预/active/post readback 与人工供电/DO 电压分开保存。
人工数值标为 OPERATOR_REPORTED_NOT_FIELD_ACCEPTANCE；不自动把 register bool 转成电压。

bag 默认 disabled/NOT_CONFIGURED；state_only 不含 raw points，event_debug 默认也不持续录 raw。
必须确认 storage、每文件 size/duration 和总 storage/duration 四项正整数上限及 namespace/topics，再通过显式 recording config 启动。
原始点云还需 event_debug 与独立 high_bandwidth approval；停止只针对 owned recorder，不清除证据或其他进程。
JSONL 是 append-only 事件记录，不是已验证的长期工业存储/掉电完整性系统；现场容量/留存仍 OPEN。

## 本阶段结束条件

本轮状态 WINDOWS_STATIC_PASS_UBUNTU_ACCEPTANCE_PENDING；纯算法与 headless PASS 不代替 ROS 安装态验收。
未来只有真实完成软件验收后才能至多 PASS_SOFTWARE_SHADOW_NOT_FIELD / COMMISSIONING_SIMULATION_STATUS=PASS_SOFTWARE。
BENCH_DO、FIELD_CONTACT/LOAD/SURFACE_REFRESH/GRASP、FIELD motor direction/limit chain/control E2E、FULL_AUTO 一律 NOT_RUN。
实机 bag 重放、真实 load polarity/absolute threshold、坑/轴/外参标定、可停能力、料面流变组合、存储容量须另行提供真实证据。
本轮交付后停止，不进入下一阶段或启用自动运行。
