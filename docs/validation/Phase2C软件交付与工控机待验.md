# Phase2C 软件交付与工控机待验

日期：2026-10-14。分支：codex/phase2c-grasp-interaction-evidence-v1。
BASE_SHA=077d0bfe44d8fe34fc7572f52cff65362cbc83d8。
CODE_SHA=4af2bb0378c134800d34de6178b004b6fa4d8f6e。

## 当前结论

PHASE2C_SOFTWARE_STATUS=WINDOWS_STATIC_PASS_UBUNTU_ACCEPTANCE_PENDING。
这是正式代码与 Windows 静态/纯算法交付，不是完整 ROS 软件封板或现场验收。
用户最新明确要求：本轮无需 Ubuntu 安装/测试，交工控机 Ubuntu 22.04 执行。
因此最终推送采用 `[skip ci]`，不继续发起云端 Ubuntu 作业，也不更改仓库设置、保护规则或权限。
最终 HEAD 的 CI、colcon build/test、launch/DDS、bag 和全部现场验收均待工控机，不能以历史父 SHA 代验。

Windows：425 项 unittest，419 通过、6 项缺 ROS 跳过；pytest 同为419通过/6跳过。
git diff --check、repository contracts、compileall、46个架构场景通过；Node.js 前端语法检查通过。
主机 Python 3.11.9 虚拟环境仅做静态/原生测试；运行目标仍 Python 3.10 / GCC11 / Humble，不声称本机具有 ROS。
Commissioning backend/headless 14项通过，fake transport 仅标 SYNTHETIC，不连接真实 ADAM。
本地浏览器核查八页、点击模拟方向释放后 STOP、六个 DO 按钮 disabled；仅 headless UI，不是 ROS/硬件验收。

## 实现范围

新增11个证据/调试消息；LoadState、CycleStatus、UnloadSafeZone、UnloadingCartState 统一来源元数据，RunManifest追加V2字段。
不可变 binding/canonical digest、有界账本、接触联合证据、局部交互失效、空斗 baseline/MAD/迟滞、稳态载荷、提升/卸料证据。
局部 binding+swept+margin mask，冻结N保留、旧点阻断、清遮挡后真实新点与原B质量门禁刷新；不重写原B算法。
第8个独立 commissioning 包：localhost Web、隔离 SRC2 模拟遥控、严格台架复用、万用表人工记录、append-only记录、有界bag入口。
source/epoch/counter/身份变化与 refresh 世代倒退被拒绝；现有 TaskCycles 的拥有权与顺序不改。
真实 transport 只能显式 PHYSICAL+VALID，mock factory 只能 SYNTHETIC；未知/混标配置在连接前拒绝。
production 不反向依赖或默认启动 console；没有接通 safety/authorized/actuation 链。

纯算法集成使用实际 B grid、planner 和局部 lifecycle，不以手写 refresh-ready bool 代替新观测。

| deterministic scenario | Windows结果 |
| --- | --- |
| SUCCESSFUL_GRASP | stable payload确认、真实局部刷新、synthetic卸料确认 |
| FAILED_GRASP_AFTER_INTERACTION | payload未确认，9个局部cells仍失效并用新点刷新 |
| LOAD_ACCELERATION_FALSE_POSITIVE | 最终payload不确认，未刷新则不updated-clear |
| TARGET_SWAP_ATTACK | 同cycle换点拒绝，原digest不变 |

TARGET_SWAP_ACCEPTED_COUNT、ACCELERATION_PAYLOAD_CONFIRMED_COUNT、
INTERACTION_WITHOUT_LOCAL_INVALIDATION_COUNT、OLD_GENERATION_NEXT_BIND_ACCEPTED_COUNT 四计数均0。
另有几何仅POSSIBLE、无overlap载荷尖峰、IO冲突、未clear旧点阻断、load未配置/失效、
联合校准接触正证、代际/时序倒序、同源同序号改状态、缺任一bench gate、异常OFF、并发方向拒绝、释放/失焦/租约STOP反证。
所有 PASS 仅限 Windows native/headless；SYNTHETIC_VALIDATED 不升级 FIELD_VALIDATED。

## 已发现问题、根因与处理

1. 原 TaskCycles 的 VERIFY_LOAD 在 RAISE 之前，不能提前宣称最终载荷。
   采用 evidence admission adapter：前者仅闭合/交互，RAISE出口才final payload+lift；原FSM文件字节冻结，native反证通过。
2. 交互使下一次规划无候选时不应污染当前冻结目标；移动Grab不应在启用交互层后无条件清整坑。
   仅显式启用C层时忽略无效下一提议、移动期间暂停融合并按原TTL过期；默认B路径/核心统计不改，native局部失效测试通过。
3. 新 ROS 节点使用 rclpy 的只读 publishers 属性导致启动失败。
   改 evidence_publishers，并锁定保留属性AST反证；修复后的安装态确已进入稳定提升阶段，但未完成整链验收。
4. verification_evidence=False、SYNTHETIC真实TCP混标会削弱台架证据可信度。
   严格非占位字符串、source/transport双向匹配与真实VALID配置；fake-only反证通过，没进行真实输出。
5. 逻辑Cycle/car/zone倒序、同序号改值与旧refresh复活存在时序漏洞。
   增加successor/role、cart/zonewatermark及refresh generation floor；新增native反证通过，最终ROS待工控机。
6. 30ms sleep租约单测在Windows完整负载下偶发失败。
   改确定性monotonic deadline前后单测；真实ROS租约测试入口仍保留，未放松实际350ms租约。

## 云端尝试审计（历史，不是最终HEAD验收）

- 0b40efb： [Static 38527772255](https://github.com/guolichen007/chili-crane-automation/actions/runs/38527772255)通过；
  [ROS 38527772434](https://github.com/guolichen007/chili-crane-automation/actions/runs/38527772434)正常八包/366包测试与旧入口通过，
  C节点只读属性冲突失败，未执行symlink后续链；vendor通过。
- ea20f4a：[Static 38529514120](https://github.com/guolichen007/chili-crane-automation/actions/runs/38529514120)通过；
  [ROS 38529514069](https://github.com/guolichen007/chili-crane-automation/actions/runs/38529514069)依赖阶段 DNS 无法解析 packages.ros.org，代码验收未执行。
  连接器重试请求403、没有实际重试或权限更改。追加真实APT有界重试与不完整索引硬失败规则；vendor通过。
- 0f8db3d：[Static 38532376714](https://github.com/guolichen007/chili-crane-automation/actions/runs/38532376714)通过；
  [ROS 38532376741](https://github.com/guolichen007/chili-crane-automation/actions/runs/38532376741)正常八包/372包测试、旧A/B/R1.1/PTP/dual/安装捕获/RViz入口通过，
  C成功场景 stable lift 的 payload/updated-clear等待 timeout，尚未定位具体门槛；symlink最终链未完成，vendor通过。

最后一次 timeout 有 state=SHADOW_EVIDENCE_NOT_CONTROL、refresh=POST_INTERACTION_REAL_OBSERVATIONS，
但不能据此推断 payload、baseline、settling 或 clear 各门槛已通过。没有用改阈值/伪造ready强行通过。
CODE_SHA 增加 timeout-diagnostics.json，以及黑匣子的事件 evidence_summary/source counter/calibration/config字段。
工控机需查看 baseline_valid、sample gap/fault、closure、速度/加速度、load settling与clear分别判断根因。
本轮停止 Ubuntu 验收由用户要求，不是将上述失败改写为PASS。

## 工控机接手

先 fetch/checkout 最终交付SHA，确认工作区状态和运行基线，重建全部八包消息，不能混用旧消息类型。
按[运行手册](../deployment/Phase2C抓取交互与调试监控台手册.md)运行完整入口，保留 normal/symlink 的旧回归及新 C/console 验证。
现场参数不确认时继续NOT_CONFIGURED；不得把synthetic阈值复制为生产参数。
优先定位上述 stable lift timeout，不声称其由某一已猜测原因造成。
最终安装态程序、真实bag、实机设备及物理隔离/输出均由工控机和现场owner给出精确SHA证据。

## 不变边界与 OPEN

```text
SafetyPermit=false
AuthorizedCommand=0
ActuationRequest=0
physical_output_enabled=false
automatic_control_enabled=false
camera_control_authority=false
valid_for_control=false
BENCH_DO_STATUS=NOT_RUN
FIELD_CONTACT_CALIBRATION=NOT_RUN
FIELD_LOAD_CALIBRATION=NOT_RUN
FIELD_SURFACE_REFRESH_VALIDATION=NOT_RUN
FIELD_GRASP_VALIDATION=NOT_RUN
FULL_AUTO=NOT_RUN
```

静态publisher反证证明C层不生成三类控制消息，历史0f8隔离安装态失败前未发动作；最终SHA的运行计数待工控机，不能冒称最终ROS运行证明。
main、PTP、vendor patch、A/B核心、原TaskCycles冻结；Windows逐文件字节/历史回归通过。
OPEN：最终SHA八包normal/symlink与DDS链、稳定提升timeout、实机bag、现场load极性/门槛/单位、
坑/轴/外参标定、料面流变与遮挡组合、停车/限位/遥控硬互锁、FSV/WDT/OFF/万用表、存储容量/留存/掉电完整性。
CI容器仍浮动tag、image digest未采集；软件性能不等于现场实时SLA。
本轮不进入下一阶段。LOCAL_PROJECT_CONTEXT.md、runs、bag、原始ZIP、截图与下载凭证不提交。
