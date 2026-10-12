# Phase1B PTP TimeScale R2 跟踪

BASE_SHA: 48674999d07c5df24e8b3e36ded55b8b779794a2

分支 codex/phase1b-ptp-timescale-r2，main 不修改。

| 项目 | 状态 | 验证 |
| --- | --- | --- |
| S13/S14 真实事实与非生产 UTC 边界 | IMPLEMENTED | 用户回传事实文档、LOCAL 配置 |
| TimeScale / Relation / 不要求 host=PTP domain | IMPLEMENTED | 纯合同负例 |
| header + 每点正规化、raw provenance | IMPLEMENTED | 整数纳秒及数组不变性 |
| future/stale 后置、关闭 0.12 秒假设 | IMPLEMENTED | 帧起/中/末/接收分开 |
| 持续 DIFOP/MSOP、TTL、重复证据与降级 | IMPLEMENTED | 纯 parser/tracker 与 ROS transport |
| clock / scan phase 独立 | IMPLEMENTED | FrameSync0/1，status0/1/2 |
| LOCAL 台架与 VALID 生产门禁 | IMPLEMENTED | 分别验证，禁止字符串越级 |
| Windows 静态 | IN_PROGRESS | 全套 tests_static、repo checks |
| Ubuntu22/Humble clean build/test | IN_PROGRESS | exact-SHA CI |
| 远端精确 SHA 全绿 | IN_PROGRESS | Static、ROS、vendor jobs |

本轮 Codex 未现场采集，S13/S14 回传不被软件 CI 重新升级。
driver disabled，physical_output_enabled=false，automatic_control_enabled=false，
camera_control_authority=false。vendor patch/lock 不变，不提交 bag。
