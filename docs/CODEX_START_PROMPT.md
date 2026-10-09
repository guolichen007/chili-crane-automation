# Desktop Codex 当前启动交接

先完整阅读 AGENTS.md、PROJECT_CONTEXT.md、SYSTEM_ARCHITECTURE_AND_ROADMAP.md、
ADR 0003、NDT_REUSE_PLAN.md、api 契约及硬件台架手册，再修改代码。
当前Phase0.5-R1基于SHA fce3fa0aa49d4278b72a65b350741cd73f85cf4b，
分支codex/architecture-freeze-r1；不得改main或NDT源仓库。
先读ADR0004/0005、api/PHASE05_CONTRACTS、验收矩阵和外部审查入口。
严格按CONTRIBUTING规范提交/推送，消息破坏性变更需clean rebuild。

## 不可遗忘的当前需求

- Ubuntu 22.04.x / ROS2 Humble / Python 3.10 / GCC 11 / C++17。
- 6052 8DI8DO + 6251 16DI = **初始 24DI，包括遥控器输入**。
- 完整24点分配、极性、遥控电气接口、设备 IP/unit、拉绳寄存器/格式/标定 OPEN。
- DO0..5 预留六动作；6/7 保留，正常运行 physical_output_enabled=false。
- 硬接线急停、方向互锁、限位、制动、遥控/自动物理互斥归原厂电气。
- 遥控输入用于观测/仲裁，不直接透传自动 DO；回自动必须验证静止/OFF并接新任务。
- 双雷达和 NDT 仍只是算法接口；不得报告算法、bag 或实机成功。
- Windows 只写代码和静态测试；Ubuntu/CI 才可做 ROS2 build/test/mock。
- 每次提交运行 AGENTS 的三个检查，Linux LF / UTF-8无BOM / C++17。
- SSH git 推送；不需要 gh。忽略本机 LOCAL_PROJECT_CONTEXT.md 并持续维护。
- 命令accept一次后tick复核；输出帧短租约，真实deadline/WDT/FSV仍需现场确认。
- DI v4只填写physical_channels，禁止另填digital_inputs；manifest v2带sensor identity。

下一步先取得厂家签字的24点表、有效电平、隔离继电器边界与 WDT/FSV OFF 证据，
然后 Ubuntu 精确 SHA 原生验证，随后台架只读；有真实双雷达/伺服 rosbag2 后才迁算法。
未知参数继续 NOT_CONFIGURED；不通过“缺报警”推导 SAFE。
