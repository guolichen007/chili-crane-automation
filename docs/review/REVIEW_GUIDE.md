# 外部技术审查入口

## 项目定位

本仓库处于Phase0.5架构冻结：七包ROS2、类型化接口、纯策略/mock和验证脚手架。
不是实际算法、实时安全控制器或可直接上线的无人行车产品。
架构冻结是模块和契约基线，不是禁止以后经ADR演进。

## 阅读顺序

1. README -> PROJECT_CONTEXT（已确认需求/OPEN冲突）。
2. decisions/0003、0004 -> SYSTEM_ARCHITECTURE_AND_ROADMAP。
3. api/PHASE05_CONTRACTS.md -> phase05_acceptance.yaml。
4. control/contracts.py、execution.py、cycle.py及对应测试。
5. hardware接口与HARDWARE_BENCH_RUNBOOK，确认感知/安全/执行分层。
6. CONTRIBUTING、PR模板及两个CI工作流。
7. 对照所审查分支的精确SHA和Actions日志，不能审查main后把结论用于其他分支。

## 核心审查问题

- 是否把reported资产、可选能力或mock误称为现场确定能力？
- Y/Z配置是否独立；停车/响应/安全速度未验收时是否阻断？
- STOP是否不受过期许可阻挡；它是否仍不产生energize？
- session/epoch/sequence能否拒绝迟到、重复、重启前命令？
- Task/Cycle是否独立SystemMode；每斗及重试是否强制重新扫描？
- 24物理DI是否与逻辑语义分离；缺失安全反馈是否保持未知？
- 接收时间/心跳是否伪造测量新鲜度；纯预测是否被误用于下降？
- 云端证据是否绑定精确SHA，且明确hardware/bag/field未执行？

审查输出给出优先级、文件/行、复现条件、风险和建议，不仅写“看起来可以”。
改架构需ADR；补接口需配置/测试/文档同步；现场值必须有记录。
当前可调速及固定低速均只是策略框架，不承诺厂家点动能力或动态性能。

## 精确SHA证据入口

[Phase0.5软件验证记录](../validation/PHASE05_SOFTWARE_470089b.md)记录已实际验证的软件SHA；
[当前评审分支](https://github.com/guolichen007/chili-crane-automation/tree/codex/architecture-freeze-v1)
及该分支最新Actions确定最终HEAD。不要把文档归档提交的SHA与历史验证记录混为一谈。
