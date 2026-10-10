# Phase 1A 实施跟踪

BASE_SHA: d4b8443d40d0e3bbb7c46d631be4b234667a706b

| 项目 | 状态 |
| --- | --- |
| 脱敏硬件事实与泄漏检查 | IMPLEMENTED；本地检查通过，远端以对应 SHA CI 为准 |
| 拉绳模数/开发标定 | IMPLEMENTED；跨界/异常/连续性单元测试；开发标定不授权生产 |
| R2 策略/权限/单调 lease | IMPLEMENTED；自动许可、卸料、能力和单调时钟反证测试 |
| 证据来源/可选 DI/runtime | IMPLEMENTED；模拟独立 topic，生产逐项 physical 检查 |
| site 只读硬件部署 | IMPLEMENTED；三种入口均不加载 DO writer |
| 精确固定官方 SDK | IMPLEMENTED；上游/子模块/message SHA + XYZIRT 最小补丁 |
| dual lidar health/sync/merge | IMPLEMENTED；纯核心合成测试，实际 ROS2 验证由 CI 执行 |
| bag/RViz/部署工具 | IMPLEMENTED；缺外参时分别使用两个 raw RViz，不造 TF |
| Windows static | 已执行；最终精确 SHA/count 见交付报告，1 项 ROS2 依赖测试在 Windows 跳过 |
| Ubuntu CI | IN_PROGRESS；七包 build/test/mock、synthetic ROS2、vendor compile 独立证据 |
| Ubuntu 实机验证 | NOT_RUN |

## 验证范围

- 基线固定为上方 SHA；仅提交 `codex/phase1a-algorithm-deployment-v1`，main 不修改。
- CI 同时归档 exact-SHA 日志、apt 依赖版本、vendor 补丁和来源清单。
- ROS2 synthetic 测试验证实际消息传输、XYZIRT、TF、单雷达丢失、禁止旧帧复用和缺外参失败关闭。
- 本轮复核修复了 TF 初始化顺序、Node 内部 subscription 列表重名，以及
  auto-required 可选 DI 来自另一模拟设备时不应获得 physical readiness 的漏洞。
- `PHASE05_R2` 指软件边界测试，不代表整机/电气/停车标定通过。
- 不在 CI 未通过或现场事实未完成时创建 R2 验收 tag。
- 本文件不自行预填远端 PASS；下载对应 SHA Actions artifact 或查看最终报告核对。
- 首次 Ubuntu ROS2 transport 测试捕获 NumPy concatenate 自动压缩结构化 padding，
  导致 merged 声明 32-byte point_step 而实际 27-byte。追加独立修复提交：强制 canonical
  dtype、序列化前校验，并保留 merged buffer 长度/offset/roundtrip 回归测试。
  首次失败日志不删除，最终状态以修复 SHA 的重新执行为准。
- readiness 的 receive_stamp 保留配对时实际接收事实，不随 10Hz 发布计时器刷新；
  合成 ROS2 transport 断一路后同时验证零复用、降级与 receive_stamp 不变。

## 必须保留

LIVE_ADAM_READONLY_STATUS=NOT_RUN
LIVE_PULLWIRE_STATUS=NOT_RUN
LIVE_ER1_204_STATUS=NOT_RUN
LIVE_ER1_205_STATUS=NOT_RUN
REAL_LIDAR_BAG_STATUS=NOT_RUN
EXTRINSIC_CALIBRATION_STATUS=NOT_RUN
X_SERVO_STATUS=NOT_CONFIGURED
PHYSICAL_ACTUATION_STATUS=NOT_RUN
FIELD_STATUS=NOT_RUN
