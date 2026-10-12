# ADR0007：Phase1B PTP 时间尺度与运行证据

- 状态：ACCEPTED，2026-10-12。
- 基线：48674999d07c5df24e8b3e36ded55b8b779794a2。
- 分支：codex/phase1b-ptp-timescale-r2；main 不修改。

PTP domain 和 HOST_UTC domain 不能作相等判断，必须显式表达尺度映射。
37 秒来自 S14 本地 UTC 关系，不是 SDK latency correction。
vendor 原样；canonical header 与每点一起减 offset；无关系 raw-only。
S14 currentUtcOffsetValid=0/timeTraceable=0 不否定双 ER1 相对同步，
但阻止正式 production VALID。LOCAL_VERIFIED 只供算法台架。
相对时钟同步与 scan phase 独立；持续只读 DIFOP/MSOP 和 TTL 决定运行 readiness，
不使用一次启动观察永久授权。生产 timing 与完整空间/control readiness 独立。
不修改 vendor，不改变相机时间流程，不做外参/融合/NDT/Grab/Pit/控制。
