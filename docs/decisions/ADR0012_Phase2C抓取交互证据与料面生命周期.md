# ADR0012：Phase2C抓取交互证据与料面生命周期

状态：ACCEPTED_IMPLEMENTATION_DIRECTION。日期：2026-10-14。基线077d0bf。
本轮实现独立shadow证据层，不重写Phase2A/2B核心，不新增TaskCycles，不授予真实动作权限。

## 不可变目标与来源

TargetBinding冻结run/task/cycle、proposal/candidate、XYZ、footprint、surface身份与完整空间/配置lineage。
摘要算法为PHASE2C_BINDING_CANONICAL_V1：字段排序UTF-8 JSON，无空白，float64归一化signed zero后
编码为显式$f64 hexadecimal字符串；排除digest字段自身。禁止NaN/Inf。ROS往返必须保持digest。
binding_stamp为绑定事件时间，evidence.measurement_stamp保留原提议测量时间；两个时间不混用。
同cycle第二次绑定必须拒绝，即使目标看起来相同；只允许同digest查询/传输，不允许覆写。
当前cycle保留抓前frozen snapshot作为证据。新proposal不得改变已绑目标；lineage变更要求ABORT_REQUIRED。

## 证据而非第二状态机

Contact以保守gap interval与下降/实测tracking/载荷特征/持续性融合；几何重叠最多POSSIBLE。
LoadFeatureEstimator实现时序有界buffer、rolling median/MAD、空斗长窗baseline、delta/z-score/
slope/persistence/hysteresis、fault/TTL。FIELD极性与绝对门槛未知只能candidate/unknown。
Closure须闭限位与实测LiDAR CLOSED一致、无矛盾且时间正确；命令issued不是closed。
Payload最终确认须交互、闭合、真实上升、脱离料面、稳定低速度/低加速度和load settling，不能以加速尖峰确认。
Lift区分bound-surface clear、updated-surface clear、pit clear和safe lift height；未知坑高程不能clear pit。
Unload要求已确认lift/payload、卸料区/车状态、真实打开与载荷下降，缺任一证据不得确认。
所有证据的source/TTL/measurement/counter/calibration与cycle/binding身份逐项验证，绝不多数投票。

既有TaskCycles顺序VERIFY_LOAD先于RAISE，与“最终载荷在稳定提升后确认”存在语义冲突。
解决方式是证据admission adapter：VERIFY_LOAD仅检查闭合/已交互且无矛盾，允许进入shadow RAISE，
不声称payload confirmed；RAISE→MOVE_TO_UNLOAD必须等待最终payload与lift证据成熟。
不改旧CycleState顺序，不通过提前确认载荷绕过物理时序。未来真实admission仍另行授权。

## 料面生命周期

现有SurfaceDisturbance保持SYNTHETIC/REPLAY原意。独立SurfaceInteractionEvidence承载真实交互来源链。
可靠下降侵入、共时geometry+load持续或真实交叠中闭合可commit；空抓也立即幂等局部失效。
受扰mask为binding footprint、实测Grab swept cells与显式collapse margin的并集，只影响坑内交叠格。
复用Grid.invalidate与既有统计/质量方法，adapter阻断未clear/interaction前旧点恢复，未受扰区保留。
CLEAR_OF_BOUND_SURFACE只开启post-interaction观察；继续Grab mask、不插值。
刷新必须受扰区新点满足原B support/coverage/TTL/stationarity/lineage，世代大于绑定世代，
再证明CLEAR_OF_UPDATED_SURFACE。没有新点保持UNKNOWN/DEGRADED；下一抓禁止复用N。

## 验证与边界

新增消息为breaking接口扩展，所有消费者重建；LoadState统一EvidenceMetadata同时迁移mock/consumer。
SYNTHETIC/REPLAY配置与FIELD分离；FIELD未知数值null/NOT_CONFIGURED，不以synthetic配置伪造标定。
8个ROS包normal/symlink、旧A/B/PTP/vendor/全部反证保留；new installed证据链与三控制topic零消息另验。
最终最多PASS_SOFTWARE_SHADOW_NOT_FIELD；physical/automatic/camera/valid_for_control全false，FIELD与BENCH均NOT_RUN。
