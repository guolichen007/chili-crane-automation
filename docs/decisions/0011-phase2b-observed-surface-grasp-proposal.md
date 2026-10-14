# ADR0011：Observed-only料面与保守抓取提议

状态：ACCEPTED_IMPLEMENTATION_DIRECTION；软件验收另绑定精确SHA。日期：2026-10-13。
基线d6f846f1f98e621f18e2f6a124253e083ee68bae；分支codex/phase2b-pit-surface-grasp-planning-v1。

Phase2A-R1.1正式软件封板：PASS_SOFTWARE_SHADOW_NOT_FIELD；进一步Grab synthetic扩展STOP。
新阶段是Pit Surface Perception + Conservative Grasp Proposal，不是控制。
冻结Grab registration/ESKF/envelope、Geofence v2、PTP/vendor与SemanticMap V1。

## 模块与身份

七包不变；perception承载PitSurfaceGrid/动态mask与扰动生命周期，control承载独立纯规划策略。
ROS surface/proposal适配器均在perception；既有perception→slam→control依赖方向保持，无control→perception环。
复用PitSurfaceStatus概念；旧GraspTarget仍是历史占位，不把它悄悄升级为控制目标。
显式PlanningSpatialContext证明source→map transform、map/pit/calibration/version、measurement epoch、
逐雷达provenance和平台静止证据。相同frame名字也不能替代transform证明。
PHYSICAL需FIELD外参、axis-to-map、CranePose证据；不复制Phase3定位算法。
移动且无已验证补偿一律不融合；当前实现只接受短时静止窗口。

## 料面与提议

每帧分格median/MAD，短窗robust融合；上下界包含样本/时间离散与显式sensor/spatial不确定性。
边界是配置的经验保守统计模型，不是任意离群分布的概率保证。真实参数全部NOT_CONFIGURED。
未返回不是EMPTY；UNKNOWN不得插值成observed。局部坡度只用真实observed邻域。
动态Grab envelope独立mask，不能改变Grab位姿；mask失效阻断融合并废弃旧料面。
扰动先失效旧格，再等待新测量恢复；邻域recent-grab只做软惩罚。
候选先受既有Geofence可达域与track约束，再以真实footprint精确检查坑/墙/observed覆盖。
ESDF仅快速拒绝；最终精确polygon检查不可省略。评分不能救回hard rejected候选。
volume仅material_volume_proxy_m3，不是payload、装斗预测或载荷标定。
yaw_control_available默认false；只能固定已知机械方向或保守包络，不能优化不可控yaw。

## 安全与验证

所有运行节点仅shadow，valid_for_control永远false；不发布SafetyPermit/AuthorizedCommand/ActuationRequest。
生产未知参数null/NOT_CONFIGURED；synthetic配置明确NON_PRODUCTION并独立于生产模板。
B1→B5分别验证：grid、mask、候选、排序/扰动、installed ROS/RViz/fuzz；旧Phase2A完整回归不缩减。
Windows仅静态/native；Ubuntu/Humble CI normal/symlink与现场证据严格分离。
FIELD及PHYSICAL全部NOT_RUN，X协议NOT_CONFIGURED，不由CI升级。

## R1 amendment：FIELD-readiness（2026-10-13）

基线ac0dec1保持PASS_SOFTWARE_SHADOW_NOT_FIELD；以下是增量契约，不重写历史决策。

空间lineage只哈希frame/map/semantic/pit/source/transform-state、完整链calibration、FIELD外参、
axis-to-map和两个sensor calibration身份。地图世代由冻结map_id/semantic_version绑定，换图必须换身份。
逐帧transform_id、motion/CranePose证据ID、epoch、数值T/uncertainty不属于lineage；证明仍逐帧检查。
同lineage每帧使用自身T，针对窗口首帧T限制translation/rotation，而非相邻帧，避免慢速pose walk。
高度不确定性增加允许平移界+2sin(rotation_limit/2)×最大输入杠杆距离；超界拒绝/清空。
Grab遮罩状态epoch必须与MID_SCAN共时，TTL不足以证明共时；遮罩extrinsic_id匹配FIELD外参ID，
输入cloud/meta匹配完整链calibration_id，二者允许不同且synthetic故意不同。

全活动域扫描+积分表bbox预分数+有界heap；精确footprint验证只对shortlist进行。
预分数不是安全许可或volume LB。完整扫描不等于证明全局最优；K裁剪明确GLOBAL_OPTIMUM_NOT_PROVEN，
预算不足另标SEARCH_TRUNCATED。空坑volume LB/底部余量先硬门禁，再用最大值锚定epsilon带排序，
同带按coverage、recent penalty、roughness、slope、墙距、ID排序，无猜测生产权重。
ESDF在精确几何前拒绝；使用kernel内切圆而非外接圆，避免异形细长抓斗错误几何推断。
保守ESDF可能拒绝安全候选，故仅在未用ESDF裁剪且穷举精确候选时宣称GLOBAL_OPTIMUM_PROVEN。

V2 planning transport仅observed indexed cells，UNKNOWN隐式；status分母为polygon有正交叠面积的活动格。
snapshot generation变化才发布；status独立心跳、debug低频1Hz。规划按snapshot/generation/
disturbance/lineage/config身份缓存，独立freshness每次复核；重复heartbeat不续epoch或counter。
recent penalty在该snapshot计算时冻结，TTL到期只在下一新世代重排；心跳不能续旧penalty或复活旧料面。

QoS分型：raw/cloud与高频空间/Grab/evidence为BEST_EFFORT、KEEP_LAST depth=1，兼容既有sensor发布者，
丢帧由epoch配对和TTL fail-closed处理；large snapshot为RELIABLE、TRANSIENT_LOCAL depth=1，
只保留最新世代供迟加入者，planner拒绝倒序/重复世代且不更新arrival；proposal为RELIABLE volatile depth=1，
保留状态语义不堆积控制指令；synthetic disturbance为RELIABLE volatile depth=1，ACK缺失阻断旧料面。
status/debug为BEST_EFFORT depth=1，不提供控制许可。可靠性不等于共时、freshness或实时SLA，
DDS实现可能阻塞，现场吞吐/网络/丢包另验。事件若丢失，不能以普通新帧冒充ACK。

性能分FAST_REJECT/FULL_VALID_PROPOSAL/DENSE_SURFACE，逐stage P50/P95/MAX及payload/容量统计。
Windows JSON diagnostic byte count不是DDS；Ubuntu实跑CDR。64/256/1024格是NON_PRODUCTION容量样本，
不把消息上限100000格当作生产默认频率。所有生产新数字null；现场组合缺口不扩算法、不升级状态。

## R1.1 amendment：两个P1的最终软件封板（2026-10-14）

基线8ee41886dfb8d084b2b01d505b770c98bf2fefde；冻结全部料面统计、候选搜索/排序、ESDF与proxy策略。
R1的“MID_SCAN共时”意图不等于header落在点范围内。上游canonical header保持FIRST_POINT，
merged header=max(两个FIRST_POINT)，不改PTP/dual LiDAR。Phase2B单独计算点时间min/max及
mid=min+(max-min)/2；frame.epoch保留header/evidence identity，frame.mid_epoch专用于Grab共时。
每雷达点时间非递减；两雷达拼接产生的跨sensor时间回退不是单scan regression。
nonfinite、单scan regression、后继帧MID_SCAN回退、header超点范围均fail-closed；名称明确区分header和mid。
latest-only真实Grab测量保留；超过skew拒绝，不插值、不预测、不以arrival伪造measurement。

surface producer启动时生成一次UUID hex surface_session_id，不随clear/expire/世代变化。
snapshot_id=session:config前12位:generation:lineage前12位；独立字段还校验完整config与lineage。
status/snapshot/proposal及cache均绑定session。新session先清空latest/cache/floors/pending event与代际假设，
再校验真实snapshot的freshness、完整空间context与provenance，允许137→1。
同session收到更高generation即使无效也阻断旧latest；已见代际不允许后续回退或重复样本续龄。
迟加入的TRANSIENT_LOCAL最新snapshot可在真实context后到时保持一个有界pending，原arrival不重置。
已退役session集合最多128项，不LRU淘汰以免复活旧session；耗尽fail-closed，需显式重启消费者。
session是生命周期身份，不是签名/认证。节点完全重启后不能仅凭ID证明跨进程恶意重放历史，仍需freshness与空间证据。
SurfaceDisturbance消息增加session绑定以拒绝延迟旧事件；核心仍SYNTHETIC/REPLAY only，PHYSICAL执行证据链留Phase2C。
消息扩展为breaking wire变更，全体消费者重建，禁止旧新消息混跑。

search completeness != safety eligibility。SEARCH_TRUNCATED/GLOBAL_OPTIMUM_NOT_PROVEN保持；
未来truncated candidate能否进入控制由另行control admission policy决定，本轮不授权。
normal/symlink installed三种真实节点重启、旧session/世代反证、80–100ms真实点时间结构必须通过；
完整Phase2A/旧入口/vendor不缩减，三控制topic仍0。全部通过后软件封板，FIELD仍NOT_RUN，无Phase2B-R1.2。
