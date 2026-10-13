# ADR0011：Observed-only料面与保守抓取提议

状态：ACCEPTED_IMPLEMENTATION_DIRECTION；软件验收另绑定精确SHA。日期：2026-10-13。
基线d6f846f1f98e621f18e2f6a124253e083ee68bae；分支codex/phase2b-pit-surface-grasp-planning-v1。

Phase2A-R1.1正式软件封板：PASS_SOFTWARE_SHADOW_NOT_FIELD；进一步Grab synthetic扩展STOP。
新阶段是Pit Surface Perception + Conservative Grasp Proposal，不是控制。
冻结Grab registration/ESKF/envelope、Geofence v2、PTP/vendor与SemanticMap V1。

## 模块与身份

七包不变；perception承载PitSurfaceGrid/动态mask与扰动生命周期，control承载独立纯规划策略。
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
