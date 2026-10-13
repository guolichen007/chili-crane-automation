# Phase2B 料面建模与抓取候选

实现基线d6f846f；ADR0011。目标是Pit Surface Perception + Conservative Grasp Proposal。
不是控制，不连接物理执行。Phase2A进一步synthetic算法扩展STOP；旧回归仍保留。

## 结构与数据边界

canonical XYZIRT → 显式PlanningSpatialContext → stable-window gate → fixed/self + Grab mask
→ PitSurfaceGrid → 有界SurfaceSnapshot → Geofence v2 domain → capture footprint硬门禁 → ranking → GraspProposal。

perception：pit_surface.py、surface_mask.py、surface_disturbance.py；control：grasp_planning.py、distance_acceleration.py。
surface_transport.py是typed ROS适配，不引入设备寄存器、TF猜测或Phase3定位算法。
surface_runtime/grasp_runtime均安装于perception，调用control纯策略；依赖方向perception→slam→control不反转。
完整格网使用独立PitSurfaceSnapshot（最多100000格，实际配置另限）；PitSurfaceStatus仅摘要。
旧GraspTarget保持历史占位；新GraspProposal不是它的自动控制升级。新增/扩展消息需全部消费者重建。

## 空间、时间与状态

即使frame同名也要显式T_map_cloud及transform/calibration/map/pit/version身份。
transform是row-major刚体4×4；uncertainty_m是该ROI内变换误差的保守空间长度界，
必须包含旋转误差×ROI杠杆距离，不能只填平移标准差。sensor calibration IDs明确区分0/1。
PHYSICAL需要FIELD_VALIDATED外参、axis-to-map、CranePose证据，不接受BENCH作为FIELD。
这里只消费上游规范化证明，未实现FIELD证明生成器；未知配置不能启动有效融合。
平台或Grab不稳定、源/逐点时域不一致、跨度/TTL/元数据/地图错配都拒绝并清空旧窗口。
当前没有运动补偿分支，移动一律不融合；HOST_UTC物理来源不能配use_sim_time。
frame_counter和measurement epoch严格递增；重复/倒序不能续龄。
窗口最多maximum_window_frames，单帧maximum_input_points，格网maximum_grid_cells，候选maximum_candidates。
队列latest-wins；不缓存无限历史/代际。各格last_observed_epoch不被发布/提议心跳更新。

## 真实观测和统计界

只接收SemanticMap V1坑内、pit_bottom至wall_top内的实测点；不修改V1 schema。
每帧每格median/MAD，窗口各帧median再median，保留frame/sample/sensor support。
height uncertainty = 配置k×最大帧内/帧间MAD + sensor界 + spatial界 + 单雷达附加界。
lower/upper还包住短窗median范围；这是假设经标定的经验保守模型，不是任意outlier概率保证。
上下界用途不同：upper用于未来接近/碰撞判断，lower仅material abundance proxy。
坡度用真实3×3邻域的三次有界Huber平面IRLS；不够coverage/non-collinear support就UNKNOWN质量。
roughness保留帧内MAD与局部平面MAD；coverage保存邻域support，边界缺邻域不假装补齐。
无return不等于EMPTY。没有planning interpolation函数，debug也只显示observed与独立UNKNOWN。
单雷达格DEGRADED并扩大不确定性；交替单雷达帧即使窗口mask并集为3也保持DEGRADED与附加界。

## 遮罩、扰动与代际

Grab mask消费TRACKED/measurement_accepted/motion_stable、同地图/frame/calibration/epoch证据，
使用conservative footprint+bottom_lower/top_upper及显式dilation。原Grab pose与envelope不改。
当前节点要求map-frame normalized geometry/clearance；若Phase2A仍输出crane/base，缺经审核的空间适配就拒绝mask。
未实现FIELD空间证明生成器或Grab-to-map实机适配器，不宣称实机直接接通。
static/self mask从Phase2B配置引用，空列表也需要owner明确VALID；生产模板NOT_CONFIGURED。
失效mask不能把Grab点当物料，旧窗口立即失效。料面接触并不依赖Grab孤立欧氏簇。
SurfaceDisturbanceTracker只接受synthetic/replay event；未来正式TaskExecution另行授权接入。
受扰footprint+dilation与格相交即移除旧观测，固定大小tombstone记event epoch，
事件前迟到帧仍不能恢复；只有事件后真实新观测并达到原support门槛才能恢复。
旧generation提议拒绝；提议节点收到事件后等待料面端last_disturbance_id/counter确认，
不能用后续普通帧generation增长冒充失效确认。recent事件deque有界且过期；周边penalty不替代失效。

## 候选与proxy

直接引用冻结OperationalGeofence v2的safe_region/track/margin/provenance，不复制另一坑polygon。
中心只在可达域生成；最大候选数限制会截断空间枚举，不声称找到全局最优。
固定机械yaw或经证明的yaw-conservative footprint；默认无yaw控制，不优化不可控角度。
真实kernel/CAD/capture depth/provenance未知则生产NOT_CONFIGURED。
每个候选先检查exact polygon/footprint墙距、Geofence margin、真实格面积intersection coverage、
UNKNOWN fraction、逐格TTL、height bounds、slope、roughness、support和地图/配置/代际身份。
一格即使带有数值高度，缺真实sample/frame/sensor/邻域support也不得参与覆盖或proxy。
distance transform只fast reject，不能跳过最终精确polygon检查；最乐观错误距离场也不能产生false-safe。

material_volume_proxy_m3 = Σ真实observed交叠面积×min(配置capture_depth,max(0,z_median-pit_bottom))。
lower_bound使用z_lower_bound。UNKNOWN面积不估高、不补量；不是expected_payload_kg或实际bucket fill。
真实形变、关斗动力学、密度、载荷/tension标定留Phase2C。
通过hard gates后lexicographic排序：lower proxy降序、coverage降序、roughness升序、slope升序、recent penalty升序。
每个分量、拒绝原因、上下界、kernel/Geofence/snapshot/transform身份均输出，不用黑盒confidence决定安全。

## ROS与验收

phase2b_surface_shadow.launch.py只启动pit_surface_shadow_node和grasp_proposal_shadow_node。
默认模板不可用；需要显式提供配置、上游空间context、canonical cloud/evidence和Phase2A geometry/clearance。
topic：lidar/merged_points、perception/pit_surface_input_evidence、planning/spatial_context为输入；
perception/pit_surface_snapshot/status、observed/unknown debug、planning/grasp_proposal_shadow为输出。
planning/surface_disturbance_shadow仅接受synthetic/replay，不是执行命令。
三控制topic零发布，DO writer不加载；valid_for_control和三控制配置flags永远false。
RViz显示height/roughness/coverage、UNKNOWN、pit、独立Grab包络、可达域、top-10与selected shadow。

Windows：仓库静态测试和tools/validate_phase2b_synthetic.py；21场景、64确定性序列×7帧。
序列实际移动Grab footprint，moving/non-compensated帧拒绝融合；每个序列候选独立做墙/可达域/UNKNOWN反证。
Ubuntu：原验收入口追加normal/symlink installed Phase2B、实际RViz加载；Phase2A和旧双雷达/PTP/Phase1C/vendor不缩减。
tools/validate_phase2b_ros.py用规范化synthetic Phase2A消息验证传输，不冒充真实Grab CAD或FIELD联机。
性能P50/P95/MAX分平台记录，不冻结生产时延；所有计数是有限fixture反证，不是现场概率保证。
PIT_SURFACE_FIELD_STATUS/GRASP_PROPOSAL_FIELD_STATUS/MATERIAL_VOLUME_PROXY_FIELD_STATUS均NOT_RUN。
