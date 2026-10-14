# Phase2B 料面建模与抓取候选

实现基线d6f846f；ADR0011。目标是Pit Surface Perception + Conservative Grasp Proposal。
不是控制，不连接物理执行。Phase2A进一步synthetic算法扩展STOP；旧回归仍保留。

## 结构与数据边界

canonical XYZIRT → 显式PlanningSpatialContext → stable-window gate → fixed/self + Grab mask
→ PitSurfaceGrid → 有界SurfaceSnapshot → Geofence v2 domain → capture footprint硬门禁 → ranking → GraspProposal。

perception：pit_surface.py、surface_mask.py、surface_disturbance.py；control：grasp_planning.py、distance_acceleration.py。
surface_transport.py是typed ROS适配，不引入设备寄存器、TF猜测或Phase3定位算法。
surface_runtime/grasp_runtime均安装于perception，调用control纯策略；依赖方向perception→slam→control不反转。
R1 planning transport使用独立PitSurfaceSnapshot V2，仅发送observed格及index，省略格为UNKNOWN；
消息边界最多100000 observed格，实际配置另限，不是默认发布规模。PitSurfaceStatus仅摘要。
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
height uncertainty = 配置k×最大帧内/帧间MAD + sensor界 + spatial界 + 允许pose jitter界 + 单雷达附加界。
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
R1完整扫描active domain，以积分表bbox预分数和稳定ID tie-break维护maximum_candidates大小heap；
预分数不是最终料量下界。ESDF先拒绝明显不可行中心，再对shortlist执行精确footprint验证。
最大候选数裁剪只限制shortlist，不提前结束raster枚举，正序/反序/随机输入结果一致。
若maximum_evaluated_cells不足，SEARCH_TRUNCATED+GLOBAL_OPTIMUM_NOT_PROVEN显式输出；
完整扫描但有shortlist或ESDF裁剪也不声称全局最优。只有全部中心经精确验证才证明搜索最优（仍是shadow）。
固定机械yaw或经证明的yaw-conservative footprint；默认无yaw控制，不优化不可控角度。
真实kernel/CAD/capture depth/provenance未知则生产NOT_CONFIGURED。
每个候选先检查exact polygon/footprint墙距、Geofence margin、真实格面积intersection coverage、
UNKNOWN fraction、逐格TTL、height bounds、slope、roughness、support和地图/配置/代际身份。
一格即使带有数值高度，缺真实sample/frame/sensor/邻域support也不得参与覆盖或proxy。
distance transform只fast reject，不能跳过最终精确polygon检查；最乐观错误距离场也不能产生false-safe。

material_volume_proxy_m3 = Σ真实observed交叠面积×min(配置capture_depth,max(0,z_median-pit_bottom))。
lower_bound使用z_lower_bound。UNKNOWN面积不估高、不补量；不是expected_payload_kg或实际bucket fill。
真实形变、关斗动力学、密度、载荷/tension标定留Phase2C。
R1先增加minimum_material_volume_proxy_lower_bound_m3和minimum_conservative_material_height_above_pit_bottom_m
两个hard gates。近底部高coverage/平整表面仍输出MATERIAL_INSUFFICIENT/FLOOR_RESERVE_INSUFFICIENT。
门槛不是payload能力，生产为null。通过hard gates后按最大volume LB锚定volume_equivalence_epsilon带，
同带按coverage降序、recent penalty升序、roughness/slope升序、wall clearance降序、ID排序。
epsilon生产null，不引入猜测权重；接近料量时避开近期抓取，显著更高LB仍在更高带优先。
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

## R1 空间lineage、逐帧transform与遮罩共时

lineage_key包含frame、map/semantic/pit、source/transform_state、完整空间链calibration、
FIELD外参、axis-to-map及两雷达标定身份。冻结地图的世代由map_id/semantic_version标识，换图必须换身份。
逐帧transform_id/motion/CranePose证据、measurement epoch、数值transform/uncertainty不属于lineage，
但仍逐帧验证证明身份、时间、rigid T和motion_stable，不能省略空间证明。
新lineage硬reset；同lineage用自身T_map_cloud(k)变换，漂移相对窗口首帧，而不是相邻帧，防慢速累积。
maximum_stationary_transform_translation_delta_m/rotation_delta_rad界内允许融合；
超界SPATIAL_MOTION_EXCEEDS_STATIONARY_WINDOW清空/拒绝。窗口无帧时重新锚定，窗口内锚点不逐帧漂移。
允许jitter的z误差保守取translation_limit+2sin(rotation_limit/2)×max(norm(source_xyz))，
加到已有transform uncertainty，不隐含毫米pose等于零误差。两limit生产null。

GrabMask.calibration_id在此是geometry.extrinsic_calibration_id，必须等于context.field_extrinsic_id；
context.calibration_id独立表示完整空间链，由cloud/meta确认。Synthetic两ID故意不同，禁止偶然字符串相等掩盖错误。
abs(cloud MID_SCAN epoch-Grab state epoch)必须≤maximum_grab_mask_time_skew_sec（生产null）；
旧但未超TTL的Grab不能掩蔽移动抓斗，当前帧不得入料面。mask还必须TRACKED/stable/fresh且map-frame规范化。

## R1 活动坑coverage、稀疏transport与世代缓存

pit_active_cell_mask预计算polygon与格有正面积交集，坑外bbox格不进入coverage分母；
status提供pit_active_cell_count/observed_cell_count/support_qualified_cell_count/dual_supported_cell_count。
support-qualified还要求坡度可计算、邻域coverage与高度不确定性符合策略，不等于控制许可。
局部footprint仍严格按observed交叠面积积分，活动域摘要不能放松候选局部UNKNOWN门禁。
PitSurfaceSnapshot.cell_encoding=OBSERVED_INDEXED_V2，仅发送observed格；消费者检查容量、重复/越界index、
observed标志、摘要身份和数目，然后在有界内部恢复UNKNOWN格。旧DENSE_V1/空encoding一律拒绝。
V2是breaking wire语义，生产者和全部消费者重建；禁止V1/V2混跑。

surface缓存fusion snapshot；仅generation变化做重序列化/CDR准备/DDS发布，status心跳单独老化。
observed/unknown debug和marker独立1Hz，非planning transport。planner cache绑定snapshot_id/generation/
disturbance_counter/lineage/config hash及recent counter；同不可变snapshot不重解，map/pose/TTL仍每次验证。
ROS adapter只解码新snapshot一次，拒绝倒序/重复世代，不重置arrival。proposal心跳不更新source epoch/counter。
耗时规划完成后publication_gate再次检查当前源时间、surface TTL与独立context TTL，避免开始fresh、发布时stale；
包含monotonic residence age，不通过改header或measurement epoch隐藏耗时。
同世代recent penalty固定为计算时的诊断值，过期后只在下次新世代重排；不能当作新观测。
QoS分型和可靠性/最新状态/迟加入取舍见ADR0011，不把DDS reliability等同于实时或安全保证。

## R1 性能方法与容量

tools/validate_phase2b_r1.py分别执行FAST_REJECT（fresh-but-skewed mask）、FULL_VALID_PROPOSAL（256格）
和DENSE_SURFACE（1024格，配置上限1024）的端到端软件workload；不混合P95，不制定生产SLA。
13个stage分别记录P50/P95/MAX：transform、grab mask、pit filtering、cell ingest、temporal fusion、
height bounds、slope/roughness、snapshot serialization、DDS payload preparation、ESDF reject、exact candidate geometry、
ranking、proposal serialization。跳过的stage为0执行时间，Windows DDS stage明确NOT_RUN_NO_ROS。
配置/活动mask预计算和fixture生成不计入total；冷planner布局/ESDF创建计入total，不能视为稳态频率。
ingest stage取该workload最后一个accepted frame的单次调用，fusion/geometry等取该snapshot一次调用；
total包含两帧摄入，不是所有stage值的简单和，未把DDS发送/网络/RViz延迟计入该离线总时长。
input_points/active/observed/serialized cells/bytes、scanned centers/shortlist、ESDF/exact计数随每次样本保存。
Windows byte count为NATIVE_JSON_DIAGNOSTIC_NOT_DDS；Ubuntu installed测实际ROS2 CDR及转换时间。
64/256/1024格partial容量检查UNKNOWN隐式，CDR边界16384+192×serialized_cells用于当前有界synthetic身份，
不是任意超长字符串/任意生产配置的通用DDS保证；FIELD尺寸、频率、CPU、网络必须独立验收。
旧八类安全计数和新增五类反证均要求0；ROS installed另验证世代快照静默、缓存age增长、旧DDS世代不重算。
installed专用NON_PRODUCTION时间profile为frame TTL 2s/surface TTL 2.5s/window 3s/最多2帧，
将共享runner中的缓存暂停与自然窗口淘汰分开；停止输入expiry仍单独断言。native原0.7s与FIELD null不变。

## R1.1 时间锚点与进程session收口

本轮只改时间共时与生命周期，不改上述统计/候选/ranking/ESDF/proxy算法。
上游canonical header为FIRST_POINT；merged header为两header最大值，保持原ROS/evidence/frame.epoch身份。
surface_time.scan_epochs从canonical point timestamps求start=min、end=max、mid=start+(end-start)/2。
header落在点范围内仅证明HEADER范围合法，不证明MID_SCAN。Grab测量与frame.mid_epoch比较skew；
未显式提供mid_epoch拒绝，绝不退回header。每sensor内部时间regression/nonfinite与后继帧mid回退拒绝。
无插值/预测，仅真实latest Grab测量；FIRST_POINT Grab即使TTL fresh，只要半跨度超过skew也拒绝。

producer进程UUID hex surface_session_id贯穿PitSurfaceStatus、PitSurfaceSnapshot、snapshot_id、
GraspProposal provenance与planner cache。snapshot_id=session:config前12位:generation:lineage前12位；
wire内外session一致且ID组合正确，仍独立验证完整hash、context与freshness。
新session清latest/缓存/floors/pending，再以valid/fresh/provenance/context建立基线。137→1不是旧世代。
同session已见更高无效世代也清latest，不准较低旧snapshot复活。旧session重放不续arrival、不重算cache。
TL迟加入只保留一个pending，context晚到仍使用原到达时间和原measurement epoch；不能续龄冒充新观测。
退役session拒绝集合有界128项，容量耗尽fail-closed而非LRU淘汰。完全消费者重启仍依赖TTL/空间证明，非认证系统。
disturbance ROS session绑定拒绝跨session旧事件；核心PHYSICAL拒绝策略不变。
所有扩展ROS消息需生产者/消费者同步重建。独立installed脚本validate_phase2b_r11_ros.py
使用未修改DualLidarMerger与80/100ms扫描、两个FIRST_POINT相差4ms，真实geometry测量戳在点时间mid。
单scan regression/nonfinite、FIRST_POINT Grab、旧但fresh Grab均拒绝；surface-only/planner-only/both重启验证恢复。
planner-only验证保留的TL快照，无新cloud/snapshot，仅重传epoch不变且age增长的真实context。
专用NON_PRODUCTION启动profile frame5s/surface6s/window6s/max2frames/span.12s，原R1期限/现场null不变。
原R1性能三workload与全部反证保留，冻结算法AST与基线比较。FIELD性能不由CI合成P95升级。

SEARCH_TRUNCATED/GLOBAL_OPTIMUM_NOT_PROVEN保持；search completeness != safety eligibility。
是否允许truncated candidate进入控制留后续control admission policy，当前输出仍valid_for_control=false。
最终证据见[软件封板验收](../validation/Phase2B-R1.1软件封板验收.md)，只封软件shadow，不封现场或执行链。
