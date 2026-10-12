# Phase2A 抓斗状态估计与几何安全

实现范围：模型驱动的纯算法与ROS2 shadow，不是FIELD外参或安全控制器。
基线72234843d5850947c5a2fd5e230516bc8fa64839；ADR0009；七包/Ubuntu22/Humble保持。
现场CAD、生产参数、pit实测、源到common-frame变换须owner确认。

## 1. 算法职责与模型

Grab是已知自身机构，不能把普通目标聚类centroid当控制位姿。
GeometricGrabCandidateProvider只做ROI、self-mask、voxel和有界欧氏聚类，输出多个候选。
TRACKED使用预测局部ROI，LOST_HOLD扩大，LOST使用配置搜索体积/可选悬挂走廊。
强度仅保留，不成为未经验证的硬阈值。不同位置的合法模型匹配视为关联歧义，
不会悄悄挑一个锁定。PCA仅为LOST的yaw初值，不发布PCA最终位姿。

GrabGeometryModel独立保存详细sampled surface（可由CAD离线采样）、配准缓存
（两层small_gicp PointCloud/normal/covariance/KdTree）和安全convex vertices。
启动检查安全hull包含样本、底/顶参考和版本/provenance；未确认配置拒绝算法启动。
本轮不提供通用CAD mesh导入器，owner应先导出模型坐标的采样点与安全顶点。
OPEN/CLOSED分别建模；TRANSITION不估计精确开度，安全模型取二者并集，
观测不给ACCEPTED，只允许短时预测/保守显示。owner还须验证极限包络及中间机构行程。
symmetry_yaw_period_rad由型号确认，不能普遍猜π；合成fixture专门为π对称盒模型。
真实几何与噪声/速度/门限在模板中全null/NOT_CONFIGURED，fixture不是现场建议。

## 2. 配准与观测门禁

缓存MODEL为target，候选common-frame云为source：原生GICP解T_model_common，
求逆得到T_common_model。coarse/fine两级；模型缓存不随每帧重建。
Python small_gicp1.0.1绑定没有robust-kernel参数，不能假称native GICP本身带Huber。
本项目在native初解后显式做Huber IRLS point-to-plane精化，截断correspondence并限制发散步长。
最后重新计算残差/Hessian，不能沿用更新前统计。
沿用既有校准数学/XYZIRT契约，未改冻结PTP五文件或vendor内容。

等价yaw群枚举后选择与预测最近的旋转计算innovation；π跳变不会当瞬间转动。
LOST使用PCA及其正交初值覆盖不同姿态假设；已知等价姿态不需重复解相同几何。
当前V1重捕获为MULTI_HYPOTHESIS_GICP，不引TEASER++或学习模型。

GrabObservationValidator检查convergence、inlier、残差P50/P95、spatial extent、
支持octants、sensor IDs、信息特征值/条件数、translation/rotation innovation、
线/竖向/角速度以及OPEN/CLOSED一致性。ACCEPTED才进入滤波，
弱可观/支持不足为DEGRADED，其他失败为REJECTED；两者均不更新测量。
converged不是VALID，支持计数也不是安全置信证书。
测量协方差来自最终加权H、残差尺度与支持率；单侧/单雷达膨胀，弱方向方差增大，
不是固定R。Huber规模、传感器噪声和可观阈值仍需现场冻结。

## 3. ESKF、对称性与丢失

nominal为common-frame平移、world线速度、body角速度、SO(3)旋转。
误差顺序δp_world、δθ_body、δv_world、δω_body；12x12协方差。
p+=v dt，R*=Exp(ω dt)；旋转误差F使用Exp(-ω dt)和right Jacobian。
Q为配置的连续白加速度谱密度，position/velocity及angle/rate有交叉块。
innovation为world位移与Log(R_pred^T R_meas)，不对Euler直接KF。
Joseph更新加右误差reset；对称yaw变换时同步旋转measurement covariance的body基。
ROS pose covariance转world/fixed-axis，诊断角速度转common-frame。
滤波为小误差近似，超限innovation拒绝，不把大角不连续交给线性滤波硬修。

CANDIDATE需连续多帧ACCEPTED后TRACKED；测量失败清零连续计数。
短时LOST_HOLD只发PREDICTED，超时LOST清位姿，重捕获再次确认。
重复/回退时间拒绝；预测不能更新measurement timestamp。
没有command-time积分Z；未来hoist/task/trolley仅prior，不代替LiDAR测量。

## 4. Swing、稳定与时间

悬挂点与model suspension reference变换后的相对向量给出horizontal offset、vertical drop、
atan2摆角/方位、lateral velocity和摆角变化率。缺新鲜独立prior或drop非法就是UNKNOWN，
不能凭抓斗位置“变化不大”断言稳态。
稳定窗口综合横向速度、摆角、摆速、innovation RMS；进入/退出门限及持续时间独立配置。
窗口有界，预测/丢失立即取消motion_stable；迟滞抑制临界抖动。

XYZIRT每点timestamp保留，frame_span取max-min。
平移/旋转scan smear为速度上界*span/2；尚无可信twist时用配置机械最大速度，
之后加入twist covariance的kσ界，不把第一帧零速度当真。
模糊进入测量R和安全包络；本轮不执行deskew，不伪称补偿扫描运动。
GrabPointMotionCompensator留作V2：按timestamp和验证twist补偿到MID_SCAN。

## 5. Bottom与保守几何

GrabState.bottom_z_m定义为T_common_grab*model.bottom_reference的nominal Z。
不是raw min(z)，不是安全lower bound；消费者须读companion并核对source/validity。
GrabGeometrySafetyState另给bottom_lower/upper和top_upper；倾斜时检查全部安全hull顶点，
不能只旋转中心底点后忽略最低角。几何/pose/extrinsic/smear均进入界。

XY包络为安全顶点投影convex hull与各向异性kσ椭圆的Minkowski和，
叠加旋转、几何、scan displacement。椭圆/圆用外切多边形，不以内接采样虚报安全。
pose/extrinsic相关性未知时使用保守协方差上界，pose cross blocks也作Loewner包络，
不以固定distance-margin代替几何传播。
这是条件化模型界：CAD/外参误差上界与噪声分布未FIELD验证，kσ不是安全认证。

## 6. Pit与实时clearance

PitBoundaryModel复用SemanticMap.PitDefinition的polygon_map_m、
wall_top_z_m、pit_bottom_z_m、safety_inset_m，不造第二套schema。
凹多边形用完整covers/intersection，不只检查footprint顶点。
包含时精确segment/polygon minimum distance；安全inset用distance>=inset判断，
不把近似buffer曲线当安全依据，buffer只用于表示safe region。
跨界/外部明确负距离，联合包围盒直径为保守侵入界，不称精确穿透深度。
raw和conservative分开，后者不高于前者。
vertical envelope与[pit bottom,wall top]无重叠则collision_relevant=false，
仍报告有限horizontal clearance，不返回伪infinity。
GrabClearanceState为当前状态，GraspTarget.wall_clearance为规划目标，不混。
未配置地图/坐标不符/预测/过期则UNKNOWN，risk保守true。
未来ESDF只加速规划，不替代此几何证据。

## 7. Shadow输入与部署

Ubuntu命令：

    source /opt/ros/humble/setup.bash
    source /path/to/exact-sha/install/setup.bash
    ros2 launch chili_crane_bringup phase2a_perception_shadow.launch.py config_file:=/path/outside/repo/owner-reviewed-grab.yaml

config/perception/phase2a_grab.template.yaml保持NOT_CONFIGURED。
owner副本须CAD采样/安全顶点、政策、source/extrinsic ID/common frame及semantic map路径。
time_domain必须明确HOST_UTC或ROS_TIME；PHYSICAL只能HOST_UTC，replay/simulation需明确ROS时钟。
回放/模拟可显式use_sim_time:=true并提供/clock；PHYSICAL拒绝sim clock，不能混淆时域。
验证源时钟age与monotonic receive age共同阻止DDS处理前积压刷新“新鲜”。
input=/crane_01/perception/grab_input（common-frame canonical XYZIRT、MID_SCAN header），
同帧grab_input_evidence（calibration/config/source/counter/measurement stamp），
独立GrabIoState及可选SuspensionPrior。不启动driver，不造source→common identity。
真实共同坐标转换须审核外参/rig父关系提供；本轮无FIELD外参。
PHYSICAL/REPLAY/SIMULATED/SYNTHETIC分离；synthetic模型不能用于physical输入。

DDS depth1+最新帧槽，输入/工作callback分组，至多2线程；metadata同帧匹配不受topic乱序影响。
云/元数据/IO超时、重复、frame mismatch或完成时超deadline无效，
evidence age包含处理时长；保留source measurement/counter，不心跳刷新测量。
无输入发布LOST/NOT_CONFIGURED；模型或地图缺失不崩成伪READY。
shadow所有GrabState/companion/clearance为DEGRADED或NOT_CONFIGURED，
valid_for_control始终false；不发布SafetyPermit/AuthorizedCommand/ActuationRequest。
algorithm TRACKED不等于整机可下降，PREDICTED更不能授权。

## 8. 验收与现场计划

tests_static/test_phase2a_grab.py覆盖native coarse/fine、Huber、初值扰动、单侧/部分遮挡、
single-sensor covariance、开度冲突/TRANSITION、对称innovation、ESKF PSD/旋转、
lost/reconfirmation、摆动迟滞、双候选歧义、倾斜bottom、各向异性包络、凹pit、
触壁/越界/高于墙顶、timestamp smear及latest-wins。
tools/validate_phase2a_synthetic.py扫描noise/occlusion/outlier/sensor/span/运动/tilt/外参扰动，
报告pose/bottom/swing/clearance误差、TRACKED数量、false TRACKED与保守违规、分阶段P50/P95/MAX。
拒绝场景保留；INIT扰动/重捕获/墙状假物体另在单测验证，不声称扫遍现场条件。
目标P95<0.8*实测frame period；合成性能仅基线，生产数字不冻结。

Ubuntu CI分别normal和独立symlink-install构建，列新旧pkg executables，
真实启动installed preview/shadow/capture入口，注入synthetic检查TRACKED、几何、
input timeout、无控制topic；原PTP/Capture/RViz/mock及vendor包装继续验证。
Windows仅静态/纯native synthetic，不能声称ROS运行成功。
FIELD还需rig/CAD/真实开度、外参、地图、噪声/速度/遮挡bags、独立GT、
最坏延迟与owner参数冻结；现场/物理动作/完整循环全部NOT_RUN。
ObservedPitBoundary、PitSurfaceEstimator、ExpectedVolumeGraspPlanner、GrabContactEstimator
仅后续职责声明，不在本轮做料面/planner/contact/load/NDT/servo/Camera AI。

参考：[small_gicp作者说明](https://github.com/koide3/small_gicp/blob/master/docs/paper.md)
为局部配准/缓存工具背景；[Open3D鲁棒核说明](https://www.open3d.org/docs/latest/tutorial/t_pipelines/t_robust_kernel.html)
为鲁棒残差背景。本项目不引Open3D运行依赖。
附件其他2026论文未能完整核验，不据其性能数字制定本项目阈值。
