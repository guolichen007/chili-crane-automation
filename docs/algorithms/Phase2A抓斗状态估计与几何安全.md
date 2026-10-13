# Phase2A 抓斗状态估计与几何安全

实现范围：模型驱动的纯算法与ROS2 shadow，不是FIELD外参或安全控制器。
基线72234843d5850947c5a2fd5e230516bc8fa64839；ADR0009；七包/Ubuntu22/Humble保持。
现场CAD、生产参数、pit实测、源到common-frame变换须owner确认。

## 1. 算法职责与模型

Grab是已知自身机构，不能把普通目标聚类centroid当控制位姿。
GeometricGrabCandidateProvider在LOST时做ROI、self-mask、voxel和有界欧氏聚类，输出多个候选。
R1的TRACKED/CANDIDATE使用预测模型表面门控，LOST_HOLD扩大门控加局部聚类，
LOST使用配置搜索体积/可选悬挂走廊。
强度仅保留，不成为未经验证的硬阈值。不同位置的合法模型匹配视为关联歧义，
不会悄悄挑一个锁定。PCA仅为LOST的yaw初值，不发布PCA最终位姿。

GrabGeometryModel独立保存详细sampled surface（可由CAD离线采样）、配准缓存
（两层small_gicp PointCloud/normal/covariance/KdTree）和安全convex vertices。
启动检查安全hull包含样本、底/顶参考和版本/provenance；未确认配置拒绝算法启动。
本轮不提供通用CAD mesh导入器，owner应先导出模型坐标的采样点与安全顶点。
OPEN/CLOSED分别建模；TRANSITION不估计精确开度，安全模型取二者并集，
R1允许配置不变刚性TRACKING_CORE，其有效且可观时接受core配准测量更新ESKF；
缺失/不可观仍仅短时预测并超时失锁。owner还须验证极限包络及中间机构行程。
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
R1取消负包围盒直径：内部精确非负最小距离，接触为0；跨界/外部数值0只是哨兵，
clearance_defined=false且geometry_violation=true，不定义signed penetration depth。
raw和conservative分开，后者不高于前者。
vertical envelope与[pit bottom,wall top]无重叠则collision_relevant=false，
仍报告有限horizontal clearance，不返回伪infinity。
GrabClearanceState为当前状态，GraspTarget.wall_clearance为规划目标，不混。
配置缺失/坐标不符为NOT_CONFIGURED，预测/弱支持为DEGRADED，过期为STALE，无法分类为UNKNOWN；risk保守true。
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
无输入发布LOST/STALE；模型或地图缺失为NOT_CONFIGURED，不崩成伪READY。
shadow所有GrabState/companion/clearance为DEGRADED、STALE、UNKNOWN或NOT_CONFIGURED，
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

## 9. 固定作业区强先验与动态Grab几何协同（R1）

```text
Y坑边方向限位 ──物理/驱动禁止链（待现场确认）
      │ ADAM DI只读证据
Y pull-wire ──限位窗口交叉验证──┐
X servo ──fresh/homed/calibrated─┤
                               ↓
                        axis-to-map标定
                               │
SemanticMap V1 pit ──版本绑定───┤
                               ↓
                   OperationalGeofence（悬挂点域）
                               │ 独立gate，不改Grab位姿
Grab pose/opening/swing/covariance/extrinsic/smear
                               ↓
                    Grab dynamic envelope/clearance
                               ↓
              必要条件shadow诊断（永不成为控制许可）
```

`config/safety/pit_operational_geofence.template.yaml` 只引用原地图和pit。
X/Y轴的方向、比例、零点、map原点以及标定身份显式配置；不能将servo X等同map.x。
参考点为SUSPENSION_POINT。圆形保守静态域内缩半径为所有static_margin加pit safety inset；
owner必须保证grab_horizontal_radius包含所有开度/姿态的静态最大水平占用。
Shapely圆弧内缩半径除以cos(pi/64)，抵消16段/象限弦近似的乐观误差；
参考点判定还核验对原pit boundary的精确距离。支持凹坑/MultiPolygon，空域拒绝。
未来可以采用方向各向异性Minkowski erosion，本轮圆形更保守但不放大真实可达域。

Y_NEGATIVE/POSITIVE_PIT_LIMIT 不等同整轨道机械限位。模板不绑定现有DI语义；
owner确认后才可绑定y_left_limit/y_right_limit。要求known_signals明确包含两个来源。
asserted且连续位置不在窗口→Y_POSITION_LIMIT_MISMATCH；窗口内未触发→Y_EXPECTED_LIMIT_NOT_ASSERTED；
双触发→Y_LIMIT_CONFLICT；未知/过期/标定不符阻断。窗口、NO/NC、断线和硬接状态均现场确认。
单侧禁止危险方向，反向经过独立几何步长检查才可输出shadow true；STOP仅去励磁。
direction_probe只诊断该有限路径，不代表制动/持续动作已安全。

限位停止线必须早于坑壁，考虑抓斗半径、最大摆幅、位置/标定误差、
传感/处理/通信/输出释放延迟、停车滑行、伺服following error与额外margin。
真实PHYSICAL位置资格需生产标定批准、FIELD axis-to-map、已验证stop/reaction能力。
未配置不产生任何几何控制资格。soft-limit支持/配置/实机验证仅能力契约，未控制驱动。

Geofence结果包含map/version/pit、X/Y标定、axis-to-map、geofence版本与规范JSON配置真实SHA256。
动态clearance绑定map/version/pit、Grab model/version、extrinsic ID/state/evidence。
独立pure FIELD外参资格函数要求FIELD_VALIDATED+PHYSICAL模型/来源；BENCH/CANDIDATE只支持shadow。
当前valid_for_control与production_control_eligible始终false，即使所有合成必要条件通过。

## 10. R1模型门控、观测与消息迁移

TRACKED距离带包含base band、pose uncertainty与scan mechanical smear，超过owner上限拒绝。
同时要求观测法线与预测模型法线一致、法向残差不超owner plane band，
每个CAD采样仅取最近测量，防止密集料面重复增加某模型点权重。
plane band是显式跟踪连续性约束：快速未建模运动会拒绝并走LOST_HOLD/LOST，不能无界扩大。
LOST_HOLD扩大带但仍有界、局部聚类；LOST重新有界多假设，不把料面吸附为Grab。

最后内点定义同时满足correspondence欧氏距离与配置法向残差上限；
离群点仍保留在inlier_ratio分母。P50/P95、observed-normal Hessian、空间支持和R均重算，
P95内点门槛不能单独证明质量；需内点率/extent/归一化信息/innovation联合检查。
归一化门禁用bbox support center及RMS特征长度，单位一致；物理Hessian协方差不做单位伪转换。
confidence=inlier_ratio只是匹配诊断比例，不是概率或安全证书。

OPEN primary同时运行有限CLOSED诊断，反向同理。primary失败且另一个高质量成立记录
GEOMETRY_IO_OPEN/CLOSED_CONFLICT，拒绝测量。TRANSITION安全包络永远OPEN/CLOSED union。
TRACKING_CORE必须是经CAD/实测证明的不变机构，不得用插值斗瓣或随意盒子替代真实模型。
core使用独立owner配置coarse/fine voxel尺度，不能将大抓斗尺度直接套给细小不变机构。
原生迭代可配置0..2次有界续算，但未收敛永远拒绝，绝不放宽阈值或篡改converged。

GrabState/GrabGeometry header保持LiDAR MID_SCAN；GrabClearance继承其epoch。
发布/处理延迟独立诊断。失效心跳沿用最后epoch（无测量时0），不能刷新测量时间。
clearance_defined=false时0值不可用于排序、距离门禁或当作“仅刚触壁”。
新增消息字段须全量重建ROS消费者；不兼容旧的负距离语义，详见ADR0010。

200 deterministic seeded sequences、连续flat/rough料面、接触/25%/50%穿入、
core长TRANSITION、单雷达/遮挡/噪声/断流/多候选/IO冲突与Geofence反证为有限软件验收。
近20万输入点测量latest-wins与Python tracemalloc retained allocation；
native heap/长时间现场内存与生产时延仍需Ubuntu实机验证，不冻结生产性能。

参考：[small_gicp作者说明](https://github.com/koide3/small_gicp/blob/master/docs/paper.md)
为局部配准/缓存工具背景；[Open3D鲁棒核说明](https://www.open3d.org/docs/latest/tutorial/t_pipelines/t_robust_kernel.html)
为鲁棒残差背景。本项目不引Open3D运行依赖。
附件其他2026论文未能完整核验，不据其性能数字制定本项目阈值。
