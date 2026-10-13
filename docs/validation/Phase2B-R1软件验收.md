# Phase2B-R1 FIELD-readiness 软件验收

日期：2026-10-13。BASE_SHA=ac0dec1d589d92ab8215d1a0f7b5207762bddbd4。
分支：codex/phase2b-field-readiness-r1；main、Phase2A/PTP/vendor/Geofence v2/V1冻结。
原Phase2B保持PASS_SOFTWARE_SHADOW_NOT_FIELD，本轮增量记录不覆盖旧验收。

## 当前证据边界

CODE_SHA=PENDING_COMMIT；WINDOWS_STATUS=PASS_STATIC_NATIVE；GITHUB_ACTIONS_STATUS=NOT_RUN。
Ubuntu22/Humble normal/symlink七包、A/B installed shadow、旧双雷达/PTP/Phase1C/Capture/RViz/mock/vendor
须由同一精确SHA的CI实跑确认。Windows不能代替ROS构建/消息/CDR/现场验收。

首代码406a1ae29926cbc096add87d2a2bdba207775346：Static [38375110250](https://github.com/guolichen007/chili-crane-automation/actions/runs/38375110250)通过，
[ROS 38375110164](https://github.com/guolichen007/chili-crane-automation/actions/runs/38375110164)失败；vendor通过、normal七包构建通过，
313包测试中R1独立pytest缺slam源路径而失败（包含CTest包装失败），没有执行后续installed验收。
修复仅R1测试自身源路径；不放松断言/核心/Phase2A门限。新SHA完整CI必须重新执行。
同时保留具体hard-reject reason，不再被无观测snapshot泛化为SURFACE_STALE；增加遮罩共时原因持久反证。

Windows完整unittest：370项，366通过、4项无ROS环境跳过；仓库契约/格式通过。
旧21场景+64确定性序列×7帧通过，八类旧反证全部0；R1新增五类反证全部0。
R1专项16项（15通过、1缺ROS跳过），含真实ESDF拒绝减少精确检查，顺序/空坑/活动域/缓存反证。
初次真实ESDF fixture未产生拒绝（冻结可达域已排除近墙中心）；改用NON_PRODUCTION大kernel制造真实拒绝，
没有放松任何门禁或删除断言。此为提交前测试fixture修正，不是现场证据。
Windows四次workload样本P95：FAST_REJECT 0.60ms、FULL_VALID_PROPOSAL 175.94ms、DENSE_SURFACE 586.36ms。
最大JSON诊断快照395328bytes，不是DDS载荷；Ubuntu CDR数值待实跑。数字仅本机有限样本，不是生产SLA。

专项覆盖：lineage与逐帧T、窗口锚点drift、遮罩共时和独立外参、顺序不变有界搜索、
空坑/底部余量、epsilon带排序、真实ESDF提前拒绝、活动坑coverage、稀疏传输、缓存与QoS。
运行tools/validate_phase2b_r1.py保存逐stage原始统计；workloads分开，不将mixed P95包装为生产SLA。
Windows JSON diagnostic bytes不是DDS；CI installed才记录实际CDR字节。64/256/1024格均NON_PRODUCTION。
旧八类安全反证与新增五类计数必须零；实际ROS的旧世代/心跳/aging反证另列，不由纯算法测试替代。

## 兼容性与回退

PitSurfaceSnapshot OBSERVED_INDEXED_V2语义及status/proposal消息扩展为breaking变更，消费者全量重建，禁止混跑。
回退应部署独立的原ac0dec1软件组合，不把R1生产者接V1消费者；禁止强推或改写公开历史。
配置新增漂移/共时/物料门槛/epsilon/搜索预算，所有FIELD数值仍null/NOT_CONFIGURED。

## 现场与控制

FIELD_EXTRINSIC_STATUS=NOT_RUN
GRAB_FIELD_VALIDATION_STATUS=NOT_RUN
PIT_SURFACE_FIELD_STATUS=NOT_RUN
GRASP_PROPOSAL_FIELD_STATUS=NOT_RUN
MATERIAL_VOLUME_PROXY_FIELD_STATUS=NOT_RUN
AXIS_TO_MAP_CALIBRATION_STATUS=NOT_RUN
OPERATIONAL_GEOFENCE_FIELD_VALIDATION_STATUS=NOT_RUN
PHYSICAL_ACTUATION_STATUS=NOT_RUN
FULL_AUTO_CYCLE_STATUS=NOT_RUN

两个Phase2A组合仍POST_R1_FIELD_VALIDATION_REQUIRED：LOST+dense chili+wall clutter+multiple candidates；
TRANSITION core+real moving jaws+continuous material。不在此轮扩展Grab算法。
physical_output_enabled=false，automatic_control_enabled=false，camera_control_authority=false，valid_for_control=false。
没有SafetyPermit/AuthorizedCommand/ActuationRequest的发布授权；实际控制topic消息须为0。
