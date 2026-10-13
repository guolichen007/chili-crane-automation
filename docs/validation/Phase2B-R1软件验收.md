# Phase2B-R1 FIELD-readiness 软件验收

日期：2026-10-13。BASE_SHA=ac0dec1d589d92ab8215d1a0f7b5207762bddbd4。
分支：codex/phase2b-field-readiness-r1；main、Phase2A/PTP/vendor/Geofence v2/V1冻结。
原Phase2B保持PASS_SOFTWARE_SHADOW_NOT_FIELD，本轮增量记录不覆盖旧验收。

## 当前证据边界

CODE_SHA=643008abc1b39e46a36327b31289decf658a706d；WINDOWS_STATUS=PASS_STATIC_NATIVE。
PHASE2B_R1_STATUS=PASS_SOFTWARE_SHADOW_NOT_FIELD；GITHUB_ACTIONS_STATUS=PASS_FOR_CODE_SHA。
[Static 38378081528](https://github.com/guolichen007/chili-crane-automation/actions/runs/38378081528)与
[ROS/vendor 38378081685](https://github.com/guolichen007/chili-crane-automation/actions/runs/38378081685)精确绑定CODE_SHA，均通过。
Ubuntu22/Humble normal/symlink七包、A/B installed shadow、旧双雷达/PTP/Phase1C/Capture/RViz/mock/vendor均实跑通过。
文档归档及验收脚本清理提交的最终HEAD仍需独立CI；不以此历史CODE_SHA自动代替最终HEAD验收。
Windows不能代替ROS构建/消息/CDR/现场验收；Ubuntu容器不等于现场原生主机/真实bag/硬件通过。

首代码406a1ae29926cbc096add87d2a2bdba207775346：Static [38375110250](https://github.com/guolichen007/chili-crane-automation/actions/runs/38375110250)通过，
[ROS 38375110164](https://github.com/guolichen007/chili-crane-automation/actions/runs/38375110164)失败；vendor通过、normal七包构建通过，
313包测试中R1独立pytest缺slam源路径而失败（包含CTest包装失败），没有执行后续installed验收。
修复仅R1测试自身源路径；不放松断言/核心/Phase2A门限。新SHA完整CI必须重新执行。
同时保留具体hard-reject reason，不再被无观测snapshot泛化为SURFACE_STALE；增加遮罩共时原因持久反证。

5af95cedab4763861fe7b02b2831f5948056c82f：[Static 38376530473](https://github.com/guolichen007/chili-crane-automation/actions/runs/38376530473)通过；
[ROS 38376530444](https://github.com/guolichen007/chili-crane-automation/actions/runs/38376530444)的normal七包构建、313包测试、
R1 installed CDR/performance、Phase2A installed、旧前置入口及vendor通过。
Phase2B installed harness仍按原高频debug同步假设断言，遇到1Hz debug初始空帧而失败，symlink后续未执行。
修复为独立等待非空observed/UNKNOWN debug，仍保留原非空断言，不修改1Hz设计或任何生产门禁。

Windows完整unittest：370项，366通过、4项无ROS环境跳过；仓库契约/格式通过。
旧21场景+64确定性序列×7帧通过，八类旧反证全部0；R1新增五类反证全部0。
R1专项16项（15通过、1缺ROS跳过），含真实ESDF拒绝减少精确检查，顺序/空坑/活动域/缓存反证。
初次真实ESDF fixture未产生拒绝（冻结可达域已排除近墙中心）；改用NON_PRODUCTION大kernel制造真实拒绝，
没有放松任何门禁或删除断言。此为提交前测试fixture修正，不是现场证据。
Windows四次workload样本P95：FAST_REJECT 0.60ms、FULL_VALID_PROPOSAL 175.94ms、DENSE_SURFACE 586.36ms。
最大JSON诊断快照395328bytes，不是DDS载荷；Ubuntu实际CDR见下节。数字仅本机有限样本，不是生产SLA。

Ubuntu构建前unittest：370项，367通过、3项未生成ROS消息跳过；normal与symlink各313项包测试，0错误/失败/跳过。
R1共时/lineage/jitter/校准链/外参拆分、末端mound/遍历不变、空坑、epsilon带排序、实际ESDF拒绝、L坑coverage、
稀疏消息往返/CDR与QoS反证通过。老八类计数和新五类计数全部0；21场景/64序列仍保留。
两个安装树分别验证实际snapshot世代发布、缓存复用、age增长且epoch不变、旧DDS世代不重算，
扰动ACK/旧观测失效/真实新帧恢复/地图错配/停止输入expiry均通过；A/B各三控制topic消息0。
软件artifact=11802029859，ZIP SHA256=4f887045b2220651346d0f3388b0a96a126fd704ee01100ca4b3121963f9b960；
vendor artifact=11801344656。仅通过Actions页面取artifact，临时下载凭证与完整ZIP不入Git。

## 实际CDR性能与容量（CODE_SHA，非生产SLA）

每workload八个样本，normal/symlink分开。cold类型支持初始化计入实际serialization，FAST_REJECT的MAX并不代表稳态。
配置/fixture/活动mask预计算不计入total；total计两帧，ingest阶段取末帧单次，不将各stage直接相加。

| workload | normal P50/P95/MAX ms | symlink P50/P95/MAX ms |
| --- | --- | --- |
| FAST_REJECT | 2.43 / 17.76 / 25.96 | 8.81 / 25.03 / 33.66 |
| FULL_VALID_PROPOSAL | 209.11 / 210.63 / 210.77 | 216.51 / 217.34 / 217.59 |
| DENSE_SURFACE | 696.09 / 707.15 / 711.40 | 699.81 / 701.84 / 701.93 |

完整三workload逐stage P50/P95/MAX及每样本input/active/observed/serialized cells/bytes、
scanned/shortlist/ESDF/exact计数在artifact的phase2b-r1-normal.json与phase2b-r1-symlink.json，以下摘要为normal P95。
0表示该拒绝路径未执行该stage，不代表无成本完成有效路径。

| stage | FAST_REJECT ms | FULL_VALID_PROPOSAL ms | DENSE_SURFACE ms |
| --- | --- | --- | --- |
| transform | 0 | 0.023 | 0.090 |
| grab mask | 0 | 14.412 | 57.127 |
| pit filtering | 0 | 13.009 | 52.000 |
| cell ingest | 0 | 8.218 | 38.488 |
| temporal fusion | 0 | 6.685 | 26.206 |
| height bounds | 0 | 1.009 | 3.902 |
| slope/roughness | 0.013 | 35.575 | 145.490 |
| snapshot serialization | 2.978 | 2.739 | 10.016 |
| DDS payload preparation | 13.415 | 2.117 | 4.189 |
| ESDF reject | 0 | 0.150 | 0.542 |
| exact candidate geometry | 0 | 68.312 | 146.873 |
| ranking | 0 | 0.133 | 0.152 |
| proposal serialization | 0.599 | 4.991 | 5.414 |

两安装树最大planning snapshot=109262bytes实际ROS2 CDR（dense observed964/active1024，短名单128）。
partial容量64/256/1024格分别传30/120/482 observed格，CDR分别4654/14734/55278bytes；UNKNOWN保持隐式。
这些格大小/规模是NON_PRODUCTION fixture，不是100000格默认发布能力或实际网络吞吐SLA。
CI镜像tag为ros:humble-ros-base-jammy，image digest未捕获；软件包版本随artifact归档。
FIELD的实际CPU、分辨率、频率、丢包与调度负载仍需实测，不承诺这里的P95可直接部署。

## 交付清理

全绿CODE_SHA的退出日志显示own-group SIGINT与launch转发叠加，会二次中断destroy_node。
收尾仅改为先给launch PID一次SIGINT、超时才对本任务拥有的session group发送SIGTERM，仍有界清理，
不改变运行算法/门禁/namespace隔离；最终HEAD需要完整CI验证，不能仅凭Windows宣称清理成功。

cdc2e7ece085f1e99f1cce7f2328c1c056a9392d的[Static 38380150855](https://github.com/guolichen007/chili-crane-automation/actions/runs/38380150855)与vendor通过；
[ROS 38380150802](https://github.com/guolichen007/chili-crane-automation/actions/runs/38380150802)的normal七包/313测试/前置入口通过，
在installed旧世代重放的缓存暂停中，共享runner延迟使原0.7秒输入TTL到期，runtime正确fail-closed，测试等待cache命中超时。
因此最后HEAD未通过，不能以历史643全绿替代。修复installed专用NON_PRODUCTION profile：
frame_ttl=2s、surface_ttl=2.5s、window=3s、最多2帧，隔离缓存暂停与合法窗口淘汰；
仍保留停止输入后的expiry/epoch不续期断言。原native fixture的0.7s与旧八类反证不变，FIELD模板仍null，Phase2A门限不变。
另加publication_gate，在计算结束后用当前源时间与monotonic residence age再次检查surface/独立context TTL；
计算开始fresh、完成stale必须空提议，保留原epoch。Windows专项增加该反证，最终修复SHA完整CI另核对。

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
