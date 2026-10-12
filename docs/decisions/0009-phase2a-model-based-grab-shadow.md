# ADR0009：模型驱动Grab状态估计与shadow几何安全

状态：采用实现方向；尚未声明软件或现场验收。
基线72234843d5850947c5a2fd5e230516bc8fa64839；保留七包及ROS2 Humble边界。

Grab为已知机械体，聚类/PCA只产生候选；最终观测来自模型配准，经过几何、
可观测性、运动、开度一致性验证，再进入SE(3)误差状态滤波。
模型对称性使用等价旋转最小innovation；短时预测只能支持显示/重捕获，不能授权下降。
三层几何（详细、配准、安全）分离；TRANSITION使用OPEN/CLOSED安全并集及不确定性。
bottom是模型变换量，不取原始点最低值；保守界传播pose、外参、几何与scan模糊。
坑壁读取现有PitDefinition schema，精确/保守几何作为证据；ESDF留给Phase2B规划。

PTP五文件和vendor lock/patch冻结。Phase1C三项P1先修，额外验证symlink-install及旧新节点。
源类型PHYSICAL/REPLAY/SIMULATED/SYNTHETIC保持隔离；无FIELD外参不得宣称物理控制有效。
shadow不启动硬件、Safety、Executor或任务，不生成自动授权；参数未知就NOT_CONFIGURED。
不实现Pit料面、抓取规划、接触、载荷、NDT、X servo、Camera AI或TEASER++依赖。
后续工作接口仅在算法文档声明，不制造可运行的伪实现。
