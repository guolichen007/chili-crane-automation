# chili_crane_perception

Phase2A-R1增加不变刚性core TRANSITION、预测模型表面门控、连通料面/IO反证，
修正MID_SCAN epoch、normalized observability及带身份的clearance defined语义。
固定坑Geofence由control独立纯策略实现，不反向clamp感知，不变成自动授权。

Phase2A已实现模型Grab候选/原生GICP/Huber精化/验证/ESKF、bottom/swing、
各向异性包络与pit几何的纯算法及ROS shadow。不驱动设备，不产生自动授权。
地池表面/抓取规划/接触/载荷仍只保留接口，不在此阶段实施。
CAD、FIELD外参和政策未知时NOT_CONFIGURED；TRACKED只是算法状态，不能代替控制有效性。
源码见src/chili_crane_perception，文档见docs/algorithms/Phase2A抓斗状态估计与几何安全.md。

运行目标 Ubuntu22.04 / ROS2 Humble / Python3.10 / GCC11 / C++17 / ament。
Windows只做静态检查；详见 docs/PROJECT_CONTEXT.md 和硬件台架手册。
