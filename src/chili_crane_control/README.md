# chili_crane_control

安全监督、请求/授权/执行仲裁与遥控模式策略，rclpy mock CSV和可靠VOLATILE QoS。所有自动许可false；厂商协议在hardware。

运行目标 Ubuntu22.04 / ROS2 Humble / Python3.10 / GCC11 / C++17 / ament。
Windows只做静态检查；详见 docs/PROJECT_CONTEXT.md 和硬件台架手册。

Phase0.5: contracts/execution/cycle/configuration均为ROS独立的纯策略模型；
executor_node是永不energize的ROS2 mock。X/G执行和生产Task/Cycle调度仍是接口桩。
真实readiness/许可/反馈必须集成并专项验收后才可讨论自动控制。
