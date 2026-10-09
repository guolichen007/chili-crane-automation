# ROS2 迁移交接记录

INPUT_SHA: 355051277167d83797e0cf985b79d515c17f4fda
BRANCH: codex/ros2-humble-hardware-bench-v1
OUTPUT_SHA: 由最终Git提交及CI运行页面确定，文档不自引用提交SHA。
RUNTIME: Ubuntu22.04.x / Humble / Python3.10 / GCC11 / C++17
DI: 初始24DI，包括遥控器输入；实际完整点表OPEN。

本次交付：七包ament/rosidl、rclpy fail-closed、ROS2 launch/config安装、
ADAM6052/6251只读与隔离台架工具、拉绳RTU/校准、遥控模式仲裁、静态与云端软件验证入口。
本次不交付：NDT/PCL算法迁移、生产控制闭环、MES、真实硬件验收。

Windows检查在提交前执行。CI状态按最终精确SHA的实际Actions结果查看，
不可根据工作流文件存在推导PASS。云端build/test/mock不等于原生Ubuntu测试。
NATIVE_UBUNTU: NOT_RUN
BAG: NOT_RUN
LIVE_ADAM_DI: NOT_RUN
LIVE_ADAM_DO: NOT_RUN
LIVE_PULL_WIRE: NOT_RUN
FIELD: NOT_RUN

主要OPEN：24点端子与极性、遥控接收器接口、安全链/急停独立性、物理互斥、
ADAM地址/unit/固件/看门狗FSV证据、拉绳串口/寄存器/端序/比例/零点、
X伺服、双雷达设备/外参/时钟、地图测量、抓斗尺寸、停止距离和现场阈值。
