# chili_crane_core

硬件无关的状态、任务、语义地图与几何接口。ament INTERFACE C++17 公共头文件，gtest smoke。不含厂商协议。

R1 operational_geofence.hpp只定义SemanticMap版本引用和默认fail-closed的shadow结果。
不复制pit polygon，valid_for_control固定false；纯策略在control，动态Grab几何在perception。

运行目标 Ubuntu22.04 / ROS2 Humble / Python3.10 / GCC11 / C++17 / ament。
Windows只做静态检查；详见 docs/PROJECT_CONTEXT.md 和硬件台架手册。
