# RoboSense ER1 厂商驱动来源

SDK 使用官方 https://github.com/RoboSense-LiDAR/rslidar_sdk ，已核对支持 RSE1、ROS2 Humble、XYZIRT。精确 SDK、rs_driver 子模块和 rslidar_msg SHA 见 `third_party/rslidar_sdk.lock.yaml`；三者 LICENSE 均为 BSD-3-Clause。上游版权和许可保留在独立 vendor 工作区。

上游 CMake 固定 XYZI，使用仓库中的最小 `rslidar_xyzirt.patch` 改为 XYZIRT；没有重写 UDP decoder。安装工具核对 SHA，并记录补丁/lock SHA256 和实际系统依赖版本。安装和编译过程不启动 driver。

端口角色与 clock_mode 默认未确认。`render_er1_config.py` 拒绝猜 MSOP/DIFOP，只有现场确认后才输出官方配置。HOST_RECEIVE 只能用于 raw 数据诊断，不能证明双雷达已同步。

首次部署先安装 Ubuntu Jammy 的 libyaml-cpp-dev、libpcap-dev、ROS2 Humble 依赖，运行固定下载工具，再在独立工作区 colcon build。实际依赖版本随 CI/部署 evidence 记录，不能把发行版包名当作实机版本验收。
