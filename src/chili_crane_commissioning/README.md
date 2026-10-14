# 辣椒行车调试与运行监控台 V1

独立第8包；production不得依赖或默认启动。stdlib Web+rclpy，无云网站/重型GUI依赖。
仅软件shadow。模拟输入固定在commissioning/simulation/remote_control_state，不是物理授权。

## Ubuntu启动

先完成Humble build并source install/setup.bash。使用自己验收过的exact SHA与独立run_id：

```sh
ros2 run chili_crane_commissioning commissioning_console.py \
  --config config/commissioning/console.yaml \
  --git-sha "$(git rev-parse HEAD)" \
  --crane-namespace crane_01 --host 127.0.0.1 --port 8080 \
  --runs-directory runs --recording-config config/recording/rosbag2_profile.yaml
```

打印LOCAL_URL后本机打开。启动参数默认不读取/写入ADAM。
非loopback监听需--allow-nonlocal及明确allowed-host；armed bench禁止非loopback。
仅UI预览可--headless-preview，明确ROS_NOT_CONNECTED，禁止物理台架输出和bag启动，不作为ROS/现场验收。
八页：总览、设备、task/cycle、2A/2B/2C证据、模拟遥控、隔离DO、事件时间线、记录。
pointer按下/释放/cancel/失焦/页面隐藏/断连lease到期均release，多个方向拒绝；STOP只释放模拟输入。

## 隔离台架（当前NOT_RUN）

DO0 Y左 / DO1 Y右 / DO2 Z上 / DO3 Z下 / DO4 G开 / DO5 G关，DO6/7 reserved，X未配置。
页面默认disabled，HTTP请求不能arm gate、改设备地址或调用任意coil。
仅在真实隔离、其他客户端停止、FSV/WDT/safe OFF证据已实测后由owner配置bench文件及显式CLI flags：
--bench-config、--enable-isolated-bench-output、--confirm-isolated-bench、--confirm-exclusive-device-session。
这不启用production自动控制；全局runtime flags仍false，台架是独立有限门禁。
严格1ms < duration <=1000ms，默认500ms；exclusive session+crash latch，写前/异常/结束OFF与readback。
网络失联可能无法证明OFF；设备WDT必须真实验证，未获OFF保留恢复锁。此软件无“保证电机安全”的声明。
人工万用表值关联bench_test_id，operator reported与软件readback分开，绝不自动升级FIELD。
本轮测试只用SOURCE_SYNTHETIC假transport，BENCH_DO_STATUS=NOT_RUN。

## 记录与容量

runs/run_id含manifest.json、events/operator_actions/bench_measurements.jsonl、rosbag、reports。
run_id安全路径、创建不覆盖、JSONL append-only，wall/ROS/monotonic和完整身份/SHA/hash/source。
HEADLESS的ROS timestamp为null；ROS接通后使用真实ROS clock，不冒充传感器measurement epoch。
ROS bag默认NOT_CONFIGURED，state_only与event_debug分开；启用必须声明storage与分包/总容量/总时长。
总容量/总时长到限停止本工具进程，不自动删除证据。原始points仅event_debug且独立批准，默认不录。
run目录/git本机上下文不上传；记录不含会话token，设备事实由配置hash绑定，owner负责备份/隐私。
