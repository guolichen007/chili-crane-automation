# Phase 1A Ubuntu 只读算法部署交接

> 历史 Phase1A 入口。当前 Phase1B 传感器部署以
> [Phase1B 交接](PHASE1B_UBUNTU_BENCH.md)为准；旧时钟名称及融合启动说明不用于本轮。

## 范围与硬关闭

本手册面向 Ubuntu 22.04 / ROS2 Humble / Python 3.10 / GCC11。
Windows 已做软件检查，现场 ADAM/拉绳/ER1/bag/外参/运动验收全部未执行。
三种入口 physical_output_enabled=false、automatic_control_enabled=false，均无真实 DO writer。
`shadow_control` 是安全隔离的 mock 输出入口，不代表 SCAN/PLAN/MOVE/LOWER 算法已完成。
不要为了让 ready 变绿补假限位、假外参或假时钟。不要再测 DO。

脱敏设备事实见 [桌面收口](../hardware/20261010_桌面控制硬件联调收口.md) 和
[部署硬件基线](../hardware/20261010_Ubuntu算法部署硬件基线.md)。
原始调试上下文含凭据，禁止上传；本仓库只留参数、边界及历史验收范围。

## 1. 固定 SHA clean build（软件，不通信硬件）

确认分支 `codex/phase1a-algorithm-deployment-v1`，使用交付报告 OUTPUT_SHA，
不要用 main 替代，不执行 git pull 覆盖本地变化。预先按组织流程安装 ROS/rosdep/colcon。

```sh
cd /absolute/chili-crane-automation
git status --short
git rev-parse HEAD
python3 -m pip install -r requirements-dev.txt
. /opt/ros/humble/setup.sh
bash scripts/validation/ubuntu22_ros2_phase0_validate.sh \
  --workspace "$PWD" --evidence-dir /absolute/artifacts/software \
  --expected-sha <OUTPUT_SHA>
```

验证脚本将 build/install/log 放到仓库外，每次新目录，测试七包、mock 和双雷达 synthetic
ROS2 transport；不连接现场设备。随后 source 本次证据目录内 `install/setup.sh`。
实际 apt 包版本归档；Humble CI 镜像目前是 floating distro tag，不宣称完整二进制锁定。

## 2. 只读 ADAM 和拉绳（由现场人员执行）

确认控制网 enp4s0=10.0.0.10/24、ADAM-CONTROL、无 gateway/DNS/never-default。
办公 eno1 与 ADAM 控制网不变。6052 10.0.0.1、6251 10.0.0.2，Unit1 TCP502。

```sh
sh scripts/validation/phase1_preflight.sh
sh scripts/validation/phase1_adam_readonly_smoke.sh
sh scripts/validation/phase1_pullwire_smoke.sh
ros2 launch chili_crane_bringup algorithm_dev.launch.py config_root:="$PWD/config"
```

前两设备只读 FC01；拉绳只读 FC03。失败返回 INVALID/UNKNOWN，不输出“全部 DI=0”。
正常 profile 不发送 STOP/DO OFF 写指令；6052 断网后的 OFF 由已验收 WDT/FSV 负责。
“STOP”软件策略不能被解释成脚本执行了写入，更不能替代硬急停和电气互锁。

串口稳定 by-id、9600 8N1、slave1、reg0×2；modulus409600。
18.162 counts/mm、零点409585 和 max_delta2000 均为开发基线，最终安装重标。
跳变/间隙异常会锁存连续性故障，核实原因并建立新会话；禁止靠反复重启掩盖故障。
Y 开发标定不允许 Y_AUTO_READY；X 固定模拟位于 `/crane_01/sim/servo_state`。
未接 DI 模拟位于 `/crane_01/sim/io_observations`，不注入 physical topic。
真实站点 24DI 绑定仍待确认；6251 桌面 16/16 回路通过不等于语义点表已绑定。

另外两个入口：

```sh
ros2 launch chili_crane_bringup shadow_control.launch.py config_root:="$PWD/config"
ros2 launch chili_crane_bringup phase1_production.launch.py config_root:="$PWD/config"
```

production 禁止 simulation/replay/synthetic，仍硬关闭输出。旧 production.launch.py 为历史 mock。

## 3. 独立雷达网络与只读探测

```sh
sh scripts/network/create_lidar_profile.sh --dry-run
sh scripts/network/check_lidar_network.sh
sudo sh scripts/validation/phase1_er1_packet_probe.sh
```

dry-run 只打印建议，绝不自动改 NetworkManager；已有 profile 返回 NO_CHANGE_REQUIRED
或 PROFILE_REVIEW_REQUIRED，绝不建议重复创建。当前配置从 site 读取：
enp3s0 / SENSOR-NET / 192.168.1.102/24，无 gateway/DNS、never-default。
旧 .10 / SENSOR-LIDAR 已被 S3-FINAL 取代。不得改 eno1/enp4s0。
抓包需要 Linux CAP_NET_RAW，仅 enp3s0，只保存 IP/端口/长度/计数和最小签名判定，不保存 payload。
204=[7799,6688]、205=[6699,7788]；不能凭默认示例认定 MSOP/DIFOP。
资产 physical_side、端口角色、设备目标 IP 均须现场记录，未经确认 driver_enabled=false。

## 4. 官方 SDK 精确固定与端口确认后启动

上游：[RoboSense rslidar_sdk](https://github.com/RoboSense-LiDAR/rslidar_sdk)，BSD-3-Clause。
SDK、rs_driver、rslidar_msg 精确 SHA 及 reviewed XYZIRT 补丁见 `third_party/rslidar_sdk.lock.yaml`。
不 git pull main，不重写 UDP decoder，不复制 NDT/ROS1 节点。

```sh
sh scripts/setup/install_rslidar_sdk.sh --workspace /absolute/vendor-er1
colcon build --base-paths /absolute/vendor-er1/src \
  --build-base /absolute/vendor-er1/build --install-base /absolute/vendor-er1/install \
  --packages-select rslidar_msg rslidar_sdk --executor sequential
. /absolute/vendor-er1/install/setup.sh
```

安装器不安装系统包；需提前准备 git/gcc/g++/libyaml-cpp-dev/libpcap-dev/ROS2。
已有 vendor tree SHA 或补丁不符时拒绝，不 reset 用户文件。
现场确认后修改两个 er1 site YAML：port_roles=VALID、msop_port/difop_port、driver_enabled=true，
明确 clock_mode。HOST_RECEIVE 仅诊断时间，不等于时钟同步已验证。

```sh
python3 scripts/sensors/render_er1_config.py --site "$PWD/config/sites/crane_01" \
  --output /absolute/artifacts/er1-official.yaml
ros2 launch chili_crane_bringup phase1_lidar.launch.py config_root:="$PWD/config" \
  start_vendor:=true vendor_config:=/absolute/artifacts/er1-official.yaml
sh scripts/validation/phase1_er1_ros_smoke.sh
```

## 5. 先各自 raw RViz 与第一份 bag，再做外参

无外参时必须分别使用两个自身坐标系窗口，不能先发布 identity TF：

```sh
rviz2 -d src/chili_crane_bringup/rviz/phase1_er1_204_raw.rviz
rviz2 -d src/chili_crane_bringup/rviz/phase1_er1_205_raw.rviz
sh scripts/validation/phase1_record_bag.sh /absolute/artifacts/first-er1-bag PHYSICAL '固定行车双ER1首采' 30
```

record 需已有 hardware/pipeline publisher；本入口只启动 recorder 与 RunManifest。
bag 必须有双 normalized topic 和 RunManifest，缺数据返回失败并保留证据。
bag_manifest 保存 SHA256、size、duration、topic counts、git/config hash、scene、calibration。
capture_complete 仅表示最低 topic 有消息，不等于数据质量/实机/外参验收。
真实 db3/mcap/pcap 不进普通 Git，放 NAS/artifact。仅经脱敏审查后的 manifest 可入 Git。

完成真实外参后填两个 calibration 文件（VALID、id、source/target frame、proper rotation、m translation）。
根据现场时间同步与采样验证填 dual_lidar.yaml 的 pair delta、stale timeout、point/header tolerance、
clocks_synchronized、coverage_verified；不要照抄 synthetic fixture 的门限作为实机参数。

```sh
rviz2 -d src/chili_crane_bringup/rviz/phase1_dual_lidar.rviz
sh scripts/validation/phase1_dual_lidar_smoke.sh
```

现场单雷达失联、时间倒退、旧帧、覆盖不足必须保持降级且无自动运动。
后续在真实 bag 上开发 GrabTracker/WallClearance/PitSurfaceBuilder，当前只有 ROI 产品接口。

## OWNER / NEXT INPUTS

| 所属 | 仍缺事实 |
| --- | --- |
| 网络/ER1现场人员 | 专用网卡实际配置、设备目标主机、MSOP/DIFOP角色、左右安装位置 |
| 标定人员 | 两个真实外参、时间域/同步误差、每点时间单位/时间差、覆盖证据与 ROI |
| Y安装人员 | 最终零点、方向、量程、生产 counts/mm、最大速度/每采样跳变上限 |
| 控制/电气人员 | 24DI语义/极性、真实DO继电器点表、停车/限位/硬互锁验收 |
| X厂家 | 伺服协议、标定、健康/速度/控制 capability |
| 数据负责人 | 首份双ER1 bag与hash、采集场景、现场验收证据位置 |

以上不阻塞软件部署基础版；全部真实 motion 保持关闭。
