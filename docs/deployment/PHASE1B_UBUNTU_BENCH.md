# Ubuntu Phase 1B：双 ER1 与一台海康相机交接

## 1. 边界与前置条件

原生 Ubuntu22.04 / ROS2 Humble / Python3.10 / GCC11。按精确 SHA checkout
`codex/phase1b-sensor-timebase-v1`；不修改 main，不把台架当生产验收。
physical_output_enabled=false、automatic_control_enabled=false、camera_control_authority=false。
不接 DO，不做运动、NDT、外参、视觉 AI。外参保持 NOT_CONFIGURED。
Phase1B 启动强制 timing-only，即使错误提供 VALID 外参也不发 TF/merged。

先运行已有 Ubuntu 软件门禁；MVS 是外部可选 SDK，不自动安装。

```sh
python3 tools/check_repo_contracts.py
python3 -m unittest discover -s tests_static -p 'test_*.py'
sh scripts/setup/install_rslidar_sdk.sh --workspace /absolute/isolated-vendor-workspace
# 依照 Phase1A runbook 编译 vendor workspace 与七包，再 source 两个 install/setup.bash。
```

Windows 只能静态；GitHub Ubuntu 容器软件 build/test 仍不是本机硬件验收。

## 2. 先只读测网络与 PTP

```sh
sh scripts/time/check_ptp_capabilities.sh enp3s0
python3 tools/ptp_capabilities.py --interface enp3s0 --output /absolute/artifacts/nic-time.json
sh scripts/time/ptp_packet_probe.sh --interface enp3s0 --seconds 60 --output /absolute/artifacts/ptp-packets.json
sh scripts/time/ptp_dry_run.sh hardware UDPv4
```

AF_PACKET 需要现场管理员提供 CAP_NET_RAW 或人工选择特权终端；脚本不自动 sudo。
只输出 metadata，不保存完整数据包。ethertype 0x88f7、UDP319/320 都探测；报文计数
包含 Sync/Follow_Up/Delay_Req/Delay_Resp/Pdelay_Req/Pdelay_Resp/Announce。
PTP 是 IEEE1588；E2E/P2P 是 delay mechanism，不预先假设。配置 AUTO，可按实测改为
E2E/P2P 或 UDPv4/L2。示例 GM 模板也不是证明此主机会当选 GM，须记录实际 GM identity。

hardware 模板要求 ethtool -T 的 hardware transmit/receive/raw clock 与可用 PHC；
只有 PHC 或服务启动不算。software 模板仅无 PHC/VM 的经确认 fallback；不默认 -S。
linuxptp 候选命令只打印，不运行；owner 评审 chrony/timesyncd/PTP 冲突并在现场决定。
本轮不启用 systemd、不停止任一时间服务、不运行 phc2sys、不改系统时间。
参考：[linuxptp ptp4l 官方手册](https://www.linuxptp.org/documentation/ptp4l/)。

## 3. 配置两种时间模式（独立的现场 config 副本）

双 ER1 必须同模式/domain；port_roles、msop/difop、frame/topic 仍需现场确认。
SENSOR_PTP：use_lidar_clock=true，VALID + 真实证据 ID。
HOST_DERIVED：use_lidar_clock=false，PROVISIONAL，PTP_VERIFIED=false，domain 指向同一台主机时钟。
不要混用。不复用过去 SENSOR_SYNCHRONIZED/HOST_RECEIVE 名称。
dual_lidar.yaml 明确 config_state=VALID、MID_SCAN、配对差、stale、future、FIRST_POINT 门限；
这些门限当前 null，需测量，不从理论 100ms 填生产常量。clocks_synchronized 旧 bool 不再足够。

```sh
python3 scripts/sensors/render_er1_config.py --site /absolute/config/sites/crane_01 --output /absolute/artifacts/er1-driver.yaml
ros2 launch chili_crane_bringup phase1b_sensors.launch.py config_root:=/absolute/config start_vendor:=true vendor_config:=/absolute/artifacts/er1-driver.yaml
# 另一终端，传入 owner 选择的实验门限；变量未设则 shell 拒绝。
python3 scripts/sensors/dual_er1_timing_probe.py --seconds 120 --pair-delta "${PROBE_PAIR_DELTA:?}" --future-skew "${PROBE_FUTURE_SKEW:?}" --stale "${PROBE_STALE:?}" --output-dir /absolute/artifacts/dual-timing
```

生成 JSON/CSV/Markdown，分析 Hz、span、header/mid delta、overlap、receive latency、
重复/倒退/未来/丢帧。探测工具不会改 site config，探测统计不独自升级 VALID。

## 4. 海康一台基础图像和时间

现场从官方安装 MVS SDK，设置 MVS_PYTHON_PATH 到官方 `MvImport`，不向仓库复制 SDK。
运行身份必须精确匹配型号 MV-CS060-10GC 以及 serial/IP，禁止首次设备自动绑定。
保留 camera_control_authority=false；driver_enabled 默认 false。

```sh
export MVS_PYTHON_PATH=/absolute/MVS/Samples/64/Python/MvImport
sh scripts/camera/hik_ptp_probe --config /absolute/config/sites/crane_01/sensors/hik_01.yaml --output /absolute/artifacts/camera-nodes.json
# 已在相机配置低 fps raw8 后，可人工选择采样（不自动改时间单位）。
sh scripts/camera/hik_ptp_probe --config /absolute/config/sites/crane_01/sensors/hik_01.yaml --capture-frames --seconds 60 --output /absolute/artifacts/camera-timestamps.json
ros2 launch chili_crane_bringup phase1b_sensors.launch.py config_root:=/absolute/config start_camera:=true
```

默认低带宽台架上限 5fps，不是生产帧率；只支持 Mono8/Bayer8，不默认 BGR8。
相机原配置速率高或未知时明确拒绝启动，owner 先在 MVS 设低速，或显式 configure_camera=true
配置 rate/pixel format/ROI/binning。不要把发布端 throttle 当成降低网卡带宽。
冻结曝光仅在 owner 明确 freeze_exposure=true 和设置 exposure_time 后执行；不猜生产曝光。
记录 width/height/pixel/fps/fExposureTime/raw unit，以及 frame gap/lost packet。

读取 PTP 节点存在性、权限和值；SDK 不支持 access API 时 UNKNOWN，不猜 WRITEABLE。
`--force-slave-only` 是显式可选相机写入，默认不写；失败只报告 FORCE_UNSUPPORTED。
GevIEEE1588Status 必须实测 Slave，时间单位/epoch 还需官方文档、tick frequency 和
CLOCK_REALTIME 对照。填写证据 ID 与已测单位，否则 RECEIVE_ESTIMATE/DEGRADED。
本轮没有厂商 SDK 或实体相机，不能宣称 MVS 动态绑定/图像成功。
CameraInfo 未标定 K[0]=0，rolling shutter 未补偿，禁止拿它做精确动态融合。

## 5. 网络与 DDS 诊断

```sh
sh scripts/sensors/sensor_network_benchmark.sh --interface enp3s0 --output /absolute/artifacts/network-120s.json
printenv RMW_IMPLEMENTATION
ros2 topic hz /crane_01/lidar/er1_204/points
ros2 topic hz /crane_01/lidar/er1_205/points
```

网络工具 120s 只读，记录 NIC counters、ethtool -S、两 ER1 UDP rate、CPU/memory、
相机 diagnostics（未接时 NOT_CONNECTED）。不改 MTU/Jumbo/offload/ring。
对比 UDP、driver、RViz 与 topic hz 后再评估 DDS，不自动安装或切换 CycloneDDS。

## 6. 真数据录包

```sh
python3 tools/phase1b_manifest.py create --site /absolute/config/sites/crane_01 --source PHYSICAL --scene '室内台架静态传感器时间实验' --output /absolute/artifacts/new-run --ptp-snapshot /absolute/artifacts/ptp-packets.json --camera-snapshot /absolute/artifacts/camera-timestamps.json
sh scripts/validation/phase1b_record_bag.sh /absolute/artifacts/new-run 120
```

新目录必须在仓库外；拒绝覆盖已有目录/bag。采集前创建 manifest，记录精确 SHA/config hash、
两 LiDAR 时钟、PTP 接口及未知项、camera 请求/实际测量快照、RMW 与禁输出边界。
requested 与 measured 分离，服务状态不写成 sync success。录全 14 topics，不含 merged/TF。
finalize 对 db3/mcap 做 SHA256、逐 topic count、duration；缺任一 required topic 就 incomplete。
capture_complete 仅表示录包完整，field_acceptance 始终 NOT_RUN，需独立现场报告。

## 7. OWNER / NEXT INPUTS

| Owner | 需确认 |
| --- | --- |
| Ubuntu 网络负责人 | enp3s0 PHC/硬件能力、报文 transport、delay、domain、实际 GM、时间服务冲突 |
| ER1 现场人员 | UDP 端口角色、双雷达时钟模式/domain、源单位/epoch、future/stale/配对/first-point 门限与证据 |
| 相机负责人 | 完整 serial/IP、SDK 版本/API、PTP 实测状态、tick 与 host 单位/epoch、低 fps/raw format/ROI/曝光 |
| 验收负责人 | 室内静态场景说明、真实 bag、网络时间报告、生产时间验收计划 |

ER1_204/205_LIVE、PTP_LIVE、CAMERA_LIVE、REAL_SENSOR_BAG、EXTRINSIC、SPATIAL_MERGE
均 NOT_RUN，生产同步验收 DEFERRED_TO_FIELD。未知项不阻塞本轮软件准备，但阻止现场就绪声明。
