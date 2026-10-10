# 辣椒地池行车自动抓取框架

当前分支为 **Phase 1B Bench R1 / 台架接入前补丁**，基线为 8118f993。
网络配置以[传感器网络 S3-FINAL 收口](docs/hardware/20261010_传感器网络接入收口.md)为准：
SENSOR-NET、enp3s0、192.168.1.102/24，双 ER1 单播到本机；
相机地址 192.168.1.180。这些是用户回传网络证据，不是本轮实机采集。
入口：[R1 跟踪](docs/validation/PHASE1B_BENCH_R1_TRACKER.md)。

Phase 1B 承接 Phase 1A，
提供双 ER1 每点帧时间、中点配对、PTP 探测、一台海康辅助图像/时间与录包工具。
只做 timing-only，不做外参、空间融合、NDT、视觉 AI 或无人控制。
入口：[Ubuntu 交接](docs/deployment/PHASE1B_UBUNTU_BENCH.md)、
[时间合同](docs/api/PHASE1B_TIMEBASE_CONTRACT.md)、[验收跟踪](docs/validation/PHASE1B_TRACKER.md)。
运行基线：Ubuntu 22.04.x、ROS2 Humble、Python 3.10、C++17、GCC 11、ament/colcon。
ROS1 旧框架已由 ADR 0003 取代；NDT-SLAM-Warehouse 仅为只读算法参考。

## 当前设备与边界

- 双 3D 雷达、X 伺服位置先验、固定轨道/语义地图；算法待迁移。
- Y 小车以 RS485 Modbus RTU 拉绳位移为主位置反馈；Z 以雷达抓斗跟踪为主。
- ADAM-6052：8DI + 8DO；ADAM-6251：16DI。**初始合计 24DI，包含遥控器输入**。
- 24DI 是物理容量，不是已确认点表。遥控命令、模式、安全、限位、故障的
  分配及每点极性全部待现场确认，禁止照旧版“最低 8DI”实施。
- DO0..5 预留 Y左/右、Z上/下、G开/关；DO6/7 保留，实际继电器接线仍需确认。
- ADAM 是远程 I/O，不是安全控制器；原厂硬接线急停、限位、方向互锁、
  制动和遥控/自动物理互斥必须保留。责任边界为中间继电器干接点输出。

遥控输入用于观测、冲突诊断和自动控制仲裁，不把遥控动作转发成自动 DO。
AUTO→REMOTE 释放自动输出、作废旧指令；REMOTE→AUTO 必须静止、六路输出
已验证 OFF、模式/安全有效且无遥控冲突，重新接受任务。本版正常运行不产生有效自动许可。
Y/Z分别支持可调速与固定低速策略契约，现场能力及停车验收未确认，不能启用自动运动。

## Phase 0.5 架构

任务/抓取周期 → 意图 → 安全许可 → 授权 → ActionExecutor → ActuationRequest → 硬件。
新增会话/epoch/序号拒绝旧命令，STOP始终允许向OFF退化；每斗完成后重新扫描料面。
R1补充命令只接收一次的执行上下文、短输出租约、模式转换矩阵、独立Z上下 readiness；
24DI只维护物理点表，标定出处绑定sensor_id。见[ADR0005](docs/decisions/0005-phase05-r1-execution-lifecycle.md)。
当前包含 ADAM/拉绳只读适配、固定 SHA 的官方 ER1 驱动安装入口和双雷达核心；
不含 NDT、料面识别或抓斗跟踪算法，也未执行现场动作。
详见[接口契约](docs/api/PHASE05_CONTRACTS.md)与[验收矩阵](docs/api/phase05_acceptance.yaml)。

## 七包边界

| 包 | 职责 |
|---|---|
| chili_crane_msgs | ROS2 rosidl 消息 |
| chili_crane_core | 硬件无关状态、地图、几何、任务契约 |
| chili_crane_slam | 双雷达/轨道定位接口；NDT 待迁移 |
| chili_crane_perception | 地池表面、抓取目标、抓斗接口；算法待实现 |
| chili_crane_control | 安全、仲裁、任务与 QoS，不含设备寄存器 |
| chili_crane_hardware | ADAM、拉绳、MVS 相机时间适配、标准化状态、mock、隔离台架工具 |
| chili_crane_bringup | 安装后的配置与 launch.py 组合 |

## 检查与启动

Windows 只编辑、静态测试、Git；不在 Windows 声称 ROS 编译/实机成功。

```text
git diff --check
python tools/check_repo_contracts.py
python -m unittest discover -s tests_static -p "test_*.py"
```

Ubuntu 精确 SHA 验证见 [运行手册](docs/validation/UBUNTU_VALIDATION_RUNBOOK.md)。
构建后：

```bash
ros2 launch chili_crane_bringup mock_system.launch.py
ros2 launch chili_crane_bringup production.launch.py
ros2 launch chili_crane_bringup bench_io.launch.py config_root:=/absolute/site/config
```

production 当前也只是 fail-closed mock；bench_io 为只读采集，不启动 DO 写入器。
所有自动许可默认 false，未知参数保持 NOT_CONFIGURED。

## 交接与证据

对外/GPT审查先读[审查指南](docs/review/REVIEW_GUIDE.md)，贡献与推送遵循
[CONTRIBUTING](CONTRIBUTING.md)，已接受决策与OPEN项分开记录。
设备资产地址不自动映射左右或端口角色；相机地址已确认网络可达，
但 SDK 型号/serial/出图仍未确认，不进入控制闭环。

先读 [项目上下文](docs/PROJECT_CONTEXT.md)、[架构](docs/SYSTEM_ARCHITECTURE_AND_ROADMAP.md)、
[台架手册](docs/hardware/HARDWARE_BENCH_RUNBOOK.md)、
[启动交接](docs/CODEX_START_PROMPT.md)。
GitHub Actions 提供 Python 3.10 静态检查与 Humble 容器 build/test/mock；
只有实际成功的精确 SHA CI 才构成云端软件证据，不代表原生 Ubuntu、bag 或现场验收。
所有 LIVE_* 在未连接设备前保持 NOT_RUN。
Humble 支持窗口至 2027-05，后续升级需要独立 ADR。
本机维护的 LOCAL_PROJECT_CONTEXT.md 已被忽略，不上传 GitHub。

## Phase 1A 现场数据部署

先读[Ubuntu 只读部署交接](docs/deployment/PHASE1A_UBUNTU_READONLY.md)、
[双雷达数据合同](docs/api/PHASE1A_DUAL_LIDAR_CONTRACT.md)、
[实施和验证跟踪](docs/validation/PHASE1A_TRACKER.md)。

`config/sites/crane_01` 保存脱敏的实测设备参数，原始 `*.template.yaml` 不变。
三种运行入口为 `algorithm_dev.launch.py`、`shadow_control.launch.py` 和
`phase1_production.launch.py`；均保持 physical/automatic 输出关闭。
旧 `production.launch.py` 仍是历史 fail-closed mock，不能当作现场入口。
端口角色、外参、时钟/覆盖均未配置：允许 raw/health，禁止有效融合/ready。
禁止把软件 CI、合成消息或操作者声明的 bag source 当作现场验收。
