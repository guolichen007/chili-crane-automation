# ADR 0003 ROS 2 Humble 与桌面硬件基线

- Status: `ACCEPTED`
- 日期：2026-10-09
- 替代：ADR 0001；ROS1 历史分支保留在 `355051277167d83797e0cf985b79d515c17f4fda`。

## 决策

当前开发与验收目标为原生 Ubuntu 22.04.x LTS（推荐 22.04.5）、
ROS 2 Humble、Python 3.10、GCC 11.x、C++17、ament/colcon。
七个包保留业务边界，新增 chili_crane_hardware 独占厂家协议、原始 I/O、
归一化映射与桌面测试工具。主动分支只维护 ROS2 实现。

24DI 从第一版即完整接入：ADAM-6052 的 8DI + ADAM-6251 的 16DI。
用途包含 MODE_AUTO、SAFETY_OK、限位、遥控器输入及扩展故障反馈。
V1.1 首批 8DI 是语义子集，不能理解成当前总容量仅为 8DI。
设备来源、通道、有效电平、遥控器完整点表仍需确认。

遥控器输入是观测/请求证据，不直接生成输出授权。厂家保持 REMOTE/AUTO
硬件互斥；AUTO 到 REMOTE 清除自动请求，后续真实执行器必须释放动作输出。
REMOTE 到 AUTO 必须停止、实际输出全部 OFF、模式和安全链证据有效，并接收
新的自动任务。旧授权不得跨模式或时间跳变重用。

## 实施边界

保留 ControlIntent -> SafetyPermit -> AuthorizedCommand -> adapter ->
CommandExecutionState。命令使用 RELIABLE/VOLATILE/KEEP_LAST，绑定身份、代次和
有效期；Phase 0 拒绝所有动作。bench 的有限脉冲是独立人工测试路径，
不接生产节点。真实行车、继电器后端、电机与 NDT/LiDAR 算法迁移留待后续。

ADAM 是远程 I/O，厂家硬件互锁、急停、限位、过载、抱闸持续有效。
Communication WDT/FSV 必须单独配置和断网验证，不能用软件 finally 保证
断网后的 OFF，也不能将 ADAM 称为安全 PLC。

## 生命周期

Humble 官方维护至 2027 年 5 月。需在此之前评估下一 ROS2 LTS/OS 升级、
设备兼容性和部署回滚；此次按已确认工控机 Ubuntu 22.04 推进。
参考：[Open Robotics 发布说明](https://www.openrobotics.org/blog/2022/5/24/ros-2-humble-hawksbill-release)。

Windows 仅作静态、纯 Python、格式和 Git 验证。云端 ROS2 build/test、原生
Ubuntu mock、设备桌面与现场验收分别记录精确 SHA，不能相互代替。
