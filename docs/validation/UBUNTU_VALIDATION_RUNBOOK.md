# Ubuntu / ROS2 精确 SHA 验证

当前基线：Ubuntu 22.04.x、ROS2 Humble、Python 3.10、GCC 11、C++17、
ament/colcon。ADR 0003 取代 ADR 0001，旧 ubuntu20_phase0_validate.sh
仅保留历史复现，不适用于当前分支。

## 执行顺序

1. SSH 拉取目标分支，检出交接 SHA，核对 HEAD，要求工作区干净。
2. 记录原生系统版本、ROS/Python/GCC、依赖来源，安装 colcon/rosdep、
   Python YAML/jsonschema、ROS2 消息和测试依赖。
3. 使用外部证据目录，不污染源码，不复用既有 build/install。
4. 运行下方脚本；脚本先检查系统/精确 SHA，再静态检查、rosdep、
   colcon build/test/test-result、mock launch 和消息观察。
5. 检查 result.yaml、日志和 mock_result.json，只记录实际完成的 PASS。
6. bag、硬件、现场另行记录，不将 mock 当实机。小修复提交新 SHA 后重新验证。

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
bash scripts/validation/ubuntu22_ros2_phase0_validate.sh \
  --workspace "$PWD" \
  --evidence-dir "$PWD/../chili-crane-evidence" \
  --expected-sha "$(git rev-parse HEAD)"
```

GitHub Actions 的 Humble 容器使用同一脚本：这是云端软件验证，
不是原生 Ubuntu、传感器、台架或工厂验收。

## mock 预期

所有 SafetyPermit 动作许可 false；6个基础 mock 硬件状态及遥控状态 NOT_CONFIGURED；
safety_ok_known/e_stop_known=false；请求仅得到无效授权和OFF请求；STOP逻辑接收不等于实物OFF。
观察system/mode、system/readiness及actuation_request，确认自动enable=false。
同一脚本额外执行全部synthetic fault scenarios（含R1生命周期/租约回归），
证据写入architecture-scenarios.json，数量以实际日志为准。
CSV 回放节点用于输入契约验证，不使整机 READY。
production/mapping/localization 当前均不启动真实控制输出。

## 分层记录

记录 INPUT_SHA、OUTPUT_SHA、BRANCH、OS/ROS/Python/GCC、证据路径；
WINDOWS_STATIC、CI_BUILD、CI_TEST、CI_MOCK、NATIVE_UBUNTU、
BAG、LIVE_ADAM_DI、LIVE_ADAM_DO、LIVE_PULL_WIRE、FIELD 分别报告。
未执行是 NOT_RUN，不是 PASS。失败保留日志和失败原因。
