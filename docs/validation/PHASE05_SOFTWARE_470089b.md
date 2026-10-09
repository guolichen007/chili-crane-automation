# Phase 0.5 软件验证证据

EVIDENCE_SHA: 470089b9c20ea5fefc0b6a007c8eff43ffe2666d
BASE_SHA: 0f8d749085d7d9f9903047eb2522fd4580b57e27
BRANCH: codex/architecture-freeze-v1
DATE: 2026-10-09
SCOPE: Ubuntu22/Humble容器软件，不是原生现场Ubuntu/设备验收。

## 实际结果

WINDOWS_STATIC_STATUS: PASS
WINDOWS_TESTS: 101项，100通过，1项Linux flock跳过；Python3.11.9加3.10语法契约。
CI_STATIC_STATUS: PASS
CI_STATIC_TESTS: Python3.10，101项通过。
ROS2_BUILD_STATUS: PASS
ROS2_BUILD_PACKAGES: 7
ROS2_TEST_STATUS: PASS
ROS2_TESTS: 66 tests, 0 errors, 0 failures, 0 skipped
MOCK_SCENARIO_STATUS: PASS
SYNTHETIC_SCENARIOS: 27
MOCK_FAIL_CLOSED_STATUS: PASS

[静态CI与提交标题/公开路径门控](https://github.com/guolichen007/chili-crane-automation/actions/runs/37902205161)；
[Humble build/test/场景/mock及证据artifact](https://github.com/guolichen007/chili-crane-automation/actions/runs/37902205264)。

mock实际观察：无效授权、Executor仅OFF、硬件拒绝注入的energize、
STOP逻辑接收但未声称physical OFF、遥控BLOCKED、system/readiness全未就绪。
产物名ros2-humble-software-evidence，含精确SHA validation.log、result.yaml、
architecture-scenarios.json、mock-evidence.json和colcon测试结果。

## 修复历史与局限

首次云端编译发现旧task_transition_policy仍引用MANUAL_OVERRIDE；
已改ABORTED，并补C++中止任务不能恢复下降的回归及静态枚举引用检查。
这是软件接口冻结证据，不代表闭环控制、算法性能、停止距离或工业安全认证。
本文件只声明上述EVIDENCE_SHA；后续文档提交/修复需查看对应最新Actions，不能沿用旧PASS。

NATIVE_UBUNTU_STATUS: NOT_RUN
HARDWARE_STATUS: NOT_RUN
BAG_STATUS: NOT_RUN
FIELD_STATUS: NOT_RUN
MAIN_CHANGE: NONE
SERVER_BRANCH_PROTECTION_CHANGE: NONE
