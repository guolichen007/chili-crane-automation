# Phase 0.5 验证与交接计划

BASE_SHA: 0f8d749085d7d9f9903047eb2522fd4580b57e27
BRANCH: codex/architecture-freeze-v1
OUTPUT_SHA: 以分支HEAD及Actions对应的40位SHA为准，不自引用本文件所在提交。

Windows：三项提交前检查、27项synthetic scenarios、check_delivery预推送检查。
云端：Python3.10静态、Ubuntu22/Humble容器colcon clean build/test，
ROS2 mock授权->Executor->OFF请求->硬件逻辑STOP接收，模式/readiness阻断观察。
新消息消费者必须clean rebuild；按同一OUTPUT_SHA复核日志和CI artifacts。
状态及计数以实际日志为准，不能根据此计划推导PASS。

此验证不含生产Executor闭环、真实Task/Cycle调度、实时安全认证或设备接线。
NATIVE_UBUNTU: NOT_RUN
HARDWARE: NOT_RUN
BAG: NOT_RUN
FIELD: NOT_RUN

15项合同的入口和范围见api/phase05_acceptance.yaml；
scope_status表示实现层次，验收结果需要精确SHA的实际证据。

OPEN：ER1左右/端口角色/驱动/外参/时钟/覆盖、CameraA完整IP、
24点表/极性/安全回路、Y/Z调速能力/停车/响应/安全速度/点动、
X厂家协议、Grab模型/料流/卸料区域/现场阈值。

按CONTRIBUTING正常SSH推送独立分支；不修改main、不强推、不发布production release。
维护者再决定PR、必需审查/CI与分支保护；本轮未改GitHub仓库设置。
