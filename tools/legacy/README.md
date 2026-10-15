# LEGACY_R1_ARCHITECTURE

`validate_phase2c_r12_ros.py` 保存 R1.2 validator 源码，用于法证，不属于 R2 acceptance。
重现其 ROS 运行必须使用精确历史提交 `7f8378d9e670c4145836264e268ce3a500a438a7`
的工具路径、消息和 full stack；不要把本目录源代码用于 R2 安装验收。
历史测试可用 CHILI_RUN_LEGACY_R1_ARCHITECTURE=1 显式启用；历史 ROS budget 合同只适用于历史提交。
历史 PASS/FAIL 保留；R2 运行 `tools/validate_phase2c_ros.py` 的 snapshot-cycle probe/suite。
