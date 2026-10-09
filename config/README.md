# Configuration

All files in `sensors/`, `hardware/`, `perception/`, and `semantic/` are
templates. `NOT_CONFIGURED` and `null` are deliberate and must not be replaced
with guessed production values.

Runtime profiles define lifecycle behavior:

- `mapping.yaml`: map mutation contract is enabled, implementation not ready;
- `localization.yaml`: frozen map required and mutation disabled;
- `replay.yaml`: simulated-time/static replay contract;
- `mock.yaml`: fail-safe interface development.

Site-specific calibrated files should be versioned only after field review.
The dual-camera template keeps both auxiliary cameras disabled and all topic,
frame, intrinsic, and extrinsic identities `NOT_CONFIGURED` until calibration.

## ROS2 硬件模板

bringup 将整个 config 安装到包 share/config；config_root 可显式替换为场地绝对目录。
6052/6251 默认不写输出，未知host/unit无法连接；io_mapping 明确24DI并保留所有未知极性。
拉绳需全部协议+标定配置后才产生米制位置；仅寄存器探测不能使状态 VALID。
bench_io 只读；DO单脉冲必须独立 CLI + 配置门控 + 隔离/独占确认 + WDT/FSV证据。
