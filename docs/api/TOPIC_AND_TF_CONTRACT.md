# Topic and TF Contract

## Phase2B shadow additions

Under `/crane_01`: `planning/spatial_context` proves cloud-to-map identity; no implicit TF.
`lidar/merged_points` plus `perception/pit_surface_input_evidence` feed the surface node.
Existing GrabGeometrySafetyState/GrabClearanceState remain independent masking evidence.
`perception/pit_surface_snapshot` carries a bounded typed planning grid; status is summary only.
`perception/pit_surface_observed_debug` and `pit_surface_unknown_debug` are read-only visualizations.
`planning/grasp_proposal_shadow` is NOT GraspTarget or a control intent. All control flags remain false.
`planning/surface_disturbance_shadow` accepts synthetic/replay events only, not physical execution.
`perception/pit_surface_markers` and `planning/grasp_proposal_markers` label SHADOW/NOT FOR CONTROL.

## Namespace rule

All per-crane topics are relative and launched under `/crane_<id>`. Phase 1
uses `/crane_01`.

## Reserved topics

```text
hardware/servo_state
hardware/trolley_state
hardware/hoist_state
hardware/load_state
hardware/grab_io_state
hardware/control_board_state
hardware/remote_control_state
hardware/adam6052/raw_di
hardware/adam6251/raw_di
hardware/sensor_health
lidar/merged_points
lidar/diagnostics
localization/pose
perception/pit_surface
perception/grasp_target
perception/grab_state
perception/unloading_cart
safety/permit
control/requested_intent
control/authorized_command
control/execution_state
task/status
```

No first-party node publishes an un-namespaced global topic.

## Frame ownership

- semantic/map manager owns `map` and fixed semantic frames;
- localization owns `map -> crane_01/base`;
- trolley state owns or supports `crane_01/base -> crane_01/trolley`;
- grab tracking owns the measured `crane_01/grab` observation;
- sensor calibration owns fixed LiDAR/camera extrinsics.

Servo coordinates are measurements, not TF frames, until calibrated.

`hardware/grab_io_state` carries only raw limit/mode/current evidence.
`perception/grab_state` carries the fused grab pose, geometry, height and
opening classification. The hardware adapter must never fabricate the latter.

## Timestamp rule

Evidence age is based on the original sensor timestamp. A timer publication or
heartbeat must not refresh measurement evidence time.

## ROS2 通信契约

使用 rclpy / rosidl；时间字段为 builtin_interfaces/Time。
状态 QoS reliable + VOLATILE + depth=1 + 有限 lifespan；命令 depth=1、
有限 lifespan，同时校验 issue/expiry。禁止 transient-local 复用旧授权。
原始8/16DI必须带设备身份、原始极性、通信状态、采样计数与 evidence_age_sec。
24DI 的语义映射在 hardware 层完成；定时发布不得刷新原始测量年龄。

## Phase 0.5新增接口

system/mode、system/readiness、task/cycle_status、run/manifest、safety/event、safety/fault_event、
control/actuation_request；雷达reserved raw topics及registration/pit/grab/wall四产品见模板。
ActionExecutor订阅authorized_command，hardware仅订阅actuation_request。
相机保留左右image_raw/camera_info接口且disabled；A/B到左右映射未知。
新增消息契约未发布对应真实算法/录包/周期管理节点，不能根据topic预留推导实现。
