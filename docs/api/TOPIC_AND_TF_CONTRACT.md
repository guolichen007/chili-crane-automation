# Topic and TF Contract

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
hardware/sensor_health
lidar/merged_points
lidar/diagnostics
localization/pose
perception/pit_surface
perception/grasp_target
perception/grab_state
perception/unloading_cart
safety/permit
control/intent
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
