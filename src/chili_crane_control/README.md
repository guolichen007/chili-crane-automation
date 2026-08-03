# chili_crane_control

Phase 0 provides:

- a mock normalized hardware publisher;
- an optional servo CSV replay publisher;
- a safety supervisor that grants no permissions;
- a one-time authorization contract between requested intent and hardware.

There is no real board, servo, trolley, hoist, grab, or load protocol in this
release. Future real adapters must preserve the normalized message contract and
the board must independently enforce direction interlocks, limits, timeouts,
heartbeat loss, and safe output release.

The raw grab electrical interface is `GrabIoState`; fused grab pose and height
remain perception outputs in `GrabState`. Control-board evidence has a separate
`ControlBoardState` contract.

The adapter subscribes only to `control/authorized_command`, never raw
`control/requested_intent`. Phase 0 rejects every command and publishes a
`CommandExecutionState`; it does not toggle a physical output.

Servo CSV replay schedules samples with `time.monotonic()` and never waits on
ROS time. `stamp_policy:=mapped` maps source offsets onto the ROS time observed
at replay start; `stamp_policy:=source` preserves CSV source stamps. If
`/use_sim_time` has no clock yet, publication still progresses on the wall
schedule and evidence age is computed from the selected evidence timestamp
rather than refreshed by a timer heartbeat.
