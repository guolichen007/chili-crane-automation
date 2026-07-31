# chili_crane_control

Phase 0 provides:

- a mock normalized hardware publisher;
- an optional servo CSV replay publisher;
- a safety supervisor that grants no permissions.

There is no real board, servo, trolley, hoist, grab, or load protocol in this
release. Future real adapters must preserve the normalized message contract and
the board must independently enforce direction interlocks, limits, timeouts,
heartbeat loss, and safe output release.

The raw grab electrical interface is `GrabIoState`; fused grab pose and height
remain perception outputs in `GrabState`. Control-board evidence has a separate
`ControlBoardState` contract.
