# Phase 0 Architecture Review

## Review result

The bootstrap architecture is approved for implementation with unresolved
hardware and runtime choices isolated behind configuration/adapters.

The system is not approved for physical automatic motion.

## Inputs reviewed

- current project handoff documents;
- latest V2 project plan, its tables, and presentation;
- historical V1 plan and procurement spreadsheets;
- site and grab reference images;
- local NDT-SLAM-Warehouse at
  `42f921e91574ff0f29b91fc08cbade67c976aa36`;
- key NDT merger, registration, observability, EKF, pose-constraint, fitness,
  stationary-motion, relocalization, and map-lifecycle documentation/code.

## Decisions

1. Build a new chili repository; do not rename/copy the NDT workspace.
2. Keep six stable ROS packages.
3. Split localization, pit-surface, and grab-tracking point-cloud products.
4. Treat the servo as a normalized prior and NDT as gated spatial correction.
5. Treat CAD/DWG as an offline semantic-map source.
6. Keep mapping and production localization as explicit modes.
7. Track a known grab mechanism with open/closed geometry, not warehouse cargo.
8. Treat LiDAR as primary Z evidence and load/tension as auxiliary evidence.
9. Require typed, false-by-default safety permissions before control.
10. Start with mock adapters that remain `NOT_CONFIGURED`.
11. Keep ScanContext/global place recovery disabled until real data supports it.
12. Resolve Ubuntu/ROS baseline before claiming a build target.

## Direct reuse classification

### Extract with small adaptation

- NDT observability;
- NDT fitness circuit breaker;
- generic rail/body pose constraints.

### Extract after deliberate decoupling

- dual-LiDAR merger;
- registration cloud builder;
- EKF/stationary motion;
- relocalization;
- immutable map snapshot concepts.

### Algorithm reference only

- cargo OBB/bottom/temporal filters;
- hook load-state filters;
- map post-processing utilities.

### Excluded

- warehouse cargo public API;
- 14/17/18/30-35 safety protocol;
- warehouse-specific payload lifecycle and warning zones;
- continuous online map mutation as the production default;
- old topic names, sensor IDs, and paths.

## Open architecture risks

- ROS1 Noetic versus Ubuntu 22.04 procurement baseline;
- repetitive rail geometry and false global matches;
- unknown LiDAR coverage/reflectivity near the 10 m range;
- grab self-occlusion and open/closed envelope calibration;
- Z control latency when LiDAR is the primary position feedback;
- ordinary-motor stopping behavior and VFD availability;
- control-board watchdog/interlock semantics;
- accurate CAD-to-map survey and pit-wall margins;
- multi-crane collision responsibility;
- absence of real chili bags and repeated grasp evidence.

## Phase 0 completion gate

- repository structure and governance present;
- messages contain explicit validity/reason fields;
- semantic and hardware templates are visibly `NOT_CONFIGURED`;
- localization profile forbids map mutation;
- mock adapter is the only default adapter;
- safety output is false by default;
- static checks pass on Windows;
- Ubuntu/bag/field status remains `NOT_RUN`.
