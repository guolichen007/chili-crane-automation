# Open Hardware and Field Questions

Resolve each item with an owner, evidence source, and date. Do not replace
unknowns with production constants for convenience.

## Runtime and compute

- final edge-server model and GPU requirement;
- Ubuntu/ROS baseline decision;
- network topology, bandwidth, time synchronization, and clock source;
- data retention and bag/video storage budget.

## LiDAR and cameras

- exact left/right LiDAR models, drivers, topics, frames, rates, and timestamp
  source;
- usable range/accuracy/reflectivity on real chili from high to low fill;
- blind zones through the full grab Z range;
- final mounting locations and 6DoF extrinsics;
- camera models, interfaces, lighting, cleaning, and evidence retention.

## X bridge axis

- servo/encoder brand and protocol;
- raw units, absolute/relative behavior, direction, scale, offset, and homing;
- update rate, position validity, motion/fault bits, limits, and stop behavior;
- independent surveyed references and drift/slip expectations.

## Y trolley axis

- draw-wire model, range, signal type, resolution, update rate, and fault modes;
- cable routing and mechanical protection;
- motor contactor/VFD mode, speed control, braking, inertia, and jog support;
- left/right limits, home, running, and fault signals.

## Z hoist

- motor/VFD/contactor and brake interface;
- upper/lower limits and safe-height definition;
- stopping distance/latency for normal, jog, and emergency behavior;
- behavior when grab tracking is stale/occluded;
- whether any independent redundant Z evidence is required by safety review.

## Grab and load

- grab CAD/dimensions for open, closed, and transition envelopes;
- open/closed limit electrical characteristics and conflict behavior;
- motor current/running/fault availability;
- load/tension sensor type, mounting, range, units, calibration, and dynamics;
- empty/contact/loaded evidence definitions and false-positive tolerance.

## Control board and electrical layer

- board model, protocol, DI/DO/AI/AO/RS485 capacity;
- command acknowledgement, sequence ID, watchdog, and heartbeat semantics;
- output release/stop behavior on timeout or reboot;
- direction interlocks, brake sequence, limit enforcement, and manual/auto mode;
- e-stop/power/fault wiring and responsibility boundary.

## Survey, map, and operations

- DWG coordinate origin/units/layers and field survey accuracy;
- track origin/direction/range and pit polygons/elevations;
- unloading station ROI and presence/full sensing method;
- safe wait pose, no-go regions, and maintenance zones;
- two-crane collision/scheduling responsibility;
- access to real chili and repeated production grasp cycles.
