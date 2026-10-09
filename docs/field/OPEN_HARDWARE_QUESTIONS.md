# Open Hardware and Field Questions

Resolve each item with an owner, evidence source, and date. Do not replace
unknowns with production constants for convenience.

## Runtime and compute

- final edge-server model and GPU requirement;
- native Ubuntu 22.04 / Humble deployment and later support-window migration;
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

## ADAM, remote controller and electrical layer

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

## 当前24DI点表（必须厂家确认）

物理容量已确认：6052 DI0..7 + 6251 DI0..15；包含遥控器输入。
须逐点记录 device/channel/端子/线号/信号名/常开常闭/raw电平/invert/断线表现。
核实遥控接收器是否提供独立干接点、方向/停止/就绪信号，哪些语义不存在，
X遥控方向是否独立于原伺服接口；不得假设24通道能同时覆盖全部可选语义。

确认中间继电器隔离、电流/电压、原厂接线责任、模式先断后合、六输出OFF读回、
ADAM 固件与 FSV 全LOW、WDT实际超时和其他TCP客户端影响。
确认 IP/unit ID、拉绳 /dev/serial/by-id、串口/地址/寄存器/端序/比例/零点/方向/范围。
全量点表未确认前 I/O配置保持 NOT_CONFIGURED，台架输出门控保持关闭。
