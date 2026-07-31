# State and Safety Contract

## Validity

Public state messages use:

```text
UNKNOWN
NOT_CONFIGURED
STALE
DEGRADED
VALID
```

Every message also carries a reason. `VALID` means the message-specific
contract is satisfied; it does not mean all automatic motion is safe.

## Normalized state

Adapters translate vendor data into SI units, explicit timestamps, validity,
faults, limits, calibration identity, and reason strings. Core modules never
read vendor registers.

`GrabIoState` is the raw electrical boundary for grab limits, drive state and
mode. The perception stack may consume it but exclusively owns the fused
`GrabState` pose, geometry, height and opening classification.
`ControlBoardState` separately carries E-stop, power, mode, heartbeat and board
fault evidence.

## Control intent

`ControlIntent` expresses a semantic request such as move X, move Y, raise,
lower, open, close, or stop. It is not permission and must not be sent directly
to electrical outputs.

## Safety permit

`SafetyPermit` contains false-by-default permissions:

- X movement;
- Y movement;
- lower;
- raise;
- grab open;
- grab close;
- unload;
- automatic task.

The actuator adapter requires both a fresh intent and a fresh permit. A permit
heartbeat does not make stale input evidence fresh.

## Conflict examples

- both grab limits active -> `CONFLICT`, block grab motion;
- LiDAR grab state disagrees with a limit -> `DEGRADED`/`CONFLICT`;
- invalid localization -> block geometry-dependent automatic X/Y;
- stale grab track -> block lowering;
- load missing -> never classify contact/load success;
- manual mode or e-stop -> block autonomous output;
- communication timeout -> release/stop outputs according to board contract.
