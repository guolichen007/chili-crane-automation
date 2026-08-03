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
lower, open, close, or stop on `control/requested_intent`. It is not permission
and must never be consumed by a hardware adapter.

## One-time command authorization

The safety/authorization boundary creates an `AuthorizedCommand` only by
binding one requested `intent_id` to a specific `permit_id`, permit generation,
issue time, and expiry time. The hardware adapter consumes only
`control/authorized_command` and rejects commands that are invalid, stale,
expired, identity-incomplete, or not bound to a permit.

```text
Planner / task -> control/requested_intent
Safety gate    -> control/authorized_command
Adapter        -> control/execution_state
```

A prior permit cannot authorize a later intent by topic freshness alone. Phase
0 publishes only invalid/expired authorization records for integration testing
and never produces physical output.

## Safety permit

`SafetyPermit` contains false-by-default permissions:

- X positive and X negative movement independently;
- Y positive and Y negative movement independently;
- lower;
- raise;
- grab open;
- grab close;
- unload;
- automatic task.

Directional permissions allow a separately implemented safety policy to block
motion toward a hazard while preserving an explicitly evaluated escape
direction. A permit heartbeat does not make stale input evidence fresh.

## Conflict examples

- both grab limits active -> `CONFLICT`, block grab motion;
- LiDAR grab state disagrees with a limit -> `DEGRADED`/`CONFLICT`;
- invalid localization -> block geometry-dependent automatic X/Y;
- stale grab track -> block lowering;
- load missing -> never classify contact/load success;
- manual mode or e-stop -> block autonomous output;
- communication timeout -> release/stop outputs according to board contract.
