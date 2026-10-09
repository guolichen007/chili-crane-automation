# ADR 0005: Phase 0.5-R1 execution lifecycle repair

Status: Accepted for interface and synthetic/mock implementation only.
Base: fce3fa0aa49d4278b72a65b350741cd73f85cf4b
Review branch: codex/architecture-freeze-r1

## Context and bounded decision

The Phase0.5 review found seven lifecycle/contract gaps. Preserve ADR0004,
the seven packages and the Ubuntu22.04/Humble baseline; do not expand into
production control, device protocols, MES, SLAM or perception algorithms.

1. Bind fixed-slow strategy reset to a newly admitted command/epoch, never a target tick.
2. Admit a command once; retain an immutable ActiveExecutionContext and revalidate every tick.
   Separate command admission sequence from finite leased actuation-frame sequence.
3. Validate routing, direction, target and configured strategy before consuming sequence.
   Keep one typed SafetyPermit direction mapper for all eight directions and STOP release.
4. Enforce a SystemMode transition matrix, physically verified SAFE_IDLE and explicit
   reset to SELF_CHECK/SAFE_IDLE only. Activation always requires a new task.
5. Evaluate Z raise and lower separately. No tracking/jam evidence means no blind raise.
6. Distinguish CANCELLED/ABORTED from FAILED; retain only SafetyEvent with typed domains.
7. Derive logical DI from the single physical24 table and bind calibration provenance
   to sensor identity and extrinsic/intrinsic versions.

## Evidence and remaining boundaries

Regression entry: src/chili_crane_control/test/test_architecture.py, R1Regressions.
It runs both through tests_static and ament pytest, using synthetic-only parameters.
See PHASE05_CONTRACTS section9 for lifecycle, migration and lease semantics.
Pure lease expiry produces logical OFF, not verified physical OFF.
The ROS executor still only publishes OFF; the hardware mock rejects all ON frames.
Trust setup, real polling deadlines, disconnect WDT/FSV, algorithms, bag and field
validation remain NOT_CONFIGURED or NOT_RUN.

CI delivery ranges now use event bases; first creation has no prior tip and checks
only HEAD's title plus all tracked public paths. Full first-push commit range must
still be checked locally with an explicit base. Push/PR updates check their event range.
CI archives apt versions and ROS/toolchain metadata; image digest/dependency lock
remains OPEN. This improves traceability but is not full environment reproducibility.

## Breaking changes

Clean-rebuild all ROS consumers. ActuationRequest replaces sequence with
command_sequence/actuation_sequence; CommandExecutionState gains two terminal states;
FaultEvent is removed; CalibrationRef replaces unkeyed manifest calibration IDs.
Migrate DI templates to v4 and manifest templates to v2; parallel DI bindings are rejected.
No automatic migration of guessed field assignments is allowed.
