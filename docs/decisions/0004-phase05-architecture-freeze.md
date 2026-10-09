# ADR 0004: Phase 0.5 Architecture Freeze

Status: `ACCEPTED`
Date: 2026-10-09
Base: `0f8d749085d7d9f9903047eb2522fd4580b57e27`
Scope: interfaces and mock policies only; no physical motion.

## Decisions

- Retain seven ROS2 packages and the Ubuntu22/Humble baseline from ADR 0003.
- Insert ActionExecutor between AuthorizedCommand and ActuationRequest;
  hardware consumes actuation requests, never planning intents or raw authorization.
- Select VARIABLE_SPEED/FIXED_SLOW separately for Y and Z; unknown capability blocks motion.
- FIXED_SLOW requires verified speed, reaction time and stop distance; no guessed values
  or implicit pulse jogging. Predicted-only/lost grab evidence blocks Z lowering.
- STOP/de-energize dominates expired/missing permits; it never authorizes energization.
- Session identity changes on restart; epoch changes on ownership transitions and new tasks;
  monotonic sequence rejects duplicate/reordered/old-session commands.
- Separate SystemMode from TaskState; MANUAL_OVERRIDE is removed from task semantics,
  numeric task value21 becomes ABORTED. Interfaces are breaking changes, rebuild all consumers.
- Run -> Task -> repeated Cycle; each successful cycle or authorized retry requires a new scan.
- Physical DI inventory is exactly24 channels; logical capability names are independent.
- Report explicit readiness, typed safety/fault events, evidence metadata and run provenance.
- Unloading is validated against a safe zone, not a single precision coordinate.
- Cameras remain auxiliary and disabled for control; no automatic single-LiDAR dangerous lowering.

## Configuration and limits

Reported ER1 endpoints belong in asset inventory, not algorithm source code.
Port roles, left/right installation and calibration remain NOT_CONFIGURED.
Camera A reported_ip=192.168.180 is NEEDS_CONFIRMATION; do not repair it by assumption.
Public inventory is a reported asset record, not connection authority.
No hardware, bags or field acceptance is implied by this ADR or mock tests.
Production remains fail-closed. Real controllers and safety evidence require later field review.

## Review impact

Changes are reviewable commits with Conventional Commit titles, PR template,
exact-SHA validation and a contract acceptance matrix. No direct main push,
force push, history rewrite or unverified production-release claim.
