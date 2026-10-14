# Phase2C-R1.1 evidence contracts

BASE_SHA: `1c14ac89d6429a31fa1cfcd3253b9eb7feabe56a`.
All eight packages and consumers must rebuild generated messages at one exact
SHA. This additive wire change is not compatible with old generated types.
Phase2A, Phase2B statistics/planner, TaskCycles and control authority stay frozen.

## Payload and surface

PAYLOAD_CANDIDATE requires closure, committed interaction and observed raise.
PAYLOAD_CONFIRMED additionally requires clear of the frozen bound surface,
settled vertical motion, fresh calibrated load with valid frozen baseline,
settling window and the existing absolute/normalized delta thresholds.
Neither refresh nor updated clear participates in payload confirmation.

SurfaceRefreshEvidence describes a post-interaction generation, not global
graspability. `generation_ready` means a fresh stable post-clear snapshot in
the same session/lineage; `refresh_ready` additionally has at least one
qualified affected cell. Counts partition affected cells into refreshed and
UNKNOWN; observed_fraction is refreshed/affected. Unqualified affected cells
are published as UNKNOWN through a C adapter projection. Stored B statistics
and the previous cycle's frozen snapshot are not modified by the projection.
Invalidation removes all old affected samples; only point epochs strictly
later than interaction and bound-clear observation enable time can recover.

Updated clear requires refresh plus zero unknown affected cells and the
existing clearance margin. Its string state is UNKNOWN until that evidence
exists; the compatibility bool is false for UNKNOWN. Thus payload CONFIRMED,
refresh pending, updated clear UNKNOWN and next plan not ready is legal.

Only the existing B planner's fresh candidate hard gates admit a next target.
The compatibility `next_cycle_surface_ready` now represents that local
candidate evidence, never coverage alone. Binding requests must match a fresh
actual B proposal from bounded history; receive/residence bookkeeping cannot
change its measurement/source identity. Same-session next binding after an
interaction requires a new generation; stale generation proposals are rejected.

## Session and run lifecycle

ACTIVE_CYCLE is a binding whose original owner has not reported COMPLETE or
ABORTED. Producer-session or run swaps during it reject and latch abort required.
Fresh producer status after a terminal cycle clears C refresh/load state and
bounded proposal history, then requires full SCAN, a fresh B proposal and a
new cycle/binding. Generations are compared only within one producer session;
retired-session evidence cannot become current again. No automatic target swap.

BindingLedger belongs to one active run. Its configured maximum remains 1024.
The next same-run binding fails closed without eviction. A valid new run after
a terminal cycle gets a new ledger and load state. The old ledger is released;
RunManifest, events.jsonl and optional rosbag own audit history.

## Diagnostics, storage and launch

GraspInteractionState.gate_diagnostics_json reports every payload gate, motion
and load value, bound/refresh session-generation-counts, updated clear and
cycle/binding/source identity. Early rejection reports NOT_EVALUATED and null
values. Timer heartbeats cannot renew source epoch or persistence.

Recording requires explicit minimum_free_space_bytes and
global_recording_budget_bytes in addition to single-capture limits. FIELD
capacity remains null/NOT_CONFIGURED. STOP_ONLY is the supported default;
limits block starts or stop the owned process and emit STORAGE_LIMIT. Historical,
pinned, active and fault evidence is never automatically deleted. Deletion
policies are unsupported and fail closed. Limits are polling safeguards, not
filesystem reservations or field capacity certification.

RunManifest V3 distinguishes journal_enabled, rosbag_configured,
rosbag_enabled, rosbag_process_alive and recording_profile. Compatibility
recording_enabled is an alias for an alive rosbag process, never JSONL logging.
Process starts/stops/unexpected exits update the disk manifest and ROS status.

phase2c_shadow_stack.launch.py composes existing surface runtime (with C
interaction_config_file), B planner, C evidence and optional localhost console.
commissioning defaults false; no bench writer flags are passed. Production
launches have no commissioning dependency. Single-node debugging remains.

## Validation boundary

Native tests do not prove DDS or field behavior. The exact-base native
reproducer demonstrates updated_clear blocking otherwise passing payload
gates. Historical 0f8db3d installed evidence lacks load/motion gate samples:
its root cause remains OPEN, not guessed from the native reproduction.
Installed validation must separately report PAYLOAD_CHAIN,
SURFACE_REFRESH_CHAIN and NEXT_PLAN_CHAIN with diagnostics on any timeout.
