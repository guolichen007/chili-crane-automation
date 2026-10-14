# Phase2C-R1.2 current-cycle closeout

Base: `7389e88a4f45268b302320e0f805ca5da953c166`.
R1.1 payload, partial UNKNOWN, producer restart, per-run ledger and B hard gates
remain in force. Phase2A/B cores, SemanticMap V1, Geofence v2, TaskCycles
enumeration/ownership, vendor/PTP and control authorization are unchanged.
All eight packages must rebuild the extended LiftEvidence at the same SHA.

## Three independent facts

PAYLOAD_CONFIRMED retains R1.1 load/motion/bound-surface gates. It never
requires refresh, updated clear or a next candidate.

PIT_EXIT_CONFIRMED is a frozen historical event for one run/cycle/binding/digest
and lineage. Only RAISE can create it, using valid fresh measured grab geometry,
ordered binding/interaction/closure/raise evidence, bound clear, valid semantic
pit geometry for the configured source, bottom_lower strictly above wall/rim
plus pit margin, and bottom_lower at or above safe lift height. Updated surface
coverage and NEXT_PLAN_READY cannot create or block the event. Its geometry
provenance, original measurement stamp and counter are retained.

RAISE admission now requires PAYLOAD_CONFIRMED + PIT_EXIT_CONFIRMED. Subsequent
MOVE_TO_UNLOAD, VERIFY_CART, POSITION_UNLOAD, OPEN_GRAB and VERIFY_EMPTY consume
the historical exit while still requiring current geometry/IO/load/cart/zone
evidence. A stale/missing pit refresh or pit planning context cannot revoke an
already valid exit. Fresh conflicting lineage, measurement/source chronology,
static pit geometry provenance or illegal phase regression invalidate it and
fail closed. Active session/run swaps and abort also clear it; new binding/run
or terminal-session reset starts without an exit. No new TaskCycles state or
actuator authorization was added.

VERIFY_EMPTY admission requires UNLOAD_CONFIRMED, independently of next plan.
The original owner can COMPLETE and then next_cycle -> SCAN. Without a fresh
post-interaction B proposal the next binding still rejects. A terminal binding
can consume fresh refresh evidence at binding admission even if the next owner
has already entered SCAN; session/digest/source/epoch/generation checks and the
node's authoritative generation floor remain enforced. The existing B planner
and proposal history checks still decide local next-target eligibility.

CLEAR_OF_UPDATED_SURFACE remains independent material-surface evidence; its
UNKNOWN semantics and conservative unknown-cell gate are unchanged.

## Diagnostics and bounded storage

payload_delta_gate_pass means absolute + normalized load thresholds only.
payload_gate_pass is the full conjunction of closure, interaction, raise,
bound clear, vertical settled, settling window, fresh load, valid baseline,
allowed load calibration/source and delta gate. It matches PAYLOAD_CONFIRMED.
Diagnostics also expose pit_rim_clear, safe_lift_height, pit_exit_confirmed,
surface_refresh_ready, updated_clear and next_plan_ready. Unevaluated gates
remain explicitly null/NOT_EVALUATED.

storage_check_interval_sec must be finite and within [1,5] seconds. The FIELD
template remains null/NOT_CONFIGURED. Bag start always checks storage; active
fast ticks only check owned-process state, configuration and total duration.
Global/capture recursive scans and free-space checks run at the bounded
monotonic interval. More than 100000 root entries (including symlinks), scan
OSError, budget or free-space violations stop the owned recorder and retain
STORAGE_LIMIT. STOP_ONLY never deletes historical, pinned, fault or active
evidence. The polling interval does not reserve storage or certify capacity.

## Packaging and test budget

Bringup declares commissioning as an exec dependency because the optional
full shadow stack can launch it. Production launch still has no console node
or writer flags and commissioning defaults false. Runtime safety boundaries
remain false/zero.

phase2c_delivery's six unchanged integrated scenarios have a bounded CTest
TIMEOUT 180, using the supported Humble ament_cmake_pytest TIMEOUT argument.
The upstream source is
[ament_add_pytest_test](https://github.com/ament/ament_cmake/blob/humble/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake).
This changes the test runner budget only; installed payload/refresh waits,
algorithm thresholds and existing success assertions are retained. The new
phase2c_r12 CTest has TIMEOUT 180 and exercises the original owner-driven cycle.

Native and installed tests cover partial-current-cycle exit, stale refresh
after exit, unload using history, COMPLETE without next plan, rejection before
fresh B proposal and acceptance after real local recovery, exit provenance
conflict, aggregate diagnostics, scan frequency and package closure. Installed
validation includes an additional original TaskCycles-driven partial cycle;
the previous scenarios and their internal success conditions remain intact.
