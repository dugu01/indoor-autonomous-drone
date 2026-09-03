# S2.5 v1.0.7 — remaining four-case correction

1. **NAV_HIGH_NOISE seed 4 lifecycle clock**: one `routeAffected && routeExists` transition entered `LIFECYCLE_REPLAN_BRAKE` without resetting `stateEntryTime`. The 90 s state timeout therefore measured time inherited from the previous state. v1.0.7 adds only `stateEntryTime=t` at that transition. No timeout or braking threshold changes.

2. **P3/P4/C1 fault phasing**: the 1.20 s brief perception faults previously used fixed wall-clock windows. Some seeds received the injected packets while not in a lifecycle state where perception freshness can transition to `MAP_DEGRADED_HOLD`, causing evidence-only failures despite safe goal completion. v1.0.7 keeps the same 1.20 s fault duration and hold requirement, but arms each brief qualification fault at the first eligible state after t>=18 s (`TRACK_OUTBOUND`, `TRACK_RTL`, `SCAN_HOLD`). Once armed, the fault runs continuously for 1.20 s even after it causes `MAP_DEGRADED_HOLD`.

Frozen S2.4-G and S2.3 parents remain unchanged.
