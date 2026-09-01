# S2.4-F — Execution-Time Safety Revalidation Candidate v1.1.2

## 1. Sourced/inherited mechanisms confirmed by source audit
S2.3 already provides future-route newly-blocked-cell checking on the inflated planner grid,
brake/replan recovery, planning-time known-free stopping checks, perception-degraded hold, scan hold,
RTL, landing and failsafe. S2.4-E retained those paths. F therefore adds no new obstacle planner.

The frozen S2.3 helper `validate_known_free_stop_S2_3` is called by the planning path to accept a newly
planned segment. The inherited live supervisor instead watches newly blocked cells affecting the
future active route and uses the existing brake/replan state machine.

## 2. Actual request-level gap
E validates an exploration request immediately before execution but did not revalidate the
accepted request itself during `TRACK_OUTBOUND`. Its expiry, remaining request route, viewpoint/
hold support and retreat authority could therefore become stale even while generic S2.3 route
tracking remained active. E's admission validator also lacked full retreat-route geometry checking.

## 3. F execution authority
Each accepted exploration request is an execution lease. During active `TRACK_OUTBOUND`, F checks:
1. request schema/valid flag and expiry;
2. the remaining route from current estimated position;
3. current viewpoint executability;
4. local hold support;
5. current stopping reserve;
6. stored retreat geometry, with current-grid inherited A* refresh if required;
7. map-version change followed by current geometric revalidation.

The validated E request TTL remains exactly 1.0 s. If and only if every CURRENT check passes,
F renews `validUntil_s = tNow + requestValidity_s`. Expiry is tested first, so F8 cannot revive an
expired request.

## 4. v1.1.1 runtime finding and v1.1.2 stop-reserve correction
The user's v1.1.1 coupled MATLAB run showed three no-fault authority generations invalidated with
`KNOWN_FREE_STOP_INVALID`; none of the F2-F14 validation faults could inject first. The defect was
not the inherited planner or braking logic. F had incorrectly reused frozen
`validate_known_free_stop_S2_3` on every execution cycle.

That frozen helper is a planning-time terminal reserve check: it requires sufficient route length
for the stopping-distance model and applies an additional terminal footprint clearance test at the
route endpoint. Applying that complete terminal check at every intermediate sample can false-abort
an otherwise unchanged accepted exploration route.

v1.1.2 keeps the frozen planning gate unchanged and uses a runtime-specific reserve check:

`d_stop = v^2/(2 a_decel) + v t_delay + d_margin`.

- If the current remaining known-free route arc length is at least `d_stop`, stopping support passes.
  The route is checked on the already-inflated execution grid, so no extra vehicle-radius inflation
  is added.
- If the remaining route is shorter than `d_stop`, the shortfall must be covered by a known-free
  terminal overrun disk around the already validated viewpoint, and local hold support must remain
  valid.
- A dedicated negative regression verifies that a high-speed short route is rejected when this
  overrun support is unavailable. The gate was not weakened merely to obtain parity.

Invalid authority is revoked before future exploration motion, then uses inherited
`LIFECYCLE_REPLAN_BRAKE` and fresh `PLAN_OUTBOUND`. Repeated authority invalidations are bounded
by `maxAuthorityInvalidations = 3`.

## 5. Validation-only fault scheduling
Faults are injected only when:
- an exploration request has been accepted;
- that same authority generation is active;
- state is `TRACK_OUTBOUND`;
- configured acceptance delay and route-progress threshold have been reached.

F14 repeats once per new authority generation; all other injected cases are one event. F11 waits
for substantial progress and changes a laterally offset known-free cell in the traversed region,
so it does not intentionally corrupt the stored retreat route. F10 blocks a future route cell and
seals cells around the retreat goal without marking the goal/current cell itself unsafe.

Fault hooks use only derived planner-grid/map freshness/request data. They do not read truth.

## 6. Backtest strengthening after v1.1.1
The previous Python semantic model omitted stopping support and therefore could not catch the
actual MATLAB failure. v1.1.2 makes stopping reserve part of the independent backtest and adds:
- `R1`: reproduces the old planning-terminal-gate false abort while the runtime remaining-route
  reserve remains valid;
- `STOP`: verifies that genuinely insufficient stopping support is rejected;
- F1-F14 and scheduler semantics as before.

## 7. F15
Observed dynamic occupancy remains covered by inherited map/route revalidation. The predictive
S2.4 dynamic-risk sidecar is not connected to the live coupled execution interface, therefore F15
future moving-obstacle intersection is explicitly **NOT QUALIFIED / N/A**. No MADER/FASTER
optimizer is copied.

## 8. MATLAB qualification
From the package root:

```matlab
gate = run_validate_S2_4_F_all();
```
