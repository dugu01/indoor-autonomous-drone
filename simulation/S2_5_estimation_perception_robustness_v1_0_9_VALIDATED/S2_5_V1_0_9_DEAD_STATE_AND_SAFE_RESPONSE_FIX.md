# S2.5 v1.0.9 — NAV dead-state + degraded-perception response correction

This candidate is based on the user-executed v1.0.8 remaining-four failure and the subsequent NAV internal pending-state trace.

## Runtime evidence that drives this change

NAV_HIGH_NOISE seed 4 entered its final `LIFECYCLE_REPLAN_BRAKE` at 44.660 s and remained there until the unchanged 90 s state timeout at 134.680 s. The internal trace showed `pendingReplan=false` and `pendingExplorationReplan=false` for all 4501 brake samples, while brake readiness and retry readiness were true for 4500/4501 samples. Therefore the failure was a lifecycle dead state, not a velocity/acceleration threshold problem.

Source tracing showed the sequence: a pending replan brake is interrupted by `NAV_DEGRADED_HOLD`; the NAV hold intentionally cancels `pendingReplan`; recovery then blindly restored `navigationResumeState='LIFECYCLE_REPLAN_BRAKE'`. v1.0.9 does not preserve the stale pending work. If NAV recovery interrupts a replan brake, it returns to `PLAN_OUTBOUND` or `PLAN_RTL` according to the saved pending resume leg and requires a fresh current-state/current-map plan.

For PERCEPTION_DUAL_BRIEF seed 2, all 43 stale samples occurred while the system was already in `LIFECYCLE_REPLAN_BRAKE`, with goal and execution safety PASS. v1.0.9 therefore records one S2.5 safe degraded-perception response episode when stale perception occurs while already in this zero-translation safety brake. It does not force a redundant `MAP_DEGRADED_HOLD` transition. The existing `perceptionHoldCount` remains unchanged and the prolonged perception-loss fail-safe case still requires an actual perception hold before emergency behavior.

## Safety invariants unchanged

No change to estimator thresholds, `replanBrakeSpeed_mps`, `replanBrakeAccel_mps2`, `missionStateTimeout_s`, perception age/hold/failsafe thresholds, 0.602 m inflation, unknown-as-occupied semantics, route freshness, trajectory limits, stopping reserve, authority invalidation bound, scan budget, map-extension bound, collision/geofence checks, or truth isolation.

## Required runtime sequence

Run the remaining four first:

```matlab
setenv('S2_5_WORKERS','4');
report = run_validate_S2_5_v1_0_9_preflight();
```

Do not launch the full 71-mission matrix unless this reports PASS 4/4.
