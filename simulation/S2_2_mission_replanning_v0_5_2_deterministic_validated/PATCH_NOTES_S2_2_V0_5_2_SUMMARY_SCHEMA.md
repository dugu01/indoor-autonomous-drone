# S2.2 v0.5.2 summary-schema compatibility patch

## Failure

All seven preserved v0.4 scenarios completed with `RESULT: PASS`, but
`run_S2_2_mission_replanning` then failed while writing the text summary:

`Unrecognized field name "maxEstimatorPositionErrorObservable_m".`

## Cause

The v0.5.2 lifecycle summary contains observability-loss diagnostics that do
not exist in the deliberately frozen v0.4 summary schema. The console printer
was guarded correctly, but the text-file writer accessed those fields
unconditionally after the scenario had already passed.

## Correction

`write_summary` now writes these fields only when
`lifecycleEnabled == true`. No planner, estimator, controller, safety gate,
scenario, or numerical threshold is changed.
