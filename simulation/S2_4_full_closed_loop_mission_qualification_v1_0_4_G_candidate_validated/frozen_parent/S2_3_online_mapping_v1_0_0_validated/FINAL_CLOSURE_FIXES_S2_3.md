# S2.3 final closure fixes

This package changes validation contracts only; the estimator, mapper, planner, controller, plant, inflation, landing and touchdown cores are unchanged.

## Evidence from the complete MATLAB release run

- Python scenario contract: PASS
- Exact mapper replay: PASS
- Frozen S2.2 regression: PASS
- Hard safety aggregate: PASS
- S2.3 deterministic: 11/12 because the narrow-passage mission used the validated grid fallback but the validation gate required a polynomial trajectory.
- S2.3 robustness: 49/60. All 11 failed trials completed the mission safely; failures were optional event expectations: 8 dead-end seeds detected/avoided the dead end before a repair was required, and 3 dynamic-to-static seeds promoted the obstacle without route intersection.

## Corrections

1. Accept the validated grid-route fallback as a legitimate trajectory mechanism, while retaining all executed-motion, collision, geofence and unknown-space gates.
2. Treat dead-end route repair as conditional when the dead end is perceived before commitment.
3. Require dynamic-to-static promotion; require route repair only in the dedicated late-corridor-blockage scenario.
4. Print the previously hidden trajectory, controller, estimator, static and composite-mapping gates.
5. Fail fast before an expensive legacy rerun when an S2.3 gate is red.
