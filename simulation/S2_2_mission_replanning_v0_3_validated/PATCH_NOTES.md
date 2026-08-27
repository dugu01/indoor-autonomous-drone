# S2.2 v0.3 cumulative patch — mode-aware tracking validation

This patch is based on `S2_2_v0_3_patch_all_scenarios`.

## Root cause

`maxTrackingError_m` mixed nominal polynomial tracking error with intentional deviation while the dynamic collision-avoidance layer overrode the trajectory tracker. The two dynamic-crossing scenarios therefore failed despite reaching the goal safely.

## Correction

- Added an explicit `REJOIN` hybrid state.
- Paused the trajectory clock during dynamic avoidance, hold, and rejoin.
- Evaluated the tracking pass only while the reference is locked in `TRACK`.
- Preserved all-mode deviation as `maxSafetyOverrideDeviation_m`.
- Added `rejoinCount` and separate plot/console metrics.
- No clearance, speed, acceleration, jerk, continuity, or tracking threshold was reduced.
