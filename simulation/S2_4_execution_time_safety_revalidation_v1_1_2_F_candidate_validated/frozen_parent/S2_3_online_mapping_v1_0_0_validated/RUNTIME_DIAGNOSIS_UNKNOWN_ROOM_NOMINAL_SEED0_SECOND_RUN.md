# Runtime diagnosis — UNKNOWN_ROOM_NOMINAL seed 0, second MATLAB run

## Evidence

The diagnosis uses the uploaded MATLAB v7.3 trial file
`S2_3_v1_0_0_candidate_trial_data(1).mat` and the exact cumulative-corrected
candidate source that produced it. MATLAB was not rerun during this analysis.

## Recorded behaviour

- The preflight freshness correction worked: the system armed and completed takeoff.
- `GOAL_UNREACHABLE` occurred at 48.98 s.
- The software reported 12 map extensions, but maximum physical displacement
  from launch before that declaration was only 0.05695 m.
- Before `GOAL_UNREACHABLE`, the vehicle spent only 1.54 s in outbound tracking
  and 34.58 s in scan hold.
- The run generated 2255 replans, 2243 trajectories and 53 brake/retry cycles.
- Landing descent began at 216.30 s. The unchanged 5.50 s landing profile would
  nominally complete at 221.80 s, beyond the 220.00 s simulation horizon.
- Map false-free rate was 0.0230145 and occupied recall was 0.581212.
- The map recorded 11740 static-promotion events.
- Maximum reference speed was 0.320099 m/s against the unchanged 0.320000 m/s limit.
- Maximum attitude error was 2.6554 deg and was localized to scan-related states;
  TRACK_RTL remained below 1.40 deg.

## Root causes

1. **Planned extensions were counted as completed extensions.**
   The extension counter was incremented when a frontier target was selected,
   even if no frontier motion was completed.

2. **Map changes anywhere reset the active trajectory.**
   Any newly blocked inflated cell triggered repair. In the saved snapshots,
   213 of 219 intervals changed somewhere, while only 57 intersected the logged
   two-second reference corridor.

3. **Free-ray endpoint contamination.**
   Stopping a free ray half a voxel before a hit did not prevent nearest-node
   quantization from selecting the endpoint voxel. An independent one-million
   phase test reproduced overlap in 50.0119% of rays.

4. **Repeated weak static promotion.**
   A persistent candidate received only one static hit at promotion and then
   reset. A cell saturated at log-odds -4 required six repeated promotions to
   reach the occupied threshold.

5. **Generator/validator hard-limit mismatch.**
   The inherited generator accepted ratios through 1.0005, while the final
   validator correctly used the hard limit with only numerical tolerance.

6. **Scan-reference discontinuity and rate.**
   Each scan state restarted yaw from an absolute zero reference at 55 deg/s,
   causing repeated yaw-reference resets and concentrating attitude error in
   scan/degraded-hold states.

7. **Landing failure was secondary.**
   The recorded landing logic did not have enough remaining simulation time;
   there is no evidence from this run that the frozen S2.2 touchdown latch failed.

## Cumulative corrections

- Route repair is now conditioned on intersection between newly blocked,
  already-inflated cells and the active future trajectory or grid fallback.
- Planned and physically completed frontier extensions are logged separately.
- Completed extension progress is measured from actual goal-distance reduction.
- Map-version change alone is no longer treated as mission progress.
- Hit endpoint voxels are explicitly excluded from free-ray updates.
- Temporary occupancy is height-aware in 3-D.
- Persistent static promotion clamps a voxel above the occupied threshold once
  and records a unique promotion.
- Independent occupied recall uses repeated physical hit endpoints, not ordinary
  ray traversal into truth-occupied cells.
- A strict S2.3 trajectory adapter preserves the S2.2 generator but rejects or
  rescales any result above the unchanged speed, acceleration or jerk limits.
- Scan yaw begins from the retained current yaw and uses 35 deg/s, still giving
  approximately 91 deg of observation sweep over a 2.6 s scan hold.
- The 220 s horizon was not increased to hide the replan storm.

## Validation status

The failing mechanisms were reproduced from recorded MATLAB data and the
cumulative source passes static, truth-isolation and independent mechanism
backtests. The corrected package has not yet been run in MATLAB and is not a
validated S2.3 release.
