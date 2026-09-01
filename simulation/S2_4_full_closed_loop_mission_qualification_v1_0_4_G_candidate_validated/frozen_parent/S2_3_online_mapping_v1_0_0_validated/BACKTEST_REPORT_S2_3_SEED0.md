# Stage S2.3 seed-0 cumulative cross-check and backtest report

## Scope

This report was completed before issuing the next code package. It combines:

- full recorded-state inspection of the second MATLAB nominal trial;
- source-level data-flow cross-checks;
- independent synthetic mapper and lifecycle mechanism tests;
- inherited S2.2 file-hash verification;
- MATLAB textual/source sanity;
- truth-isolation audit.

It does not claim a MATLAB rerun of the corrected candidate.

## Recorded-state cross-check

| Finding | Recorded value |
|---|---:|
| GOAL_UNREACHABLE time | 48.98 s |
| Reported extensions | 12 |
| Maximum displacement before unreachable | 0.05695 m |
| Outbound tracking before unreachable | 1.54 s |
| Scan hold before unreachable | 34.58 s |
| Total replans | 2255 |
| Trajectories generated | 2243 |
| Brake/retry cycles | 53 |
| Landing descent start | 216.30 s |
| Nominal landing profile end | 221.80 s |
| Simulation horizon | 220.00 s |
| False-free rate | 0.0230145 |
| Occupied recall | 0.581212 |
| Promotion events | 11740 |
| Maximum reference speed | 0.320099 m/s |
| Maximum attitude error | 2.6554 deg |

The complete machine-readable cross-check is stored in
`RECORDED_TRACE_CROSSCHECK_S2_3_SEED0.json`.

## Independent mechanism backtests

Fifteen mechanism tests pass:

1. temporary occupancy is height-aware;
2. occupied endpoint voxel never receives a free update;
3. extension completion is distinct from frontier selection;
4. a far-away map change does not invalidate the active route;
5. an idle control cycle is not a rejected packet;
6. the launch footprint can qualify after the preflight scan;
7. the near-ground map layer is not scored as physical floor occupancy;
8. the old half-voxel method reproduces approximately 50% endpoint overlap;
9. sensor insertion uses timestamp-aligned pose interpolation;
10. preflight uses accepted-map freshness rather than exact packet arrival;
11. persistent promotion clamps once to occupied;
12. raycast geometry is consistent;
13. map-version change is not a progress proxy;
14. scan yaw is continuous and rate-reduced;
15. the strict trajectory adapter preserves the unchanged hard kinematic limits.

## Source and package gates

- Frozen S2.2 files hash-checked: **110 files, 0 missing, 0 changed**.
- Cumulative S2.3 static audit: **PASS**.
- Truth-isolation static audit: **PASS**.
- MATLAB source sanity: **81 MATLAB files, PASS**.
- Recorded failing-trace diagnosis replay: **PASS**.
- Python mechanism checks: **15/15 PASS**.

## Safety and threshold statement

No inherited S2.2 estimator, collision, clearance, tracking, speed,
acceleration, jerk, attitude, landing, touchdown or disarm threshold was
relaxed. The simulation horizon was not extended after the failure. The
geometric controller, multi-lane ESKF, D* Lite, A*, minimum-snap core,
emergency landing and touchdown latch remain inherited unchanged.

## Remaining gate

The corrected cumulative candidate must now be rerun in MATLAB for
`UNKNOWN_ROOM_NOMINAL`, seed 0. Only the coupled MATLAB run can determine
whether the interacting corrections produce mission completion and meet all
map, control, estimator and safety metrics.
