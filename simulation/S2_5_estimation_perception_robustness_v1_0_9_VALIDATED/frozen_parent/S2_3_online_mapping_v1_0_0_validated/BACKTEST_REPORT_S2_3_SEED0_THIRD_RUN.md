# S2.3 UNKNOWN_ROOM_NOMINAL Seed-0 — Third-Run Cross-Check and Backtest

## Status

This report analyzes the MATLAB trial saved as
`S2_3_v1_0_0_candidate_trial_data(2).mat` against the exact cumulative
backtested candidate source. MATLAB was not executed during this cross-check.

## Recorded MATLAB outcome

The third run was a large improvement over the preceding run:

- final lifecycle state: `COMPLETE`;
- mission completion time: 98.76 s;
- takeoff, RTL, landing and disarming: completed;
- collision violations: 0;
- geofence violations: 0;
- unknown-space commitments: 0;
- truth-isolation audit: PASS;
- completed/planned map extensions: 2/2;
- all replans: 1;
- reference XY speed/acceleration/jerk: within hard limits;
- executed XY speed/acceleration/jerk: within configured executed limits.

The overall result remained FAIL because:

1. the nominal goal was declared unreachable at 70.34 s;
2. map false-free rate was 0.00675894, above the 0.005 limit;
3. occupied recall was 0.914254, below the 0.95 limit.

## Root cause 1 — grid inflation added an unconfigured cell

The final estimator-aware inflation radius was 0.602 m and the planning grid
resolution was 0.10 m. The previous implementation used:

```matlab
rCells = ceil(inflationRadius / resolution);
```

This converted 0.602 m to seven cells and therefore blocked grid nodes 0.70 m
from an occupied node. The nominal goal at `[5.3, 5.3]` is 0.70 m from the
known room walls. Its physical clearance exceeds the configured 0.602 m radius
by approximately 0.098 m, but the ceil-cell disk still marked it occupied.

The recorded map contained wall surface cells at `[5.3, 6.0]` and `[6.0, 5.3]`.
The ceil-cell disk inflated both directly onto the goal.

### Independent saved-map backtest

- old ceil-cell inflation: goal occupied;
- physical-radius centre test: goal free;
- physical-radius A* route from the recorded unreachable position: available;
- the route also remains available with persistent static latching enabled.

The correction uses physical distances between grid-node centres:

```text
hypot(dx * resolution, dy * resolution) <= inflationRadius
```

This preserves the configured clearance. It does not reduce the numerical
inflation radius or any safety threshold.

## Root cause 2 — static occupancy was eroded by later free rays

The endpoint-exclusion correction prevented a ray from clearing its own hit
voxel, but other noisy rays could later traverse the same quantised voxel and
apply repeated free log-odds updates. Consequently, fixed walls and obstacles
could have many physical endpoint hits while their final log odds became free.

Recorded final-map evidence:

- false-free voxels: 272;
- occupied observable voxels: 2,694;
- missed occupied voxels: 231;
- many missed voxels had repeated endpoint hits but negative final log odds.

This violates the S2.3 design rule that static occupancy must not be cleared by
isolated missed returns.

### Independent saved-map latch backtest

Using the recorded hit evidence to represent the result of a persistent static
latch produced:

| Metric | Recorded implementation | Latch backtest | Required |
|---|---:|---:|---:|
| False-free rate | 0.00675894 | 0.00447957 | <= 0.005 |
| Occupied recall | 0.914254 | 0.982554 | >= 0.95 |

The source correction adds a dedicated `staticOccupied` latch:

- repeated direct endpoint evidence latches static occupancy;
- dynamic-to-static promotion also sets the latch;
- ordinary free rays do not erase latched static voxels;
- planner `knownFree` is explicitly disjoint from static occupancy;
- the independent map validator uses the same persistent classification.

No map acceptance threshold was weakened.

## Corrected source gates

- cumulative static audit: PASS;
- truth-isolation static audit: PASS;
- MATLAB source sanity: 81 files PASS;
- independent mechanism tests: 19/19 PASS;
- recorded third-run cross-check: PASS;
- inherited S2.2 source immutability: PASS.

## Limitations

These results establish that the recorded failures are reproducible and that
the isolated corrections address those mechanisms. They do not prove the
coupled MATLAB mission will pass. The corrected cumulative candidate must be
rerun in MATLAB before any S2.3 validation claim.
