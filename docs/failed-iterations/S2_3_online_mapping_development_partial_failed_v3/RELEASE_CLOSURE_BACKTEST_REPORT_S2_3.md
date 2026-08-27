# S2.3 release-closing backtest report

The revised deterministic geometries were checked on a 0.10 m grid using the
unchanged 0.602 m physical inflation radius and 8-connected A* with diagonal
corner-cut prevention.

| Scenario | Truth-map result | Minimum centre clearance |
|---|---:|---:|
| Occluded obstacle | Feasible | 0.632 m |
| Unknown narrow passage | Feasible | 0.750 m |
| Goal requires scan | Feasible | 0.632 m |
| Hidden obstacle before insertion | Feasible | — |
| Hidden obstacle after insertion | Alternate route feasible | 0.632 m |
| Unreachable goal | No route, as intended | — |

The pre-insertion hidden-obstacle route intersects the delayed obstacle's
inflated footprint, so the scenario can exercise genuine active-route repair.

This is a mechanism-level feasibility test only. Coupled MATLAB validation is
still required.
