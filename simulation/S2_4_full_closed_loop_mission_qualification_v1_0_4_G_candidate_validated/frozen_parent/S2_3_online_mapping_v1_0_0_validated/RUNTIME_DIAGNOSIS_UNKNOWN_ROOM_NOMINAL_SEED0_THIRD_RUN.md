# Runtime Diagnosis — UNKNOWN_ROOM_NOMINAL Seed 0, Third Run

## State timeline

| Time (s) | State | Position summary |
|---:|---|---|
| 0.60 | ARM | Preflight passed |
| 1.10 | TAKEOFF | Launch initiated |
| 5.88 | INITIAL_HOVER | Nominal altitude reached |
| 8.92 | PLAN_OUTBOUND | First unknown-map plan |
| 30.08 | SCAN_HOLD | First frontier completed near `[5.19, 3.41]` |
| 32.70 | PLAN_OUTBOUND | Second frontier plan |
| 62.42 | SCAN_HOLD | Second frontier completed near `[4.60, 4.79]` |
| 65.04–70.32 | PLAN/SCAN cycles | Direct goal rejected by inflated goal cell |
| 70.34 | GOAL_UNREACHABLE | No reachable frontier after three scans |
| 71.34 | PLAN_RTL | Safe contingency initiated |
| 91.46 | LAND_HOVER | Landing area reached |
| 92.28 | LAND_DESCENT | Descent initiated |
| 98.16 | DISARM | Touchdown completed |
| 98.76 | COMPLETE | Lifecycle completed |

## Why the goal was rejected

The goal was raw known free in the final map. It became occupied only after
inflation. The nearest wall-surface occupied nodes were 0.70 m from the goal.
The configured radius was 0.602 m, but ceil-cell inflation treated it as 0.70 m.

A physical-radius inflation backtest on the saved map makes the goal free and
finds a route from the recorded position at the time of the unreachable event.

## Why map validation still failed

Static wall and obstacle voxels accumulated hits but could later receive enough
free-ray updates to cross the free threshold. The failure therefore remained
in static-evidence persistence, not in endpoint exclusion.

The corrected source latches repeated static evidence and makes free and static
classes disjoint.
