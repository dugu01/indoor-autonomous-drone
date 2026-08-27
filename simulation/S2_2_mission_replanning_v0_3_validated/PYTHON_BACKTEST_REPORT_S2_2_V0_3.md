# Stage S2.2 v0.3 Python Backtest Report

Overall: **55/55 PASS**

| Scenario | Runs | Pass | Worst speed | Worst accel | Worst jerk | Minimum clearance | Max C3 jump |
|---|---:|---:|---:|---:|---:|---:|---:|
| `forced_time_rescale` | 5 | 5 | 0.2868 | 0.0738 | 0.0348 | 0.5362 | 8.01e-17 |
| `midflight_replan` | 5 | 5 | 0.3367 | 0.6900 | 2.2015 | 0.5653 | 7.52e-15 |
| `planned_room_route` | 5 | 5 | 0.3398 | 0.1036 | 0.0580 | 0.5362 | 4.73e-16 |
| `random_room_route` | 30 | 30 | 0.3398 | 0.4547 | 2.1281 | 0.5099 | 2.93e-14 |
| `sharp_corner` | 5 | 5 | 0.3398 | 0.0824 | 0.0407 | 0.8000 | 1.78e-15 |
| `straight_route` | 5 | 5 | 0.3398 | 0.0412 | 0.0102 | 0.8000 | 0.00e+00 |

Limits:

- speed <= 0.35 m/s
- acceleration <= 0.7 m/s^2
- jerk <= 2.5 m/s^3
- static/wall centre clearance >= 0.502 m

The generator uses seventh-order polynomial segments, shared waypoint derivatives for C3 continuity, dense collision checking, iterative time scaling, and a zero-internal-velocity safe fallback when shape overshoot is detected.
