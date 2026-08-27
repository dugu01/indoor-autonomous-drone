# Changelog — Stage S2.2 v0.2

## Retained

- S2.1 remains frozen.
- Inflated occupancy-grid safety geometry.
- A* availability as an independent fallback.
- Fixed-altitude 2-D mission abstraction.

## Added

- D* Lite incremental repair and expansion counters.
- Dynamic-obstacle truth and noisy measurement simulation.
- Alpha-beta obstacle tracking.
- Finite-horizon velocity-obstacle candidate filtering.
- Dynamic object promotion after stopped persistence.
- Braking-distance speed restriction with delay.
- No-data hold and prolonged-dropout failsafe.
- Six-scenario MATLAB validator.

## Python pre-validation

The independent Python reference passed 30/30 runs: six scenarios over five random seeds.

## Claim boundary

OMR-IDS is the project integration name. Component algorithms remain attributed to their original authors and official projects.


## S2.2 v0.3 candidate

Added smooth dynamically feasible trajectory generation:

- seventh-order piecewise minimum-snap interpolation;
- C3 waypoint continuity;
- iterative velocity/acceleration/jerk time scaling;
- dense inflated-grid trajectory validation;
- safe stop-and-go polynomial fallback;
- feedforward + PD tracking;
- jerk-limited point-mass update;
- reference and replan continuity metrics;
- new `trajectory_time_rescale` validation scenario;
- docked six-panel dashboard saved under the `v0_3` result tree.

Python reference backtest: 55/55 passed. MATLAB runtime validation remains required on the user machine.
