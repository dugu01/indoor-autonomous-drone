# Stage S2.2 v0.2 — Incremental and Dynamic Obstacle Replanning

This package upgrades the validated S2.2 v0.1 static/unknown-obstacle planner while keeping Stage S2.1 frozen.

## Added in v0.2

- D* Lite incremental repair after persistent occupancy changes.
- A* scratch-expansion benchmark and safety fallback.
- Alpha-beta moving-obstacle tracker with position noise.
- Finite-horizon velocity-obstacle candidate filter.
- Braking-distance speed cap including sensor/control delay.
- No-obstacle-data XY hold after 0.5 s and failsafe after 5 s.
- Promotion of a stopped persistent moving object into the static map.

## Run

```matlab
restoredefaultpath;
rehash toolboxcache;
cd('/Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2_2_mission_replanning');
addpath(pwd,'-begin');
clear functions; clear classes; close all; clc;

report = validate_S2_2(true);
```

Individual run:

```matlab
results = run_S2_2_mission_replanning(0,'dynamic_crossing_yield',true,false);
```

## Validation scenarios

1. `incremental_static_insert`
2. `dynamic_crossing_yield`
3. `dynamic_blocker_becomes_static`
4. `sensor_dropout_recover`
5. `sensor_dropout_failsafe`
6. `two_dynamic_crossings`

## Current boundary

This remains a 2-D mission/planning simulation at fixed altitude. Full S2.1 estimator coupling, 6-DOF vehicle dynamics, jerk-limited/minimum-snap trajectories, and EGO-Planner-style continuous trajectory optimization are later S2.2 work.
