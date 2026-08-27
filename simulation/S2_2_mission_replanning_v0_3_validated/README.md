# Stage S2.2 v0.3 — Smooth Dynamically Feasible Replanning

This package extends the MATLAB-validated v0.2 D* Lite and dynamic-obstacle logic with:

- piecewise seventh-order minimum-snap interpolation for fixed endpoint derivatives;
- shared waypoint derivatives and C3 continuity;
- sampled collision and kinematic feasibility checks;
- iterative segment-time scaling for speed, acceleration and jerk limits;
- zero-internal-derivative collision-safe fallback;
- feedforward + PD trajectory tracking;
- jerk-limited point-mass response;
- continuity-preserving regeneration after map changes and obstacle avoidance.

It remains a 2-D fixed-altitude planning/control simulation. Full 6-DOF F450 dynamics and S2.1 estimator coupling are not yet claimed.

Run:

```matlab
clear functions; clear classes; close all force; clc;
report = validate_S2_2(true);
```

Results are saved under:

```text
simulation/results/S2_2_mission_replanning/v0_3/<scenario>/seed_000/
```
