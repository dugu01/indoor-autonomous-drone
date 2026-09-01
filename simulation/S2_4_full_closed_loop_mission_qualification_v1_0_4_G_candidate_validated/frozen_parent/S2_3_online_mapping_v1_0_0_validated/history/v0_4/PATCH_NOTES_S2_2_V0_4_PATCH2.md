# Stage S2.2 v0.4 Patch 2 — Remaining Dynamic Failures

This cumulative patch is applied after v0.4 Patch 1.

## MATLAB failures addressed

### `dynamic_crossing_6dof`

Observed failure:

- goal reached;
- no collision or geofence violation;
- executed speed reached 0.669 m/s;
- executed-kinematic gate failed.

Root cause:

`REJOIN` used both the paused trajectory position and the bounded rejoin
velocity. The position error therefore created an additional acceleration
request, allowing the 6-DOF vehicle to exceed the executed-speed limit even
though the velocity reference itself was bounded.

Correction:

- `REJOIN` now remains a velocity-mode controller state;
- the XY position reference is anchored at the current local estimate;
- the rejoin velocity is acceleration/jerk shaped;
- TRACK resumes only after both position and velocity lock;
- the geometric controller has a soft speed envelope below the unchanged
  0.45 m/s executed-speed validation limit.

### `dynamic_blocker_becomes_static_6dof`

Observed failure:

- dynamic obstacle was correctly promoted;
- D* Lite and A* both found routes;
- immediate smooth-trajectory generation from the moving avoidance state
  was rejected;
- the recoverable kinodynamic mismatch was incorrectly converted to FAILSAFE.

Correction:

- a new `REPLAN_BRAKE` mission state is used when a map-changing replan has a
  route but cannot generate a collision-free trajectory from the current
  velocity/acceleration;
- the vehicle brakes in velocity mode while holding the current safe area;
- D* Lite/A* are refreshed from the new continuous state after near-hover;
- the new trajectory retains exact position/velocity/acceleration continuity;
- FAILSAFE is used only if no route exists or the controlled recovery cannot
  complete within the configured timeout.

## Threshold policy

The following validation limits were not relaxed:

- executed speed: 0.45 m/s;
- executed acceleration: 2.5 m/s^2;
- executed jerk: 25 m/s^3;
- controller tracking, altitude and tilt gates;
- static/dynamic clearance gates;
- estimator and uncertainty gates.

## Files replaced

- `init_S2_2_config.m`
- `geometric_controller_S2_2.m`
- `mission_manager_S2_2.m`
- `plot_S2_2_dashboard.m`
- `run_S2_2_mission_replanning.m`

## Focused regression

The included Python regression is a mechanism-level test, not a substitute
for the seven-scenario MATLAB validator.

Expected report:

- rejoin executed speed below 0.45 m/s;
- brake-before-replan reaches near-hover before 4 seconds;
- no increase of the hard MATLAB pass thresholds.
