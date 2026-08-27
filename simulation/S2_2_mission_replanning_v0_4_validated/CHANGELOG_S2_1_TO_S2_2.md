# Changelog — Stage S2.2

## v0.4 — Estimator-in-the-Loop 6-DOF Integration

Added:
- 6-DOF F450 rigid-body truth dynamics;
- quaternion SO(3) utilities;
- near-hover geometric position/attitude controller;
- synthetic primary/backup IMU, VIO, accepted LiDAR-local-pose, range and barometer measurements;
- four online S2.1-style ESKF lanes;
- innovation, covariance and freshness health scoring;
- output blending during estimator lane changes;
- causal IMU disagreement attribution using the two all-aid lanes;
- selected estimated pose/velocity driving planning and control;
- covariance-aware obstacle inflation and inflation-triggered replanning;
- truth-only collision, geofence and estimator-performance validation;
- seven-scenario MATLAB validator;
- v0.4 docked/tabbed dashboard and 3-D replay animation;
- versioned results under `v0_4`;
- independent Python 6-DOF/ESKF/control regression.

Preserved:
- D* Lite incremental repair and A* benchmark/recovery;
- static insertion, dynamic avoidance, promotion and obstacle no-data logic;
- seventh-order smooth trajectories and conservative fallback;
- nominal tracking versus safety-override metric separation;
- frozen S2.1 LiDAR/pose-graph package.

Known limitations:
- raw LiDAR scan matching is represented by an accepted local-pose measurement from the frozen S2.1 front-end;
- fixed nominal altitude and fixed yaw are used for planner integration;
- motor/ESC/propeller electrical dynamics and mixer saturation are not yet modelled individually;
- model parameters are nominal and not hardware-identified;
- no hardware-in-the-loop or flight validation is claimed.

## v0.3 — Smooth Dynamically Feasible Replanning

- seventh-order polynomial trajectories;
- speed/acceleration/jerk checking and time scaling;
- exact continuous replan anchoring;
- safety override, hold and rejoin modes;
- seven MATLAB scenarios validated by the user.

## v0.2 — Incremental and Dynamic Replanning

- D* Lite repair;
- velocity-obstacle safety filter;
- dynamic-to-static promotion;
- obstacle-sensor dropout hold/failsafe;
- six MATLAB scenarios validated by the user.

## v0.1 — OMR-FailSafe Baseline

- inflated occupancy grid;
- A* planning;
- live collision monitor;
- hover-and-replan;
- blocked-goal and narrow-passage failsafes;
- four MATLAB scenarios validated by the user.
