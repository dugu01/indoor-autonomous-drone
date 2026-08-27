# Stage S2.2 v0.4 — Estimator-in-the-Loop 6-DOF Replanning

Stage S2.2 v0.4 integrates the validated S2.2 planning stack with a simulated F450 rigid-body model, a near-hover geometric controller, synthetic onboard sensors, and a four-lane error-state Kalman filter.

## What is implemented

- 6-DOF rigid-body truth dynamics using quaternion attitude.
- Near-hover geometric position/attitude controller.
- Four online ESKF lanes:
  1. primary IMU + VIO + LiDAR + range + barometer;
  2. backup IMU + VIO + LiDAR + range + barometer;
  3. primary IMU + VIO + range + barometer;
  4. backup IMU + LiDAR + range + barometer.
- Innovation/covariance/freshness lane health and causal lane switching.
- D* Lite incremental repair with fresh A* comparison and recovery.
- Seventh-order smooth trajectory generation with speed, acceleration and jerk checks.
- Dynamic-obstacle velocity filtering, hold/rejoin logic, and stopped-object promotion.
- Obstacle-sensor no-data hold and failsafe.
- Covariance-aware obstacle inflation:

  `applied inflation = base F450 safety radius + 2 sigma_xy`, capped at 0.72 m.

- Truth is used only for collision/performance validation. Planning and control use the selected local estimator output.
- Docked/tabbed dashboards and versioned results.

## Important implementation boundary

The LiDAR measurement in v0.4 represents an **accepted local pose output** from the frozen S2.1 LiDAR front-end. Raw scan generation, ICP, Scan Context, and pose-graph optimization remain in the frozen S2.1 package and are not duplicated here.

This is software-in-the-loop simulation. F450 mass, inertia, drag, thrust and moment limits are nominal simulation values and must be identified/calibrated before hardware use.

## Main interface

```matlab
results = run_S2_2_mission_replanning( ...
    seed, scenarioName, makePlots, makeAnimation);
```

## Validation scenarios

1. `nominal_6dof`
2. `incremental_static_estimated`
3. `dynamic_crossing_6dof`
4. `dynamic_blocker_becomes_static_6dof`
5. `obstacle_sensor_dropout_recover_6dof`
6. `primary_imu_fault_vio_outage`
7. `xy_aid_loss_failsafe`

## MATLAB validation sequence

```matlab
restoredefaultpath;
rehash toolboxcache;

cd('/Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2_2_mission_replanning');
addpath(pwd,'-begin');

clear functions;
clear classes;
close all force;
clc;
rehash path;

which run_S2_2_mission_replanning -all
which mission_manager_S2_2 -all
which multi_lane_eskf_S2_2 -all
which validate_S2_2 -all

report = validate_S2_2(false);
```

After all no-plot tests pass:

```matlab
report = validate_S2_2(true);
```

Animation is last:

```matlab
r = run_S2_2_mission_replanning( ...
    0,'primary_imu_fault_vio_outage',true,true);
```

## Results

Results are saved under:

```text
simulation/results/S2_2_mission_replanning/v0_4/<scenario>/seed_000/
```

The full validator writes:

```text
simulation/results/S2_2_mission_replanning/v0_4/validation/
```

## Current status

- Independent Python regression: 40/40 PASS over all seven MATLAB mission/fault classes plus one supplementary uncertainty case, five seeds each.
- MATLAB source: parsed successfully as MATLAB R2021b by MISS_HIT static syntax analysis.
- MATLAB runtime: not claimed until executed on the project MATLAB installation.
