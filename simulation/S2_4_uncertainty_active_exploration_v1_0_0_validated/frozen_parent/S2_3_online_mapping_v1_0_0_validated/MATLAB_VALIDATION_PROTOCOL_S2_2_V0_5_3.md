# MATLAB Validation Protocol — Stage S2.2 v0.5.3

This candidate addresses the twelve failed seed/scenario combinations from the v0.5.2 multi-seed sweep. MATLAB execution on the target installation remains authoritative.

## 1. Path reset

```matlab
restoredefaultpath;
rehash toolboxcache;
cd('/path/to/indoor-autonomous-drone/simulation/S2_2_mission_replanning');
addpath(pwd,'-begin');
clear functions; clear classes; close all force; clc; rehash path;
```

Confirm one active copy of each production function:

```matlab
which run_S2_2_mission_replanning -all
which mission_manager_S2_2 -all
which mission_manager_v0_5_3_core_S2_2 -all
which mission_lifecycle_manager_S2_2 -all
which multi_lane_eskf_robust_S2_2 -all
which validate_S2_2 -all
```

## 2. Focused former-failure regression

```matlab
focus = validate_S2_2_v0_5_3_focus();
```

Acceptance: 12/12 PASS. This covers dynamic-crossing seeds 1 and 7; IMU-fault seeds 1, 2, 6 and 9; RTL-obstacle seed 2; and XY-loss seeds 2, 3, 6, 7 and 9.

## 3. Deterministic regression

```matlab
report = validate_S2_2(false);
```

Acceptance: 12/12 PASS.

## 4. Complete multi-seed robustness matrix

```matlab
robust = validate_S2_2_multiseed_robustness(0:9);
```

Acceptance: 60/60 PASS with zero collision and zero geofence violations. Any failed seed must be investigated; results are not averaged away.

## 5. Visual validation

Only after the no-plot tests pass:

```matlab
report = validate_S2_2(true);
```

All dashboards must remain tabbed in one MATLAB Figures window and save under:

`simulation/results/S2_2_mission_replanning/v0_5_3/`

## 6. Critical animations

Run nominal lifecycle, dynamic crossing, blocker promotion, RTL obstacle replan, alternate landing and XY-loss emergency landing. Confirm that animation timing matches obstacle insertion and mission-state transitions.

No safety, clearance, estimator, tracking, speed, acceleration, jerk, or mission threshold is relaxed in v0.5.3.
