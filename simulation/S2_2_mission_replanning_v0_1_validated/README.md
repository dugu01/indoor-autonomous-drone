# Stage S2.2 v0.1 — Mission Management, Online Obstacle Avoidance and Replanning

This folder starts Stage S2.2 separately from the frozen Stage S2.1 robust multi-lane estimator.

## Scope

S2.2 v0.1 implements a MATLAB reference of the Python-tested **OMR-FailSafe Planner**:

- inflated 2-D occupancy grid,
- A* global planning,
- line-of-sight path smoothing,
- lookahead collision monitor,
- hover-and-replan mission state machine,
- failsafe when the goal or path is unsafe.

It is not yet a full flight stack. It does not yet include D* Lite, 3-D ESDF mapping, minimum-snap trajectory generation, motor dynamics, or hardware interface code.

## Run

```matlab
cd('/Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2_2_mission_replanning')
addpath(pwd,'-begin')
clear functions; clear classes; close all; clc;

results = run_S2_2_mission_replanning(0,'unknown_obstacle_appears',true,false);
```

Validate the scenario matrix:

```matlab
report = validate_S2_2(true);
```

## Scenarios

- `static_known_obstacles`
- `unknown_obstacle_appears`
- `goal_blocked_failsafe`
- `narrow_passage_rejected`

## GitHub note

This is an engineering integration. The repo should cite the algorithms and systems used as references, and should not claim to reproduce Nav2, PX4, Voxblox, EGO-Planner, or any external stack.
