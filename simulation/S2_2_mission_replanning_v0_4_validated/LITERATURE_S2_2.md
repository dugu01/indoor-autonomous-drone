# Literature and Open-Source Basis — Stage S2.2 v0.4

The code in this package is independently written. The sources below provide the mathematical and architectural basis. The project does not claim to reproduce any cited system in full.

## Error-state inertial estimation

**Joan Solà, “Quaternion Kinematics for the Error-State Kalman Filter,” 2017.**  
arXiv:1711.02508.

Used for:
- nominal quaternion plus small-angle error-state representation;
- quaternion exponential/logarithm conventions;
- inertial propagation and covariance linearisation;
- multiplicative attitude correction.

Project adaptation:
- 16-state indoor ESKF with position, velocity, attitude error, accelerometer bias, gyro bias, and barometer bias;
- four complementary IMU/aiding lanes;
- selected local state is supplied to planning and control.

## Quadrotor geometric control

**Taeyoung Lee, Melvin Leok, N. Harris McClamroch, “Control of Complex Maneuvers for a Quadrotor UAV using Geometric Methods on SE(3),” IEEE CDC, 2010.**  
DOI: 10.1109/CDC.2010.5717652.

Used for:
- desired body-z direction from commanded force;
- rotation-matrix attitude error on SO(3);
- body moment feedback without Euler-angle singularities.

Project adaptation:
- conservative near-hover controller with explicit tilt, thrust and moment saturation;
- fixed-yaw indoor navigation during this stage;
- controller operates on selected ESKF state rather than truth.

## Incremental planning

**Sven Koenig and Maxim Likhachev, “D* Lite,” AAAI, 2002.**

Used for:
- `g/rhs` incremental shortest-path representation;
- key-based repair after local occupancy changes;
- moving the start state without rebuilding the complete search.

Project adaptation:
- 8-connected 2-D inflated occupancy grid;
- fresh A* search retained as a benchmark and recovery route;
- exact continuous vehicle position is inserted before trajectory generation.

**Peter E. Hart, Nils J. Nilsson, Bertram Raphael, “A Formal Basis for the Heuristic Determination of Minimum Cost Paths,” IEEE Transactions on Systems Science and Cybernetics, 1968.**

Used for the fresh-search A* benchmark and recovery planner.

## Smooth quadrotor trajectories

**Daniel Mellinger and Vijay Kumar, “Minimum Snap Trajectory Generation and Control for Quadrotors,” ICRA, 2011.**

**Charles Richter, Adam Bry and Nicholas Roy, “Polynomial Trajectory Planning for Aggressive Quadrotor Flight in Dense Indoor Environments,” 2016.**

Used for:
- high-order polynomial position trajectories;
- derivative constraints and smooth segment connection;
- speed, acceleration and jerk feasibility checks.

Project adaptation:
- piecewise seventh-order trajectories with endpoint position/velocity/acceleration/jerk constraints;
- iterative segment-time scaling;
- dense collision checking against the inflated grid;
- conservative zero-internal-derivative fallback when a smooth shortcut is unsafe.

Official implementation cross-check:
- ETH Zurich `mav_trajectory_generation` repository.

## Dynamic obstacle safety

**Paolo Fiorini and Zvi Shiller, “Motion Planning in Dynamic Environments Using Velocity Obstacles,” International Journal of Robotics Research, 1998.**

Used for the relative-velocity/predicted-separation concept.

Project adaptation:
- finite candidate velocity set;
- static-grid check for every candidate;
- stop/hold when no safe candidate exists;
- trajectory clock pause and explicit rejoin state after the safety override clears.

## Estimator/control architecture cross-checks

Official PX4 source repositories were reviewed for architectural comparison:
- `PX4-Autopilot/src/modules/ekf2` — estimator instances and selector separation;
- `PX4-Autopilot/src/modules/mc_pos_control` — multicopter position-control module separation.

These repositories were used as architecture references only. No PX4 source code is copied into this project.

## Future trajectory-optimisation reference

**Xin Zhou et al., “EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors,” 2020.**

The official `ZJU-FAST-Lab/ego-planner` repository is retained as a future reference for continuous collision-aware local optimisation. v0.4 does not implement EGO-Planner.

## What is ours

The project-specific contribution is the engineering integration and validation framework:

- S2.1-style four-lane indoor ESKF;
- selected local estimate driving D* Lite, dynamic avoidance and geometric control;
- uncertainty-dependent obstacle inflation;
- local trajectory continuity across replans and estimator lane changes;
- combined static, dynamic, sensor-dropout, IMU-fault and observability-loss regression;
- truth-isolated safety/performance validation.

The individual algorithms remain attributed to their original authors.
