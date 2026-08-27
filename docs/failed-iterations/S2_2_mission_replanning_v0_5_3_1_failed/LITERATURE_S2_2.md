# Literature and Open-Source Basis — Stage S2.2 v0.5

The MATLAB and Python code in this package is independently written. The cited papers, official documentation and repositories are used for mathematical, safety and architecture guidance. This project does not claim to reproduce any cited flight stack or paper in full.

## 1. Error-state inertial estimation

**Joan Solà, “Quaternion Kinematics for the Error-State Kalman Filter,” 2017, arXiv:1711.02508.**

Used for:
- multiplicative quaternion error-state representation;
- quaternion exponential/logarithm conventions;
- inertial propagation and covariance correction.

Project adaptation:
- 16-state indoor ESKF;
- four complementary estimator lanes;
- local selected state drives planning and control.

## 2. Geometric quadrotor control

**Taeyoung Lee, Melvin Leok, N. Harris McClamroch, “Control of Complex Maneuvers for a Quadrotor UAV using Geometric Methods on SE(3),” IEEE CDC, 2010, DOI: 10.1109/CDC.2010.5717652.**

Used for:
- desired body-z direction from commanded force;
- SO(3) attitude error;
- body-moment feedback without Euler-angle singularities.

Project adaptation:
- conservative near-hover controller;
- explicit tilt/thrust/moment limits;
- acceleration and jerk shaping;
- selected ESKF state rather than truth is used for feedback.

## 3. Incremental and heuristic planning

**Sven Koenig and Maxim Likhachev, “D* Lite,” AAAI, 2002.**

**Peter E. Hart, Nils J. Nilsson, Bertram Raphael, “A Formal Basis for the Heuristic Determination of Minimum Cost Paths,” IEEE Transactions on Systems Science and Cybernetics, 1968.**

Used for:
- incremental grid-path repair after map changes;
- fresh A* comparison and recovery;
- obstacle-aware outbound and return planning.

Project adaptation:
- 8-connected inflated indoor occupancy grid;
- exact continuous vehicle position inserted before trajectory generation;
- home-preferred return with safe alternate landing sites when home is blocked.

## 4. Smooth trajectories

**Daniel Mellinger and Vijay Kumar, “Minimum Snap Trajectory Generation and Control for Quadrotors,” ICRA, 2011.**

**Charles Richter, Adam Bry and Nicholas Roy, “Polynomial Trajectory Planning for Aggressive Quadrotor Flight in Dense Indoor Environments,” 2016.**

Used for:
- high-order polynomial position trajectories;
- derivative constraints;
- time scaling for speed, acceleration and jerk feasibility.

Project adaptation:
- piecewise seventh-order horizontal trajectories;
- dense collision validation against the inflated grid;
- seventh-order rest-to-rest vertical takeoff and landing profiles;
- conservative fallback when smooth internal derivatives are unsafe.

Official implementation cross-check:
- ETH Zurich `mav_trajectory_generation` repository.

## 5. Dynamic obstacle safety

**Paolo Fiorini and Zvi Shiller, “Motion Planning in Dynamic Environments Using Velocity Obstacles,” International Journal of Robotics Research, 1998.**

Used for the relative-velocity and predicted-separation concept.

Project adaptation:
- finite safe-velocity candidates;
- static-grid validation of candidates;
- hold when no safe candidate exists;
- explicit rejoin and brake-before-replan states.

## 6. Autonomous takeoff, landing and mission state machines

**Pengyu Wang, Chaoqun Wang, Jiankun Wang, Max Q.-H. Meng, “Quadrotor Autonomous Landing on Moving Platform,” 2022, arXiv:2208.05201.**

Used as a system-level reference for integrating:
- autonomous takeoff;
- tracking/planning;
- landing;
- a mission state machine.

Project adaptation:
- indoor fixed-floor mission lifecycle rather than moving-platform landing;
- separate preflight, arm, takeoff, hover, wait, outbound plan, goal hover, RTL, landing and disarm states;
- estimator-loss emergency landing state.

## 7. Safe landing-zone assessment

**Mattia Secchiero, Nishanth Bobbili, Yang Zhou, Giuseppe Loianno, “Visual Environment Assessment for Safe Autonomous Quadrotor Landing,” 2023, arXiv:2311.10065.**

Used for the principle that landing sites should be assessed for geometric safety and suitability rather than treated as unconditional points.

Project adaptation in v0.5:
- landing-site safety is currently map-geometric only;
- the centre-safe inflated occupancy grid is checked around the landing site;
- home is preferred when safe and reachable;
- alternate landing points are ranked by reachable A* path length when home is blocked.

Not yet implemented:
- semantic terrain classification;
- slope, roughness or visual-surface assessment;
- real depth-map landing-zone perception.

## 8. Official PX4 architecture and safety references

Official PX4 documentation reviewed:

- **Takeoff Mode (Multicopter)**: automatic vertical takeoff to a configured altitude followed by position hold; requires a valid local position estimate.
  - https://docs.px4.io/main/en/flight_modes_mc/takeoff
- **Land Mode (Multicopter)**: automatic descent at the engaged location and automatic disarm after landing.
  - https://docs.px4.io/main/en/flight_modes_mc/land
- **Return Mode (Multicopter)**: automatic return to a safe destination using a geofence-aware path, descent and landing sequence.
  - https://docs.px4.io/main/en/flight_modes_mc/return
- **Safety/Failsafe Configuration**: hold, return and land actions selected according to failure severity.
  - https://docs.px4.io/main/en/config/safety
- **Collision Prevention**: velocity restriction based on stopping distance, jerk/acceleration limits, sensor delay and no-data handling.
  - https://docs.px4.io/main/en/computer_vision/collision_prevention

Project adaptation:
- no PX4 code is copied;
- v0.5 uses a local indoor map and local ESKF rather than GNSS/global navigation;
- return is planned through the same inflated occupancy grid used for outbound flight;
- complete XY-aid loss does not attempt horizontal RTL and instead executes a local controlled landing.

## 9. What is ours

The project-specific contribution is the engineering integration and validation framework:

- exact preservation of the MATLAB-validated v0.4 estimator/planner/controller core;
- v0.5 lifecycle dispatcher that leaves v0.4 regression behavior unchanged;
- autonomous preflight-to-disarm mission sequence;
- map-aware home/alternate landing selection;
- obstacle-aware return-path replanning;
- controlled emergency landing after loss of observable horizontal navigation;
- combined lifecycle, estimator, planner, controller and safety validation;
- versioned result folders, tabbed plots, animation and multi-seed regression.

The component algorithms remain attributed to their original authors and official projects.

## 10. v0.5.3 robustness-specific basis

### Multi-instance estimator fault isolation

Official PX4 EKF2 documentation describes running separate estimator instances with different sensor combinations, comparing their internal consistency, and using an estimator selector to isolate IMU bias, saturation and stuck-data faults. v0.5.3 uses this only as architecture guidance. The project-specific implementation uses four indoor ESKF lanes, recent normalized innovations, causal IMU disagreement attribution, dwell/confirmation logic and a fault-aware blend duration. No PX4 source code is copied.

Reference: https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf

### Position-loss response and stopping margin

Official PX4 failsafe documentation identifies loss of recently fused horizontal position/velocity aiding and excessive position uncertainty as position-loss triggers. It also states that multicopter geofence margins should account for stopping distance and uncertainty. v0.5.3 adapts these principles to an indoor local-map vehicle: it detects stale horizontal aiding, freezes the last aid-bounded velocity, applies a bounded open-loop brake, disables stale XY position feedback, and then descends vertically. This braking implementation is ours.

Reference: https://docs.px4.io/main/en/config/safety

### Search recovery hierarchy

D* Lite remains the incremental repair basis and A* remains the independently computed recovery/benchmark. The added progress watchdog, brake-and-replan sequence, and low-speed stop-at-corner route follower are project-specific integration logic.

Reference: Sven Koenig and Maxim Likhachev, “D* Lite,” AAAI 2002, https://idm-lab.org/bib/abstracts/papers/aaai02b.pdf
