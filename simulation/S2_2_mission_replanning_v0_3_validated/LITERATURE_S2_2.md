# Literature Basis — Stage S2.2 v0.3

## Primary trajectory references

1. D. Mellinger and V. Kumar, “Minimum Snap Trajectory Generation and Control for Quadrotors,” IEEE ICRA, 2011, pp. 2520–2525.
2. C. Richter, A. Bry, and N. Roy, “Polynomial Trajectory Planning for Aggressive Quadrotor Flight in Dense Indoor Environments,” in *Robotics Research*, Springer, 2016, pp. 649–666.
3. L. Berscheid and T. Kröger, “Jerk-limited Real-time Trajectory Generation with Arbitrary Target States,” Robotics: Science and Systems, 2021.
4. X. Zhou et al., “EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors,” IEEE Robotics and Automation Letters, 2021.
5. Z. Wang et al., “Geometrically Constrained Trajectory Optimization for Multicopters,” IEEE Transactions on Robotics, 2022, DOI 10.1109/TRO.2022.3160022.

## Open-source implementation cross-checks

- ETH Zurich ASL `mav_trajectory_generation`: polynomial vertices, segments, derivative constraints and feasibility checking.
- `pantor/ruckig`: state-to-state online motion generation under velocity, acceleration and jerk constraints.
- ZJU FAST Lab `ego-planner` and `GCOPTER`: collision-aware trajectory optimization and time/dynamic feasibility concepts.

## Exact adaptation in this project

The v0.3 implementation is independently written. It does not copy those repositories and it does not claim to reproduce their complete optimizers.

Our method combines:

1. D* Lite/A* collision-free waypoint paths from v0.2.
2. Seventh-order polynomial segments. For fixed endpoint position, velocity, acceleration and jerk, the segment is the minimum-snap solution.
3. Shared waypoint derivatives for C3 continuity.
4. Iterative time scaling when sampled speed, acceleration or jerk exceeds configured limits.
5. Dense validation against the inflated occupancy grid.
6. A zero-internal-derivative fallback when smooth polynomial shape overshoot enters occupied cells.
7. Jerk-limited closed-loop point-mass tracking and continuity-preserving replanning from the current state.

This integration is the project’s `OMR-IDS-MS` method. The component algorithms remain attributed to their original authors.
