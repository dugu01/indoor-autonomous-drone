# Stage S2.2 v0.2 Literature and Open-Source Cross-Reference

## Primary research sources

1. S. Koenig and M. Likhachev, **Fast Replanning for Navigation in Unknown Terrain**, IEEE Transactions on Robotics, 21(3), 354–363, 2005. Basis for D* Lite incremental shortest-path repair.
2. S. Koenig, M. Likhachev, and D. Furcy, **Lifelong Planning A***, Artificial Intelligence, 155(1–2), 93–146, 2004. Basis for the `g`/`rhs` local-consistency framework underlying D* Lite.
3. P. Fiorini and Z. Shiller, **Motion Planning in Dynamic Environments Using Velocity Obstacles**, International Journal of Robotics Research, 17(7), 760–772, 1998. Basis for finite-horizon relative-velocity collision prediction.
4. X. Zhou, Z. Wang, H. Ye, C. Xu, and F. Gao, **EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors**, IEEE Robotics and Automation Letters / IROS, 2021. Used as a future trajectory-optimization reference; not reproduced in v0.2.

## Official implementation references

5. PX4 Collision Prevention documentation. Used for the design principles of stopping-distance speed restriction, delay allowance, and zero XY command when obstacle-sensor coverage is unavailable.
6. ROS 2 Nav2 Collision Monitor documentation. Used for the separation of an immediate sensor-level safety layer from the global costmap/planner.
7. PythonRobotics open-source project and paper. Used only to cross-check conventional search/planning organization; no source code was copied.

## Our integration

**OMR-IDS (Obstacle-Mapped Replanning with Incremental/Dynamic Safety)** combines:

- D* Lite for persistent occupancy changes;
- a finite candidate approximation to velocity-obstacle filtering for transient movers;
- alpha-beta tracking from noisy position observations;
- conversion of a stopped persistent mover into a static inflated obstacle;
- PX4-inspired delay/braking and no-data hold;
- A* as an independent consistency fallback and expansion benchmark.

The integration and scenario logic are ours. We do not claim invention of the component algorithms.
