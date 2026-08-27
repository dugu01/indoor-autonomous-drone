# Literature and open-source references for Stage S2.2

Stage S2.2 v0.1 is an engineering integration. It is not a direct copy of any paper or repository.

## Used in v0.1

1. Hart, Nilsson and Raphael, "A Formal Basis for the Heuristic Determination of Minimum Cost Paths", IEEE Transactions on Systems Science and Cybernetics, 1968.
   - Used for: A* search principle, `f = g + h` planning over a grid.
   - Our adaptation: 8-connected indoor F450 centre-point grid with inflated obstacles.

2. ROS 2 Navigation / Nav2 costmap architecture.
   - Used for: layered map concept: static obstacles, live obstacle insertion, inflation, keepout/safety buffer.
   - Our adaptation: MATLAB 2-D inflated boolean grid, not ROS costmap messages or plugins.

3. Nav2 Collision Monitor.
   - Used for: separate emergency-level collision monitor independent of the planner.
   - Our adaptation: path lookahead monitor that commands hover and replanning.

4. PX4 collision prevention.
   - Used for: speed/stop logic concept when obstacle data constrains a commanded motion direction.
   - Our adaptation: MATLAB lookahead path blocker and fail-safe hover, not PX4 uORB or firmware code.

## Planned for later S2.2 versions

5. Koenig and Likhachev, D* Lite / Lifelong Planning A* family.
   - Planned use: incremental replanning when obstacle map changes.

6. Oleynikova et al., Voxblox.
   - Planned use: 3-D ESDF-style obstacle-distance representation for MAV planning.

7. Zhou et al., EGO-Planner.
   - Planned use: fast local replanning and collision-guided trajectory adjustment for quadrotors.

8. Mellinger and Kumar, minimum-snap quadrotor trajectories.
   - Planned use: smooth dynamically feasible path-to-trajectory conversion.

9. PythonRobotics and similar educational open-source repos.
   - Used only as implementation cross-checks for algorithms, not copied directly.

## Citation rule

In the final GitHub project, cite the paper/repo/manual for each borrowed concept and explicitly state what is ours: integration, adaptation, parameters, test scenarios, MATLAB/Python implementation and validation around the S2.1 estimator.
