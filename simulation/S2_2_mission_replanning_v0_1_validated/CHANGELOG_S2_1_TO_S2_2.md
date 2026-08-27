# Changelog — Stage S2.1 to Stage S2.2 v0.1

## Frozen baseline

Stage S2.1 robust multi-lane estimator remains frozen. Do not overwrite the S2.1 folder.

## Added in Stage S2.2 v0.1

- New standalone folder: `simulation/S2_2_mission_replanning/`.
- New main interface: `run_S2_2_mission_replanning(seed, scenarioName, makePlots, makeAnimation)`.
- Inflated 2-D occupancy grid using S2.1 F450 clearance model.
- A* global planner over an 8-connected grid.
- Line-of-sight path smoothing.
- Lookahead collision monitor.
- Hover-and-replan state machine.
- Failsafe when no safe path exists.
- MATLAB scenario validator.
- Dashboard and simple animation.

## Current limitations

- Kinematic XY tracking only.
- No full S2.1 estimator coupling yet.
- No 3-D ESDF/depth occupancy map.
- No D* Lite incremental update.
- No minimum-snap trajectory generation.
- No motor/thrust dynamics.

## Patch 2 — map-version replanning logic

- Fixed hover/replan oscillation in `UNKNOWN_OBSTACLE_APPEARS`.
- Added map-version invalidation: a persistent newly detected obstacle invalidates the old plan once, the vehicle hovers, replans on the updated inflated grid, and then tracks the new path.
- Added path post-validation with raw-A* fallback if shortcut smoothing creates an invalid segment.
- Reduced dashboard legend spam by showing initial and final plans only.
