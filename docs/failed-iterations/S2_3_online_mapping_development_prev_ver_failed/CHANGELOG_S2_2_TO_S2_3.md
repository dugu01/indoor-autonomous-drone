# S2.2 -> S2.3 cumulative candidate changes

## Preserved unchanged

- Frozen S2.2 source files and public S2.2 entry points.
- 6-DOF dynamics and geometric controller.
- Four-lane lifecycle ESKF and lane switching.
- D* Lite, A*, minimum-snap generation and grid fallback.
- Estimator-aware inflation and all inherited validation thresholds.
- XY-aid-loss braking, emergency descent, touchdown latch and disarm logic.

## Added

- Raw simulated RPLidar and D435i-style depth ray packets.
- Finite range, noise, field of view, occlusion and dropout.
- Timestamped packet sequence and estimator-pose interpolation.
- Layered 3-D log-odds map with free/occupied/unknown state.
- Temporary dynamic layer and persistence-based static promotion.
- Fail-closed planner-grid projection.
- Map and frame version fields.
- Goal-directed known-free frontier segments.
- Known-free terminal stop validation.
- Scan hold, map-degraded hold and goal-unreachable lifecycle states.
- Perception-qualified launch and landing checks.
- Map accuracy, truth-isolation and unknown-commitment validation.
- One-window tabbed S2.3 dashboard.
- Deterministic and 60-trial critical validation harnesses.
- Static candidate audit and independent Python mechanism checks.

## Source-review corrections before handoff

- Duplicate asynchronous packets are rejected.
- Sensor insertion uses the estimated pose at the measurement timestamp.
- Map versions advance for free/unknown changes, not only occupied changes.
- Frontier selection uses the unknown safety boundary.
- Preflight checks the observed ground launch footprint rather than an unobserved cruise layer.
- Partial-map no-route outcomes enter stop-and-scan rather than immediate emergency landing.
- Persistent complete perception loss does not initiate translation on a stale map.

## First MATLAB nominal runtime correction

The initial `UNKNOWN_ROOM_NOMINAL`, seed 0 runtime stopped at
`PREFLIGHT_REJECT` at 0.60 s. Recorded-state replay showed that estimator,
launch-clearance and covariance gates were valid and `perceptionFresh` was
true, but the old preflight implementation required a new scan on the exact
control step. The cumulative development candidate now:

- qualifies preflight perception from the latest accepted map-observation age;
- requires a minimum number of accepted mapping packets;
- records no-sensor-event control steps as idle rather than rejected packets;
- removes the near-ground z=0 map-layer/floor-plane mismatch from independent
  map scoring.

No inherited S2.2 flight, estimator, planning, landing or safety threshold was
changed.

## Second MATLAB nominal runtime: cumulative source-grounded correction

The second `UNKNOWN_ROOM_NOMINAL`, seed 0 run armed, took off and remained
collision/geofence safe, but declared the goal unreachable and ended in
`LAND_DESCENT`. Full MAT replay established multiple coupled causes:

- 12 selected frontier plans were counted as completed extensions although the
  vehicle moved only 0.05695 m from launch before the unreachable declaration;
- any newly blocked map cell anywhere caused active-route repair, producing 2255
  replans and 2243 trajectory generations;
- landing began at 216.30 s, too late for the unchanged 5.50 s profile within
  the 220.00 s horizon;
- free-ray endpoint quantization contaminated occupied voxels;
- static promotion could repeat many times from saturated-free evidence;
- the trajectory generator and hard validator used inconsistent numerical
  acceptance tolerances;
- scan yaw was repeatedly restarted from zero at 55 deg/s.

The cumulative candidate now:

- repairs only when a newly blocked inflated cell intersects the active future
  trajectory or grid fallback;
- separates planned frontier segments from physically completed map extensions;
- measures extension progress from actual vehicle motion toward the mission goal;
- does not use changing map version as physical progress;
- explicitly excludes occupied endpoint voxels from free-ray updates;
- uses a height-aware 3-D temporary layer;
- performs one-time static promotion by clamping qualified cells above the
  occupied threshold;
- scores occupied recall only on repeatedly hit physical obstacle endpoints;
- uses an S2.3 strict adapter around the unchanged S2.2 minimum-snap generator;
- begins scans from the retained yaw and uses 35 deg/s;
- leaves the 220 s horizon unchanged rather than concealing the replan storm.

No frozen S2.2 source or inherited safety threshold was changed.

## Seed-0 third-run cross-check corrections

- Replaced ceil-cell binary inflation with physical metric-radius inflation of
  planner grid-node centres. The configured 0.602 m radius is no longer rounded
  up to 0.70 m on a 0.10 m grid.
- Added persistent `staticOccupied` voxel state.
- Prevented ordinary free rays from eroding latched static voxels.
- Made planner known-free space explicitly disjoint from static occupancy.
- Updated independent map validation to use persistent static classification.
- Added the third-run saved-map cross-check and 19-mechanism test suite.
- Did not relax map, clearance, kinematic, estimator or lifecycle thresholds.
