# S2.3 Release End-to-End Closure

## Frozen scope

No changes are made to the estimator, probabilistic mapper, D* Lite/A* cores,
trajectory generator, geometric controller, 6-DOF plant, landing logic,
touchdown latch, inflation radius, or safety thresholds.

## Scenario-contract correction

The deterministic catalogue now separates four mechanisms:

1. **Unknown static geometry** — discover obstacles through onboard perception,
   move only through sufficiently observed free space, stop and scan when needed.
2. **Late blockage of a known-free corridor** — a closing door or moved object is
   sensed while the vehicle is still far away; only changed cells intersecting
   the committed future route trigger repair.
3. **Dynamic object becoming static** — temporary occupancy is tracked and then
   promoted after the configured persistence and hit conditions.
4. **Unreachable goal** — bounded scan/no-progress attempts followed by safe
   refusal, RTL and landing.

The previous hidden-obstacle test created a large obstacle only about half a
second before the vehicle entered its collision envelope. It tested object
teleportation, not safe perception-driven replanning. The replacement late
corridor blockage appears at 12 s, while the validated nominal run is in
TRACK_OUTBOUND, more than 3 m from the obstacle and with a conservative 5.78 s
lead to the affected route section. The initial route is blocked, an alternate
inflated route exists, and the obstacle is within direct LiDAR line of sight.

The narrow-passage case no longer requires a map-extension event. With a
360-degree 6.5 m LiDAR in a 6 m room, the corridor can be observed before the
vehicle needs a frontier manoeuvre; successful direct navigation is valid.

## Literature alignment

- **FASTER** distinguishes free-known, occupied-known and unknown space and
  retains a stop-capable safe trajectory in free-known space.
- **D* Lite** reuses previous search results and repairs the relevant changed
  costs after newly discovered blockage rather than replanning for every map
  update.
- **FIESTA** supports incremental map/distance updates for online aerial-robot
  planning.
- **Dynablox** motivates conservative free-space reasoning for diverse moving
  objects without requiring semantic class labels.
- **MADER** demonstrates explicit trajectory reasoning around dynamic agents;
  S2.3 uses a simpler temporary-occupancy layer, with stronger prediction left
  to S2.4.

Primary references:

- Tordesillas et al., *FASTER: Fast and Safe Trajectory Planner for Navigation
  in Unknown Environments*, IEEE T-RO, arXiv:2001.04420.
- Koenig and Likhachev, *D* Lite*, AAAI 2002.
- Han et al., *FIESTA: Fast Incremental Euclidean Distance Fields for Online
  Motion Planning of Aerial Robots*, arXiv:1903.02144.
- Schmid et al., *Dynablox*, arXiv:2304.10049.
- Tordesillas and How, *MADER*, arXiv:2010.11061.

## Full Python backtest

`python_tests/release_end_to_end_backtest.py` checks all twelve scenarios using
exact 0.10 m grid resolution and 0.602 m metric inflation. It checks:

- route feasibility or deliberate unreachability;
- start/goal occupancy and minimum route clearance;
- occlusion and later visibility;
- scan-required direct-view blockage;
- late-blockage route intersection, alternate route, sensor visibility,
  active TRACK_OUTBOUND timing, physical notice distance and lead time;
- sensor-dropout hold/failsafe timing;
- backup estimator-aid availability;
- dynamic-to-static promotion timing and route interaction;
- source-code scenario contracts.

The complete result is stored in `RELEASE_END_TO_END_BACKTEST.txt` and JSON.
This is a scenario/mechanism gate and does not replace MATLAB coupled evidence.
