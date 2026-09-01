# Stage S2.3 cumulative candidate architecture

## Parent boundary

The frozen S2.2 release remains unchanged. The S2.3 development folder retains its numerical plant, geometric controller, four-lane ESKF, D* Lite, A*, minimum-snap trajectory generation, XY-aid-loss handling, touchdown latch and landing logic.

## New data path

```
truth world (simulation only)
  -> finite-range noisy LiDAR/depth ray generation
  -> timestamped perception packet
  -> selected-ESKF pose interpolation at packet time
  -> layered static/dynamic log-odds map
  -> fail-closed 2-D planner projection
  -> D* Lite/A* known-free path or goal-directed frontier segment
  -> known-free stop validation
  -> existing minimum-snap trajectory and geometric control
```

Truth is supplied separately to the validator. The mapper, planner and mission decisions do not accept obstacle-truth geometry.

## Localization boundary

S2.2 retains the validated LiDAR pose-aid abstraction inherited from the S2.1 scan-matching frontend. S2.3 does not add a second competing ICP correction path. Its new raw LiDAR/depth channels are obstacle-perception channels for occupancy construction.

## Unknown-space policy

Unknown space is fail-closed. If the goal is not connected through known free space, the vehicle selects a reachable known-free endpoint near the unknown boundary, stops there, scans, updates the map and replans. Failure to find a route in a partial map triggers stop-and-scan, not immediate emergency landing.

## Safety invariants

- No committed reference point may lie outside known free space.
- Every planned segment must retain terminal stopping reserve in known free space.
- A newly blocked active corridor triggers repair, braking or scan hold.
- Complete perception loss first causes map-degraded hold; persistent loss causes controlled emergency landing rather than new translation on a stale map.
- S2.2 thresholds remain unchanged.
