# S2.2 v0.3 all-scenario audit

| Scenario | Primary logic checked by this patch |
|---|---|
| `incremental_static_insert` | new-grid D* repair ordering, exact state anchoring, supercover checking, A* recovery, continuity and search-efficiency gates |
| `dynamic_crossing_yield` | static-safe velocity-obstacle candidates, hold/resume without false global replan, paused trajectory-clock recovery |
| `dynamic_blocker_becomes_static` | tracker maturity, persistence timer, promotion to costmap, D*/A* replanning fallback, continuous regenerated trajectory |
| `sensor_dropout_recover` | no-data hold, current-state trajectory anchoring, D* refresh and fresh-A* recovery |
| `sensor_dropout_failsafe` | expected failsafe outcome gate, no-data event gate, collision/geofence/core limits retained |
| `two_dynamic_crossings` | multiple track handling, no stale tracks, static-safe VO fallback, dynamic-event gate |
| `trajectory_time_rescale` | effective scale relative to requested timing, kinematic limits, C0-C3 continuity |

The patch does not reduce obstacle inflation, clearance, velocity, acceleration, jerk,
tracking, or continuity thresholds.
