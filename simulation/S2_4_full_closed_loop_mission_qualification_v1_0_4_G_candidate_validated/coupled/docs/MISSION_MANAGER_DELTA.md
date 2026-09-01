# Mission-Manager Delta from Frozen S2.3

The frozen file `frozen_parent/.../mission_lifecycle_manager_S2_3.m` is not modified.

The development copy `coupled/mission/mission_lifecycle_manager_S2_4.m` adds only:

- initialization and live update of the read-only uncertainty sidecar;
- persistent frontier state;
- replacement of the outbound no-route extension choice with `plan_active_exploration_segment_S2_4`;
- creation and immediate revalidation of a mission-level exploration request;
- execution of the selected viewpoint yaw through the inherited reference interface;
- counters and logs for requests, selected views, executed views, frontier IDs, and unsafe execution steps;
- S2.4-specific release gates in the final summary;
- sidecar, frontier state and request history in saved maps.

Unchanged mechanisms include:

- raw sensor simulation and pose estimation;
- probabilistic mapper;
- S2.3 planner grid and inflation;
- D* Lite/A*;
- trajectory generation and grid fallback;
- controller and plant;
- route-change braking and replanning;
- perception-degraded hold;
- scan hold;
- RTL, landing, emergency and failsafe behaviour;
- independent truth-based validation metrics.
