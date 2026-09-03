# S2.4-E — Mission-Manager Coupling Candidate

## Scope

S2.4-E is the first command-enabled integration candidate. The active-exploration layer still does **not** command motors, attitude, body rates, or velocity. It produces a mission-level request containing a safe viewpoint, viewing yaw, known-free route, retreat route, map version, frontier ID, and validity window. A development copy of the validated S2.3 mission manager decides whether to execute the request using the inherited planner, trajectory generator, controller, estimator, and plant.

The frozen S2.3 parent is unchanged.

## Runtime flow

```text
S2.3 LiDAR/depth packet + selected estimated pose
                    |
                    v
Authoritative S2.3 probabilistic map and planner grid
                    |
          +---------+----------+
          |                    |
          v                    v
Normal direct target route   S2.4 uncertainty sidecar
available?                   + persistent frontiers
          |                    |
       yes|                    v
          |             safe viewpoint candidates
          |                    |
          |             hard safety filtering
          |                    |
          |             target-first ranking
          |                    |
          +---------+----------+
                    |
                    v
Mission request: viewpoint + yaw + route + retreat
                    |
                    v
Request revalidation against latest S2.3 grid
                    |
                    v
Inherited trajectory generation/controller/plant
                    |
                    v
Viewpoint arrival -> inherited bounded scan -> target replan
```

## Key contracts

1. A normal known-free target route always takes priority over exploration.
2. S2.4 reads but never writes the authoritative occupancy map.
3. A request is accepted only when its position, full route, local hold area, and retreat route are known free and unblocked.
4. A map-version change triggers geometric revalidation; it is not automatically treated as failure.
5. The selected yaw is passed to the inherited reference while moving to the viewpoint, and the scan begins from that orientation.
6. Exploration is currently enabled for outbound target acquisition only. RTL remains the validated S2.3 behaviour.
7. The exploration module cannot issue low-level commands.

## New files

- `mission/init_S2_4_E_config.m`
- `mission/exploration_request_S2_4.m`
- `mission/plan_active_exploration_segment_S2_4.m`
- `mission/mission_lifecycle_manager_S2_4.m`
- `mission/run_S2_4_coupled.m`
- `execution/build_execution_grid_S2_4.m`
- `execution/project_uncertainty_2d_S2_4.m`
- `execution/validate_exploration_request_S2_4.m`
- `scenarios/scenario_S2_4.m`
- `validation/test_S2_4_E_request_contracts.m`
- `validation/validate_S2_4_E_milestone_1.m`
- `validation/validate_S2_4_E_all.m`

## First coupled milestone

Scenario: `active_goal_requires_scan`

Required outcome:

- at least one active-exploration request is generated;
- at least one request is accepted by the mission manager;
- at least one safe viewpoint is reached;
- the inherited scan state updates the map;
- a target route becomes available;
- the mission reaches the goal, returns, and lands;
- no collision, geofence violation, unknown commitment, unsafe viewpoint execution, or truth-map access occurs.

## Current qualification

Completed here:

- frozen-parent byte identity;
- source-manifest identity;
- existing S2.4 A-D 15/15 offline contracts;
- S2.4-E decision truth/command isolation;
- mission-request schema audit;
- recorded `GOAL_REQUIRES_SCAN` preflight: 103 snapshots, deterministic repeat, 15 accepted candidates, zero unsafe candidates.

Pending local MATLAB execution:

- six request-validation unit contracts;
- the first coupled move-scan-replan mission;
- coupled plots and animation;
- multi-seed and full 15-scenario integration.
