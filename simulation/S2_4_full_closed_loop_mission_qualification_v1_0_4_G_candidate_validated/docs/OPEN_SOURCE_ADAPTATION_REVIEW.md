# Open-source adaptation review for S2.4

## Decision rule

The reviewed projects are used as mechanism references, not copied wholesale. S2.4 must retain the frozen S2.3 estimator, occupancy mapper, known-free planner, trajectory safety supervisor, controller, plant, landing logic, inflation, truth-isolation boundary and exact replay contract.

## Reviewed projects

### ahmedeltaher/Autonomous-drone-navigation

Repository: https://github.com/ahmedeltaher/Autonomous-drone-navigation

Useful later for hardware integration:

- ROS 2/PX4/MAVSDK process separation;
- optical-flow, IMU and 2-D LiDAR data plumbing;
- map persistence and mission/failsafe nodes;
- a practical package split for global planning, local planning, path following and vehicle interface.

Not adopted into the S2.4 autonomy core:

- DWA is a reactive velocity sampler and does not by itself certify S2.3-style known-free stopping and retreat;
- Pure Pursuit is a path follower, not an active-perception viewpoint selector;
- the repository does not provide the deterministic uncertainty/frontier/viewpoint replay and truth-isolation gates required here.

### HKUST-Aerial-Robotics/FUEL

Repository: https://github.com/HKUST-Aerial-Robotics/FUEL
Paper: https://arxiv.org/abs/2010.11561

Adopted mechanisms:

- persistent Frontier Information Structure;
- dirty-region/incremental frontier refresh;
- deterministic subdivision of oversized frontier surfaces;
- local viewpoint generation around a frontier;
- yaw-aware travel cost and frontier visibility reasoning.

Adaptation:

FUEL is coverage-oriented. S2.4 adds a target-corridor relevance tier and rejects unrelated Tier-3 exploration, so a large frontier cannot win merely because it exposes more unknown volume.

### ethz-asl/mav_active_3d_planning

Repository: https://github.com/ethz-asl/mav_active_3d_planning

Adopted mechanisms:

- modular gain, cost and evaluator interfaces;
- explicit separation between candidate generation, hard rejection and utility ranking;
- traceable per-candidate component logging.

Not adopted:

- stochastic online tree sampling for the first A-D package. The shadow gate requires deterministic candidate sets and exact replay hashes.

### MIT-ACL/FASTER

Repository: https://github.com/mit-acl/faster
Paper: https://arxiv.org/abs/2001.04420

Adopted mechanism:

- every accepted observation viewpoint must retain a stop/retreat certificate in known-free space.

Not adopted:

- FASTER's optimization stack and solver dependencies. S2.3's validated trajectory generator and grid fallback remain frozen.

### mit-acl/MADER

Repository: https://github.com/mit-acl/mader
Paper: https://arxiv.org/abs/2010.11061

Adopted mechanism:

- evaluate a route in space and predicted arrival time against moving-object uncertainty envelopes.

Not adopted:

- decentralized multi-agent trajectory optimization or Gurobi. A-D only logs wait/replan/retreat recommendations in shadow mode.

### ZJU-FAST-Lab/ego-planner

Repository: https://github.com/ZJU-FAST-Lab/ego-planner

Decision:

- retain as a possible later comparison for the coupled trajectory stage;
- do not replace S2.3 D* Lite/A*/minimum-snap/grid-fallback mechanisms in A-D.

### uzh-rpg/dynablox

Repository: https://github.com/uzh-rpg/dynablox
Paper: https://arxiv.org/abs/2304.10049

Adopted mechanisms:

- class-agnostic dynamic confidence;
- free-space inconsistency/persistence concepts;
- conservative temporary occupancy during occlusion and track loss;
- no dependency on a semantic `person` label.

### Other projects considered

- Fast-Planner: useful ancestry and benchmark reference for fast aerial planning, but not a replacement for the frozen S2.3 core.
- NBVPlanner: useful next-best-view reference; FUEL plus the modular ETH evaluator already cover the required A-D mechanism more directly.
- TARE/FAR Planner: useful for large environments and visibility-graph reasoning. The current 6 m indoor stage uses a diagnostic target corridor only; it never executes a route through unknown space.
- RACER/EGO-Swarm: multi-UAV systems and outside the current single-F450 scope.

## Improvements made to the S2.4 design after review

1. **Persistent frontier identity** rather than rebuilding anonymous frontier points.
2. **Deterministic large-cluster subdivision** by normalized PCA, capped at an 18-cell extent.
3. **Target-first route precheck** before exploration. A known-free target route wins immediately; a stale route produces a rescan recommendation.
4. **Diagnostic target-corridor tube** from a relaxed, non-executable search, with a three-cell relevance radius.
5. **Lexicographic eligibility tiers** before the weighted utility:
   - Tier 0: target route available;
   - Tier 1: reveals blocked target corridor;
   - Tier 2: certified target-progress direction;
   - Tier 3: unrelated coverage, rejected.
6. **Hard candidate rejection before scoring**, with every reason retained.
7. **Known-free stop and retreat support** for every accepted candidate.
8. **Per-source LiDAR/depth contribution layers**, observation age, entropy, static confidence, dynamic confidence and stale-free indication.
9. **Space-time dynamic prediction interface** with covariance growth and conservative track-loss handling.
10. **Shadow-only command isolation**: the A-D package emits recommendations and evidence, never flight commands.
