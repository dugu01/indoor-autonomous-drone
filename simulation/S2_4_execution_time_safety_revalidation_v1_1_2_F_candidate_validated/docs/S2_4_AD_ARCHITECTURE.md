# S2.4 A-D cumulative shadow architecture

## Frozen boundary

```text
S2.3 perception replay
  -> exact unchanged S2.3 mapper replay
  -> authoritative occupancy / known-free / unknown projection
  -> S2.4 uncertainty sidecar
  -> incremental persistent frontier manager
  -> safe viewpoint candidates
  -> hard rejection log
  -> target-directed utility
  -> shadow recommendation only
```

The sidecar may add costs and annotations. It cannot clear an S2.3 occupied cell, convert unknown to free, reduce the 0.602 m final inflation contract, or send a command to the mission manager/controller/plant.


## Raw belief versus executable navigation mask

S2.4 preserves two deliberately different map products:

- the **raw S2.3 belief classes** (`knownFree`, `unknown`, raw static/dynamic
  occupancy) are used to define frontiers as known-free cells adjacent to
  unknown cells;
- the **executable navigation mask** inflates both physical occupancy and
  unknown space by the inherited runtime radius. All candidate positions,
  routes, stopping volumes and retreat routes are checked against this mask.

This separation is essential: applying unknown inflation to the frontier
predicate would erase valid frontiers, while omitting it from executable routes
would permit unknown-space commitment. The diagnostic target-corridor search may
assign a high finite cost to unknown cells only to identify useful observations;
that route is never executable or exposed as a flight command.

## Data products

### Uncertainty sidecar

- normalized Bernoulli occupancy entropy;
- inherited observation count;
- last-observed time and observation age;
- LiDAR hit/free contribution count;
- depth hit/free contribution count;
- source mask and fused source quality;
- static and dynamic confidence;
- stale-free indication;
- authoritative map version and independent sidecar version.

### Frontier track

- persistent track ID;
- sorted cells and deterministic geometry hash;
- centroid and creation/update versions;
- lifecycle state, failure count and cooldown;
- candidates and target relevance.

### Viewpoint candidate

- candidate/frontier IDs;
- cell, metric position and yaw;
- known-free route;
- visible unknown cells;
- information gain and target relevance;
- travel, static, dynamic, uncertainty, yaw and stop/retreat risk;
- tier, utility, accepted flag and complete rejection list.

## Core equations

For inherited log odds `l_i`:

```text
p_i = 1 / (1 + exp(-l_i))
H_i = -p_i log2(p_i) - (1-p_i) log2(1-p_i)
```

The hard-gated utility is:

```text
J(v) = 0.25 I + 0.35 G - 0.10 T - 0.08 S
       - 0.10 D - 0.05 U - 0.02 Y - 0.05 R
```

Weights rank candidates only after all hard gates pass. Tier eligibility is lexicographic and cannot be overridden by the weighted sum.

## Truth isolation

Runtime/shadow inputs are limited to recorded onboard perception packets, estimator pose, S2.3 map layers, dynamic tracks, geofence, target, and vehicle/sensor models. Scenario rectangles, truth occupancy, truth frontier labels, truth dynamic trajectories and oracle visibility are forbidden. Truth remains permitted only inside inherited post-mission validators.
