# Scenario 2: literal competing corridors

## Research question

When the quadrotor reaches an occluded physical fork with two safe unknown
branches, can it select the branch that reveals the commanded target route even
when the other branch offers more raw information?

## Physical test world

The world is a T-shaped fork in a 6 x 6 m room. A vertical wall occludes both
branches from the start. A horizontal east-side divider keeps the two branches
physically separate after the fork.

- upper branch: contains the commanded target;
- lower branch: safe, larger, irrelevant exploration decoy;
- both branches: feasible after the inherited 0.602 m safety inflation;
- autonomy: receives only its normal estimated map/frontier/candidate data.

The exact geometry is versioned in
`coupled/scenarios/literal_competing_corridors_geometry.json`.

## Decision rule under test

Hard safety gates remain authoritative. Among surviving candidates, the existing
selector remains tier-first:

1. Tier 1 — target-relevant frontier;
2. Tier 2 — safe target progress;
3. Tier 3 — unrelated exploration.

The scenario therefore tests whether Tier-1 target relevance defeats a larger
but irrelevant branch without bypassing known-free-space, retreat, clearance or
dynamic-feasibility checks.

## Stronger v0.3.2 evidence contract

A Scenario-2 coupled PASS now requires:

- the simulator truth world exactly matches the two literal walls;
- a selected decision with at least one target-relevant frontier and one
  **distinct** irrelevant frontier;
- the selected view is Tier 1 with positive target relevance;
- the distinct decoy has strictly greater information gain than the selected
  view;
- the tier-priority margin versus that decoy is positive;
- the selected view is executed and the full goal -> RTL -> landing mission
  completes without hard-safety violations or decision-layer truth access.

`S2_4_E_COMPETING_CORRIDORS_PREFLIGHT.*` from v0.3.0 is retained only as a
legacy recorded-map selector regression. It is not evidence for the literal
world.

## Release authority

The independent Python geometry audit validates only the physical geometry.
The release authority remains:

```matlab
gate2 = validate_S2_4_E_milestone_2();
assert(gate2.pass);
```

followed by:

```matlab
report2 = validate_S2_4_E_competing_corridors_multiseed(0:9);
assert(report2.pass);
```

## v0.3.6 physical benchmark revision

The first literal-world coupled seed-0 run completed safely and selected the
correct Tier-1 target frontier, but the measured maximum irrelevant information
gain (15.964386) remained below the selected target information gain
(17.925579). Therefore that world did not yet instantiate the intended
adversarial information-gain competition.

v0.3.6 changes only the truth geometry: the lower irrelevant fork is widened
and the upper target corridor is narrowed while both remain feasible under the
same inherited 0.602 m inflation. The selector, information-gain equation,
weights, target tiers, and safety gates are unchanged.
