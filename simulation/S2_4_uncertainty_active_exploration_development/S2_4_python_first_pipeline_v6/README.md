# S2.4 Python-First Layered Validation Pipeline v6

v6 fixes the multiseed release-gate problem exposed by the v5 run.

## Why v5 stopped at seed 1

Seed 0 was a valid physical benchmark activation: a clean Tier-3 decoy existed,
the Tier-1 target was selected, the mission completed, and all hard-safety checks
passed. Seed 1 also completed safely and selected the target, but stochastic
perception did not instantiate a clean Tier-3 decoy. v5 incorrectly required that
benchmark event to occur in every seed.

That mixed two different questions:

1. **Does the physical competing-corridor benchmark actually instantiate a clean
   decoy and demonstrate target-priority?**
2. **Is the closed-loop mission robust across random seeds?**

v6 separates them.

## Layer A — controlled adversarial policy contract

Mandatory in Python and MATLAB:

- target and decoy are safety-feasible;
- target is Tier 1;
- decoy is Tier 3 and rejected only by `IRRELEVANT_EXPLORATION`;
- `IG_decoy > IG_target`;
- `utility_decoy > utility_target`;
- the Tier-1 target is still selected.

## Layer B1 — physical benchmark activation

Default benchmark seed: `0`.

The benchmark seed must contain a clean Tier-3 decoy **at the same decision** in
which the Tier-1 target is selected. Mission completion, hard safety, truth
isolation, request execution, and zero irrelevant selections are also mandatory.

## Layer B2 — all-seed physical robustness

For every requested robustness seed (default `0:9`):

- mission PASS and goal reached;
- RTL + landing;
- exploration request/viewpoint executed;
- target-relevant selection occurs;
- zero irrelevant selections;
- zero collision, geofence violation, unknown commitment, unsafe viewpoint;
- truth isolation remains intact.

A clean decoy is **not forced to appear in every random seed**. If it does appear,
the same target-priority checks are applied. The benchmark seed is what guarantees
that the physical competing-corridor condition is actually instantiated.

This is not a relaxation of the adversarial policy requirement: the controlled
policy contract still mandates `IG_decoy > IG_target`, and the fixed physical
benchmark still mandates a clean feasible decoy.

## Python structural model

The Python map model is a structural pre-screen only. `visible_unknown_proxy` is
not presented as numerical parity with MATLAB entropy information gain. Truth is
used only to generate simulated observations and is not passed to candidate
scoring. Unknown space occludes the structural visibility ray.

The current v0.3.6 physical geometry passes the 11-case Python structural
robustness suite, so v6 reuses it by default instead of retuning walls. Use
`--force-search` only when intentionally searching for another physical world.

## Run

```bash
python3 run_full_pipeline.py \
  --out out \
  --matlab-project "/Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2_4_uncertainty_active_exploration_development"
```

Defaults:

- MATLAB seeds: `0:9`
- benchmark seed: `0`

The pipeline automatically backs up validation files/geometry before migrations.
It runs the benchmark seed in strict activation mode and the remaining seeds in
robustness mode, then runs the native MATLAB layered release gate.

## Important validation semantics

A robustness seed is **not** allowed to pass merely because the decoy is absent.
It must still complete the mission safely, execute exploration, select a target-
relevant candidate, and make zero irrelevant selections. Absence of a clean decoy
only means that the stochastic benchmark event did not instantiate in that seed.
