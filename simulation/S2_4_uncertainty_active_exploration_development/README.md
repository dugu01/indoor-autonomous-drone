# S2.4 — Uncertainty-Aware Active Exploration and Dynamic-Risk Navigation

Version: `S2.4-A-D-shadow-v0.1.0-candidate`

This is the cumulative **A-D offline/shadow candidate** built on a byte-frozen
copy of the uploaded S2.3 release. It contains:

- exact S2.3 parent immutability and final-manifest checks;
- non-authoritative entropy, age, source and confidence sidecars;
- deterministic incremental frontier extraction with persistent lifecycle IDs;
- safe viewpoint generation through S2.3 known-free space only;
- target-corridor relevance, information gain and traceable utility terms;
- class-agnostic dynamic prediction and route-crossing risk contracts;
- complete accepted/rejected candidate logs;
- a combined 15-scenario Python contract catalogue;
- deterministic replay of the available final S2.3 nominal raw trace;
- MATLAB shadow code and a repeat-exact MATLAB validation runner.

## Frozen safety boundary

S2.3 occupancy remains authoritative. S2.4 does not edit S2.3, clear occupied
cells, convert unknown cells to free, reduce the inherited runtime inflation, or
send position, velocity, yaw, state-machine or actuator commands.

Frontiers use the raw S2.3 known-free/unknown boundary. Candidate positions,
routes, stopping support and retreat support use a stricter executable mask in
which both physical occupancy and unknown space are inflated by the inherited
runtime radius (`0.602 m` in the final supplied trace).

## Offline checks

```bash
python3 tools/run_all_checks.py
python3 python_tests/s2_4_recorded_shadow_replay.py \
  /path/to/final/S2_3_v1_0_0_candidate_trial_data.mat \
  --output-dir evidence
```

The multi-trace runner rejects incompatible pre-release MAT schemas rather than
silently treating them as final evidence:

```bash
python3 python_tests/s2_4_recorded_replay_matrix.py \
  /path/to/final_trace_1.mat /path/to/final_trace_2.mat
```

## MATLAB gate

```matlab
addpath(fullfile(pwd,'s2_4_shadow'));
gate = validate_S2_4_AD('/absolute/path/to/final_trace.mat', ...
    fullfile(pwd,'results','S2_4_AD_shadow'));
```

`validate_S2_4_AD` performs two complete shadow replays and requires exact
uncertainty and frontier/viewpoint repeat digests. MATLAB was not available in
the package-generation environment, so this coupled execution gate is not
claimed as passed.

## Scope boundary

The package does **not** command the drone. Coupled active-exploration flight,
the deterministic MATLAB 15/15 mission matrix and the coupled 60/60 critical
multi-seed matrix remain disabled until the A-D MATLAB shadow gate passes.
