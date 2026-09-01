# S2.3 Package Manifest

- Package: cumulative exact-replay-gated known-boundary coupled candidate
- Formal release status: not frozen; coupled MATLAB rerun required
- Total files before checksum manifest: 177
- MATLAB files: 82
- Inherited S2.2 files: preserved byte-identically
- Independent mechanism tests: 20/20 PASS
- Truth-isolation static audit: PASS
- MATLAB source sanity: 82 files PASS

## Verified MATLAB replay evidence

- Accepted raw perception packets: 1342
- Exact mapper arrays: PASS
- Exact mapper metrics: PASS
- Known-boundary policy replay: PASS
- Counterfactual false-free / recall: 0.00174985 / 0.990054

## Integrated source change

- `init_probabilistic_map_S2_3.m`: registers only known `cfg.room` boundary
  voxels as persistent prohibited/static occupancy.
- `replay_perception_log_S2_3.m`: treats idle control-cycle count as
  informational and compares core mapper counters.
- No unknown obstacle truth or inherited safety threshold changed.

## End-to-end release closure additions
- `python_tests/release_end_to_end_backtest.py`
- `RELEASE_END_TO_END_BACKTEST.txt`
- `RELEASE_END_TO_END_BACKTEST.json`
- `RELEASE_END_TO_END_STATIC_CHECKS.txt`
- `S2_3_RELEASE_END_TO_END_SOLUTION.md`
- `RELEASE_END_TO_END_CHANGESET.md`
- `validate_S2_3_release_all.m` — one-command full release gate.
