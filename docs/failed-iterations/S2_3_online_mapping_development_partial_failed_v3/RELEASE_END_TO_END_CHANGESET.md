# S2.3 Release End-to-End Changeset

Changed files:

- `scenario_S2_3.m`
  - Replaced the unsafe 18 s obstacle teleportation with a 12 s late corridor blockage at `[5.19 3.31 0.30 0.30]`.
  - Removed the unrelated map-extension requirement from the replan scenario.
  - Made map extension optional for the fully visible narrow-passage scenario.
  - Tightened the narrow-passage scan bound to two.
- `python_tests/release_end_to_end_backtest.py`
  - Added complete 12-scenario feasibility, visibility, timing, dropout, estimator and dynamic-promotion contracts.
- `python_tests/release_closure_scenario_backtest.py`
  - Redirected the old partial backtest to the complete catalogue test.
- `run_static_checks_s2_3.sh`
  - Added the complete 12-scenario backtest to the release static gate.
- Documentation updated with the literature-aligned scenario contracts.

No estimator, mapper, planner core, trajectory generator, controller, plant,
landing, touchdown, inflation or safety-threshold changes were made.
