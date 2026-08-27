# S2.3 release-closing candidate manifest

## Modified MATLAB files

- `scenario_S2_3.m` — revised five deterministic scenario contracts/geometries.
- `mission_lifecycle_manager_S2_3.m` — acceptance/report fields only; no flight-state numerical logic changed.
- `run_S2_3_online_mapping.m` — explicit map gate reporting.
- `validate_S2_3.m` — deterministic gate summary table.
- `plot_S2_3_dashboard.m` — tracking-error legend clarification only.

## Added files

- `validate_S2_3_release_focus.m`
- `python_tests/release_closure_scenario_backtest.py`
- `RELEASE_CLOSURE_PLAN_S2_3.md`
- `RELEASE_CLOSURE_BACKTEST_REPORT_S2_3.md`
- `RELEASE_CLOSURE_SCENARIO_BACKTEST_FINAL.txt`
- `RELEASE_CLOSURE_STATIC_CHECKS_FINAL.txt`
- `RELEASE_CLOSURE_CHANGESET.patch`

## Frozen numerical cores

All estimator, controller, mapping, planner, trajectory, plant, landing, and
safety-threshold files are unchanged relative to the nominal-passing coupled
candidate.
