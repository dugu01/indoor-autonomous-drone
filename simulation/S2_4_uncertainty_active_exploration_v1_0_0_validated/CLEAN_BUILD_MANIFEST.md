# Clean-build contents

## Retained
- exact S2.4 A-D source and configuration;
- current coupled mission/execution/scenario/V6 validation source;
- exact validated S2.3 frozen parent source;
- only the sibling frozen-parent `goal_requires_scan/seed_000` trace required by the legacy selector regression;
- deterministic Python tests and static audit tools;
- nominal recorded S2.3 trace for A-D replay;
- current design documentation and compact evidence;
- release poster assets and release figure/video generators.

## Excluded
- development `archive/` and old Milestone-1 archive;
- `S2_4_python_first_pipeline_v6/` search/backtest harness;
- historical S2.4 result trees;
- full historical sibling frozen-parent results;
- `*_before_v*` backups;
- obsolete cumulative-overlay repair audit;
- stale generated static-gate console outputs;
- `.DS_Store`, `__MACOSX`, non-parent `__pycache__`, `.pyc`, `.pytest_cache`.

## Frozen-parent integrity rule
The frozen S2.3 parent is byte-controlled by `evidence/FROZEN_PARENT_SHA256SUMS.txt`.
Two CPython 3.13 `.pyc` files inside its `python_tests/__pycache__/` directory are
part of the validated 208-file parent byte set and therefore remain intentionally.
No other cache file is permitted inside that parent.

RC3 restores that exact parent byte set and suppresses Python bytecode generation
through `PYTHONDONTWRITEBYTECODE=1` / Python `-B` in release-side validation
launchers. This prevents validation with a different Python installation from
mutating the frozen parent.

## Release portability
Top-level MATLAB validation/figure/video launchers resolve the package root from
their own file location. No release-side source contains the old absolute S2.4
development-directory path.

## Behavioral change status
None. The RC3 corrections are package portability and frozen-parent protection only.
No uncertainty, frontier, viewpoint, information-gain, target-relevance, tiering,
mission, planner, controller, geometry or V6 validation-policy logic was changed.

## Next step
Run the static/offline aggregate and MATLAB parity checks from this exact folder.
If parity reproduces PASS, do not edit executable files; freeze the same bytes as
`S2_4_uncertainty_active_exploration_v1_0_0_validated`.
