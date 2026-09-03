# Stage S2.3 release-closing plan

## Status at candidate creation

- Nominal coupled mission: PASS.
- Exact perception-mapper replay: PASS.
- Nominal map false-free/recall: PASS.
- Previous deterministic matrix: 7/12 PASS.
- Five failures were traced to four scenario/acceptance defects rather than
  estimator, controller, mapper, planner-core, or landing defects.

## Release-closing changes

1. `hidden_obstacle_replan`
   - Replaced the time-zero static obstacle with a truth-only delayed obstacle
     inserted at 18.0 s.
   - The initial truth-map route intersects the inserted inflated footprint.
   - A post-insertion alternate truth-map route remains available.
   - Acceptance now requires at least one route-corridor safety repair.

2. `occluded_obstacle`
   - Replaced the infeasible geometry with a collinear front/rear obstacle
     arrangement that preserves initial occlusion and a feasible truth route.

3. `unknown_narrow_passage`
   - Increased the physical opening to 1.50 m.
   - The unchanged 0.602 m inflation leaves a narrow but feasible centreline.

4. `goal_requires_scan`
   - Replaced the infeasible double-barrier geometry with a single tall screen.
   - The direct route is blocked, the upper passage is feasible, and at least
     one scan hold plus one completed map extension is required.

5. `unreachable_goal`
   - Retained the enclosed goal.
   - Removed the invalid requirement for a completed forward extension.
   - Added a bounded scan-count acceptance contract.

6. Validation/reporting
   - Added explicit extension, safety-replan, scan-bound, unreachable, and
     truth-isolation gates.
   - Added a deterministic end-of-run gate table.
   - Clarified the dashboard tracking-error label.

## Frozen components

No numerical implementation changes were made to:

- rigid-body plant or actuators;
- multi-lane ESKF or lane selection;
- online mapper or known-boundary policy;
- map projection or uncertainty inflation;
- D* Lite or A*;
- polynomial trajectory generation or strict adapter;
- geometric controller;
- landing, touchdown latch, disarm, or emergency landing;
- any inherited safety threshold.

## Required MATLAB closure sequence

1. Run the five revised scenarios individually.
2. Require 5/5 PASS.
3. Run `validate_S2_3(false)` and require 12/12 PASS.
4. Rerun the nominal exact mapper replay and require PASS.
5. Run `validate_S2_3_legacy_regression()` and require the frozen S2.2
   12/12 + 12/12 + 60/60 evidence.
6. Run `validate_S2_3_multiseed(0:9)` and require 60/60 PASS.
7. Freeze as S2.3 v1.0.0 only after all evidence files are reviewed.
