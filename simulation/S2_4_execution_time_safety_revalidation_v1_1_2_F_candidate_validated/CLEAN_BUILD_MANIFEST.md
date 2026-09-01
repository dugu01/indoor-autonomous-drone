# S2.4-F v1.1.2 cumulative candidate manifest

## Baseline retained
- exact S2.4 A-D source/configuration and recorded replay evidence;
- exact validated S2.4-E exploration/scenario geometry and coupled source except the two existing
  runtime files intentionally modified for F (`mission_lifecycle_manager_S2_4.m` and
  `validate_exploration_request_S2_4.m`);
- exact frozen S2.3 parent;
- existing A-E validation tools/evidence.

## F additions
- execution-time active-request revalidator;
- validation-only fault injector;
- F config and coupled fault runner;
- deterministic MATLAB contracts and aggregate MATLAB gate;
- source-faithful Python stop/fault/scheduler semantic backtest and static/isolation audit;
- F design/remediation evidence for both coupled MATLAB failure rounds.

## v1.1.2-specific regression protection
- runtime revalidator is statically forbidden from calling the frozen S2.3 planning-time terminal stop gate;
- `R1` reproduces the v1.1.1 false-abort mechanism and requires corrected runtime continuation;
- `STOP` proves insufficient current stop reserve is still rejected;
- MATLAB failure output now reports d_stop, remaining route length, route reserve, overrun shortfall and terminal-overrun result.

## Frozen-parent rule
`frozen_parent/S2_3_online_mapping_v1_0_0_validated` is immutable. Local audit reports 208 files,
0 missing, 0 changed, 0 extra; final closure manifest PASS.

## Validated E invariants intentionally unchanged
- literal competing-corridor walls and geometry;
- 0.602 m inflation;
- target/decoy interpretation and controlled adversarial policy contract;
- D*/A*, A*, trajectory generation, controller and plant;
- E request builder, execution-grid builder, E config and scenario source.

## Current qualification status
Local static/offline aggregate: PASS.
Coupled MATLAB v1.1.2 F fault qualification: PENDING user MATLAB rerun.
F15 predictive moving-obstacle interface: NOT CONNECTED / N/A.
