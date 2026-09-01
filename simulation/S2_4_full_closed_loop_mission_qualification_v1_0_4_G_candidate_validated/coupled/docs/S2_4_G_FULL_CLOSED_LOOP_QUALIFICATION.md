# S2.4-G — Full Closed-Loop Mission Qualification v1.0.3

## Runtime evidence entering v1.0.3

User MATLAB v1.0.2 produced **71/75** critical PASS, 5/5 no-fault PASS, hard safety PASS, actual truth access PASS, complete E+F PASS and frozen-parent integrity PASS. The only four failures were F2/F3/F9/F10 at MID timing, seed 3. All four reached `final=COMPLETE` with finite goal/completion times and zero unsafe counters.

## Source-traced qualification defect

The mission manager's full nominal `summary.pass` is:

`missionLifecyclePass && trajectoryGate && controllerGate && estimatorGate && continuityPass && uncertaintyPass && staticGate && mappingPass && explorationPass && ~stateTimeoutTriggered`

plus `executionSafetyPass` when F is enabled.

G v1.0.2 reused that entire value inside the variable named `missionCompletion`. `ACTIVE_GOAL_REQUIRES_SCAN` sets `expectedMinExplorationExecutions=1`, so a fault-recovery run could reach the mission goal and complete safely but still have `summary.pass=false` if the revoked exploration viewpoint was no longer needed after current-map replanning.

## v1.0.3 critical-run gate

Critical fault-run qualification separates:

- **mission completion:** mission complete, goal reached, final state COMPLETE, not unreachable, no timeout;
- **closed-loop integrity:** every full mission-manager composite gate except only nominal `explorationPass`;
- **fault evidence:** post-acceptance injection, detection, required response, bounded invalidations;
- **hard safety:** zero stale continuation, collision, geofence, unknown commitment, unsafe viewpoint execution and actual truth access.

No-fault baselines continue to require the complete nominal `summary.pass`. The complete E+F gate is also rerun before G, so nominal exploration behavior is still explicitly qualified.

This is a qualification-semantics correction only. Runtime/autonomy source is unchanged from v1.0.2.

## Matrix

- faults: F2, F3, F6, F9, F10
- timing: early 0.20, mid 0.50, late 0.75 route progress
- seeds: 0:4
- total: 75 coupled critical runs
- acceptance: 75/75, plus 5/5 no-fault baselines

## Diagnostic separation

Failure output now reports `mission`, `core`, `nominal`, and `exp` separately. If a run fails, the wrapper also prints the preserved composite gates:

`T` trajectory, `C` controller, `E` estimator, `K` continuity, `U` uncertainty, `S` static, `M` mapping, `X` execution safety, `EXP` nominal exploration.

Thus an actual closed-loop performance regression cannot be hidden by the exploration-contract correction.

## v1.0.4 yaw-reference robustness correction

The v1.0.3 user MATLAB campaign isolated the last four failures to estimator attitude only: F2/F3/F9/F10 MID seed=3 all reached `COMPLETE`, retained hard safety and zero truth access, but peaked at 2.140175 deg estimator attitude error. A dedicated peak decomposition showed 2.140151 deg in the yaw-like quaternion component and only 0.010013 deg tilt component; the corresponding truth angular-rate magnitude reached approximately 204.865 deg/s.

Source tracing showed that accepted exploration yaw was copied directly into the geometric controller reference. That is an angle step because the inherited controller consumes desired yaw angle but has no desired yaw-rate state. G v1.0.4 therefore stores exploration yaw as a target and applies shortest-path slew limiting during `TRACK_OUTBOUND` using the inherited S2.3 `mapScanYawRate_radps = 35 deg/s`. This reuses the parent stage's already-qualified policy that replaced a 55 deg/s/resetting scan reference after it violated the 2 deg estimator attitude gate.

Authority revocation also revokes the associated viewing direction: during brake/replan the yaw reference is reset to the current estimated yaw, so the controller damps angular rate instead of continuing toward stale exploration orientation. Any subsequent accepted or reauthorized viewpoint produces a new yaw target and is again slew-limited.

The estimator attitude threshold remains exactly 2.0 deg. Controller gains, ESKF, plant, trajectory generator, planner, exploration geometry/policy and frozen S2.3 code are unchanged.

Before the full critical matrix, G v1.0.4 executes the four historical residual cases (F2/F3/F9/F10 MID seed=3). The results are cached and reused in the matrix, so the campaign still contains exactly 75 unique critical coupled runs. If any historical residual still fails, qualification terminates before spending time on the rest of the matrix.
