# S2.4-G — Full Closed-Loop Mission Qualification

**Candidate:** `v1.0.4-G-candidate`  
**Qualification ID:** `v0.5.4-full-closed-loop-yaw-slew-candidate`

## v1.0.4 — source-traced yaw transient correction

The user-executed v1.0.3 G campaign remains **71/75**, but the residual mechanism is now fully localized. F2/F3/F9/F10 MID seed=3 all complete the mission with hard safety and truth isolation PASS; only the estimator attitude gate fails. A five-case peak diagnostic shows the four faults have the same 2.140175 deg maximum at t=28.02 s, with 2.140151 deg in the yaw-like quaternion component and only 0.010013 deg tilt component. The corresponding truth angular-rate magnitude peaks at about 204.865 deg/s. Position estimation remains well inside its 0.10 m bound.

Source tracing found that accepted exploration yaw was assigned directly (`yawCommand=plannedRequest.yaw`) into a geometric controller that has no desired-yaw-rate state. That makes a new viewing orientation an angle step. v1.0.4 replaces this with a bounded shortest-path yaw target transition using the existing inherited S2.3 `mapScanYawRate_radps = 35 deg/s`, which was itself introduced after a 55 deg/s/resetting scan yaw caused a >2 deg attitude-estimator failure. The estimator threshold is **not relaxed**.

When exploration authority is revoked, its viewing direction is revoked too: the brake/replan phase resets the yaw reference to the current estimated yaw, so the controller damps residual angular rate rather than continuing toward stale exploration orientation. A newly admitted/reauthorized request may set a new yaw target, but it is again slew-limited.

The v1.0.4 local yaw-axis backtest uses the inherited `Jz=0.055 kg m^2`, yaw attitude gain 2.2, rate damping 0.25, angular drag 0.02, moment cap 0.35 N m and dt=0.02 s. A 90 deg angle step yields ~199.8 deg/s peak yaw rate; a 35 deg/s target slew yields ~45.2 deg/s in the same model. This supports the mechanism but does not replace the required MATLAB ESKF qualification.


The user-executed v1.0.2 coupled campaign improved the targeted matrix from **52/75 to 71/75**. More importantly, it showed:

- 5/5 no-fault seeds PASS;
- the complete A-D/E/F gates PASS;
- hard safety PASS across the matrix;
- actual map/uncertainty truth-access isolation PASS;
- the earlier `unknown=23` failures were eliminated by the outbound+RTL reference guard;
- every late F2/F3/F9/F10 recovery case now completed through fresh reauthorization where needed.

The only four failures were **F2/F3/F9/F10, MID, seed=3**. Each printed `inj=1 det=1 resp=1`, zero stale/collision/geofence/unknown/truth violations, `timeout=0`, `goalUnreachable=0`, `final=COMPLETE`, `tGoal=57.90`, and `tComplete=108.04`. Therefore v1.0.3 does **not** add another runtime/autonomy patch. It corrects a qualification-wrapper conflation found by tracing the exact source.

## Exact v1.0.2 qualification bug

For critical fault runs, v1.0.2 defined mission completion as:

`missionComplete && goalReached && ~goalUnreachable && ~stateTimeoutTriggered && summary.pass`

But `summary.pass` is the full nominal S2.4 scenario composite. For `ACTIVE_GOAL_REQUIRES_SCAN` it contains `explorationPass`, which requires at least one **completed exploration viewpoint**. After a fault revokes the selected authority, the current map can legitimately reveal a safe direct route to the mission goal. In that case the fault response can be correct and the mission can finish, while the revoked viewpoint is never executed. Reusing nominal `summary.pass` therefore mislabels a completed fault-recovery mission as `mission=0`.

## v1.0.3 qualification semantics

No-fault baselines still require the complete nominal `summary.pass`, so E's exploration-execution contract is not weakened.

Critical fault runs now require all of the following separately:

1. literal mission completion: `missionComplete`, `goalReached`, final state `COMPLETE`, no `GOAL_UNREACHABLE`, no state timeout;
2. the complete mission-manager composite **except only `explorationPass`**:
   - mission lifecycle outcome;
   - trajectory gate;
   - controller gate;
   - estimator gate;
   - continuity gate;
   - uncertainty gate;
   - static/collision/geofence gate;
   - mapping composite gate;
   - execution-safety gate;
3. post-acceptance fault injection;
4. fault detection;
5. fault-specific required response;
6. bounded authority invalidations;
7. zero stale command continuation, collision, geofence, unknown commitment, unsafe viewpoint execution and actual truth access.

E's nominal exploration behavior remains independently re-qualified by the full E+F gate before the 75-run campaign. A critical fault cannot inject until an exploration authority has already been accepted.

## Runtime scope

The S2.4 runtime in v1.0.3 is byte-identical to G v1.0.2. In particular, the reviewed v1.0.2 one-file delta to validated F v1.1.2 remains unchanged:

`coupled/mission/mission_lifecycle_manager_S2_4.m`

No D*/A*, A* recovery, frontier selection, trajectory generation, controller, estimator, plant, E geometry/policy or frozen S2.3 logic is changed in v1.0.3.

## Qualification composition

1. Complete A-D MATLAB shadow qualification.
2. Complete E + F coupled qualification.
3. Five no-fault `ACTIVE_GOAL_REQUIRES_SCAN` seeds (`0:4`) using the **full nominal stage pass**.
4. F2/F3/F6/F9/F10 × early/mid/late × seeds 0:4 = 75 critical runs.
5. Post-run frozen-parent and reviewed-F-delta integrity audit.

The 75-run safety/mission threshold remains **75/75**; no pass threshold is relaxed.

## Cross-check/backtests added in v1.0.3

`backtest_S2_4_G_v102_user_log.py` parses the actual user MATLAB v1.0.2 console log and verifies the exact four residual signatures.

`audit_S2_4_G_qualification_semantics.py` compares the G critical gate against the mission manager's source composite and fails unless the critical gate preserves every composite component except nominal `explorationPass`. It also verifies no-fault baselines still require `summary.pass`.

`backtest_S2_4_G_matrix.py` contains positive and negative controls for the semantic split. A completed fault run with only nominal `explorationPass=false` qualifies, while any failure of controller, estimator, trajectory, mapping, continuity, uncertainty, static safety, execution safety, truth isolation, mission completion, fault response or bounded retries still fails.

## Local command

```bash
python3 coupled/validation/run_all_checks_S2_4_G.py
```

MATLAB/Octave is unavailable in the packaging environment, so coupled MATLAB PASS is not claimed locally.

## One MATLAB command

```matlab
gate = run_validate_S2_4_G_all();
```

Do not freeze S2.4-G unless the final result is **5/5 no-fault + 75/75 critical PASS**, with mission completion, closed-loop integrity, hard safety, actual truth isolation and frozen-parent integrity all PASS.
