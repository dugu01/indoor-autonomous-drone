# S2.5 Estimation + Perception Robustness — v1.0.9 VALIDATED

## Release status

**Status:** VALIDATED / FROZEN PARENT FOR S2.6

**Qualified candidate archive SHA-256:** `cf49d8912fd384618097235249d807ae5ac357bea290a53bac76cc695830661f`

**MATLAB runtime evidence SHA-256:** `6c66f7ddbe77667dee797e287f7bbb503ebef9eb570e22d912efd17b76f2fa15`

The runtime/autonomy source in this validated release is the same v1.0.9 source that was qualified in MATLAB. Release-only documentation and manifests were added after qualification; no behavioral source, safety threshold, estimator threshold, controller threshold, planner threshold, mapping threshold, timeout, inflation radius, route-freshness rule, or unknown-space rule was changed during the freeze step.

## MATLAB qualification result — 2026-09-03

Focused remaining-four preflight:

- NAV_HIGH_NOISE seed 4: PASS — goal reached, no failsafe, no state timeout, execution safety PASS.
- PERCEPTION_DUAL_BRIEF seed 2: PASS — safe stale-brake response observed.
- PERCEPTION_STALE_BURST seed 2: PASS — safe stale-brake response observed.
- COUPLED_IMU_PERCEPTION seed 2: PASS — safe stale-brake response observed.
- Remaining-four gate: **4/4 PASS**.

Full S2.5 qualification:

- Validated S2.4-G parent: **PASS**.
- Inherited S2.4-F regression: **PASS**.
- No-fault baselines: **5/5 PASS**.
- Historical recovery preflight: **5/5 PASS**.
- Recoverable fault matrix: **60/60 PASS**.
- Fail-safe fault matrix: **6/6 PASS**.
- Total unique coupled missions: **71/71 PASS**.
- Post-run S2.4-G parent byte identity: **353/353 PASS**.
- Final MATLAB verdict: **S2.5 ESTIMATION + PERCEPTION ROBUSTNESS: PASS**.

MATLAB results path recorded by the qualification run:

`results/S2_5_qualification/20260903_181538`

The complete user-executed MATLAB console log is archived at:

`s2_5/evidence/validated_release/MATLAB_RUNTIME_QUALIFICATION_20260903.txt`

## Rechecked offline / source gates on the exact qualified candidate

Before freezing, the exact candidate ZIP was freshly extracted and rechecked in the build environment:

- S2.4-G frozen parent byte identity: **353/353 PASS**.
- v1.0.9 static/isolation audit: **PASS**.
- MATLAB source sanity: **PASS** (16 S2.5 MATLAB source files).
- Source-faithful fault/gate backtest: **PASS**.
- v1.0.9 lifecycle integration backtest: **PASS**.
- Perception-integrity microtests: **PASS**.
- Parallel 71-mission harness semantics: **PASS**.
- Exact historical snapshot replay: **5/5 PASS**.
- Negative controls: **14/14 PASS**.
- Fixed-seed local perturbations: **10/10 PASS**.
- Recovery Monte Carlo: **40/40 safe recovery, 0 unsafe**.
- Candidate package SHA inventory before freeze metadata: **495/495 PASS**.

MATLAB itself was not available in the build environment; the MATLAB claims above are grounded in the archived user-executed qualification log.

## Validated v1.0.9 corrections

1. **Continuous Start Egress + Safe Informative Excursion** retained for recovery from continuous/discrete start-admissibility mismatches and informative safe relocation.
2. **Perception packet integrity overlay** retains one occupied endpoint update per voxel per packet and static-occlusion consistency, preventing a single corrupted packet from promoting a false persistent obstacle through duplicate ray hits.
3. **NAV degraded-hold recovery dead-state correction:** when a pending replan was cancelled during navigation degradation, recovery does not restore an empty `LIFECYCLE_REPLAN_BRAKE`; it returns through `PLAN_OUTBOUND`/`PLAN_RTL` and creates a fresh current-state plan.
4. **Brief perception safe-response evidence:** stale perception while already in zero-translation `LIFECYCLE_REPLAN_BRAKE` counts as the S2.5 safe degraded-perception response. The inherited `perceptionHoldCount` remains exposed, and prolonged perception loss still requires the actual named hold/failsafe behavior.

## Freeze contract

This release is the immutable validated parent for **S2.6 ROS 2 / PX4 Software-in-the-Loop integration**.

Downstream work must not modify this tree in place. Any behavioral change discovered during S2.6 or later phases must be implemented as a new child/version and regression-tested against this frozen release.
