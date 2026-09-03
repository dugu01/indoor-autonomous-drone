# S2.5 v1.0.9 VALIDATED — Clean Build / Freeze Manifest

- Release status: **VALIDATED / FROZEN PARENT FOR S2.6**.
- Qualified MATLAB runtime: **71/71 unique coupled missions PASS**.
  - No-fault baselines: **5/5 PASS**.
  - Recoverable fault matrix: **60/60 PASS**.
  - Fail-safe fault matrix: **6/6 PASS**.
  - Focused v1.0.9 remaining-four preflight: **4/4 PASS**.
- Inherited S2.4-F regression: **PASS**.
- Validated parent S2.4-G v1.0.4: **353/353 byte-identical before and after MATLAB qualification**.
- Frozen S2.3 / closure manifests remain inherited and unchanged.
- Estimator, controller, map-safety, trajectory, stopping, authority, scan, inflation, route-freshness, unknown-space, and mission-state timeout thresholds: **unchanged**.
- CSE/SIE recovery architecture retained.
- Packet-integrity sanitizer retained: per-packet voxel endpoint de-duplication and static-occlusion consistency.
- NAV degraded-hold recovery dead-state correction validated: cancelled pending replans are not restored as empty `LIFECYCLE_REPLAN_BRAKE`; recovery replans fresh via `PLAN_OUTBOUND`/`PLAN_RTL`.
- Brief perception safe-response evidence validated: stale perception while already in zero-translation replan brake is recorded as an S2.5 safe response; prolonged perception loss still requires actual named hold/failsafe behavior.
- Exact historical snapshot replay: **5/5 PASS**.
- Negative controls: **14/14 PASS**.
- Fixed-seed local perturbations: **10/10 PASS**.
- Recovery Monte Carlo: **40/40 safe recovery; 0 unsafe**.
- Per-packet perception-integrity microtests: **PASS**.
- v1.0.9 static/isolation audit: **PASS**.
- MATLAB source sanity: **PASS**.
- Source-faithful fault/gate backtest: **PASS**.
- v1.0.9 lifecycle integration backtest: **PASS**.
- Parallel 71-mission harness semantics: **PASS**.
- Qualified runtime/source byte identity after release documentation edits: **PASS**.
- Validated release package inventory: regenerated and re-audited.

See `S2_5_VALIDATION_REPORT_v1_0_9.md` for the final evidence summary and `s2_5/evidence/validated_release/MATLAB_RUNTIME_QUALIFICATION_20260903.txt` for the complete MATLAB console log.
