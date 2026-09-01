# S2.4-F — Execution-Time Safety Revalidation

**Candidate:** `v1.1.2-F-candidate`  
**Runtime ID:** `v0.4.2-execution-safety-revalidation-candidate`  
**Baseline:** exact uploaded validated S2.4-E RC3 clean archive  
**Frozen parent:** `S2_3_online_mapping_v1_0_0_validated`

This is a cumulative S2.4 package: validated A-D/E source plus the F execution-time exploration-authority supervisor and validation-only post-acceptance fault suite.

## What F changes
- revalidates an accepted exploration authority while it is actually executing;
- checks remaining known-free route, viewpoint, hold support, current stopping reserve, retreat authority, map version and expiry;
- renews E's existing 1.0 s request TTL only after a successful CURRENT revalidation;
- never revives an already expired request;
- revokes authority on inherited perception-degraded hold;
- routes invalid authority into the inherited `LIFECYCLE_REPLAN_BRAKE` and fresh `PLAN_OUTBOUND` path;
- bounds repeated invalidations at three;
- injects F faults only after acceptance and only while the current exploration authority is in `TRACK_OUTBOUND`.

No new planner/controller/plant was introduced. D*/A*, A*, trajectory generation, geometric controller, RTL, landing and failsafe are retained.

## v1.1.2 correction
The user's v1.1.1 coupled MATLAB run preserved the full S2.4-E gate but showed that F revoked three consecutive no-fault exploration authorities with `KNOWN_FREE_STOP_INVALID` before validation faults could inject. The root cause was specific: F was re-running the frozen S2.3 **planning-time terminal stop gate** on every execution sample. That gate includes endpoint-footprint clearance and is not the inherited runtime supervisor.

v1.1.2 removes that misuse. Planning still uses frozen `validate_known_free_stop_S2_3` unchanged. Runtime F now uses the same first-principles stopping-distance model against the **remaining already-inflated known-free route**. If remaining route length is shorter than conservative stopping distance, only the shortfall must be covered by a known-free terminal overrun region around the already validated viewpoint. This avoids the demonstrated false abort without accepting insufficient stopping support.

The local backtest now explicitly recreates the v1.1.1 false-abort geometry (`R1`) and also checks the converse (`STOP`): a high-speed short route with unavailable terminal overrun support is rejected.

## Local validation completed
Run in the packaging environment:

```bash
python3 coupled/validation/run_all_checks_S2_4_F.py
```

Result: inherited A-E static/offline regression PASS, frozen parent 208/208 byte-identical, F static/isolation PASS, source-faithful F1-F14 stop/fault/scheduler backtest PASS, including `R1` and `STOP` regression cases.

MATLAB and Octave are not installed in the packaging environment. Therefore no coupled v1.1.2 MATLAB PASS is claimed.

## One MATLAB command
Start a fresh MATLAB session, `cd` to this extracted project root, then run:

```matlab
gate = run_validate_S2_4_F_all();
```

Do not call S2.4-F validated until that coupled gate passes. F15 remains N/A until a live predictive moving-obstacle interface exists.
