# S2.4-G v1.0.4 Clean Build Manifest

Basis: S2.4-G v1.0.3 candidate whose user MATLAB campaign achieved 71/75 critical PASS with mission completion, hard safety and truth isolation PASS. The residual four cases were estimator-attitude-only failures.

## Runtime delta

Relative to G v1.0.3 autonomy runtime, exactly one runtime source file is changed:

- `coupled/mission/mission_lifecycle_manager_S2_4.m`

Reviewed SHA-256: `880aa4bb8057bf3f1b999fd2bf2262d92a363e278331426fa511529d72b80d63`

Changes in that file are limited to:

1. exploration viewing yaw stored as `yawTargetCommand` rather than stepped directly into the controller;
2. shortest-path slew using inherited `cfg.mapScanYawRate_radps` (35 deg/s) during `TRACK_OUTBOUND`;
3. stale viewing-yaw target discarded on execution/perception/reference-guard authority revocation by resetting reference to current estimated yaw;
4. added diagnostic `yawRef` log and `maxTruthYawRate_degps` summary field.

No change to the 2.0 deg estimator-attitude threshold, ESKF, geometric-controller gains, plant, trajectory generator, D*/A*, A* recovery, exploration policy/geometry or frozen S2.3.

## Qualification-wrapper changes

- historical residual preflight: F2/F3/F9/F10 MID seed=3;
- cached reuse of those four results in the 75-run matrix, preserving 75 unique fault runs;
- failure output includes estimator-attitude maximum and maximum truth yaw rate;
- v1.0.4 yaw-slew source/first-principles backtest and runtime-delta audit added.

## Local backtests

- A-D 15/15 static/offline: PASS
- E static/offline + literal geometry: PASS
- F static/isolation + source-faithful F1-F14/STOP/SCH: PASS
- G v1.0.2 recovery/reference-guard regressions: PASS
- G v1.0.3 qualification semantics regressions: PASS
- G v1.0.4 yaw-slew source/first-principles regression: PASS
- frozen S2.3: 208 checked, 0 missing/changed/extra
- MATLAB/Octave: unavailable in packaging environment; coupled v1.0.4 qualification is not claimed locally.
