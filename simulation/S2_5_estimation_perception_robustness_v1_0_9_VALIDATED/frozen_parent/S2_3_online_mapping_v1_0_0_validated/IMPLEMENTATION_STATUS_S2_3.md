# S2.3 implementation status

## Current package

This is one cumulative Stage S2.3 development candidate. It is not a formal
intermediate version and it is not a validated v1.0.0 release.

## Completed

- Full source-level integration from the supplied frozen S2.2 release.
- Perception simulation, probabilistic mapping, fail-closed map projection,
  unknown-space planning, mission integration, validation harness and tabbed
  plotting.
- First MATLAB runtime diagnosis and freshness-based preflight correction.
- Second MATLAB runtime diagnosis from the complete saved v7.3 MAT file.
- Recorded-state reproduction of extension exhaustion, replan storm, landing
  horizon exhaustion, map-quality defects and hard-limit mismatch.
- Cumulative corrections for route-aware replanning, extension completion,
  endpoint exclusion, unique static promotion, strict trajectory generation and
  continuous scan yaw.
- Frozen S2.2 SHA-256 comparison: 110 inherited files unchanged.
- MATLAB source sanity: 81 files pass.
- Truth-isolation static audit: pass.
- Cumulative static architecture audit: pass.
- Recorded seed-0 cross-check: pass.
- Independent mechanism backtests: 15/15 pass.

## Not completed

- MATLAB rerun of the cumulative backtested candidate.
- Coupled nominal S2.3 acceptance.
- Deterministic scenario acceptance.
- Focused MATLAB runtime regressions for newly corrected mechanisms.
- Frozen S2.2 regression from the S2.3 folder.
- S2.3 60-trial critical robustness matrix.

The folder must remain named `S2_3_online_mapping_development` until every
MATLAB acceptance gate passes.

## Third-run cumulative correction status

The latest supplied MATLAB run completed the full lifecycle but failed the
nominal goal and map-quality gates. Saved-map cross-checking identified
ceil-cell over-inflation and static-evidence erosion. Both mechanisms are now
corrected in the cumulative development candidate. Independent tests pass, but
the corrected code has not yet been rerun in MATLAB.

## Exact replay and boundary-policy gate

- Raw perception records captured in MATLAB: 1342.
- Exact mapper arrays reproduced: PASS.
- Exact production map metrics reproduced: PASS.
- Old idle-cycle counter mismatch: informational only; accepted-packet replay
  does not invoke no-sensor-event control cycles.
- Known room/geofence boundary policy replay: PASS.
- Replayed candidate metrics: false-free 0.00174985, recall 0.990054.
- Coupled boundary integration: implemented; MATLAB rerun still required.
