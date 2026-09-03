# S2.5 v1.0.2 Recovery Design Review

## Source evidence

The user v1.0.1 recovery trace shows `direct=0` for every failed PLAN_OUTBOUND sample. Three cases repeatedly report `no_safe_active_viewpoint`. Two perception cases combine invalidations from request 16 and a later fresh request 25, reaching a global count of three and prematurely suppressing exploration.

## Design interpretation

A failure of the stricter S2.4 information-viewpoint policy is not proof that no safe known-free map-extension motion exists. The frozen S2.3 planner already supplies a conservative goal-directed frontier mode. S2.5 therefore reuses it as a recovery fallback instead of inventing a new planner.

An authority invalidation limit is logically scoped to the authority/request being retried. Reauthorizing the same request remains within one episode; a newly planned request is a new episode. Mission-level progress remains independently bounded by the inherited map-extension budget.

## Safety invariants retained

- S2.4-G parent is byte frozen.
- S2.4-F execution-safety runtime is unchanged.
- S2.5 sensor/perception fault models are unchanged from user-tested v1.0.1.
- `maxAuthorityInvalidations=3` unchanged.
- `mapMaxNoProgressScans=3` unchanged.
- `mapMaxExtensionAttempts` unchanged.
- estimator and mapping safety thresholds unchanged.
- fallback uses the frozen S2.3 known-free frontier planner, strict trajectory generator and known-free stop validator.
- no stale exploration request regains authority.

## Runtime claim boundary

The packaging environment has no MATLAB/Octave. Source/static/backtest PASS does not establish that the five user-failed cases are repaired. The validator therefore runs those exact cases first and aborts before the long matrix if any fails.
