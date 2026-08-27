STAGE S2.2 v0.3 — CUMULATIVE ALL-SCENARIO PATCH

This patch supersedes the earlier incremental-only and dynamic-crossing patches.
Replace all ten .m files together; do not mix old and new copies.

Audited validation scenarios
1. incremental_static_insert
2. dynamic_crossing_yield
3. dynamic_blocker_becomes_static
4. sensor_dropout_recover
5. sensor_dropout_failsafe
6. two_dynamic_crossings
7. trajectory_time_rescale

Cumulative corrections
- Conservative supercover grid traversal for path and trajectory collision checking.
- Exact continuous-state anchoring at every initial plan/replan.
- D* Lite repair/refresh uses the NEW occupancy grid before moving the start.
- Fresh A* recovery if a D* path or generated polynomial is invalid.
- Temporary dynamic avoidance resumes the paused validated trajectory instead of unnecessary replanning.
- VO fallback may not cross an inflated static cell; no safe command means hold.
- Stale dynamic tracks and stop timers reset when an object disappears.
- Stopped-object promotion requires multiple tracker updates before persistence timing.
- Static braking lookahead uses the same conservative segment checker.
- Time-scale metric is measured against the requested nominal timing, including fallback mode.
- No-data recovery has D* refresh plus A* fallback before declaring failsafe.
- All hidden core, mission, and event validation gates are printed and saved.
- Infinite dynamic-clearance values are plotted as N/A instead of sent to bar().
- Validator runs all seven scenarios before issuing a consolidated error.

Static checks completed here
- Patch file/function presence: PASS
- Delimiter/static source audit: PASS
- Independent Python trajectory backtest: 55/55 PASS

MATLAB runtime is not available in this environment. Final runtime confirmation must come
from validate_S2_2(false) on the project MATLAB installation.
