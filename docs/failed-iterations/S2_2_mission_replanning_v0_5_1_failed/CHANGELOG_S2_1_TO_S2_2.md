# Changelog — S2.1 to S2.2

## S2.2 v0.5.1 — RTL planning recovery and transition-reference correction

- Corrected normal RTL failure when a grid route exists but the current moving state cannot immediately form a collision-free polynomial trajectory.
- Added brake-to-hover and repeated trajectory generation for outbound and RTL planning.
- Removed one-step ground/home reference leakage during `PLAN_OUTBOUND`, `PLAN_RTL`, recovery and emergency transitions.
- Recorded `rtlExecuted` correctly after a successful brake/retry recovery.
- Separated horizontal path tracking from vertical takeoff/landing altitude validation.
- Isolated outputs under `simulation/results/S2_2_mission_replanning/v0_5_1/`.
- Added a faithful runtime-semantic regression: 19/19 PASS.

## S2.2 v0.5 — Consolidated autonomous lifecycle candidate

Built from the exact user-uploaded, MATLAB-validated v0.4 package. This release supersedes the earlier v0.5 candidate and both standalone preflight patches.

### Parent isolation

- preserved the exact v0.4 mission core in `mission_manager_v0_4_core_S2_2.m`;
- restored the exact v0.4 ESKF as `multi_lane_eskf_S2_2.m` for legacy scenarios;
- added `multi_lane_eskf_lifecycle_S2_2.m` solely for lifecycle preflight/freshness outputs;
- verified 129 legacy configuration assignments remain unchanged, excluding only version/method labels.

### Added

- complete preflight-to-disarm mission state machine;
- arm/disarm, automatic takeoff, initial hover and goal-wait states;
- outbound flight, goal hover, obstacle-aware RTL, landing hover and descent;
- safe alternate landing-site selection when home is blocked;
- local emergency landing after observable XY navigation is lost;
- contact-confirmed land detector;
- accepted-aid age, covariance, eligibility and accepted-update preflight gates;
- takeoff, goal and RTL arrival dwell confirmation;
- vertical reference/executed speed, acceleration and jerk validation;
- immediate motor cutoff on transition to disarm;
- temporal obstacle replay in animation;
- full five-scenario lifecycle Monte Carlo matrix.

### Corrected during cumulative audit

- removed dependence on exact-step asynchronous packet flags during preflight;
- stopped treating the airborne goal as a landing zone;
- removed direct truth-state mission decisions;
- prevented recurring RTL requests from resetting emergency-hold time;
- disabled stale horizontal control during XY-loss descent;
- tied RTL obstacle event count to actual path repair;
- separated lifecycle ESKF modifications from validated v0.4 regressions;
- made source/result paths portable.

### Executed non-MATLAB evidence

- lifecycle/grid backtest: 50/50 PASS;
- focused lifecycle mechanism audit: 9/9 PASS;
- preserved v0.4 focused mechanism regressions: PASS;
- cumulative static/source audit: 42/42 PASS;
- Python syntax compilation: PASS.

### Runtime boundary

MATLAB v0.5 runtime is not claimed until the user executes the nominal lifecycle, all 12 scenarios, tabbed visual validation, critical animations and the five-seed lifecycle matrix.
