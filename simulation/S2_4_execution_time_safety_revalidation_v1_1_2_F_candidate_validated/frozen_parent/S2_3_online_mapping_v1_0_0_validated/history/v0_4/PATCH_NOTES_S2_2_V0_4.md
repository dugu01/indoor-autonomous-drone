# Stage S2.2 v0.4 Patch 1 — dynamic control, promotion, and sensor recovery

This is a cumulative replacement patch for the first v0.4 MATLAB candidate.

## Failures addressed

1. `dynamic_crossing_6dof`
   - The finite-candidate VO command changed discontinuously at 50 Hz.
   - A position lead (`p = p_est + 0.40 v_safe`) multiplied those changes through the position loop.
   - The geometric controller allowed up to 3 m/s² horizontal command acceleration and used an under-damped attitude-rate gain.

2. `dynamic_blocker_becomes_static_6dof`
   - Raw alpha-beta velocity at 50 Hz was too noisy for a 0.10 m/s stopped threshold.
   - The stopped-persistence timer repeatedly reset, so promotion and replanning never occurred.

3. `obstacle_sensor_dropout_recover_6dof`
   - Coverage recovery rebuilt a trajectory even though the static map had not changed.
   - The restart used an instantaneous finite-difference ESKF acceleration, which could reject an otherwise valid recovery trajectory and cause a false failsafe.

## Corrections

- Added acceleration/jerk-limited safety velocity shaping.
- Changed dynamic avoidance to velocity-mode reference tracking anchored at the current local estimate.
- Added horizontal command-acceleration and controller jerk limits.
- Increased roll/pitch rate damping.
- Added filtered obstacle velocity and used it for prediction and stopped detection.
- Reduced alpha-beta beta gain from 0.12 to 0.05 for the 50 Hz/noise configuration.
- Coverage recovery now rejoins the paused validated trajectory; it does not replan unless the map changed.
- Added filtered/clamped ESKF acceleration for genuine map-change replans.
- Tightened the rejoin-to-TRACK transition to 0.30 m while leaving all pass thresholds unchanged.

## Files to replace

- `init_S2_2_config.m`
- `alpha_beta_track_S2_2.m`
- `geometric_controller_S2_2.m`
- `mission_manager_S2_2.m`

## New file

- `shape_velocity_command_S2_2.m`

## Important Python audit finding

The original v0.4 Python backtest was not validation-equivalent to MATLAB:

- blocker promotion was hard-coded at 10.5 s rather than produced by the noisy alpha-beta tracker;
- the Python pass expression did not enforce MATLAB's executed speed, acceleration, jerk, tilt, and controller-tracking gates.

Therefore the earlier `40/40 PASS` result must be treated as an architecture smoke test, not as proof of the MATLAB v0.4 controller and tracker gates.
