# Stage S2.2 v0.5.3.1 — XY-loss brake-before-descent correction

## Trigger
Focused MATLAB validation failed only for `xy_loss_emergency_land`, seed 7.
The mission and emergency landing completed, but minimum wall clearance was
0.057 m, below the unchanged protected-clearance requirement.

## Root cause
The v0.5.3 emergency state began the vertical profile only 0.05 s after
entering `EMERGENCY_HOLD`, while the XY braking pulse continued for about
0.75 s. Horizontal braking and vertical descent therefore competed for the
same jerk-limited acceleration command. The remaining XY velocity then
persisted during the blind descent.

## Correction
1. Keep constant altitude until the XY brake pulse has completed and either:
   - trusted inertial XY speed is below 0.055 m/s; or
   - the short velocity-trust window has expired.
2. Within the trust window, combine the frozen last-reliable-velocity braking
   pulse with bounded inertial velocity damping to correct command lag.
3. After brake release, perform a level blind descent with no XY position,
   velocity, or feedforward-acceleration command.
4. Record brake-release time and speed.
5. Save results under `v0_5_3_1`.

No safety threshold, clearance requirement, estimator bound, mission
expectation, or kinematic limit was relaxed.

## Python evidence
- Cumulative source audit: 29/29 PASS.
- Updated multi-seed mechanism regression: PASS.
- Dedicated XY-loss regression: 10,000 trials PASS.
- Worst predicted horizontal drift: 0.310 m.
- 99th percentile predicted drift: 0.253 m.
- Worst predicted final XY speed: 0.066 m/s.

These are mechanism/source checks, not MATLAB runtime validation.
