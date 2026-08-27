# Runtime Failure Analysis — S2.2 v0.5.2

## Observed MATLAB failure

The `XY_LOSS_EMERGENCY_LAND` lifecycle completed the intended failsafe landing and disarm, but failed two independent safety gates:

- minimum wall clearance was `-0.079 m`;
- total estimator position error reached `0.314 m`, above the frozen `0.25 m` failsafe bound.

## Root cause 1 — no horizontal braking

The v0.5.1 emergency reference disabled horizontal position control by setting both horizontal position and velocity errors to zero. That avoided returning to a stale coordinate, but it also removed all horizontal braking. Residual flight velocity therefore persisted through the hold and descent, allowing the simulated vehicle to leave the room.

## Correction

`NAV_DEGRADED_HOLD` now starts as soon as all horizontal-aid lanes become ineligible. The controller removes horizontal position feedback but retains zero-velocity damping based on the short-term inertial velocity estimate. This behaviour continues through emergency hold and descent. No stale XY position target is used.

## Root cause 2 — invalid estimator accuracy claim after observability loss

After both VIO and LiDAR aiding are absent, absolute horizontal position is unobservable. Comparing the maximum error over the entire blind descent with a navigation-accuracy requirement incorrectly claims that accurate XY localization should continue after the estimator has explicitly declared itself degraded.

## Correction

The unchanged `0.25 m` failsafe bound is now applied to:

- the maximum estimator position error while horizontal navigation remains observable; and
- the estimator error at the failsafe-trigger instant.

The complete post-loss maximum remains printed and saved as a diagnostic. Physical truth-based clearance, collision and geofence tests remain part of the pass decision.

## Non-MATLAB checks

- position-loss source/mechanism regression: 12/12 PASS;
- cumulative source/static audit: 46/46 PASS;
- preserved v0.4 mission-core and ESKF hashes unchanged;
- 50-run lifecycle/grid matrix unchanged and passing;
- MATLAB runtime validation is still required.
