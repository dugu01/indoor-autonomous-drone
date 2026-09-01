# Runtime Failure Analysis — S2.2 v0.5.1

The nominal v0.5 MATLAB run passed preflight, takeoff, outbound planning, goal arrival, landing, estimator and geometric safety checks, but incorrectly entered emergency landing at the goal instead of executing normal RTL.

## Evidence from MATLAB

- Goal reached: yes
- Preflight gates: all pass
- Normal RTL executed: no
- Emergency landing/failsafe: yes
- Selected emergency landing point: approximately the goal
- D*/A* expansions: a geometric return route existed
- Controller gate: failed due one-step reference jumps during planning transitions

## Root causes

1. `PLAN_RTL` treated `routeExists && ~traj.valid` as an immediate emergency. Near the wall, a valid grid route can exist while the current nonzero velocity/acceleration cannot be connected to that route by a collision-free seventh-order polynomial. The correct response is brake-to-hover and retry.
2. `PLAN_OUTBOUND` and `PLAN_RTL` began each loop with the default ground reference. When either state changed inside its switch case, that ground reference remained active for one integration step, causing artificial horizontal/altitude tracking spikes and an unsafe transient command.
3. A successful pending retry into `TRACK_RTL` did not set `rtlExecuted` or initialise the RTL tracking start time.
4. The reported tracking metric mixed horizontal path tracking with vertical takeoff/landing motion. Horizontal tracking and altitude tracking are now validated independently.

## Corrections

- Route present but trajectory invalid → `LIFECYCLE_REPLAN_BRAKE` → repeated near-hover trajectory attempts.
- No route present → emergency landing remains the correct response.
- Accepted plans use the exact trajectory start state on the same integration step.
- Planning and recovery states always command a safe airborne hold reference.
- Pending RTL recovery explicitly records normal RTL execution.
- Results are isolated under `v0_5_1`.

## Verification completed before packaging

- Cumulative static/source audit: 43/43 PASS
- Runtime-semantic planning regression: 19/19 PASS
- Python lifecycle matrix: 50/50 PASS
- Focused lifecycle mechanism audit: 9/9 PASS
- Preserved v0.4 mechanism regressions: PASS
- MATLAB source sanity scan: PASS

MATLAB runtime remains the authoritative coupled validation.
