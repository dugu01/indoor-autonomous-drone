# Stage S2.2 v0.5.2 — Position-Loss-Safe Autonomous Mission Lifecycle Candidate



## v0.5.2 position-loss correction

The MATLAB `xy_loss_emergency_land` run exposed two distinct defects in v0.5.1:

1. disabling horizontal position control also disabled horizontal velocity damping, so the vehicle retained its translational velocity during the hold and descent and drifted outside the room;
2. the pass gate compared the complete post-loss position error with an accuracy requirement even though absolute XY position is unobservable after both VIO and LiDAR aiding are lost.

v0.5.2 now pauses the mission immediately when every horizontal-aid lane becomes ineligible, damps short-term inertial horizontal velocity without using a stale XY position target, and then performs the local descent when the configured no-aid delay raises the failsafe. The total post-loss estimator error remains reported, but the frozen 0.25 m failsafe gate is applied only to the observable interval and the failsafe-trigger instant. Physical truth-based wall/obstacle clearance, collision and geofence gates remain mandatory.

Results are isolated under `simulation/results/S2_2_mission_replanning/v0_5_2/`.

See `RUNTIME_FAILURE_ANALYSIS_S2_2_V0_5_2.md` and run:

```bash
python3 s2_2_v0_5_2_emergency_regression.py
python3 audit_S2_2_v0_5.py
```

## v0.5.1 runtime correction

The first consolidated v0.5 MATLAB nominal run exposed a semantic integration defect: a valid RTL grid route existed, but a nonzero near-wall start state made the first smooth RTL trajectory invalid. v0.5.1 brakes to a near-hover and retries rather than declaring an emergency. Planning transitions also retain a safe airborne reference, and horizontal tracking is validated separately from altitude tracking. See `RUNTIME_FAILURE_ANALYSIS_S2_2_V0_5_1.md`.

The prior v0.5.1 results remain under `simulation/results/S2_2_mission_replanning/v0_5_1/` for traceability.

This package was rebuilt and cumulatively audited from the exact user-uploaded, MATLAB-validated Stage S2.2 v0.4 archive. It supersedes the earlier v0.5 package and the two preflight patches. **Do not apply either older v0.5 patch to this package.**

## Preservation of the validated parent

The seven v0.4 regression scenarios remain isolated from the new lifecycle code:

```text
legacy scenario
  → mission_manager_v0_4_core_S2_2.m
  → exact validated v0.4 multi_lane_eskf_S2_2.m

lifecycle scenario
  → mission_lifecycle_manager_S2_2.m
  → multi_lane_eskf_lifecycle_S2_2.m
```

The preserved v0.4 mission-core normalized SHA-256 is:

```text
9ac6336160b3ee68f032b6610fa6684a83854109ecddd3a854bd221a1353a483
```

The preserved v0.4 ESKF file SHA-256 is:

```text
b4cb3f8a18b3936520cf91fd0215c3f0e28c230294b6a9c367319aaf2b742d25
```

All 129 v0.4 configuration assignments other than the version and method-name labels are checked against `V0_4_CONFIG_BASELINE_S2_2.json`.

## v0.5 lifecycle

```text
PREFLIGHT
→ ARM
→ TAKEOFF
→ INITIAL_HOVER
→ WAIT_FOR_GOAL
→ PLAN_OUTBOUND
→ TRACK_OUTBOUND
→ GOAL_HOVER
→ PLAN_RTL
→ TRACK_RTL
→ LAND_HOVER
→ LAND_DESCENT
→ DISARM
→ COMPLETE
```

Safety branches:

```text
PREFLIGHT_REJECT
NAV_DEGRADED_HOLD → EMERGENCY_HOLD → EMERGENCY_LAND → DISARM
LIFECYCLE_REPLAN_BRAKE
FAILSAFE
```

## Consolidated corrections

The cumulative review corrected issues that were not covered by the first v0.5 candidate:

- preflight uses ages of **accepted** ESKF aiding updates, not sensor arrivals on one simulation step;
- aid freshness starts invalid and becomes valid only after accepted filter updates;
- preflight also checks active-lane eligibility, covariance, finite state and accepted-update count;
- the airborne mission goal is checked as a safe reachable cell, not as a landing footprint;
- lifecycle mission decisions use the selected local ESKF state and simulated onboard signals, not direct truth state;
- takeoff and arrival declarations require persistence dwell;
- landing requires persistent simulated contact, low estimated vertical speed and near-ground estimated altitude;
- complete XY-aid loss triggers once, holds briefly and then performs vertical-only local descent without returning to stale XY;
- repeated RTL requests cannot reset the emergency-hold timer;
- vertical reference and executed speed, acceleration and jerk are measured and gated;
- motor command is cut immediately when landing transitions to `DISARM`;
- RTL obstacle counts are recorded at actual path repair;
- animation replays static obstacle insertions at their actual times;
- Monte Carlo validation includes all five lifecycle scenarios.

## Main interface

```matlab
results = run_S2_2_mission_replanning( ...
    seed, scenarioName, makePlots, makeAnimation);
```

## Installation

Keep the parent unchanged:

```text
simulation/S2_2_mission_replanning_v0_4_validated/
```

Extract this package as:

```text
<project-root>/simulation/S2_2_mission_replanning/
```

## Static and Python audit

From a terminal in the package folder:

```bash
python3 audit_S2_2_v0_5.py
```

Expected final line:

```text
CUMULATIVE STATIC AUDIT: PASS
```

The audit reruns rather than merely reads:

- the 50-run lifecycle/grid matrix;
- the 9-test focused lifecycle mechanism audit;
- both preserved v0.4 patch-mechanism regressions;
- MATLAB source/function/call/field sanity checks.

These checks are not a substitute for MATLAB runtime validation.

## MATLAB validation order

Reset the path and add only the working v0.5 folder:

```matlab
restoredefaultpath;
rehash toolboxcache;

cd('<project-root>/simulation/S2_2_mission_replanning');
addpath(pwd,'-begin');

clear functions;
clear classes;
close all force;
clc;
rehash path;
```

Confirm one active copy:

```matlab
which run_S2_2_mission_replanning -all
which mission_manager_S2_2 -all
which mission_manager_v0_4_core_S2_2 -all
which multi_lane_eskf_S2_2 -all
which multi_lane_eskf_lifecycle_S2_2 -all
which mission_lifecycle_manager_S2_2 -all
which validate_S2_2 -all
```

Run the nominal lifecycle first:

```matlab
r = run_S2_2_mission_replanning( ...
    0,'full_mission_nominal',false,false);
```

The normal preflight gates should be:

```text
H/V/A/L/C/U/home/goal/route = 1/1/1/1/1/1/1/1/1
```

Then run the complete 12-scenario matrix:

```matlab
report = validate_S2_2(false);
```

After all no-plot runs pass:

```matlab
report = validate_S2_2(true);
mc = validate_S2_2_monte_carlo(0:4,false);
```

Results remain versioned under:

```text
simulation/results/S2_2_mission_replanning/v0_5/
```

## Scenario matrix

Seven preserved v0.4 scenarios:

```text
nominal_6dof
incremental_static_estimated
dynamic_crossing_6dof
dynamic_blocker_becomes_static_6dof
obstacle_sensor_dropout_recover_6dof
primary_imu_fault_vio_outage
xy_aid_loss_failsafe
```

Five v0.5 lifecycle scenarios:

```text
full_mission_nominal
rtl_obstacle_replan
alternate_landing_zone
preflight_reject_unsafe_home
xy_loss_emergency_land
```

## Engineering boundary

The landing-site model is map-geometric. It does not yet classify floor semantics, slope, roughness, people, moving platforms or surface material. Hardware deployment still requires actuator and battery models, calibration, HIL/SIL testing, real contact/land detection and safety review.
