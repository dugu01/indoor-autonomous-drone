# MATLAB Validation Protocol — Stage S2.2 v0.5 Consolidated Candidate

## 1. Preserve the validated parent

Keep unchanged:

```text
simulation/S2_2_mission_replanning_v0_4_validated/
```

Install this package as:

```text
simulation/S2_2_mission_replanning/
```

Do not apply the earlier v0.5 Patch 1 or Patch 2; their corrections are already incorporated and expanded here.

## 2. Run the cumulative audit

From Terminal in the v0.5 folder:

```bash
python3 audit_S2_2_v0_5.py
```

Expected:

```text
42/42 checks passed
CUMULATIVE STATIC AUDIT: PASS
```

## 3. Reset the MATLAB path

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

## 4. Nominal lifecycle first

```matlab
r = run_S2_2_mission_replanning( ...
    0,'full_mission_nominal',false,false);
```

Expected preflight pattern:

```text
Preflight H/V/A/L/C/U/home/goal/route: 1 / 1 / 1 / 1 / 1 / 1 / 1 / 1 / 1
```

Expected lifecycle outcome:

```text
arm = 1
takeoff = 1
goal = 1
RTL executed = 1
landed = 1
disarmed = 1
preflight reject = 0
emergency = 0
mission complete = 1
collision/geofence = 0/0
RESULT = PASS
```

## 5. Remaining lifecycle cases

```matlab
r1 = run_S2_2_mission_replanning(0,'rtl_obstacle_replan',false,false);
r2 = run_S2_2_mission_replanning(0,'alternate_landing_zone',false,false);
r3 = run_S2_2_mission_replanning(0,'preflight_reject_unsafe_home',false,false);
r4 = run_S2_2_mission_replanning(0,'xy_loss_emergency_land',false,false);
```

Required outcomes:

| Scenario | Required event |
|---|---|
| `rtl_obstacle_replan` | at least one actual RTL map repair; home landing |
| `alternate_landing_zone` | home rejected after blockage; safe reachable alternate selected |
| `preflight_reject_unsafe_home` | home gate fails geometrically; no arm/takeoff |
| `xy_loss_emergency_land` | one-shot RTL request; hold; vertical-only local landing; disarm |

## 6. Complete regression

```matlab
report = validate_S2_2(false);
```

This runs seven preserved v0.4 cases plus five v0.5 lifecycle cases. It completes all cases before reporting the consolidated failures.

## 7. Tabbed plots and animation

After all 12 no-plot runs pass:

```matlab
report = validate_S2_2(true);
```

All dashboards must appear as tabs in one MATLAB Figures window and save under:

```text
simulation/results/S2_2_mission_replanning/v0_5/
```

Critical animations:

```matlab
run_S2_2_mission_replanning(0,'full_mission_nominal',true,true);
run_S2_2_mission_replanning(0,'rtl_obstacle_replan',true,true);
run_S2_2_mission_replanning(0,'alternate_landing_zone',true,true);
run_S2_2_mission_replanning(0,'xy_loss_emergency_land',true,true);
```

Verify that inserted obstacles do not appear before their insertion times.

## 8. Five-seed lifecycle matrix

```matlab
mc = validate_S2_2_monte_carlo(0:4,false);
```

This now includes all five lifecycle scenarios, including geometric preflight rejection.

## Acceptance boundary

Do not freeze v0.5 until:

1. all 12 seed-0 scenarios pass;
2. plots are reviewed in one tabbed window;
3. critical animations are reviewed;
4. all 25 lifecycle Monte Carlo runs pass;
5. no safety, clearance, estimator, kinematic or lifecycle gate is relaxed to force a pass.
