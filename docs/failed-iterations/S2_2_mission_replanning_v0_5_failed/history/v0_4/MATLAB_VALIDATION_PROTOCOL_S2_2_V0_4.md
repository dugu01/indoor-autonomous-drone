# MATLAB Validation Protocol — Stage S2.2 v0.4

## 1. Preserve v0.3

Rename the validated source folder to:

```text
S2_2_mission_replanning_v0_3_validated
```

Do not put v0.3 and v0.4 on the MATLAB path simultaneously.

## 2. Install v0.4

Place the new package at:

```text
simulation/S2_2_mission_replanning/
```

## 3. Path audit

```matlab
restoredefaultpath;
rehash toolboxcache;
cd('/Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2_2_mission_replanning');
addpath(pwd,'-begin');
clear functions; clear classes; close all force; clc; rehash path;

which run_S2_2_mission_replanning -all
which mission_manager_S2_2 -all
which multi_lane_eskf_S2_2 -all
which validate_S2_2 -all
```

Every command should return exactly one v0.4 path.

## 4. No-plot validation

```matlab
report = validate_S2_2(false);
```

The validator completes all seven scenarios and writes a consolidated report even when one scenario fails.

## 5. Plot validation

Only after no-plot validation passes:

```matlab
clear functions; clear classes; close all force; clc;
report = validate_S2_2(true);
```

Confirm seven docked figure tabs and versioned PNG/FIG files.

## 6. Animation

```matlab
r = run_S2_2_mission_replanning(0,'primary_imu_fault_vio_outage',true,true);
```

## Failure reporting

Paste the complete console block for every failed scenario. The console prints separate gates for:
- static and dynamic safety;
- reference and executed kinematics;
- controller performance;
- estimator performance;
- uncertainty inflation;
- mission, failsafe and required events.
