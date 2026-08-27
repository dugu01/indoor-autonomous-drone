# MATLAB Run Protocol — S2.4-E

Project root used below:

```matlab
projectRoot = '/Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2_4_uncertainty_active_exploration_development';
```

## 1. Run static/offline checks from MATLAB

```matlab
clear; clc; close all force;
projectRoot = '/Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2_4_uncertainty_active_exploration_development';
cd(projectRoot);
[status,out] = system('python3 coupled/validation/run_all_checks_S2_4_E.py');
fprintf('%s\n',out);
assert(status==0,'S2.4-E static/offline gate failed.');
```

Required ending:

```text
S2.4-E AGGREGATE STATIC/OFFLINE GATE: PASS
MATLAB coupled mission execution: PENDING LOCAL MATLAB RUN
```

## 2. Add the development paths

```matlab
restoredefaultpath;
parentRoot = fullfile(projectRoot,'frozen_parent','S2_3_online_mapping_v1_0_0_validated');
shadowRoot = fullfile(projectRoot,'s2_4_shadow');
missionRoot = fullfile(projectRoot,'coupled','mission');
executionRoot = fullfile(projectRoot,'coupled','execution');
scenarioRoot = fullfile(projectRoot,'coupled','scenarios');
validationRoot = fullfile(projectRoot,'coupled','validation');
addpath(parentRoot,'-begin');
addpath(shadowRoot,'-begin');
addpath(executionRoot,'-begin');
addpath(scenarioRoot,'-begin');
addpath(missionRoot,'-begin');
addpath(validationRoot,'-begin');
clear functions; rehash;
```

Confirm resolution:

```matlab
which run_S2_4_coupled -all
which mission_lifecycle_manager_S2_4 -all
which plan_active_exploration_segment_S2_4 -all
which validate_exploration_request_S2_4 -all
```

All four must resolve inside the `coupled` folder.

## 3. Run the fast request contracts

```matlab
requestReport = test_S2_4_E_request_contracts();
assert(requestReport.pass);
```

Required:

```text
S2.4-E REQUEST CONTRACTS: 6/6 PASS
```

## 4. Run the first coupled mission

```matlab
result = run_S2_4_coupled(0,'active_goal_requires_scan',false,false);
disp(result.summary);
```

Initial required fields:

```matlab
assert(result.summary.explorationRequestCount >= 1);
assert(result.summary.explorationSelectedCount >= 1);
assert(result.summary.explorationExecutedCount >= 1);
assert(result.summary.goalReached == 1);
assert(result.summary.rtlExecuted == 1);
assert(result.summary.landed == 1);
assert(result.summary.collisionCount == 0);
assert(result.summary.geofenceViolationCount == 0);
assert(result.summary.unknownCommitmentCount == 0);
assert(result.summary.unsafeViewpointExecutionCount == 0);
assert(result.summary.truthIsolationPass == 1);
assert(result.summary.explorationPass == 1);
assert(result.summary.pass == 1);
```

## 5. Run the combined gate

```matlab
gate = validate_S2_4_E_all(true);
assert(gate.pass);
```

## 6. Optional plots and animation after the numerical gate passes

```matlab
result = run_S2_4_coupled(0,'active_goal_requires_scan',true,true);
```

Do not enable plots/animation for the first diagnostic run because they can obscure the first numerical error.

## 7. Send back after the first run

Send:

- complete MATLAB console output;
- generated `summary_S2_4_E_*.txt`;
- generated `S2_4_E_*_trial_data.mat`;
- the full error and stack trace if any gate fails.
