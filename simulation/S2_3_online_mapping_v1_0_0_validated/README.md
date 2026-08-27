# Stage S2.2 v0.5.3.3 — Multi-Seed Robust Autonomous Replanning Candidate

This candidate is built from the deterministic 12/12-PASS v0.5.2 package and addresses the four mechanisms exposed by the 48/60 multi-seed result.

## Cumulative v0.5.3.3 changes

- REJOIN progress watchdog and route rebuilding;
- verified low-speed grid-route fallback before failsafe/emergency landing;
- recent-NIS fault attribution and short blend for confirmed faulty-IMU switching;
- early XY-aid-loss detection;
- open-loop horizontal braking from the last aid-bounded velocity;
- blind vertical descent without stale XY position or drifting-velocity feedback;
- detailed recovery, fault-switch, final-state and drift diagnostics;
- a dedicated 60-run MATLAB robustness validator.

The exact validated v0.4 files remain in the package as frozen references:

- `mission_manager_v0_4_core_S2_2.m`
- `multi_lane_eskf_S2_2.m`

The active legacy-scenario derivatives are:

- `mission_manager_v0_5_3_core_S2_2.m`
- `multi_lane_eskf_robust_S2_2.m`


### Inherited v0.5.3.2 XY-loss control correction

- removes post-loss ESKF velocity damping from the frozen braking pulse;
- bypasses the normal speed guard whenever horizontal state is unobservable;
- waits 0.16 s after the pulse so jerk-limited horizontal acceleration returns to zero before descent.

## Acceptance scripts

Use only these v0.5.3.3 acceptance entry points:

- `audit_S2_2_v0_5_3_3.py`
- `s2_2_v0_5_3_3_touchdown_replay_regression.py`
- `validate_S2_2_v0_5_3_focus.m`
- `validate_S2_2.m`
- `validate_S2_2_multiseed_robustness.m`

Older version-specific regressions are retained under `history/` for traceability and are not v0.5.3 acceptance tests.

## First checks

```bash
python3 audit_S2_2_v0_5_3_3.py
python3 s2_2_v0_5_3_3_touchdown_replay_regression.py
```

## MATLAB validation

```matlab
restoredefaultpath;
rehash toolboxcache;
cd('/path/to/indoor-autonomous-drone/simulation/S2_2_mission_replanning');
addpath(pwd,'-begin');
clear functions; clear classes; close all force; clc; rehash path;

focus = validate_S2_2_v0_5_3_focus();             % require 12/12 former failures
report = validate_S2_2(false);                    % require 12/12 deterministic
robust = validate_S2_2_multiseed_robustness(0:9); % require 60/60
```

Only after both pass:

```matlab
report = validate_S2_2(true);
```

All plots remain docked as tabs in one MATLAB window. Results are isolated under:

`simulation/results/S2_2_mission_replanning/v0_5_3_3/`

See `MULTISEED_ROBUSTNESS_REPORT_S2_2_V0_5_3.md` and `LITERATURE_S2_2.md`.

## v0.5.3.3 exact-trace touchdown correction

The v0.5.3.2 seed-7 trial proved that XY braking succeeded but landing
recognition was delayed by contact rebound. v0.5.3.3 gates contact detection on
vertical-profile completion and latches the first qualified contact event.
See `EXACT_TRACE_DEBUG_REPORT_S2_2_V0_5_3_3.md` and run:

```bash
python3 audit_S2_2_v0_5_3_3.py
python3 s2_2_v0_5_3_3_touchdown_replay_regression.py
```
