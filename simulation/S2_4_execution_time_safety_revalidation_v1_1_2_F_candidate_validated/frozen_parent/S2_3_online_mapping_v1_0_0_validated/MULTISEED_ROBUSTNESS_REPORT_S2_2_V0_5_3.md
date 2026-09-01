# S2.2 v0.5.3 Multi-Seed Robustness Correction Report

## Evidence that triggered this revision

The MATLAB v0.5.2 deterministic matrix passed all 12 scenarios, but the six-scenario, ten-seed sweep passed only 48/60 runs:

| Scenario | v0.5.2 result |
|---|---:|
| Dynamic crossing | 8/10 |
| Dynamic blocker becomes static | 10/10 |
| Primary IMU fault + VIO outage | 6/10 |
| RTL obstacle replan | 9/10 |
| Alternate landing zone | 10/10 |
| XY-loss emergency landing | 5/10 |

The failed mechanisms were not merged into one generic failure:

1. Dynamic crossing: safe but non-progressing REJOIN states.
2. IMU fault: successful lane switch but 0.102–0.119 m peak estimator transients against the unchanged 0.100 m requirement.
3. RTL obstacle seed 2: a grid route existed, but repeated smooth-trajectory rejection escalated to emergency landing.
4. XY loss: emergency landing completed, but several seeds violated the 0.502 m protected clearance because blind-descent horizontal drift reached 1.75–3.37 m.

## v0.5.3 corrections

### 1. Progress watchdog and conservative terminal route follower

A REJOIN now records entry time, best distance to goal, and last meaningful progress. A stalled or overlong REJOIN brakes and replans from the current estimate. After repeated unsuccessful smooth recovery, a verified A* path is followed at low speed with stop-at-corner velocity commands. Failsafe remains reserved for a target with no safe grid route.

Literature basis: Koenig and Likhachev, *D* Lite*, AAAI 2002. The use of incremental repair and a fresh A* recovery is attributed to the original search methods. The watchdog and route-follower hierarchy are project-specific integration logic.

### 2. Fault-aware estimator transition

The exact validated v0.4 estimator is retained unchanged as a frozen reference file. The v0.5.3 robust derivative adds recent normalized-innovation attribution and a short 0.08 s output blend only after a confirmed faulty-IMU attribution. Ordinary quality-based switches retain the 0.30 s blend.

Architecture basis: official PX4 EKF2 documentation describes multiple EKF instances using different sensor combinations and selection based on internal consistency to isolate IMU bias, saturation, or stuck-data faults. No PX4 code is copied.

Mathematical basis: Joan Solà, *Quaternion Kinematics for the Error-State Kalman Filter*, arXiv:1711.02508.

### 3. Route-before-land RTL recovery

When a valid D*/A* route exists but polynomial trajectory generation remains invalid after braking retries, the system executes the verified low-speed grid route instead of landing at the current location. Emergency landing is used only if no safe route exists.

### 4. Confidence-limited XY-loss braking

The lifecycle manager stores the last velocity estimate obtained while horizontal aiding is fresh. At aid loss it generates a bounded, short open-loop braking pulse from that frozen velocity. Absolute XY position feedback is disabled, and the later blind descent neither follows a stale XY target nor continuously damps a drifting inertial velocity estimate. After the braking pulse, the vehicle commands level vertical descent.

Safety architecture basis: official PX4 failsafe documentation uses fused-aid timeout and position-estimate quality to trigger position-loss handling, and notes that geofence margins should include stopping distance and uncertainty. The exact braking pulse and indoor local-landing integration are project-specific.

## Independent Python mechanism regression

`python3 s2_2_v0_5_3_multiseed_regression.py`

Result at packaging:

- source invariants: PASS;
- liveness terminal recovery: PASS;
- fault-aware transient model: 0.040 m versus 0.137 m old-model transient;
- route-before-land hierarchy: PASS;
- 2,000 randomized XY-loss brake trials: worst drift 0.339 m, 99th percentile 0.294 m, worst residual speed 0.088 m/s;
- overall focused mechanism regression: PASS.

These are focused software regressions, not a claim that MATLAB has passed.

## MATLAB acceptance sequence

1. Run `validate_S2_2_v0_5_3_focus()` and require 12/12 former-failure cases PASS.
2. Run `validate_S2_2(false)` and require 12/12 deterministic cases PASS.
3. Run `validate_S2_2_multiseed_robustness(0:9)` and require 60/60 PASS.
4. Run `validate_S2_2(true)` and inspect the single-window tabbed dashboards.
5. Run critical animations only after all no-plot validators pass.

No safety, clearance, estimator, tracking, speed, acceleration, jerk, or mission expectation threshold has been relaxed.
