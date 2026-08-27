# Package Manifest — Stage S2.2 v0.5.3 Multi-Seed Robustness Candidate

New production files: `mission_manager_v0_5_3_core_S2_2.m`, `multi_lane_eskf_robust_S2_2.m`, and `validate_S2_2_multiseed_robustness.m`. New evidence: `MULTISEED_ROBUSTNESS_REPORT_S2_2_V0_5_3.md`, `s2_2_v0_5_3_multiseed_regression.py`, and `MULTISEED_MECHANISM_REGRESSION_S2_2_V0_5_3.txt`. The exact v0.4 core and estimator remain present as frozen references.

# Package Manifest — Stage S2.2 v0.5.2 Audited Candidate

## Public entry points

- `run_S2_2_mission_replanning.m`
- `validate_S2_2.m`
- `validate_S2_2_monte_carlo.m`

## Mission integration

- `mission_manager_S2_2.m` — scenario dispatcher
- `mission_manager_v0_4_core_S2_2.m` — preserved validated v0.4 mission core
- `mission_lifecycle_manager_S2_2.m` — v0.5 autonomous lifecycle
- `init_S2_2_config.m`
- `scenario_S2_2.m`
- `preflight_check_S2_2.m`
- `land_detector_S2_2.m`
- `vertical_profile_S2_2.m`
- `landing_zone_clear_S2_2.m`
- `select_safe_landing_zone_S2_2.m`

## Vehicle, controller and estimator

- `init_quadrotor_state_S2_2.m`
- `quadrotor_dynamics_S2_2.m`
- `geometric_controller_S2_2.m`
- `multi_lane_eskf_S2_2.m` — exact validated v0.4 ESKF for legacy scenarios
- `multi_lane_eskf_lifecycle_S2_2.m` — lifecycle-only freshness/covariance outputs
- `simulate_sensor_packet_S2_2.m`
- `init_sensor_model_S2_2.m`
- `uncertainty_inflation_S2_2.m`

## Planning and trajectories

- `dstar_lite_S2_2.m`, `astar_grid_S2_2.m`
- `build_occupancy_grid_S2_2.m`, `inflate_obstacles_S2_2.m`
- `segment_occupied_grid_S2_2.m`, `smooth_path_S2_2.m`
- `generate_min_snap_trajectory_S2_2.m`
- `sample_min_snap_trajectory_S2_2.m`, `sample_min_snap_state_S2_2.m`
- `eval_min_snap_segment_S2_2.m`, `track_smooth_trajectory_S2_2.m`
- `dynamic_obstacle_state_S2_2.m`, `alpha_beta_track_S2_2.m`
- `velocity_obstacle_filter_S2_2.m`, `static_braking_speed_S2_2.m`
- `shape_velocity_command_S2_2.m`

## Quaternion/SO(3) helpers

- `qnormalize_S2_2.m`, `qmul_S2_2.m`, `qconj_S2_2.m`
- `qexp_S2_2.m`, `qlog_S2_2.m`
- `q2R_S2_2.m`, `R2q_S2_2.m`, `q2rpy_S2_2.m`
- `skew3_S2_2.m`, `wrap_pi_S2_2.m`

## Visualisation

- `plot_S2_2_dashboard.m` — one docked MATLAB window, scenario tabs, PNG/FIG export
- `animate_S2_2_flight.m` — lifecycle animation with time-correct obstacle replay

## Audit and provenance

- `README.md`
- `LITERATURE_S2_2.md`
- `CHANGELOG_S2_1_TO_S2_2.md`
- `SOURCE_BASELINE_S2_2_V0_5.md`
- `CONSOLIDATED_AUDIT_NOTES_S2_2_V0_5.md`
- `MATLAB_VALIDATION_PROTOCOL_S2_2_V0_5.md`
- `PLOTTING_AND_RESULTS_CONVENTION.md`
- `V0_4_CONFIG_BASELINE_S2_2.json`
- `audit_S2_2_v0_5.py`
- `matlab_source_sanity.py`
- `STATIC_AUDIT_S2_2_V0_5.md`

## Python evidence

- `s2_2_v0_5_lifecycle_backtest.py`
- `s2_2_v0_5_audit_backtest.py`
- `PYTHON_BACKTEST_RESULTS_S2_2_V0_5.json`
- `PYTHON_BACKTEST_REPORT_S2_2_V0_5.md`
- `AUDIT_BACKTEST_RESULTS_S2_2_V0_5.json`
- `AUDIT_BACKTEST_REPORT_S2_2_V0_5.txt`
- `patch_regression_s2_2_v0_4.py`
- `patch2_regression_s2_2_v0_4.py`

`history/v0_4/` contains earlier validation and patch-regression documentation for traceability and is not called by runtime MATLAB code.

## v0.5.1 runtime-correction evidence

- `RUNTIME_FAILURE_ANALYSIS_S2_2_V0_5_1.md`
- `s2_2_v0_5_1_runtime_semantic_regression.py`


## v0.5.2 position-loss evidence

- `RUNTIME_FAILURE_ANALYSIS_S2_2_V0_5_2.md`
- `s2_2_v0_5_2_emergency_regression.py`
- `POSITION_LOSS_REGRESSION_REPORT_S2_2_V0_5_2.txt`


## v0.5.3 validation utilities

- `validate_S2_2_v0_5_3_focus.m` — exact twelve former failed cases.
- `validate_S2_2_multiseed_robustness.m` — complete six-scenario, ten-seed matrix.
- `audit_S2_2_v0_5_3.py` — cumulative source/package audit.
- `s2_2_v0_5_3_multiseed_regression.py` — focused mechanism regression.
