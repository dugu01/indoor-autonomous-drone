# Package Manifest — Stage S2.2 v0.4

## Entry points

- `run_S2_2_mission_replanning.m`
- `validate_S2_2.m`

## Integration core

- `mission_manager_S2_2.m`
- `init_S2_2_config.m`
- `scenario_S2_2.m`
- `geometric_controller_S2_2.m`
- `quadrotor_dynamics_S2_2.m`
- `multi_lane_eskf_S2_2.m`
- `simulate_sensor_packet_S2_2.m`
- `uncertainty_inflation_S2_2.m`

## Planning and trajectory

- `dstar_lite_S2_2.m`
- `astar_grid_S2_2.m`
- `build_occupancy_grid_S2_2.m`
- `inflate_obstacles_S2_2.m`
- `segment_occupied_grid_S2_2.m`
- `smooth_path_S2_2.m`
- `generate_min_snap_trajectory_S2_2.m`
- `sample_min_snap_trajectory_S2_2.m`
- `sample_min_snap_state_S2_2.m`
- `eval_min_snap_segment_S2_2.m`
- `track_smooth_trajectory_S2_2.m`

## Dynamic-obstacle logic

- `dynamic_obstacle_state_S2_2.m`
- `alpha_beta_track_S2_2.m`
- `velocity_obstacle_filter_S2_2.m`
- `static_braking_speed_S2_2.m`

## Quaternion/SO(3) helpers

- `qnormalize_S2_2.m`
- `qmul_S2_2.m`
- `qconj_S2_2.m`
- `qexp_S2_2.m`
- `qlog_S2_2.m`
- `q2R_S2_2.m`
- `R2q_S2_2.m`
- `q2rpy_S2_2.m`
- `skew3_S2_2.m`
- `wrap_pi_S2_2.m`

## Visualisation

- `plot_S2_2_dashboard.m`
- `animate_S2_2_flight.m`

## Documentation and audit

- `README.md`
- `LITERATURE_S2_2.md`
- `CHANGELOG_S2_1_TO_S2_2.md`
- `MATLAB_VALIDATION_PROTOCOL_S2_2_V0_4.md`
- `PLOTTING_AND_RESULTS_CONVENTION.md`
- `PYTHON_BACKTEST_REPORT_S2_2_V0_4.md`
- `STATIC_AUDIT_S2_2_V0_4.md`
- `audit_S2_2_v0_4.py`
