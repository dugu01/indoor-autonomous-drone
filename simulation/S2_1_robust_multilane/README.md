# Stage S2.1 — Robust Multi-Lane Navigation

This package upgrades the production Stage S2 architecture while retaining the locked interface:

```matlab
results = run_S2_lidar_slam(seed, runStress, makePlots, makeAnimation);
```

## MATLAB files

- `run_S2_lidar_slam.m` — four-lane quaternion ESKF, ICP, ScanContext, pose graph, scenarios, saving.
- `plot_S2_dashboard.m` — the same separate tabbed-dashboard architecture, extended with a multi-lane resilience tab.
- `animate_S2_flight.m` — the production animation path, renamed output prefix for S2.1.
- `validate_S2_1.m` — path-resolution/static check and optional nominal+stress execution.

## Lane configuration

1. Primary IMU + VIO + LiDAR + rangefinder + barometer
2. Backup IMU + VIO + LiDAR + rangefinder + barometer
3. Primary IMU + VIO + rangefinder + barometer
4. Backup IMU + LiDAR + rangefinder + barometer

Every lane used for horizontal control has a horizontal aid. There is no IMU+barometer-only lane eligible for flight-control position.

## Run

Place the three production files together under `simulation/S2_visual_slam/`, retain the existing `assets/F450/` folder, then run:

```matlab
clear functions; clear classes; close all; clc;
which run_S2_lidar_slam -all
which plot_S2_dashboard -all
which animate_S2_flight -all

validation = validate_S2_1(false);
results = run_S2_lidar_slam(0, true, true, true);
```

Results are written below:

```text
simulation/results/S2_1_robust_multilane/
  nominal/seed_000/
  stress_primary_imu_plus_vio/seed_000/
```

Each trial saves PNG report tabs, the dashboard FIG, MAT trial data, summary text, and the animation when requested.

## Python validation

```bash
cd python
python run_backtests.py --output backtest_results --seeds 3
```

Use `--full-frontend` for the slower scan/ICP/ScanContext/pose-graph regression.

## Important

MATLAB was not available in the generation environment. The estimator and fault logic were independently exercised in Python, and the MATLAB source was statically audited. Run `validate_S2_1(true)` on the target MATLAB installation before replacing a working Stage S2 file or attempting HIL/flight tests.
