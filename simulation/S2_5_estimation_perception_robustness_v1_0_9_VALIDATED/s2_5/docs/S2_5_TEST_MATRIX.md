# S2.5 qualification matrix

## Baseline — 5 runs

`baseline`, seeds 0..4. Full nominal S2.4/S2.5 mission and all inherited gates must pass.

## Recoverable matrix — 60 runs

Each case is run at seeds 0..4. Every individual run must pass; no percentage/average criterion is used.

| ID | Case | Injected condition | Required evidence |
|---|---|---|---|
| N1 | nav_vio_dropout | VIO removed 18–24 s | mission/estimator/safety pass |
| N2 | nav_lidar_dropout | estimator LiDAR pose aid removed 18–24 s | mission/estimator/safety pass |
| N3 | nav_vio_outlier | gross VIO pose/velocity/attitude burst 18–18.6 s | lane rejection count increases; mission pass |
| N4 | nav_lidar_outlier | gross LiDAR XY/yaw burst 18–18.6 s | lane rejection count increases; mission pass |
| N5 | nav_imu_fault_vio_outage | primary IMU bias step + VIO loss | lane switch observed; mission pass |
| N6 | nav_high_noise | 1.5x all measurement noise | nominal thresholds still pass |
| P1 | perception_lidar_dropout | raw obstacle LiDAR removed 18–26 s | depth-only recovery; mapping/safety pass |
| P2 | perception_depth_dropout | depth rays removed 18–26 s | LiDAR-only recovery; mapping/safety pass |
| P3 | perception_dual_brief | both obstacle sensors removed 1.2 s | perception hold observed, recovery + mission pass |
| P4 | perception_stale_burst | valid rays timestamped 0.45 s late for 1.2 s | mapper rejection increases, hold + recovery |
| P5 | perception_range_spike | one-frame +0.8 m hit-range spike | map false-free/safety gates remain pass |
| C1 | coupled_imu_perception | primary IMU bias + 1.2 s total obstacle-perception dropout | lane switch + perception hold + mission pass |

## Persistent-loss fail-safe matrix — 6 runs

Seeds 0..2, each must safely terminate via controlled emergency landing:

- N7 `nav_xy_loss`: persistent VIO + estimator-LiDAR horizontal-aid loss.
- P6 `perception_dual_prolonged`: 7 s loss of both obstacle-perception modalities.

Required: failsafe=1, emergency landing=1, mission lifecycle completes, zero collision/geofence/unknown commitment, truth isolation PASS.

Total unique S2.5 runs = 5 + 60 + 6 = **71**.
