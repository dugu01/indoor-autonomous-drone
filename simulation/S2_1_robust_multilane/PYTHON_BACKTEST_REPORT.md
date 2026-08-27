# Stage S2.1 Python Backtest Report

## Scope

The Python reference independently reproduces the 16-error-state quaternion ESKF, four parallel lanes, sensor-specific NIS gates, observability checks, hysteretic failover, conditional gravity aiding, output blending, collision validation, and degraded/RTL-request logic.

### Seed-0 scenario matrix

| Scenario | Max 3-D error | RMSE | Attitude max | Switches | Final lane | Degraded | RTL | Result |
|---|---:|---:|---:|---:|---:|---:|:---:|:---:|
| nominal | 2.14 cm | 0.90 cm | 0.812° | 0 | 1 | 0.00 s | no | PASS |
| vio_outage | 2.54 cm | 1.00 cm | 0.812° | 0 | 1 | 0.00 s | no | PASS |
| lidar_degraded | 2.15 cm | 0.91 cm | 0.809° | 0 | 1 | 0.00 s | no | PASS |
| range_outage | 2.14 cm | 0.93 cm | 0.812° | 0 | 1 | 0.00 s | no | PASS |
| baro_drift | 2.15 cm | 0.94 cm | 0.812° | 0 | 1 | 0.00 s | no | PASS |
| primary_imu_bias | 5.80 cm | 1.01 cm | 1.297° | 1 | 2 | 0.00 s | no | PASS |
| primary_imu_freeze | 5.46 cm | 1.00 cm | 1.927° | 1 | 2 | 0.00 s | no | PASS |
| backup_imu_bias | 2.14 cm | 0.90 cm | 0.812° | 0 | 1 | 0.00 s | no | PASS |
| primary_imu_plus_vio | 9.47 cm | 1.35 cm | 1.633° | 1 | 2 | 0.00 s | no | PASS |
| all_xy_outage | 45.91 cm | 7.31 cm | 0.812° | 0 | 1 | 9.06 s | yes | EXPECTED DEGRADED |

The `all_xy_outage` case intentionally removes both VIO and LiDAR horizontal aiding. It is not expected to satisfy the 10 cm navigation requirement; the required behaviour is to declare degradation and request RTL rather than select an unobservable lane.

### Three-seed nominal check

| Seed | Max 3-D error | RMSE | Attitude max | Switches |
|---:|---:|---:|---:|---:|
| 0 | 2.14 cm | 0.90 cm | 0.812° | 0 |
| 1 | 1.53 cm | 0.61 cm | 0.740° | 0 |
| 2 | 1.40 cm | 0.70 cm | 0.914° | 0 |

### Full frontend checks

These runs include ray-cast scans, scan-to-local-map ICP, ScanContext candidate retrieval, geometric loop verification, and robust pose-graph optimization. To keep the independent Python regression practical in this environment, these checks used 180 beams and reduced ICP/submap settings; the MATLAB production file retains 360 beams and the full configuration.

| Scenario | Max fused error | Local LiDAR max | LiDAR acceptance | Verified loops | Switches | Result |
|---|---:|---:|---:|---:|---:|:---:|
| nominal | 2.11 cm | 3.60 cm | 100.00% | 7 | 0 | PASS |
| lidar_degraded | 2.12 cm | 2.77 cm | 84.29% | 7 | 0 | PASS |
| primary_imu_bias | 2.15 cm | 3.60 cm | 100.00% | 7 | 1 | PASS |

## Conclusions

- Nominal accuracy remained well below 10 cm across the recorded seeds.
- VIO, LiDAR, rangefinder and barometer disturbances remained bounded.
- Primary-IMU bias/freeze cases selected the backup-IMU lane.
- A backup-IMU fault did not pull the output away from the healthy primary lane.
- The combined primary-IMU fault plus VIO outage completed with one controlled switch to Lane 2.
- Complete horizontal-aid loss produced explicit degradation and RTL request instead of a false healthy solution.

Python validation is not a substitute for MATLAB execution, HIL testing, time-synchronization checks, calibrated sensor extrinsics, vibration testing, or flight-envelope safety validation.