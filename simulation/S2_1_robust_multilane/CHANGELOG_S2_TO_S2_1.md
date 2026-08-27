# Stage S2 → Stage S2.1 changes

## Retained from production Stage S2

- Locked four-argument `run_S2_lidar_slam` interface.
- 16-error-state quaternion ESKF and body-frame IMU mechanization.
- D435i host-VIO, tilt-aware rangefinder and barometer-bias updates.
- Live ESKF prior for trimmed scan-to-local-map ICP.
- ScanContext proposal followed by metric ICP/consistency verification.
- Separate smooth local/control pose and corrected global/map pose.
- Separate `plot_S2_dashboard.m` and `animate_S2_flight.m` files.
- F450 collision geometry and preflight path validation.

## Added in Stage S2.1

- Four parallel, horizontally observable lanes over primary/backup IMUs.
- Sensor-specific normalized NIS histories and robust median scoring.
- Horizontal/vertical aid-freshness and covariance eligibility gates.
- Cross-IMU disagreement attribution using parallel-lane aiding innovations.
- Switching hysteresis, confirmation delay, minimum dwell and consistency gate.
- Smooth output blending after lane changes.
- Conditional gravity pseudo-measurement with correct right-error Jacobian sign.
- Degraded-navigation timer and RTL-request flag when no lane is observable.
- Explicit primary-IMU bias fault in the stress trial to exercise real failover.
- S2.1 result hierarchy, saved paths and multi-lane dashboard tab.
- Independent Python scenario matrix and reduced full-frontend regressions.
