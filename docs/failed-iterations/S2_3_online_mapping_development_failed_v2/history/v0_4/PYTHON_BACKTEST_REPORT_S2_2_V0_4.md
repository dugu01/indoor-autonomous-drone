# Stage S2.2 v0.4 Python Backtest

Overall: **40/40 PASS**

| Scenario | Seeds | Pass | Worst estimator error [m] | Worst attitude error [deg] | Worst reference deviation [m] | Min static clearance [m] | Min dynamic clearance [m] | Replans/promotions |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `nominal_6dof` | 5 | 5/5 | 0.045 | 0.62 | 0.025 | 0.620 | N/A | 0/0 |
| `incremental_static_estimated` | 5 | 5/5 | 0.045 | 0.62 | 0.023 | 0.553 | N/A | 1/0 |
| `dynamic_crossing_6dof` | 5 | 5/5 | 0.045 | 1.41 | 0.402 | 0.615 | 0.128 | 0/0 |
| `dynamic_blocker_becomes_static_6dof` | 5 | 5/5 | 0.045 | 0.62 | 0.091 | 0.524 | 0.233 | 1/1 |
| `obstacle_sensor_dropout_recover_6dof` | 5 | 5/5 | 0.045 | 0.62 | 0.110 | 0.617 | N/A | 0/0 |
| `primary_imu_fault_vio_outage` | 5 | 5/5 | 0.052 | 0.62 | 0.056 | 0.606 | N/A | 0/0 |
| `xy_aid_loss_failsafe` | 5 | 5/5 | 0.170 | 0.62 | 0.170 | 0.794 | N/A | 0/0 |
| `uncertainty_inflation` | 5 | 5/5 | 0.045 | 0.62 | 0.023 | 0.553 | N/A | 1/0 |

## Scope

Independent Python software-in-the-loop regression of all seven MATLAB v0.4 mission/fault classes plus one supplementary uncertainty-inflation case.

The dynamic tests check physical vehicle-to-obstacle clearance and reject any collision. The blocker case requires dynamic avoidance, static promotion and replanning. The obstacle-coverage-loss case requires a no-data hold and recovery.

The MATLAB implementation is a separate integration candidate and requires MATLAB runtime validation.
