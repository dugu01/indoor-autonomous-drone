# Stage S2.1 literature cross-reference

The implementation uses the following sources as design references rather than claiming to reproduce any one flight stack exactly.

1. Joan Solà, *Quaternion kinematics for the error-state Kalman filter* (2017). Basis for the right-multiplicative quaternion error convention, attitude injection, and reset Jacobian.
2. Forster et al., *On-Manifold Preintegration for Real-Time Visual-Inertial Odometry* (2017). Reference for manifold IMU propagation and bias-aware inertial estimation.
3. Qin, Li, Shen, *VINS-Mono* (2018). Reference for six-DOF visual-inertial state estimation, initialization/failure-recovery concepts, and separation of local odometry from globally corrected consistency.
4. Xu et al., *FAST-LIO2* (2022). Reference for direct scan/map registration and Kalman-filter-based LiDAR-inertial estimation.
5. Kim and Kim, *Scan Context* / Kim, Choi, Kim, *Scan Context++*. Reference for structural LiDAR place retrieval. In Stage S2.1 a descriptor match is only a proposal; ICP and consistency gates are mandatory before adding a graph edge.
6. PX4 EKF2 multi-instance documentation. Reference for running independent estimator instances over different IMU combinations and selecting a healthy instance.
7. ArduPilot EKF3 affinity and lane-switching documentation. Reference for sensor affinity, lane health and controlled switching.

Implementation decisions:
- Four lanes are kept horizontally observable: two all-aid lanes, one VIO lane, and one LiDAR lane.
- Raw NIS values from measurements of different dimensions are never compared directly; each is normalized by its own chi-square gate.
- Lane eligibility requires recent horizontal and vertical aiding plus bounded covariance.
- A switch needs improvement margin, confirmation time and minimum dwell; output blending prevents a control-state step.
- Gravity aiding is conditional on accelerometer magnitude and angular rate.
- Global loop-closure corrections never jump the local flight-control state.
