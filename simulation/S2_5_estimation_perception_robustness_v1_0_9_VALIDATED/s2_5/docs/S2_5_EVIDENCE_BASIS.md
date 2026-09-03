# Evidence basis for S2.5

S2.5 is intentionally aligned with established estimator/perception robustness principles rather than introducing a new estimator.

- **PX4 EKF2:** observation faults are treated as either loss of data or excessive innovations; statistical innovation consistency checks reject inconsistent observations. External-vision timing/delay is explicitly treated as an estimator concern.
- **ArduPilot EKF3:** multiple EKF lanes run in parallel; innovation/error scores and sensor affinity support switching away from a degraded IMU/sensor combination.
- **MSCKF / OpenVINS / VINS-Mono:** visual-inertial estimation combines inertial propagation with visual constraints and emphasizes consistency, robust measurement handling and evaluation.
- **OctoMap:** probabilistic mapping explicitly preserves occupied, free and unknown states; representing unknown separately is important for safe planning.

Project-specific decisions:

- Gross VIO and LiDAR outliers are selected so that their innovation lower bounds exceed the inherited chi-square gates even at maximum eligible-lane covariance. This is checked offline rather than assumed.
- A 1.2 s total obstacle-perception outage is longer than the inherited 0.55 s hold threshold but shorter than the 4.0 s persistent-loss failsafe threshold, so it tests hold-and-recover semantics.
- A 0.45 s perception timestamp lag exceeds the inherited 0.30 s map packet-age gate and therefore must be rejected.
- Persistent total obstacle-perception loss is deliberately longer than 4.0 s and must produce a controlled emergency landing, not stale-map translation.

These sources support the design principles only. The exact numerical thresholds and pass/fail outcomes are project configuration and must be established by the project validation suite.

## Explicit boundary: navigation message latency

The inherited S2.2 ESKF packet API is synchronous and carries no VIO/LiDAR measurement timestamp into the filter update. Therefore S2.5 does not claim out-of-sequence navigation-measurement compensation. Packet-age robustness is exercised on the S2.3 perception/mapping interface, where timestamps are part of the real API. Navigation transport delay/time synchronization must be exercised in S2.6 ROS 2/PX4 SITL, where external-vision messages have timestamps and the PX4 EKF explicitly exposes external-vision delay handling. This is a declared interface boundary, not a silent omission.
