# Simulation — Indoor GPS-Denied Autonomous Drone
**Minor Project 2025–26 | Week 1–4 MATLAB simulation**

## Folder structure

```
simulation/
├── S1_dynamics_pid/          ← Week 1 (you are here)
│   ├── run_S1_simulation.m   ← ENTRY POINT — run this
│   ├── quadrotor_dynamics.m  ← 6-DOF physics (RK4)
│   ├── cascaded_pid.m        ← 3-loop PID controller
│   └── plot_S1_results.m     ← 4 report figures
│
├── S2_ekf_sensor_fusion/     ← Week 2 (IMU + optical flow EKF)
├── S3_obstacle_avoidance/    ← Week 3 (potential field, sonar model)
├── S4_aruco_landing/         ← Week 3 (ArUco pose estimation + landing)
└── results/                  ← Save all plot PNGs here for report
```

## How to run Stage S1

```matlab
cd simulation/S1_dynamics_pid
run_S1_simulation
```

No extra toolboxes required for S1.  
UAV Toolbox is used from S2 onward (`imuSensor`, `uavScenario`).

## What S1 validates

| Check | Pass criterion | Why it matters |
|---|---|---|
| Altitude hold | Error < ±5 cm for t > 5 s | EKF3 rangefinder source needs this stability |
| XY drift | < 5 cm over 30 s | Starting point before optical flow added in S2 |
| Motor RPM | No saturation (< 10500 RPM) | Confirms thrust-to-weight budget is correct |
| Euler angles | ± < 15° in steady state | Safe for indoor testing |

## Physical parameters (matched to real hardware)

| Parameter | Value | Source |
|---|---|---|
| Total mass | 720 g | F330 frame 150 + motors 200 + ESCs 80 + Pixhawk 38 + RPi5 46 + battery 140 + misc |
| Arm length | 165 mm | F330 wheelbase 330 mm / 2 |
| kT (thrust coeff) | 3.16 × 10⁻⁶ N/(rad/s)² | Derived from hover condition |
| kD (torque coeff) | 7.94 × 10⁻⁹ N·m/(rad/s)² | Typical for 5045 props |
| Max motor speed | 1100 rad/s | 2204-2300KV on 3S (11.1V) |
| Hover RPM/motor | ~750 rad/s (~7200 RPM) | Computed: sqrt(mg/4kT) |

## PID gains → ArduPilot transfer (Week 4)

After tuning in simulation, these gain names map to ArduPilot parameters:

```
Simulation gain          ArduPilot parameter      Starting value
─────────────────────────────────────────────────────────────────
gains.att_Kp           → ATC_RAT_RLL_P             8.0
gains.att_Ki           → ATC_RAT_RLL_I             0.05
gains.att_Kd           → ATC_RAT_RLL_D             1.5
gains.alt_Kp           → PSC_POSZ_P                2.0
gains.pos_Kp           → PSC_POSXY_P               1.2
gains.yaw_Kp           → ATC_RAT_YAW_P             4.0
```

Reduce `ATC_RAT_RLL_P` by 10% at a time if real drone oscillates.

## Simulation stages

| Stage | Week | New element added | Unit test |
|---|---|---|---|
| S1 | 1 | Dynamics + cascaded PID | Hover 30 s, alt error < ±5 cm |
| S2 | 2 | Simulated IMU + EKF fusion | Position drift < 15 cm / 60 s |
| S3 | 2–3 | 4 sonar sensors + potential-field avoidance | Navigates 6×6 m room, avoids 2 boxes |
| S4 | 3 | ArUco marker detection + landing controller | Lands within 5 cm of marker |
