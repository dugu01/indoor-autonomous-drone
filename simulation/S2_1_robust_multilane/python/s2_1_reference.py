#!/usr/bin/env python3
"""Stage S2.1 robust multi-lane navigation Python reference/backtest.

This is an independent software-in-the-loop reference for the MATLAB Stage S2.1
implementation. It mirrors the scalar-first quaternion, right-error ESKF
convention used by the production MATLAB code.

Design references (see S2_1_LITERATURE.md):
- Sola, Quaternion kinematics for the error-state Kalman filter.
- Forster et al., on-manifold inertial preintegration.
- Qin et al., VINS-Mono initialization/failure recovery and local/global split.
- PX4 EKF2 multiple estimator instances and selector.
- ArduPilot EKF3 affinity and lane switching.
- Kim & Kim, Scan Context, with metric ICP verification before graph insertion.

The fast Monte Carlo uses geometrically plausible LiDAR pose observations to
exercise the estimator and selector. The --full-frontend mode separately runs
actual 2-D ray-cast scans, scan-to-local-map ICP, Scan Context candidate search,
and robust pose-graph optimization.
"""
from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import time
from collections import defaultdict, deque
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree


# ---------------------------------------------------------------------------
# SO(3) / quaternion helpers: scalar-first, body-to-world, right perturbation.
# ---------------------------------------------------------------------------
def wrap_pi(a):
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def skew(v: Sequence[float]) -> np.ndarray:
    x, y, z = np.asarray(v, dtype=float).reshape(3)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def qnormalize(q: Sequence[float]) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(4)
    n = np.linalg.norm(q)
    if not np.isfinite(n) or n < 1e-15:
        raise FloatingPointError("invalid quaternion")
    q = q / n
    return -q if q[0] < 0.0 else q


def qconj(q: Sequence[float]) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(4)
    return np.r_[q[0], -q[1:]]


def qmul(a: Sequence[float], b: Sequence[float]) -> np.ndarray:
    a = np.asarray(a, dtype=float).reshape(4)
    b = np.asarray(b, dtype=float).reshape(4)
    return qnormalize(
        np.r_[
            a[0] * b[0] - a[1:] @ b[1:],
            a[0] * b[1:] + b[0] * a[1:] + np.cross(a[1:], b[1:]),
        ]
    )


def qexp(v: Sequence[float]) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(3)
    a = np.linalg.norm(v)
    if a < 1e-10:
        return qnormalize(np.r_[1.0, 0.5 * v])
    return np.r_[math.cos(a / 2.0), math.sin(a / 2.0) * v / a]


def qlog(q: Sequence[float]) -> np.ndarray:
    q = qnormalize(q)
    nv = np.linalg.norm(q[1:])
    if nv < 1e-10:
        return 2.0 * q[1:]
    a = 2.0 * math.atan2(nv, q[0])
    if a > np.pi:
        a -= 2.0 * np.pi
    return a * q[1:] / nv


def q2R(q: Sequence[float]) -> np.ndarray:
    w, x, y, z = qnormalize(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def R2q(R: np.ndarray) -> np.ndarray:
    tr = float(np.trace(R))
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2.0
        q = [0.25 * S, (R[2, 1] - R[1, 2]) / S, (R[0, 2] - R[2, 0]) / S, (R[1, 0] - R[0, 1]) / S]
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            S = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            q = [(R[2, 1] - R[1, 2]) / S, 0.25 * S, (R[0, 1] + R[1, 0]) / S, (R[0, 2] + R[2, 0]) / S]
        elif i == 1:
            S = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            q = [(R[0, 2] - R[2, 0]) / S, (R[0, 1] + R[1, 0]) / S, 0.25 * S, (R[1, 2] + R[2, 1]) / S]
        else:
            S = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            q = [(R[1, 0] - R[0, 1]) / S, (R[0, 2] + R[2, 0]) / S, (R[1, 2] + R[2, 1]) / S, 0.25 * S]
    return qnormalize(q)


def rpy2q(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return qnormalize(
        [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]
    )


def orientation_residual(q_pred: Sequence[float], q_meas: Sequence[float]) -> np.ndarray:
    return qlog(qmul(qconj(q_pred), q_meas))


def slerp_offset(q_offset: np.ndarray, fraction_remaining: float) -> np.ndarray:
    return qexp(fraction_remaining * qlog(q_offset))


# ---------------------------------------------------------------------------
# Configuration and scenarios.
# ---------------------------------------------------------------------------
@dataclass
class Config:
    duration: float = 60.0
    imu_rate: int = 200
    vio_rate: int = 30
    lidar_rate: float = 5.5
    range_rate: int = 30
    baro_rate: int = 25
    room: Tuple[float, float, float] = (6.0, 6.0, 2.5)
    obstacles: Tuple[Tuple[float, float, float, float], ...] = ((1.0, 1.0, 0.5, 0.5), (4.0, 3.5, 0.5, 0.5))
    obstacle_height: float = 1.60
    requirement_m: float = 0.10
    attitude_requirement_deg: float = 2.0
    g: float = 9.81
    r_BC: np.ndarray = field(default_factory=lambda: np.array([0.08, 0.00, 0.02]))
    r_BL: np.ndarray = field(default_factory=lambda: np.array([0.00, 0.00, 0.05]))
    r_BR: np.ndarray = field(default_factory=lambda: np.array([0.00, 0.00, -0.05]))
    d_BR: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, -1.0]))
    accel_nd: float = 0.003
    gyro_nd: float = math.radians(0.025)
    accel_bias_rw: float = 2e-4
    gyro_bias_rw: float = math.radians(0.002)
    vio_pos_sigma: float = 0.015
    vio_vel_sigma: float = 0.025
    vio_att_sigma: float = math.radians(0.45)
    vio_drift_pos_rw: float = 8e-4
    vio_drift_att_rw: float = math.radians(0.012)
    vio_outlier_probability: float = 0.005
    lidar_sigma_xy: float = 0.025
    lidar_sigma_yaw: float = math.radians(0.7)
    range_sigma: float = 0.012
    baro_sigma: float = 0.06
    baro_bias_rw: float = 0.002
    gravity_sigma: float = 0.20
    gravity_norm_tolerance: float = 0.30
    gravity_max_gyro: float = math.radians(120.0)
    gate_vio9: float = 27.877
    gate_lidar3: float = 16.266
    gate_1: float = 10.828
    gate_gravity3: float = 16.266
    n_beams: int = 360
    range_noise: float = 0.012
    icp_max_corr: float = 0.22
    icp_trim: float = 0.72
    icp_iterations: int = 14
    icp_step_xy: float = 0.08
    icp_step_yaw: float = math.radians(3.0)
    local_submap_scans: int = 45
    map_voxel: float = 0.045
    icp_health_rmse: float = 0.080
    icp_health_overlap: int = 80
    icp_health_correction: float = 0.12
    icp_health_yaw: float = math.radians(5.0)
    sc_rings: int = 20
    sc_sectors: int = 60
    sc_exclude_recent: int = 45
    sc_threshold: float = 0.24
    sc_verify_rmse: float = 0.065
    keyframe_stride: int = 5
    max_loop_closures: int = 8
    collision_radius: float = 0.352
    localization_margin: float = 0.10
    control_margin: float = 0.05
    # Lane selector and continuity.
    health_window: int = 10
    xy_aid_timeout: float = 1.0
    z_aid_timeout: float = 1.0
    max_xy_covariance: float = 0.10**2
    max_z_covariance: float = 0.12**2
    score_switch_margin: float = 0.20
    switch_confirm_time: float = 0.15
    minimum_dwell_time: float = 2.0
    switch_consistency_position: float = 0.75
    switch_consistency_attitude: float = math.radians(12.0)
    output_blend_time: float = 0.30
    degraded_rtl_delay: float = 2.0

    @property
    def gW(self) -> np.ndarray:
        return np.array([0.0, 0.0, -self.g])

    @property
    def geofence_margin_xy(self) -> float:
        return self.collision_radius + self.localization_margin + self.control_margin


@dataclass
class Scenario:
    name: str
    vio_outages: Tuple[Tuple[float, float], ...] = ()
    lidar_outages: Tuple[Tuple[float, float], ...] = ()
    range_outages: Tuple[Tuple[float, float], ...] = ()
    baro_outages: Tuple[Tuple[float, float], ...] = ()
    lidar_dropout_probability: float = 0.0
    lidar_false_pose_probability: float = 0.0
    vio_outlier_probability_extra: float = 0.0
    primary_imu_bias_step_time: float = math.inf
    primary_accel_bias_step: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    primary_gyro_bias_step_deg_s: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    primary_imu_freeze: Optional[Tuple[float, float]] = None
    backup_imu_bias_step_time: float = math.inf
    backup_accel_bias_step: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    backup_gyro_bias_step_deg_s: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    baro_drift_start: float = math.inf
    baro_drift_rate_mps: float = 0.0
    expected_degraded: bool = False
    expected_switch_to_backup: bool = False
    expected_no_switch: bool = False


def scenario_catalog() -> Dict[str, Scenario]:
    return {
        "nominal": Scenario("nominal", expected_no_switch=True),
        "vio_outage": Scenario("vio_outage", vio_outages=((25.0, 32.0),)),
        "lidar_degraded": Scenario("lidar_degraded", lidar_dropout_probability=0.20, lidar_false_pose_probability=0.02),
        "range_outage": Scenario("range_outage", range_outages=((18.0, 38.0),)),
        "baro_drift": Scenario("baro_drift", baro_drift_start=20.0, baro_drift_rate_mps=0.015),
        "primary_imu_bias": Scenario(
            "primary_imu_bias",
            primary_imu_bias_step_time=25.0,
            primary_accel_bias_step=(0.35, -0.25, 0.18),
            primary_gyro_bias_step_deg_s=(1.5, -1.0, 1.2),
            expected_switch_to_backup=True,
        ),
        "primary_imu_freeze": Scenario("primary_imu_freeze", primary_imu_freeze=(25.0, 40.0), expected_switch_to_backup=True),
        "backup_imu_bias": Scenario(
            "backup_imu_bias",
            backup_imu_bias_step_time=25.0,
            backup_accel_bias_step=(0.40, 0.20, -0.25),
            backup_gyro_bias_step_deg_s=(1.2, 1.0, -1.3),
            expected_no_switch=True,
        ),
        "primary_imu_plus_vio": Scenario(
            "primary_imu_plus_vio",
            vio_outages=((24.0, 38.0),),
            primary_imu_bias_step_time=25.0,
            primary_accel_bias_step=(0.35, -0.25, 0.18),
            primary_gyro_bias_step_deg_s=(1.5, -1.0, 1.2),
            expected_switch_to_backup=True,
        ),
        "all_xy_outage": Scenario(
            "all_xy_outage",
            vio_outages=((25.0, 35.0),),
            lidar_outages=((25.0, 35.0),),
            expected_degraded=True,
        ),
    }


def in_windows(t: float, windows: Sequence[Tuple[float, float]]) -> bool:
    return any(a <= t <= b for a, b in windows)


# ---------------------------------------------------------------------------
# Truth and sensor simulation.
# ---------------------------------------------------------------------------
def time_to_index(t: np.ndarray, rate: float, N: int) -> np.ndarray:
    return np.clip(np.rint(t * rate).astype(int), 0, N - 1)


def simulate_truth(cfg: Config) -> Dict[str, np.ndarray]:
    dt = 1.0 / cfg.imu_rate
    t = np.arange(0.0, cfg.duration + dt / 2.0, dt)
    # Collision-checked figure-eight retained from the production Stage S2.
    cx, cy, rx, ry, phi = 3.2, 2.0, 1.5, 0.9, math.radians(250.0)
    w1, w2 = 2 * np.pi / 30.0, 4 * np.pi / 30.0
    p = np.c_[cx + rx * np.sin(w1 * t), cy + ry * np.sin(w2 * t + phi), 1.15 + 0.16 * np.sin(2 * np.pi * t / 18.0)]
    v = np.c_[rx * w1 * np.cos(w1 * t), ry * w2 * np.cos(w2 * t + phi), 0.16 * (2 * np.pi / 18.0) * np.cos(2 * np.pi * t / 18.0)]
    a = np.c_[-rx * w1**2 * np.sin(w1 * t), -ry * w2**2 * np.sin(w2 * t + phi), -0.16 * (2 * np.pi / 18.0) ** 2 * np.sin(2 * np.pi * t / 18.0)]
    yaw = np.unwrap(np.arctan2(v[:, 1], v[:, 0]))
    roll = np.deg2rad(5.0) * np.sin(2 * np.pi * t / 8.0)
    pitch = np.deg2rad(4.0) * np.sin(2 * np.pi * t / 10.0 + 0.4)
    q = np.array([rpy2q(roll[k], pitch[k], yaw[k]) for k in range(len(t))])
    omega = np.zeros((len(t), 3))
    for k in range(len(t) - 1):
        omega[k] = qlog(qmul(qconj(q[k]), q[k + 1])) / dt
    omega[-1] = omega[-2]
    return {"t": t, "p": p, "v": v, "a": a, "q": q, "omega": omega, "rpy": np.c_[roll, pitch, yaw]}


def validate_reference_path(gt: Dict[str, np.ndarray], cfg: Config) -> Dict[str, float | bool]:
    p = gt["p"]
    wall_clear = np.min(np.c_[p[:, 0], cfg.room[0] - p[:, 0], p[:, 1], cfg.room[1] - p[:, 1]], axis=1)
    obs_clear = np.full(len(p), np.inf)
    for ox, oy, ow, od in cfg.obstacles:
        dx = np.maximum.reduce([ox - p[:, 0], np.zeros(len(p)), p[:, 0] - (ox + ow)])
        dy = np.maximum.reduce([oy - p[:, 1], np.zeros(len(p)), p[:, 1] - (oy + od)])
        horizontal = np.hypot(dx, dy)
        horizontal[p[:, 2] > cfg.obstacle_height + cfg.control_margin] = np.inf
        obs_clear = np.minimum(obs_clear, horizontal)
    minimum = float(min(np.min(wall_clear), np.min(obs_clear)))
    vertical_safe = bool(np.all((p[:, 2] >= 0.35) & (p[:, 2] <= cfg.room[2] - 0.30)))
    return {
        "safe": bool(minimum >= cfg.geofence_margin_xy and vertical_safe),
        "path_length_m": float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1))),
        "min_wall_clearance_m": float(np.min(wall_clear)),
        "min_obstacle_clearance_m": float(np.min(obs_clear)),
        "min_static_clearance_m": minimum,
        "required_clearance_m": cfg.geofence_margin_xy,
        "vertical_safe": vertical_safe,
    }


def simulate_sensors(seed: int, cfg: Config, scenario: Scenario, *, generate_scans: bool = False) -> Tuple[Dict[str, np.ndarray], Dict[str, object]]:
    rng = np.random.default_rng(seed)
    gt = simulate_truth(cfg)
    dt = 1.0 / cfg.imu_rate
    N = len(gt["t"])

    ba0 = np.zeros((N, 3)); bg0 = np.zeros((N, 3))
    ba0[0] = [0.018, -0.014, 0.012]; bg0[0] = np.deg2rad([0.06, -0.04, 0.05])
    for k in range(1, N):
        ba0[k] = ba0[k - 1] + cfg.accel_bias_rw * math.sqrt(dt) * rng.standard_normal(3)
        bg0[k] = bg0[k - 1] + cfg.gyro_bias_rw * math.sqrt(dt) * rng.standard_normal(3)
    ba1 = ba0 + np.array([0.005, -0.003, 0.002])
    bg1 = bg0 + np.deg2rad([0.010, -0.008, 0.005])

    sa, sg = cfg.accel_nd / math.sqrt(dt), cfg.gyro_nd / math.sqrt(dt)
    acc0 = np.zeros((N, 3)); gyro0 = np.zeros((N, 3)); acc1 = np.zeros((N, 3)); gyro1 = np.zeros((N, 3))
    for k in range(N):
        R = q2R(gt["q"][k])
        specific = R.T @ (gt["a"][k] - cfg.gW)
        acc0[k] = specific + ba0[k] + sa * rng.standard_normal(3)
        gyro0[k] = gt["omega"][k] + bg0[k] + sg * rng.standard_normal(3)
        acc1[k] = specific + ba1[k] + 1.15 * sa * rng.standard_normal(3)
        gyro1[k] = gt["omega"][k] + bg1[k] + 1.15 * sg * rng.standard_normal(3)

    # Fault injection after the nominal sensor generation.
    t = gt["t"]
    primary_step = t >= scenario.primary_imu_bias_step_time
    acc0[primary_step] += np.asarray(scenario.primary_accel_bias_step)
    gyro0[primary_step] += np.deg2rad(np.asarray(scenario.primary_gyro_bias_step_deg_s))
    backup_step = t >= scenario.backup_imu_bias_step_time
    acc1[backup_step] += np.asarray(scenario.backup_accel_bias_step)
    gyro1[backup_step] += np.deg2rad(np.asarray(scenario.backup_gyro_bias_step_deg_s))
    if scenario.primary_imu_freeze is not None:
        a, b = scenario.primary_imu_freeze
        i0 = int(np.searchsorted(t, a)); i1 = int(np.searchsorted(t, b))
        if i1 > i0:
            acc0[i0:i1] = acc0[i0]
            gyro0[i0:i1] = gyro0[i0]

    tv = np.arange(0.0, cfg.duration + 1e-10, 1.0 / cfg.vio_rate)
    iv = time_to_index(tv, cfg.imu_rate, N)
    dp = np.zeros((N, 3)); dth = np.zeros((N, 3))
    for k in range(1, N):
        dp[k] = dp[k - 1] + cfg.vio_drift_pos_rw * math.sqrt(dt) * rng.standard_normal(3)
        dth[k] = dth[k - 1] + cfg.vio_drift_att_rw * math.sqrt(dt) * rng.standard_normal(3)
    vio_p = np.zeros((len(tv), 3)); vio_v = np.zeros_like(vio_p); vio_q = np.zeros((len(tv), 4))
    for i, k in enumerate(iv):
        RWB = q2R(gt["q"][k]); pWC = gt["p"][k] + RWB @ cfg.r_BC
        RWCm = RWB @ q2R(qexp(dth[k] + cfg.vio_att_sigma * rng.standard_normal(3)))
        pWCm = pWC + dp[k] + cfg.vio_pos_sigma * rng.standard_normal(3)
        # Host output is converted back to the body origin: prediction must be body pose.
        vio_p[i] = pWCm - RWCm @ cfg.r_BC
        vio_q[i] = R2q(RWCm)
        vio_v[i] = gt["v"][k] + cfg.vio_vel_sigma * rng.standard_normal(3)
    vio_valid = np.array([not in_windows(float(x), scenario.vio_outages) for x in tv], dtype=bool)
    vio_outlier = rng.random(len(tv)) < (cfg.vio_outlier_probability + scenario.vio_outlier_probability_extra)
    vio_outlier[0] = False
    vio_p[vio_outlier] += 0.35 * rng.standard_normal((int(np.sum(vio_outlier)), 3))
    for i in np.flatnonzero(vio_outlier):
        vio_q[i] = qmul(vio_q[i], qexp(np.deg2rad(12.0) * rng.standard_normal(3)))

    tr = np.arange(0.0, cfg.duration + 1e-10, 1.0 / cfg.range_rate)
    ir = time_to_index(tr, cfg.imu_rate, N)
    zr = np.zeros(len(tr))
    for i, k in enumerate(ir):
        R = q2R(gt["q"][k]); pS = gt["p"][k] + R @ cfg.r_BR; dW = R @ cfg.d_BR
        zr[i] = -pS[2] / dW[2] + cfg.range_sigma * rng.standard_normal()
    range_valid = (rng.random(len(tr)) > 0.05) & np.array([not in_windows(float(x), scenario.range_outages) for x in tr])

    tb = np.arange(0.0, cfg.duration + 1e-10, 1.0 / cfg.baro_rate)
    ib = time_to_index(tb, cfg.imu_rate, N)
    baro_bias = np.zeros(len(tb)); baro_bias[0] = 0.12
    for i in range(1, len(tb)):
        baro_bias[i] = baro_bias[i - 1] + cfg.baro_bias_rw * math.sqrt(1.0 / cfg.baro_rate) * rng.standard_normal()
    drift = np.maximum(0.0, tb - scenario.baro_drift_start) * scenario.baro_drift_rate_mps
    zb = gt["p"][ib, 2] + baro_bias + drift + cfg.baro_sigma * rng.standard_normal(len(tb))
    baro_valid = np.array([not in_windows(float(x), scenario.baro_outages) for x in tb], dtype=bool)

    tl = np.arange(0.0, cfg.duration + 1e-10, 1.0 / cfg.lidar_rate)
    il = time_to_index(tl, cfg.imu_rate, N)
    lidar_truth = np.zeros((len(tl), 3)); lidar_pose = np.zeros((len(tl), 3))
    for i, k in enumerate(il):
        R = q2R(gt["q"][k]); pL = gt["p"][k] + R @ cfg.r_BL
        yaw = math.atan2(R[1, 0], R[0, 0])
        lidar_truth[i] = [pL[0], pL[1], yaw]
        lidar_pose[i] = [pL[0] + cfg.lidar_sigma_xy * rng.standard_normal(), pL[1] + cfg.lidar_sigma_xy * rng.standard_normal(), wrap_pi(yaw + cfg.lidar_sigma_yaw * rng.standard_normal())]
    lidar_valid = np.array([not in_windows(float(x), scenario.lidar_outages) for x in tl], dtype=bool)
    lidar_valid &= rng.random(len(tl)) >= scenario.lidar_dropout_probability
    false_pose = rng.random(len(tl)) < scenario.lidar_false_pose_probability
    false_pose[0] = False
    lidar_pose[false_pose, :2] += rng.uniform(-0.5, 0.5, (int(np.sum(false_pose)), 2))
    lidar_pose[false_pose, 2] = wrap_pi(lidar_pose[false_pose, 2] + rng.uniform(-math.radians(20), math.radians(20), int(np.sum(false_pose))))

    scans = None
    if generate_scans:
        scans = simulate_lidar_scans(gt, tl, il, cfg, rng)

    meas: Dict[str, object] = {
        "acc0": acc0, "gyro0": gyro0, "acc1": acc1, "gyro1": gyro1,
        "ba0_true": ba0, "bg0_true": bg0, "ba1_true": ba1, "bg1_true": bg1,
        "t_vio": tv, "i_vio": iv, "vio_p": vio_p, "vio_v": vio_v, "vio_q": vio_q, "vio_valid": vio_valid, "vio_outlier": vio_outlier,
        "t_range": tr, "i_range": ir, "zr": zr, "range_valid": range_valid,
        "t_baro": tb, "i_baro": ib, "zb": zb, "baro_valid": baro_valid, "baro_bias_true": baro_bias + drift,
        "t_lidar": tl, "i_lidar": il, "lidar_pose": lidar_pose, "lidar_truth": lidar_truth, "lidar_valid": lidar_valid, "lidar_false": false_pose,
        "scans": scans,
    }
    return gt, meas


def simulate_lidar_scans(gt: Dict[str, np.ndarray], tl: np.ndarray, il: np.ndarray, cfg: Config, rng: np.random.Generator) -> List[np.ndarray]:
    angles = np.linspace(0.0, 2.0 * np.pi, cfg.n_beams, endpoint=False)
    scans: List[np.ndarray] = []
    for k in il:
        RWB = q2R(gt["q"][k]); pL = gt["p"][k] + RWB @ cfg.r_BL
        yaw = math.atan2(RWB[1, 0], RWB[0, 0])
        ranges = np.zeros(cfg.n_beams)
        for b, angle in enumerate(angles):
            wa = angle + yaw; dx, dy = math.cos(wa), math.sin(wa)
            candidates: List[float] = []
            if abs(dx) > 1e-12:
                candidates.extend([(0.0 - pL[0]) / dx, (cfg.room[0] - pL[0]) / dx])
            if abs(dy) > 1e-12:
                candidates.extend([(0.0 - pL[1]) / dy, (cfg.room[1] - pL[1]) / dy])
            hit = min(x for x in candidates if x > 1e-6)
            for ox, oy, ow, od in cfg.obstacles:
                tx1, tx2 = ((ox - pL[0]) / dx, (ox + ow - pL[0]) / dx) if abs(dx) > 1e-12 else (-np.inf, np.inf)
                ty1, ty2 = ((oy - pL[1]) / dy, (oy + od - pL[1]) / dy) if abs(dy) > 1e-12 else (-np.inf, np.inf)
                enter = max(min(tx1, tx2), min(ty1, ty2)); exit_ = min(max(tx1, tx2), max(ty1, ty2))
                if enter > 0.0 and enter < exit_ and enter < hit:
                    hit = enter
            ranges[b] = np.clip(hit + cfg.range_noise * rng.standard_normal(), 0.15, 12.0)
        scans.append(np.c_[ranges * np.cos(angles), ranges * np.sin(angles)])
    return scans


# ---------------------------------------------------------------------------
# ESKF lanes and health selector.
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class LaneSpec:
    lane_id: int
    name: str
    imu_id: int
    use_vio: bool
    use_lidar: bool
    use_range: bool = True
    use_baro: bool = True


LANE_SPECS = (
    LaneSpec(1, "Primary all-aid", 0, True, True),
    LaneSpec(2, "Backup all-aid", 1, True, True),
    LaneSpec(3, "Primary VIO lane", 0, True, False),
    LaneSpec(4, "Backup LiDAR lane", 1, False, True),
)


class Lane:
    def __init__(self, spec: LaneSpec, meas: Dict[str, object], cfg: Config):
        self.spec = spec
        self.p = np.asarray(meas["vio_p"])[0].copy()
        self.v = np.asarray(meas["vio_v"])[0].copy()
        self.q = qnormalize(np.asarray(meas["vio_q"])[0])
        self.ba = np.zeros(3); self.bg = np.zeros(3); self.bbaro = 0.12
        s = np.r_[np.repeat(0.05, 3), np.repeat(0.12, 3), np.deg2rad([3, 3, 4]), np.repeat(0.06, 3), np.deg2rad([0.4, 0.4, 0.4]), 0.20]
        self.P = np.diag(s**2)
        self.ratios: Dict[str, deque] = defaultdict(lambda: deque(maxlen=cfg.health_window))
        self.last_accept = {"vio": 0.0, "lidar": -np.inf, "range": 0.0, "baro": 0.0, "gravity": -np.inf}
        self.last_reject = {"vio": -np.inf, "lidar": -np.inf, "range": -np.inf, "baro": -np.inf, "gravity": -np.inf}
        self.accepted = defaultdict(int); self.rejected = defaultdict(int)
        self.score = np.inf; self.eligible = False; self.reason = "initializing"

    def finite(self) -> bool:
        arrays = [self.p, self.v, self.q, self.ba, self.bg, np.array([self.bbaro]), self.P]
        return all(np.all(np.isfinite(x)) for x in arrays) and 0.98 < np.linalg.norm(self.q) < 1.02


def propagate_lane(lane: Lane, meas: Dict[str, object], k: int, dt: float, cfg: Config) -> None:
    acc = np.asarray(meas[f"acc{lane.spec.imu_id}"])[k]
    gyro = np.asarray(meas[f"gyro{lane.spec.imu_id}"])[k]
    fb = acc - lane.ba; w = gyro - lane.bg; R = q2R(lane.q); aw = R @ fb + cfg.gW
    lane.p = lane.p + lane.v * dt + 0.5 * aw * dt**2
    lane.v = lane.v + aw * dt
    lane.q = qmul(lane.q, qexp(w * dt))
    F = np.zeros((16, 16)); F[:3, 3:6] = np.eye(3); F[3:6, 6:9] = -R @ skew(fb); F[3:6, 9:12] = -R
    F[6:9, 6:9] = -skew(w); F[6:9, 12:15] = -np.eye(3)
    Phi = np.eye(16) + F * dt
    G = np.zeros((16, 13)); G[3:6, :3] = -R; G[6:9, 3:6] = -np.eye(3); G[9:12, 6:9] = np.eye(3); G[12:15, 9:12] = np.eye(3); G[15, 12] = 1.0
    qc = np.r_[np.repeat(cfg.accel_nd**2, 3), np.repeat(cfg.gyro_nd**2, 3), np.repeat(cfg.accel_bias_rw**2, 3), np.repeat(cfg.gyro_bias_rw**2, 3), cfg.baro_bias_rw**2]
    lane.P = Phi @ lane.P @ Phi.T + G @ np.diag(qc) @ G.T * dt
    lane.P = 0.5 * (lane.P + lane.P.T)


def filter_update(lane: Lane, residual: np.ndarray, H: np.ndarray, Rm: np.ndarray, gate: float, sensor: str, t: float) -> Tuple[bool, float]:
    r = np.asarray(residual, dtype=float).reshape(-1)
    Rm = np.atleast_2d(Rm)
    S = H @ lane.P @ H.T + Rm
    try:
        nis = float(r @ np.linalg.solve(S, r))
    except np.linalg.LinAlgError:
        nis = np.inf
    ratio = nis / gate if np.isfinite(nis) else np.inf
    lane.ratios[sensor].append(float(min(ratio, 100.0)))
    if not np.isfinite(nis) or nis > gate:
        lane.rejected[sensor] += 1; lane.last_reject[sensor] = t
        return False, nis
    K = np.linalg.solve(S, (lane.P @ H.T).T).T
    dx = K @ r
    lane.p += dx[:3]; lane.v += dx[3:6]; lane.q = qmul(lane.q, qexp(dx[6:9]))
    lane.ba += dx[9:12]; lane.bg += dx[12:15]; lane.bbaro += dx[15]
    I_KH = np.eye(16) - K @ H
    lane.P = I_KH @ lane.P @ I_KH.T + K @ Rm @ K.T
    reset = np.eye(16); reset[6:9, 6:9] = np.eye(3) - 0.5 * skew(dx[6:9])
    lane.P = reset @ lane.P @ reset.T
    lane.P = 0.5 * (lane.P + lane.P.T)
    lane.accepted[sensor] += 1; lane.last_accept[sensor] = t
    return True, nis


def lidar_measurement(lane: Lane, cfg: Config) -> Tuple[np.ndarray, np.ndarray]:
    R = q2R(lane.q); pL = lane.p + R @ cfg.r_BL
    h = np.array([pL[0], pL[1], math.atan2(R[1, 0], R[0, 0])])
    H = np.zeros((3, 16)); H[0, 0] = H[1, 1] = 1.0; A = -R @ skew(cfg.r_BL); H[:2, 6:9] = A[:2]
    eps = 1e-7
    for j in range(3):
        d = np.zeros(3); d[j] = eps; Rp = R @ q2R(qexp(d))
        H[2, 6 + j] = wrap_pi(math.atan2(Rp[1, 0], Rp[0, 0]) - h[2]) / eps
    return h, H


def range_measurement(lane: Lane, cfg: Config) -> Tuple[float, np.ndarray]:
    R = q2R(lane.q); pS = lane.p + R @ cfg.r_BR; dW = R @ cfg.d_BR; den = dW[2]
    H = np.zeros((1, 16))
    if den >= -0.2:
        return np.nan, H
    num = -pS[2]; h = num / den; H[0, 2] = -1.0 / den
    Ar = -R @ skew(cfg.r_BR); Ad = -R @ skew(cfg.d_BR)
    H[0, 6:9] = (-Ar[2] * den - num * Ad[2]) / den**2
    return float(h), H


def lane_health(lane: Lane, now: float, cfg: Config) -> Tuple[float, bool, str]:
    xy_ages = []
    if lane.spec.use_vio:
        xy_ages.append(now - lane.last_accept["vio"])
    if lane.spec.use_lidar:
        xy_ages.append(now - lane.last_accept["lidar"])
    xy_age = min(xy_ages) if xy_ages else np.inf
    z_ages = []
    if lane.spec.use_vio: z_ages.append(now - lane.last_accept["vio"])
    if lane.spec.use_range: z_ages.append(now - lane.last_accept["range"])
    if lane.spec.use_baro: z_ages.append(now - lane.last_accept["baro"])
    z_age = min(z_ages) if z_ages else np.inf
    pxy = float(np.trace(lane.P[:2, :2])); pz = float(lane.P[2, 2])
    finite = lane.finite()
    eligible = finite and xy_age <= cfg.xy_aid_timeout and z_age <= cfg.z_aid_timeout and pxy <= cfg.max_xy_covariance and pz <= cfg.max_z_covariance
    reasons = []
    if not finite: reasons.append("nonfinite")
    if xy_age > cfg.xy_aid_timeout: reasons.append("xy stale")
    if z_age > cfg.z_aid_timeout: reasons.append("z stale")
    if pxy > cfg.max_xy_covariance: reasons.append("Pxy high")
    if pz > cfg.max_z_covariance: reasons.append("Pz high")

    # Low is better. Ratios are normalized by each sensor's own chi-square gate.
    sensor_weights = {"vio": 1.0, "lidar": 1.0, "range": 0.35, "baro": 0.25, "gravity": 0.35}
    score = 0.0; weight_sum = 0.0
    for sensor, weight in sensor_weights.items():
        buf = lane.ratios[sensor]
        if buf:
            robust_ratio = float(np.median(np.asarray(buf)))
            score += weight * min(robust_ratio, 8.0); weight_sum += weight
    score = score / weight_sum if weight_sum else 1.0
    # Prefer all-aid lanes when health is otherwise comparable. This prevents
    # harmless innovation noise from moving the output to a sensor-reduced lane.
    if lane.spec.lane_id == 2:
        score += 0.03
    elif lane.spec.lane_id in (3, 4):
        score += 0.12
    score += 3.0 * min(pxy / cfg.max_xy_covariance, 10.0)
    score += 1.0 * min(pz / cfg.max_z_covariance, 10.0)
    score += 2.0 * max(0.0, xy_age - 0.25)
    if not eligible:
        score += 100.0
    return score, eligible, ", ".join(reasons) if reasons else "healthy"


@dataclass
class SelectorState:
    active_lane: int = 1
    candidate_lane: Optional[int] = None
    candidate_since: float = -np.inf
    last_switch_time: float = -np.inf
    switch_count: int = 0
    switch_log: List[Dict[str, float | int | str]] = field(default_factory=list)
    offset_p: np.ndarray = field(default_factory=lambda: np.zeros(3))
    offset_v: np.ndarray = field(default_factory=lambda: np.zeros(3))
    offset_q: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    blend_start_time: float = -np.inf


def choose_lane(lanes: List[Lane], selector: SelectorState, now: float, cfg: Config, output_state: Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]], imu_suspect: Optional[int] = None) -> bool:
    for lane in lanes:
        lane.score, lane.eligible, lane.reason = lane_health(lane, now, cfg)
    active = lanes[selector.active_lane - 1]
    eligible = [lane for lane in lanes if lane.eligible]
    if not eligible:
        selector.candidate_lane = None
        return False
    best = min(eligible, key=lambda x: (x.score, x.spec.lane_id))
    active_unhealthy = not active.eligible
    improvement = active.score - best.score
    # Cross-IMU fault fast path: when the active IMU family is producing much
    # larger normalized innovations than the alternate IMU family, reduce the
    # confirmation delay. Normal healthy lane changes still use conservative
    # hysteresis and dwell time.
    cross_imu_fault = (
        active.spec.imu_id != best.spec.imu_id
        and (
            (imu_suspect is not None and active.spec.imu_id == imu_suspect)
            or (active.score > 0.45 and best.score < 0.38 and improvement > 0.10)
        )
    )
    switch_margin = 0.02 if cross_imu_fault else cfg.score_switch_margin
    confirm_time = 0.03 if cross_imu_fault else cfg.switch_confirm_time
    if best.spec.lane_id == selector.active_lane or (not active_unhealthy and improvement < switch_margin):
        selector.candidate_lane = None
        return False

    # A healthy active lane uses a consistency gate. If the active lane is declared
    # unhealthy, permit failover despite divergence; continuity is handled by output blending.
    if not active_unhealthy:
        pos_difference = np.linalg.norm(best.p - active.p)
        att_difference = np.linalg.norm(orientation_residual(active.q, best.q))
        if pos_difference > cfg.switch_consistency_position or att_difference > cfg.switch_consistency_attitude:
            selector.candidate_lane = None
            return False
    if now - selector.last_switch_time < cfg.minimum_dwell_time:
        return False
    if selector.candidate_lane != best.spec.lane_id:
        selector.candidate_lane = best.spec.lane_id
        selector.candidate_since = now
        return False
    if now - selector.candidate_since < confirm_time:
        return False

    old_id = selector.active_lane
    selector.active_lane = best.spec.lane_id
    selector.last_switch_time = now
    selector.switch_count += 1
    selector.candidate_lane = None
    if output_state is not None:
        p_out, v_out, q_out = output_state
        selector.offset_p = p_out - best.p
        selector.offset_v = v_out - best.v
        selector.offset_q = qmul(qconj(best.q), q_out)
        selector.blend_start_time = now
    selector.switch_log.append({"time": now, "from": old_id, "to": best.spec.lane_id, "old_score": active.score, "new_score": best.score, "reason": active.reason})
    return True


def blended_output(active: Lane, selector: SelectorState, now: float, cfg: Config) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    if not np.isfinite(selector.blend_start_time):
        return active.p.copy(), active.v.copy(), active.q.copy()
    elapsed = max(0.0, now - selector.blend_start_time)
    if elapsed >= cfg.output_blend_time:
        selector.offset_p[:] = 0.0; selector.offset_v[:] = 0.0; selector.offset_q = np.array([1.0, 0.0, 0.0, 0.0]); selector.blend_start_time = -np.inf
        return active.p.copy(), active.v.copy(), active.q.copy()
    remaining = 1.0 - elapsed / cfg.output_blend_time
    p = active.p + remaining * selector.offset_p
    v = active.v + remaining * selector.offset_v
    q = qmul(active.q, slerp_offset(selector.offset_q, remaining))
    return p, v, q


# ---------------------------------------------------------------------------
# Full LiDAR frontend and global graph.
# ---------------------------------------------------------------------------
def transform_points(P: np.ndarray, pose: np.ndarray) -> np.ndarray:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    R = np.array([[c, -s], [s, c]])
    return P @ R.T + pose[:2]


def se2_compose(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0] + ca * b[0] - sa * b[1], a[1] + sa * b[0] + ca * b[1], wrap_pi(a[2] + b[2])], dtype=float)


def se2_inverse(p: np.ndarray) -> np.ndarray:
    c, s = math.cos(p[2]), math.sin(p[2])
    return np.array([-c * p[0] - s * p[1], s * p[0] - c * p[1], wrap_pi(-p[2])], dtype=float)


def se2_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return se2_compose(se2_inverse(a), b)


def rigid_fit(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    cs, cd = np.mean(src, axis=0), np.mean(dst, axis=0)
    X, Y = src - cs, dst - cd
    U, _, Vt = np.linalg.svd(X.T @ Y)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    trans = cd - R @ cs
    return np.array([trans[0], trans[1], math.atan2(R[1, 0], R[0, 0])])


def voxel_down(P: np.ndarray, voxel: float) -> np.ndarray:
    if len(P) == 0: return P
    keys = np.floor(P / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return P[np.sort(idx)]


def icp_to_map(scan: np.ndarray, mappts: np.ndarray, pose: np.ndarray, cfg: Config) -> Tuple[np.ndarray, float, int]:
    pose = pose.copy(); rmse = np.inf; overlap = 0
    tree = cKDTree(mappts)
    for _ in range(cfg.icp_iterations):
        Q = transform_points(scan, pose)
        d, idx = tree.query(Q, k=1)
        ids = np.flatnonzero(d < cfg.icp_max_corr); overlap = len(ids)
        if overlap < 40: break
        ids = ids[np.argsort(d[ids])[: max(40, int(cfg.icp_trim * len(ids)))]]
        delta = rigid_fit(Q[ids], mappts[idx[ids]])
        delta[:2] = np.clip(delta[:2], -cfg.icp_step_xy, cfg.icp_step_xy)
        delta[2] = np.clip(delta[2], -cfg.icp_step_yaw, cfg.icp_step_yaw)
        pose = se2_compose(delta, pose)
        rmse = float(math.sqrt(np.mean(d[ids] ** 2)))
        if np.linalg.norm(delta[:2]) < 1e-4 and abs(delta[2]) < 1e-4: break
    return pose, rmse, overlap


class LidarFrontend:
    def __init__(self, scans: List[np.ndarray], cfg: Config):
        self.scans = scans; self.cfg = cfg; self.map_blocks: List[np.ndarray] = []
        self.pose = np.zeros((len(scans), 3)); self.valid = np.zeros(len(scans), dtype=bool); self.diag = np.zeros((len(scans), 7))

    def process(self, i: int, prior: np.ndarray) -> Tuple[np.ndarray, bool, Dict[str, float]]:
        if not self.map_blocks:
            z = prior.copy(); rmse = 0.0; overlap = len(self.scans[i]); correction = np.zeros(3); healthy = True
        else:
            first = max(0, len(self.map_blocks) - self.cfg.local_submap_scans)
            mappts = voxel_down(np.vstack(self.map_blocks[first:]), self.cfg.map_voxel)
            z, rmse, overlap = icp_to_map(self.scans[i], mappts, prior, self.cfg)
            correction = se2_between(prior, z)
            healthy = np.isfinite(rmse) and rmse < self.cfg.icp_health_rmse and overlap >= self.cfg.icp_health_overlap and np.linalg.norm(correction[:2]) < self.cfg.icp_health_correction and abs(correction[2]) < self.cfg.icp_health_yaw
        self.pose[i] = z if healthy else prior
        self.valid[i] = healthy
        if healthy:
            self.map_blocks.append(transform_points(self.scans[i][::2], z))
        self.diag[i] = [rmse, overlap, float(healthy), 0.0, np.nan, np.linalg.norm(correction[:2]), abs(correction[2])]
        return self.pose[i].copy(), bool(healthy), {"rmse": float(rmse), "overlap": int(overlap), "correction_xy": float(np.linalg.norm(correction[:2])), "correction_yaw": float(abs(correction[2]))}


def scan_context_descriptor(scan: np.ndarray, cfg: Config) -> Tuple[np.ndarray, np.ndarray]:
    r = np.linalg.norm(scan, axis=1); theta = np.mod(np.arctan2(scan[:, 1], scan[:, 0]), 2 * np.pi)
    desc = np.zeros((cfg.sc_rings, cfg.sc_sectors))
    ri = np.clip(np.floor(r / 8.0 * cfg.sc_rings).astype(int), 0, cfg.sc_rings - 1)
    si = np.clip(np.floor(theta / (2 * np.pi) * cfg.sc_sectors).astype(int), 0, cfg.sc_sectors - 1)
    for n in range(len(r)):
        desc[ri[n], si[n]] = max(desc[ri[n], si[n]], 1.0 - r[n] / 8.0)
    return desc, np.mean(desc, axis=1)


def scan_context_distance(a: np.ndarray, b: np.ndarray) -> Tuple[float, int]:
    best, best_shift = 1.0, 0
    for shift in range(a.shape[1]):
        bs = np.roll(b, shift, axis=1)
        valid = (np.linalg.norm(a, axis=0) > 1e-8) & (np.linalg.norm(bs, axis=0) > 1e-8)
        if not np.any(valid): continue
        dots = np.sum(a[:, valid] * bs[:, valid], axis=0)
        den = np.linalg.norm(a[:, valid], axis=0) * np.linalg.norm(bs[:, valid], axis=0)
        d = 1.0 - float(np.mean(dots / np.maximum(den, 1e-12)))
        if d < best: best, best_shift = d, shift
    return best, best_shift


def icp_pair(cur: np.ndarray, ref: np.ndarray, pose: np.ndarray) -> Tuple[np.ndarray, float, int]:
    pose = pose.copy(); rmse = np.inf; overlap = 0; tree = cKDTree(ref)
    for _ in range(20):
        Q = transform_points(cur, pose); d, idx = tree.query(Q, k=1); ids = np.flatnonzero(d < 0.25); overlap = len(ids)
        if overlap < 50: break
        ids = ids[np.argsort(d[ids])[: max(50, int(0.75 * len(ids)))]]
        delta = rigid_fit(Q[ids], ref[idx[ids]])
        delta[:2] = np.clip(delta[:2], -0.10, 0.10); delta[2] = np.clip(delta[2], -math.radians(5), math.radians(5))
        pose = se2_compose(delta, pose); rmse = float(math.sqrt(np.mean(d[ids] ** 2)))
        if np.linalg.norm(delta[:2]) < 1e-4 and abs(delta[2]) < 1e-4: break
    return pose, rmse, overlap


def build_global_pose_graph(scans: List[np.ndarray], local_pose: np.ndarray, valid: np.ndarray, cfg: Config) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    accepted = np.flatnonzero(valid); global_pose = local_pose.copy(); loops: List[List[float]] = []
    if len(accepted) < 3: return global_pose, np.zeros((0, 4)), accepted
    kf = [int(accepted[0])]
    for idx in accepted[1:]:
        if idx - kf[-1] >= cfg.keyframe_stride: kf.append(int(idx))
    if kf[-1] != int(accepted[-1]): kf.append(int(accepted[-1]))
    desc, keys = [], []
    for idx in kf:
        d, key = scan_context_descriptor(scans[idx], cfg); desc.append(d); keys.append(key)
    keys = np.asarray(keys)
    edges: List[Tuple[int, int, np.ndarray, np.ndarray]] = []
    for a in range(1, len(kf)):
        edges.append((a - 1, a, se2_between(local_pose[kf[a - 1]], local_pose[kf[a]]), np.array([0.055, 0.055, math.radians(1.5)])))
    exclude = max(6, int(math.ceil(cfg.sc_exclude_recent / cfg.keyframe_stride))); last = -10**9
    for a in range(exclude, len(kf)):
        if len(loops) >= cfg.max_loop_closures or a - last < 6: continue
        old = np.arange(0, a - exclude + 1)
        order = old[np.argsort(np.linalg.norm(keys[old] - keys[a], axis=1))[: min(6, len(old))]]
        candidates = []
        for b in order:
            d, shift = scan_context_distance(desc[a], desc[b]); candidates.append((d, int(b), shift))
        for d, b, _ in sorted(candidates)[:4]:
            if d > cfg.sc_threshold: break
            if np.linalg.norm(local_pose[kf[a], :2] - local_pose[kf[b], :2]) > 1.25: continue
            init = se2_between(local_pose[kf[b]], local_pose[kf[a]])
            z, rmse, overlap = icp_pair(scans[kf[a]], scans[kf[b]], init)
            consistency = se2_between(init, z)
            if rmse < cfg.sc_verify_rmse and overlap > 110 and np.linalg.norm(consistency[:2]) < 0.08 and abs(consistency[2]) < math.radians(3):
                edges.append((b, a, z, np.array([0.035, 0.035, math.radians(1.0)])))
                loops.append([kf[a], kf[b], d, rmse]); last = a; break
    if loops:
        P0 = local_pose[kf]
        anchor = P0[0].copy(); x0 = P0[1:].reshape(-1)
        def residual(x):
            P = np.vstack([anchor, x.reshape(-1, 3)]); out = []
            for i, j, z, sig in edges:
                pred = se2_between(P[i], P[j]); e = se2_between(z, pred); e[2] = wrap_pi(e[2]); q = e / sig
                a = np.abs(q); big = a > 1.5; q[big] = np.sign(q[big]) * np.sqrt(2 * 1.5 * a[big] - 1.5**2); out.extend(q)
            return np.asarray(out)
        sol = least_squares(residual, x0, max_nfev=3000, ftol=1e-7, xtol=1e-7, verbose=0)
        kg = np.vstack([anchor, sol.x.reshape(-1, 3)]); kg[:, 2] = wrap_pi(kg[:, 2])
        corrections = np.array([se2_compose(kg[a], se2_inverse(local_pose[kf[a]])) for a in range(len(kf))])
        for a in range(len(kf) - 1):
            i0, i1 = kf[a], kf[a + 1]
            for i in range(i0, i1 + 1):
                u = (i - i0) / max(1, i1 - i0)
                ci = (1 - u) * corrections[a, :2] + u * corrections[a + 1, :2]
                cy = wrap_pi(corrections[a, 2] + u * wrap_pi(corrections[a + 1, 2] - corrections[a, 2]))
                global_pose[i] = se2_compose(np.r_[ci, cy], local_pose[i])
    return global_pose, np.asarray(loops, dtype=float).reshape(-1, 4), np.asarray(kf, dtype=int)


# ---------------------------------------------------------------------------
# Integrated multi-lane run.
# ---------------------------------------------------------------------------
def recent_imu_group_score(lanes: List[Lane], imu_id: int) -> float:
    values = []
    for lane in lanes:
        if lane.spec.imu_id != imu_id:
            continue
        for sensor in ("vio", "lidar", "gravity"):
            buf = lane.ratios[sensor]
            if buf:
                values.extend(list(buf)[-3:])
    return float(np.median(values)) if values else np.inf


def run_trial(seed: int, scenario: Scenario, cfg: Config, *, full_frontend: bool = False, save_history: bool = False) -> Dict[str, object]:
    gt, meas = simulate_sensors(seed, cfg, scenario, generate_scans=full_frontend)
    safety = validate_reference_path(gt, cfg)
    if not safety["safe"]: raise RuntimeError(f"unsafe reference path: {safety}")
    lanes = [Lane(spec, meas, cfg) for spec in LANE_SPECS]
    selector = SelectorState(active_lane=1, last_switch_time=-np.inf)
    dt = 1.0 / cfg.imu_rate; N = len(gt["t"])
    kv, kr, kb, kl = 1, 1, 1, 0
    Rvio = np.diag(np.r_[np.repeat(cfg.vio_pos_sigma**2, 3), np.repeat(cfg.vio_vel_sigma**2, 3), np.repeat(cfg.vio_att_sigma**2, 3)])
    Rlid = np.diag([cfg.lidar_sigma_xy**2, cfg.lidar_sigma_xy**2, cfg.lidar_sigma_yaw**2])
    p_hist = np.zeros((N, 3)); v_hist = np.zeros((N, 3)); q_hist = np.zeros((N, 4)); lane_hist = np.ones(N, dtype=int)
    degraded = np.zeros(N, dtype=bool); rtl = np.zeros(N, dtype=bool); score_hist = np.full((N, len(lanes)), np.nan); eligible_hist = np.zeros((N, len(lanes)), dtype=bool); pxy_hist = np.full((N, len(lanes)), np.nan)
    active0 = lanes[0]; p_hist[0], v_hist[0], q_hist[0] = active0.p, active0.v, active0.q
    lidar_frontend = LidarFrontend(meas["scans"], cfg) if full_frontend else None
    local_lidar_pose = np.zeros((len(meas["t_lidar"]), 3)); local_lidar_valid = np.zeros(len(meas["t_lidar"]), dtype=bool); local_lidar_diag = np.zeros((len(meas["t_lidar"]), 7))
    degraded_start = None
    imu_disagreement_count = 0
    imu_suspect: Optional[int] = None

    def process_lidar_event(i: int, now: float):
        nonlocal local_lidar_pose, local_lidar_valid, local_lidar_diag
        source_lane = lanes[selector.active_lane - 1]
        prior, _ = lidar_measurement(source_lane, cfg)
        if full_frontend:
            z, frontend_ok, d = lidar_frontend.process(i, prior)
            sensor_valid = bool(meas["lidar_valid"][i]) and frontend_ok
            local_lidar_diag[i] = lidar_frontend.diag[i]
        else:
            z = np.asarray(meas["lidar_pose"])[i].copy(); sensor_valid = bool(meas["lidar_valid"][i])
            d = {"rmse": cfg.lidar_sigma_xy, "overlap": cfg.n_beams, "correction_xy": float(np.linalg.norm(z[:2] - prior[:2])), "correction_yaw": float(abs(wrap_pi(z[2] - prior[2])))}
            local_lidar_diag[i] = [d["rmse"], d["overlap"], float(sensor_valid), 0.0, np.nan, d["correction_xy"], d["correction_yaw"]]
        any_ok = False
        if sensor_valid:
            for lane in lanes:
                if not lane.spec.use_lidar: continue
                h, H = lidar_measurement(lane, cfg); r = z - h; r[2] = wrap_pi(r[2])
                ok, nis = filter_update(lane, r, H, Rlid, cfg.gate_lidar3, "lidar", now); any_ok |= ok
        local_lidar_pose[i] = z if any_ok else prior; local_lidar_valid[i] = any_ok
        local_lidar_diag[i, 3] = float(any_ok)

    # Initial measurement timestamps at zero are processed explicitly.
    while kl < len(meas["t_lidar"]) and meas["t_lidar"][kl] <= gt["t"][0] + np.finfo(float).eps:
        process_lidar_event(kl, float(meas["t_lidar"][kl])); kl += 1

    for k in range(1, N):
        now = float(gt["t"][k])
        for lane in lanes: propagate_lane(lane, meas, k, dt, cfg)
        while kv < len(meas["t_vio"]) and meas["t_vio"][kv] <= now + 1e-12:
            if bool(meas["vio_valid"][kv]):
                for lane in lanes:
                    if not lane.spec.use_vio: continue
                    r = np.r_[np.asarray(meas["vio_p"])[kv] - lane.p, np.asarray(meas["vio_v"])[kv] - lane.v, orientation_residual(lane.q, np.asarray(meas["vio_q"])[kv])]
                    H = np.zeros((9, 16)); H[:3, :3] = np.eye(3); H[3:6, 3:6] = np.eye(3); H[6:9, 6:9] = np.eye(3)
                    filter_update(lane, r, H, Rvio, cfg.gate_vio9, "vio", float(meas["t_vio"][kv]))
            kv += 1
        while kl < len(meas["t_lidar"]) and meas["t_lidar"][kl] <= now + 1e-12:
            process_lidar_event(kl, float(meas["t_lidar"][kl])); kl += 1
        while kr < len(meas["t_range"]) and meas["t_range"][kr] <= now + 1e-12:
            if bool(meas["range_valid"][kr]):
                for lane in lanes:
                    if not lane.spec.use_range: continue
                    h, H = range_measurement(lane, cfg)
                    if np.isfinite(h): filter_update(lane, np.array([meas["zr"][kr] - h]), H, np.array([[cfg.range_sigma**2]]), cfg.gate_1, "range", float(meas["t_range"][kr]))
            kr += 1
        while kb < len(meas["t_baro"]) and meas["t_baro"][kb] <= now + 1e-12:
            if bool(meas["baro_valid"][kb]):
                for lane in lanes:
                    if not lane.spec.use_baro: continue
                    H = np.zeros((1, 16)); H[0, 2] = 1.0; H[0, 15] = 1.0
                    filter_update(lane, np.array([meas["zb"][kb] - (lane.p[2] + lane.bbaro)]), H, np.array([[cfg.baro_sigma**2]]), cfg.gate_1, "baro", float(meas["t_baro"][kb]))
            kb += 1
        if k % max(1, cfg.imu_rate // 10) == 0:
            for lane in lanes:
                acc = np.asarray(meas[f"acc{lane.spec.imu_id}"])[k]; gyro = np.asarray(meas[f"gyro{lane.spec.imu_id}"])[k]
                corrected = acc - lane.ba
                if abs(np.linalg.norm(corrected) - cfg.g) <= cfg.gravity_norm_tolerance and np.linalg.norm(gyro - lane.bg) <= cfg.gravity_max_gyro:
                    R = q2R(lane.q); g_body = R.T @ (-cfg.gW); h = g_body + lane.ba
                    H = np.zeros((3, 16)); H[:, 6:9] = skew(g_body); H[:, 9:12] = np.eye(3)
                    filter_update(lane, acc - h, H, np.eye(3) * cfg.gravity_sigma**2, cfg.gate_gravity3, "gravity", now)

        # Two-IMU disagreement alone cannot identify the failed sensor, so use
        # aiding innovations from the parallel lane families to attribute it.
        acc_disagreement = np.linalg.norm(np.asarray(meas["acc0"])[k] - np.asarray(meas["acc1"])[k])
        gyro_disagreement = np.linalg.norm(np.asarray(meas["gyro0"])[k] - np.asarray(meas["gyro1"])[k])
        if acc_disagreement > 0.22 or gyro_disagreement > math.radians(1.2):
            imu_disagreement_count += 1
        else:
            imu_disagreement_count = max(0, imu_disagreement_count - 1)
        if imu_disagreement_count >= 5:
            primary_group = recent_imu_group_score(lanes, 0)
            backup_group = recent_imu_group_score(lanes, 1)
            if primary_group > backup_group + 0.06:
                imu_suspect = 0
            elif backup_group > primary_group + 0.06:
                imu_suspect = 1
        elif imu_disagreement_count == 0:
            imu_suspect = None

        current_output = (p_hist[k - 1].copy(), v_hist[k - 1].copy(), q_hist[k - 1].copy())
        choose_lane(lanes, selector, now, cfg, current_output, imu_suspect)
        active = lanes[selector.active_lane - 1]
        p_hist[k], v_hist[k], q_hist[k] = blended_output(active, selector, now, cfg)
        lane_hist[k] = selector.active_lane
        no_eligible = not any(lane.eligible for lane in lanes)
        degraded[k] = no_eligible
        if no_eligible:
            if degraded_start is None: degraded_start = now
        else:
            degraded_start = None
        rtl[k] = degraded_start is not None and now - degraded_start >= cfg.degraded_rtl_delay
        for j, lane in enumerate(lanes):
            score_hist[k, j] = lane.score; eligible_hist[k, j] = lane.eligible; pxy_hist[k, j] = np.trace(lane.P[:2, :2])

    pos_err = np.linalg.norm(p_hist - gt["p"], axis=1)
    att_err = np.array([np.linalg.norm(orientation_residual(q_hist[k], gt["q"][k])) for k in range(N)])
    ss = gt["t"] >= 5.0
    switch_jumps = []
    for event in selector.switch_log:
        idx = int(round(float(event["time"]) * cfg.imu_rate)); idx = np.clip(idx, 1, N - 1)
        switch_jumps.append(float(np.linalg.norm(p_hist[idx] - p_hist[idx - 1])))
    lidar_local_err = np.linalg.norm(local_lidar_pose[:, :2] - np.asarray(meas["lidar_truth"])[:, :2], axis=1)
    global_lidar = local_lidar_pose.copy(); loops = np.zeros((0, 4)); keyframes = np.flatnonzero(local_lidar_valid)
    if full_frontend:
        global_lidar, loops, keyframes = build_global_pose_graph(meas["scans"], local_lidar_pose, local_lidar_valid, cfg)
    global_lidar_err = np.linalg.norm(global_lidar[:, :2] - np.asarray(meas["lidar_truth"])[:, :2], axis=1)
    metrics = {
        "seed": seed, "scenario": scenario.name,
        "max_error_m": float(np.max(pos_err[ss])), "rmse_m": float(math.sqrt(np.mean(pos_err[ss] ** 2))), "final_error_m": float(pos_err[-1]),
        "attitude_max_deg": float(np.rad2deg(np.max(att_err[ss]))),
        "lidar_local_max_m": float(np.max(lidar_local_err)), "lidar_local_rmse_m": float(math.sqrt(np.mean(lidar_local_err**2))),
        "lidar_global_max_m": float(np.max(global_lidar_err)), "lidar_acceptance": float(np.mean(local_lidar_valid)), "verified_loops": int(len(loops)),
        "switches": selector.switch_count, "final_lane": int(selector.active_lane), "max_switch_jump_m": max(switch_jumps) if switch_jumps else 0.0,
        "degraded_duration_s": float(np.sum(degraded) * dt), "rtl_requested": bool(np.any(rtl)),
        "path_safe": bool(safety["safe"]), "min_clearance_m": float(safety["min_static_clearance_m"]),
    }
    # Scenario-specific acceptance: all-XY outage is a fail-safe behavior test, not a 10 cm navigation test.
    navigation_pass = metrics["max_error_m"] < cfg.requirement_m and metrics["attitude_max_deg"] < cfg.attitude_requirement_deg
    selector_pass = metrics["max_switch_jump_m"] < 0.05
    if scenario.expected_switch_to_backup:
        selector_pass &= metrics["switches"] >= 1 and metrics["final_lane"] in (2, 4)
    if scenario.expected_no_switch:
        selector_pass &= metrics["switches"] == 0
    if scenario.expected_degraded:
        behavior_pass = metrics["degraded_duration_s"] > 0.0 and metrics["rtl_requested"] and selector_pass
    else:
        behavior_pass = navigation_pass and selector_pass and metrics["degraded_duration_s"] < 0.5
    metrics["navigation_pass"] = bool(navigation_pass); metrics["selector_pass"] = bool(selector_pass); metrics["pass"] = bool(behavior_pass)

    out: Dict[str, object] = {"metrics": metrics, "switch_log": selector.switch_log, "safety": safety}
    if save_history:
        out["history"] = {
            "t": gt["t"], "truth_p": gt["p"], "p": p_hist, "v": v_hist, "q": q_hist, "pos_err": pos_err, "att_err": att_err,
            "lane": lane_hist, "degraded": degraded, "rtl": rtl, "scores": score_hist, "eligible": eligible_hist, "pxy": pxy_hist,
            "local_lidar": local_lidar_pose, "local_lidar_valid": local_lidar_valid, "local_lidar_diag": local_lidar_diag,
            "global_lidar": global_lidar, "loops": loops, "keyframes": keyframes,
        }
    return out


# ---------------------------------------------------------------------------
# Backtest harness and reports.
# ---------------------------------------------------------------------------
def run_matrix(output_dir: Path, seeds: Sequence[int], scenario_names: Sequence[str], full_frontend: bool = False) -> List[Dict[str, object]]:
    cfg = Config(); catalog = scenario_catalog(); output_dir.mkdir(parents=True, exist_ok=True)
    rows: List[Dict[str, object]] = []
    for scenario_name in scenario_names:
        scenario = catalog[scenario_name]
        for seed in seeds:
            start = time.time(); result = run_trial(seed, scenario, cfg, full_frontend=full_frontend, save_history=False); row = dict(result["metrics"]); row["runtime_s"] = time.time() - start
            rows.append(row)
            print(f"{scenario_name:23s} seed={seed:2d} max={100*row['max_error_m']:6.2f} cm rmse={100*row['rmse_m']:6.2f} cm lane={row['final_lane']} sw={row['switches']} deg={row['degraded_duration_s']:5.2f}s pass={int(row['pass'])}")
    with (output_dir / "backtest_runs.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys())); writer.writeheader(); writer.writerows(rows)
    summary = []
    for name in scenario_names:
        group = [r for r in rows if r["scenario"] == name]
        summary.append({
            "scenario": name, "runs": len(group), "passes": sum(bool(r["pass"]) for r in group),
            "worst_max_error_m": max(float(r["max_error_m"]) for r in group), "worst_rmse_m": max(float(r["rmse_m"]) for r in group),
            "worst_attitude_deg": max(float(r["attitude_max_deg"]) for r in group), "max_switch_jump_m": max(float(r["max_switch_jump_m"]) for r in group),
            "max_degraded_duration_s": max(float(r["degraded_duration_s"]) for r in group), "switches_total": sum(int(r["switches"]) for r in group),
        })
    with (output_dir / "backtest_summary.json").open("w") as f: json.dump(summary, f, indent=2)
    with (output_dir / "backtest_summary.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(summary[0].keys())); writer.writeheader(); writer.writerows(summary)
    return rows


def save_example_history(output_dir: Path, seed: int, scenario_name: str, full_frontend: bool = False) -> Dict[str, object]:
    cfg = Config(); scenario = scenario_catalog()[scenario_name]
    result = run_trial(seed, scenario, cfg, full_frontend=full_frontend, save_history=True)
    history = result.pop("history")
    np.savez_compressed(output_dir / f"history_{scenario_name}_seed_{seed:03d}.npz", **history)
    with (output_dir / f"history_{scenario_name}_seed_{seed:03d}_metrics.json").open("w") as f: json.dump(result, f, indent=2, default=lambda x: x.tolist() if isinstance(x, np.ndarray) else x)
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("S2_1_python_results"))
    parser.add_argument("--seeds", type=int, default=5, help="number of seeds starting at zero")
    parser.add_argument("--scenarios", nargs="*", default=list(scenario_catalog().keys()))
    parser.add_argument("--full-frontend", action="store_true", help="run full scan/ICP/pose graph (use few seeds)")
    parser.add_argument("--examples", action="store_true", help="save nominal and primary-IMU histories")
    args = parser.parse_args()
    rows = run_matrix(args.output, list(range(args.seeds)), args.scenarios, full_frontend=args.full_frontend)
    if args.examples:
        save_example_history(args.output, 0, "nominal", full_frontend=args.full_frontend)
        save_example_history(args.output, 0, "primary_imu_bias", full_frontend=args.full_frontend)
        save_example_history(args.output, 0, "all_xy_outage", full_frontend=args.full_frontend)
    print(f"\nSaved {len(rows)} runs to {args.output.resolve()}")


if __name__ == "__main__":
    main()
