"""Focused regression for S2.2 v0.4 Patch 1.

This is not a substitute for the MATLAB seven-scenario validator. It checks
three corrected mechanisms independently:
  1. noisy stopped-obstacle promotion;
  2. acceleration/jerk-limited safety velocity shaping;
  3. jerk-limited near-hover 6-DOF response to changing velocity commands.
"""
from __future__ import annotations
import math
import numpy as np

DT = 0.02


def tracker_promotion(seed: int) -> tuple[float | None, bool]:
    rng = np.random.default_rng(seed)
    p = None
    v = np.zeros(2)
    vf = np.zeros(2)
    t_prev = 0.0
    updates = 0
    timer = 0.0
    promoted = None
    early = False
    for k in range(int(15.0 / DT) + 1):
        t = k * DT
        if t < 5.0:
            continue
        move_end = min(t, 9.0)
        truth = np.array([5.25 - 0.20 * (move_end - 5.0), 2.0])
        z = truth + 0.020 * rng.normal(size=2)
        if p is None:
            p = z.copy()
            t_prev = t
            updates = 1
        else:
            d = max(1e-3, t - t_prev)
            pred = p + v * d
            residual = z - pred
            p = pred + 0.72 * residual
            v = v + (0.05 / d) * residual
            vf = 0.90 * vf + 0.10 * v
            t_prev = t
            updates += 1
        if updates >= 3 and np.linalg.norm(vf) <= 0.10:
            timer += DT
        else:
            timer = 0.0
        if timer >= 1.50 and promoted is None:
            promoted = t
            early = t < 9.0
    return promoted, early


def shape(v: np.ndarray, a: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    raw_a = (target - v) / DT
    na = np.linalg.norm(raw_a)
    if na > 0.55:
        raw_a *= 0.55 / na
    da = raw_a - a
    nda = np.linalg.norm(da)
    if nda > 2.0 * DT:
        da *= (2.0 * DT) / nda
    a = a + da
    na = np.linalg.norm(a)
    if na > 0.55:
        a *= 0.55 / na
    v = v + a * DT
    nv = np.linalg.norm(v)
    if nv > 0.32:
        v *= 0.32 / nv
    return v, a


def qmul(q: np.ndarray, r: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    a, b, c, d = r
    out = np.array([w*a-x*b-y*c-z*d, w*b+x*a+y*d-z*c,
                    w*c-x*d+y*a+z*b, w*d+x*c-y*b+z*a])
    return out / np.linalg.norm(out)


def qexp(rv: np.ndarray) -> np.ndarray:
    th = np.linalg.norm(rv)
    if th < 1e-12:
        return np.array([1.0, *(0.5 * rv)])
    return np.array([math.cos(th/2), *(math.sin(th/2) * rv / th)])


def q2r(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def six_dof_command_regression() -> dict[str, float]:
    mass = 1.50
    inertia = np.diag([0.030, 0.030, 0.055])
    p = np.array([3.0, 0.8, 1.15])
    vel = np.zeros(3)
    q = np.array([1.0, 0.0, 0.0, 0.0])
    omega = np.zeros(3)
    truth_a = np.zeros(3)
    previous_truth_a = np.zeros(3)
    previous_command_a = np.zeros(3)
    v_cmd = np.zeros(2)
    a_cmd_xy = np.zeros(2)
    max_speed = max_accel = max_jerk = max_tilt = 0.0
    targets = [np.array([0.32, 0.0]), np.array([0.0, 0.32]),
               np.array([-0.32, 0.0]), np.zeros(2), np.array([0.20, 0.20])]
    for k in range(4000):
        target = targets[min(k // 500, len(targets)-1)]
        v_cmd, a_cmd_xy = shape(v_cmd, a_cmd_xy, target)
        ep = np.array([0.0, 0.0, 1.15 - p[2]])
        ev = np.r_[v_cmd - vel[:2], -vel[2]]
        a_des = np.r_[a_cmd_xy, 0.0] + np.array([2.0, 2.0, 3.2])*ep + np.array([1.5, 1.5, 2.0])*ev
        h = np.linalg.norm(a_des[:2])
        if h > 0.65:
            a_des[:2] *= 0.65 / h
        a_des[2] = np.clip(a_des[2], -1.50, 1.50)
        delta = a_des - previous_command_a
        nd = np.linalg.norm(delta)
        if nd > 6.0 * DT:
            a_des = previous_command_a + delta * (6.0 * DT / nd)
        previous_command_a = a_des.copy()
        force = mass * (a_des + np.array([0.0, 0.0, 9.81]))
        b3 = force / np.linalg.norm(force)
        b1c = np.array([1.0, 0.0, 0.0])
        b2 = np.cross(b3, b1c)
        b2 /= np.linalg.norm(b2)
        rd = np.column_stack((np.cross(b2, b3), b2, b3))
        r = q2r(q)
        e = rd.T @ r - r.T @ rd
        e_r = 0.5 * np.array([e[2,1], e[0,2], e[1,0]])
        moment = -np.array([5.0, 5.0, 2.2])*e_r - np.array([0.55, 0.55, 0.25])*omega + np.cross(omega, inertia @ omega)
        moment = np.clip(moment, [-0.90, -0.90, -0.35], [0.90, 0.90, 0.35])
        thrust = np.clip(force @ r[:,2], 0.0, 2.40 * mass * 9.81)
        truth_a = r @ np.array([0.0, 0.0, thrust]) / mass + np.array([0.0, 0.0, -9.81]) - 0.10 * vel
        omega_dot = np.linalg.solve(inertia, moment - np.cross(omega, inertia @ omega) - 0.02 * omega)
        p = p + vel*DT + 0.5*truth_a*DT*DT
        vel = vel + truth_a*DT
        omega_mid = omega + 0.5*omega_dot*DT
        q = qmul(q, qexp(omega_mid*DT))
        omega = omega + omega_dot*DT
        jerk = np.linalg.norm((truth_a-previous_truth_a)[:2]) / DT
        previous_truth_a = truth_a.copy()
        r_now = q2r(q)
        pitch = math.asin(np.clip(-r_now[2,0], -1.0, 1.0))
        roll = math.atan2(r_now[2,1], r_now[2,2])
        max_speed = max(max_speed, np.linalg.norm(vel[:2]))
        max_accel = max(max_accel, np.linalg.norm(truth_a[:2]))
        max_jerk = max(max_jerk, jerk)
        max_tilt = max(max_tilt, math.degrees(math.hypot(roll, pitch)))
    return {'max_speed': max_speed, 'max_accel': max_accel,
            'max_jerk': max_jerk, 'max_tilt_deg': max_tilt}


def main() -> None:
    promotions = [tracker_promotion(seed) for seed in range(1000)]
    times = [p for p, _ in promotions if p is not None]
    assert len(times) == 1000
    assert not any(early for _, early in promotions)
    assert max(times) < 12.0

    rng = np.random.default_rng(7)
    v = np.zeros(2)
    a = np.zeros(2)
    previous_a = a.copy()
    max_v = max_a = max_j = 0.0
    target = np.zeros(2)
    candidates = np.array([[0.32,0.0],[0.0,0.32],[-0.32,0.0],[0.0,0.0],[0.20,-0.20]])
    for k in range(10000):
        if k % 20 == 0:
            target = candidates[rng.integers(len(candidates))]
        v, a = shape(v, a, target)
        max_v = max(max_v, np.linalg.norm(v))
        max_a = max(max_a, np.linalg.norm(a))
        max_j = max(max_j, np.linalg.norm(a-previous_a)/DT)
        previous_a = a.copy()
    assert max_v <= 0.3200001
    assert max_a <= 0.5500001
    assert max_j <= 2.000001

    response = six_dof_command_regression()
    assert response['max_speed'] <= 0.45
    assert response['max_accel'] <= 2.5
    assert response['max_jerk'] <= 25.0
    assert response['max_tilt_deg'] <= 5.0

    print('PATCH REGRESSION: PASS')
    print(f"tracker promotion: {min(times):.2f} to {max(times):.2f} s; 0/1000 early")
    print(f"velocity shaper max v/a/j: {max_v:.4f} / {max_a:.4f} / {max_j:.4f}")
    print('6-DOF response max v/a/j/tilt: '
          f"{response['max_speed']:.4f} / {response['max_accel']:.4f} / "
          f"{response['max_jerk']:.4f} / {response['max_tilt_deg']:.4f}")


if __name__ == '__main__':
    main()
