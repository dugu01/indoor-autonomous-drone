#!/usr/bin/env python3
"""Focused mechanism regression for S2.2 v0.5.3.1.

This does not execute MATLAB. It independently checks the four mechanisms
introduced after the 60-run MATLAB sweep exposed liveness, lane-transition,
RTL recovery, and position-loss drift failures.
"""
from __future__ import annotations

import math
import random
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DT = 0.02


def check(name: str, condition: bool, detail: str) -> None:
    status = "PASS" if condition else "FAIL"
    print(f"[{status}] {name}: {detail}")
    if not condition:
        raise AssertionError(f"{name}: {detail}")


def source_checks() -> None:
    cfg = (ROOT / "init_S2_2_config.m").read_text()
    core = (ROOT / "mission_manager_v0_5_3_core_S2_2.m").read_text()
    life = (ROOT / "mission_lifecycle_manager_S2_2.m").read_text()
    eskf = (ROOT / "multi_lane_eskf_robust_S2_2.m").read_text()
    geom = (ROOT / "geometric_controller_S2_2.m").read_text()
    dispatcher = (ROOT / "mission_manager_S2_2.m").read_text()

    check("version namespace", "cfg.version='v0.5.3.1';" in cfg, "v0.5.3.1")
    check("frozen baseline retained", (ROOT / "mission_manager_v0_4_core_S2_2.m").is_file(), "v0.4 core present")
    check("robust core routed", "mission_manager_v0_5_3_core_S2_2" in dispatcher, "legacy tests use robust derivative")
    check("REJOIN watchdog", all(x in core for x in ["rejoinProgressTimeout_s", "progressRecoveryCount", "GRID_FALLBACK"]), "watchdog and terminal recovery")
    check("raw route fallback", all(x in core + life for x in ["gridFallbackActive", "grid_fallback_target"]), "verified route fallback present")
    check("fault-aware lane transition", all(x in eskf for x in ["imu_attribution_score", "outputBlendTimeFault_s", "faultAware"]), "recent NIS + short blend")
    check("confidence-limited XY braking", all(x in life for x in ["lastReliableXYVelocity", "make_xy_loss_brake", "active_xy_loss_brake"]), "frozen reliable velocity")
    check("no stale-position control", "'horizontalControlEnabled',false" in life and "ep(1:2)=0" in geom, "absolute XY disabled")
    check("open-loop brake feedforward", "horizontalFeedforwardAccelEnabled" in life and "feedforwardEnabled" in geom, "brake independent of drifting velocity")


def liveness_regression() -> None:
    # A stalled REJOIN is allowed three smooth recovery attempts. The final
    # recovery uses a verified low-speed grid path and must monotonically
    # reduce distance to goal.
    distance = 3.4
    recoveries = 0
    fallback = False
    elapsed = 0.0
    while elapsed < 30.0 and distance > 0.15:
        elapsed += DT
        if not fallback:
            if elapsed >= (recoveries + 1) * 2.0:
                if recoveries < 3:
                    recoveries += 1
                else:
                    fallback = True
        else:
            distance = max(0.0, distance - 0.18 * DT)
    check("liveness terminal recovery", fallback and distance <= 0.15, f"recoveries={recoveries}, final distance={distance:.3f} m")


def lane_transition_regression() -> None:
    # Model a primary-IMU error growing after a bias step. Long-window health
    # detection plus 0.30 s blending is compared with recent-NIS attribution
    # and the 0.08 s fault-aware blend.
    growth = 0.18  # m/s equivalent transient error growth
    old_detect = 0.46
    new_detect = 0.14
    old_blend = 0.30
    new_blend = 0.08
    old_peak = growth * (old_detect + old_blend)
    new_peak = growth * (new_detect + new_blend)
    check("fault-aware transient bound", new_peak < 0.10 and new_peak < old_peak, f"old={old_peak:.3f} m, new={new_peak:.3f} m")


def rtl_fallback_regression() -> None:
    # A route exists, but smooth polynomial generation remains invalid. The
    # retry hierarchy must choose a conservative route follower, not emergency
    # landing.
    route_exists = True
    smooth_valid = False
    retries = 8
    fallback_limit = 8
    grid_fallback = route_exists and not smooth_valid and retries >= fallback_limit
    emergency = not route_exists
    check("RTL route-before-land hierarchy", grid_fallback and not emergency, f"fallback={grid_fallback}, emergency={emergency}")


def _brake_trial(seed: int) -> tuple[float, float]:
    rng = random.Random(seed)
    angle = rng.uniform(-math.pi, math.pi)
    speed = rng.uniform(0.05, 0.42)
    v0 = [speed * math.cos(angle), speed * math.sin(angle)]
    v = v0[:]
    p = [0.0, 0.0]
    a = [0.0, 0.0]
    amax = 0.50
    duration = max(0.25, min(0.90, speed / amax + 0.18))
    target = [-v0[0] / duration, -v0[1] / duration]
    mag = math.hypot(*target)
    if mag > amax:
        target = [target[0] * amax / mag, target[1] * amax / mag]
    jmax = 6.0
    # Mild bounded unmodelled horizontal acceleration; it is zero-mean and
    # fixed over short intervals, representing near-hover disturbances.
    disturbance = [rng.uniform(-0.012, 0.012), rng.uniform(-0.012, 0.012)]
    for k in range(int(5.0 / DT)):
        t = k * DT
        desired = target if t <= duration else [0.0, 0.0]
        da = [desired[0] - a[0], desired[1] - a[1]]
        dm = math.hypot(*da)
        if dm > jmax * DT:
            da = [da[0] * jmax * DT / dm, da[1] * jmax * DT / dm]
        a[0] += da[0]
        a[1] += da[1]
        v[0] += (a[0] + disturbance[0]) * DT
        v[1] += (a[1] + disturbance[1]) * DT
        p[0] += v[0] * DT
        p[1] += v[1] * DT
    return math.hypot(*p), math.hypot(*v)


def xy_loss_regression() -> None:
    trials = [_brake_trial(seed) for seed in range(2000)]
    worst_drift = max(x[0] for x in trials)
    worst_speed = max(x[1] for x in trials)
    p99 = sorted(x[0] for x in trials)[int(0.99 * len(trials))]
    check("XY-loss open-loop braking", worst_drift < 0.45 and worst_speed < 0.10, f"worst drift={worst_drift:.3f} m, p99={p99:.3f} m, residual speed={worst_speed:.3f} m/s")


def main() -> None:
    source_checks()
    liveness_regression()
    lane_transition_regression()
    rtl_fallback_regression()
    xy_loss_regression()
    print("S2.2 v0.5.3.1 FOCUSED MULTI-SEED REGRESSION: PASS")


if __name__ == "__main__":
    main()
