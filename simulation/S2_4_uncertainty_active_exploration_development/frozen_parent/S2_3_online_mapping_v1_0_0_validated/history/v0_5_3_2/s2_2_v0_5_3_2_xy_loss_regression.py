#!/usr/bin/env python3
"""Focused source and reduced-model regression for S2.2 v0.5.3.2.

This does not replace MATLAB. It checks the exact failure mechanism exposed by
xy_loss_emergency_land seed 7: stale post-loss velocity was used twice, first
through explicit damping and later through the controller speed guard.
"""
from pathlib import Path
import math, random
import numpy as np

ROOT=Path(__file__).resolve().parent
cfg=(ROOT/'init_S2_2_config.m').read_text()
life=(ROOT/'mission_lifecycle_manager_S2_2.m').read_text()
ctrl=(ROOT/'geometric_controller_S2_2.m').read_text()

checks=[]
def check(name, ok):
    checks.append((name,bool(ok)))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}")

check('v0.5.3.2 namespace', "cfg.version='v0.5.3.2';" in cfg)
check('post-loss velocity damping disabled in config', 'cfg.emergencyVelocityDampingEnabled=false;' in cfg)
check('post-loss command returns damping false', 'dampingEnabled=false;' in life)
check('jerk-settle interval configured', 'cfg.xyLossBrakeSettleTime_s=0.16;' in cfg)
check('descent waits for jerk settle', 't>=xyLossBrakeEndTime+cfg.xyLossBrakeSettleTime_s' in life)
check('blind-mode flag exists', 'horizontalBlindMode=' in ctrl)
check('speed guard has two blind-mode bypasses', ctrl.count('if ~horizontalBlindMode') >= 2)
check('both speed-guard calls retained for observable flight', ctrl.count('guard_horizontal_speed(cfg,est.v(1:2),aCmd(1:2))') == 2)

# Reduced command-level Monte Carlo. It uses the package dt, jerk, brake limit,
# pulse construction and settling time. Horizontal acceleration during descent
# is zero. This is a mechanism test, not the 6-DOF MATLAB result.
dt=0.02; jerk=6.0; amax=0.50; ramp=0.18; tmin=0.25; tmax=0.90
settle=0.16; descent=4.40
rng=random.Random(5302)
residual=[]; drift=[]
for _ in range(20000):
    speed=rng.uniform(0.05,0.35)
    angle=rng.uniform(0,2*math.pi)
    v=np.array([math.cos(angle),math.sin(angle)])*speed
    p=np.zeros(2)
    aa=rng.uniform(0,2*math.pi)
    amag=rng.uniform(0,0.20)
    a=np.array([math.cos(aa),math.sin(aa)])*amag
    duration=max(tmin,min(tmax,speed/amax+ramp))
    a_brake=-v/duration
    t=0.0
    while t < duration+settle+descent-1e-12:
        target=a_brake if t<=duration+1e-12 else np.zeros(2)
        delta=target-a
        n=float(np.linalg.norm(delta))
        max_delta=jerk*dt
        if n>max_delta:
            delta*=max_delta/n
        a+=delta
        v+=a*dt
        p+=v*dt
        t+=dt
    residual.append(float(np.linalg.norm(v)))
    drift.append(float(np.linalg.norm(p)))

residual=np.array(residual); drift=np.array(drift)
check('reduced-model worst residual speed <= 0.020 m/s', residual.max()<=0.020)
check('reduced-model 99th-percentile drift <= 0.20 m', np.quantile(drift,0.99)<=0.20)
check('reduced-model worst drift <= 0.25 m', drift.max()<=0.25)
print(f"worst residual speed: {residual.max():.4f} m/s")
print(f"99th-percentile drift: {np.quantile(drift,0.99):.4f} m")
print(f"worst drift: {drift.max():.4f} m")

passed=sum(ok for _,ok in checks)
print(f"Result: {passed}/{len(checks)} checks passed")
if passed != len(checks):
    raise SystemExit(1)
print('S2.2 v0.5.3.2 XY-LOSS REGRESSION: PASS')
