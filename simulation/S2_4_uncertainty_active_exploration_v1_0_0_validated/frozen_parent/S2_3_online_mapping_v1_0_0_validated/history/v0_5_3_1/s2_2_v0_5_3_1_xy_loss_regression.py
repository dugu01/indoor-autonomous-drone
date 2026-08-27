#!/usr/bin/env python3
"""Focused mechanism regression for S2.2 v0.5.3.1 XY-loss arrest.

This is a reduced horizontal model. It checks the change that matters here:
complete XY braking at constant altitude, with confidence-limited inertial
velocity damping, before starting the vertical descent. It is not a MATLAB or
6-DOF runtime claim.
"""
from __future__ import annotations
import math, random
from pathlib import Path

DT=0.02
ROOT=Path(__file__).resolve().parent


def simulate(seed: int):
    rng=random.Random(seed)
    angle=rng.uniform(-math.pi,math.pi)
    speed=rng.uniform(0.05,0.42)
    v=[speed*math.cos(angle),speed*math.sin(angle)]
    # Last aid-bounded estimate, with bounded mismatch.
    vest=[v[0]+rng.gauss(0,0.018),v[1]+rng.gauss(0,0.018)]
    p=[0.0,0.0]
    a=[rng.uniform(-0.08,0.08),rng.uniform(-0.08,0.08)]
    amax=0.50; cmdmax=0.65; gain=2.80; jmax=6.0; drag=0.10
    trust_end=0.85; exit_speed=0.055
    duration=max(0.25,min(0.90,math.hypot(*vest)/amax+0.18))
    abr=[-vest[0]/duration,-vest[1]/duration]
    mag=math.hypot(*abr)
    if mag>amax: abr=[x*amax/mag for x in abr]
    drift_rate=[rng.uniform(-0.025,0.025),rng.uniform(-0.025,0.025)]
    release=trust_end

    # Constant-altitude braking phase. Feedforward acts until brake end;
    # bounded inertial velocity damping is allowed only within trust window.
    for k in range(int(trust_end/DT)+1):
        t=k*DT
        ev=[v[0]+drift_rate[0]*t+rng.gauss(0,0.002),
            v[1]+drift_rate[1]*t+rng.gauss(0,0.002)]
        ff=abr if t<=duration else [0.0,0.0]
        desired=[ff[0]-gain*ev[0],ff[1]-gain*ev[1]]
        dm=math.hypot(*desired)
        if dm>cmdmax: desired=[x*cmdmax/dm for x in desired]
        da=[desired[0]-a[0],desired[1]-a[1]]
        dam=math.hypot(*da)
        if dam>jmax*DT: da=[x*jmax*DT/dam for x in da]
        a=[a[0]+da[0],a[1]+da[1]]
        disturbance=[rng.uniform(-0.01,0.01),rng.uniform(-0.01,0.01)]
        v=[v[0]+(a[0]+disturbance[0]-drag*v[0])*DT,
           v[1]+(a[1]+disturbance[1]-drag*v[1])*DT]
        p=[p[0]+v[0]*DT,p[1]+v[1]*DT]
        if t>=duration and (math.hypot(*ev)<=exit_speed or t>=trust_end):
            release=t
            break

    # Level blind descent: no XY position/velocity feedback and no XY
    # feedforward. The prior acceleration command is jerk-shaped back to zero.
    for _ in range(int(4.40/DT)):
        da=[-a[0],-a[1]]
        dam=math.hypot(*da)
        if dam>jmax*DT: da=[x*jmax*DT/dam for x in da]
        a=[a[0]+da[0],a[1]+da[1]]
        disturbance=[rng.uniform(-0.01,0.01),rng.uniform(-0.01,0.01)]
        v=[v[0]+(a[0]+disturbance[0]-drag*v[0])*DT,
           v[1]+(a[1]+disturbance[1]-drag*v[1])*DT]
        p=[p[0]+v[0]*DT,p[1]+v[1]*DT]
    return math.hypot(*p),math.hypot(*v),release


def main():
    life=(ROOT/'mission_lifecycle_manager_S2_2.m').read_text()
    cfg=(ROOT/'init_S2_2_config.m').read_text()
    required=[
        'horizontalBrakeComplete',
        'xyLossBrakeExitSpeed_mps',
        "'horizontalVelocityDampingEnabled',dampingEnabled",
        "'horizontalFeedforwardAccelEnabled',false",
        'xyLossBrakeReleaseSpeed_mps',
    ]
    missing=[x for x in required if x not in life and x not in cfg]
    assert not missing, f'missing source tokens: {missing}'
    assert "cfg.version='v0.5.3.1';" in cfg

    vals=[simulate(i) for i in range(10000)]
    drifts=sorted(v[0] for v in vals)
    speeds=sorted(v[1] for v in vals)
    releases=sorted(v[2] for v in vals)
    worst=max(drifts); p99=drifts[int(0.99*len(drifts))]
    worst_speed=max(speeds)
    assert worst<0.40, (worst,p99,worst_speed)
    assert worst_speed<0.08, worst_speed
    print(f'XY-loss trials: {len(vals)}')
    print(f'Worst predicted drift: {worst:.3f} m')
    print(f'99th percentile drift: {p99:.3f} m')
    print(f'Worst final XY speed: {worst_speed:.3f} m/s')
    print(f'Latest brake release: {max(releases):.3f} s')
    print('S2.2 v0.5.3.1 XY-LOSS MECHANISM REGRESSION: PASS')

if __name__=='__main__':
    main()
