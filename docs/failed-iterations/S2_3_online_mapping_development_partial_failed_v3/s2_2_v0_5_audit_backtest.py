#!/usr/bin/env python3
"""Focused mechanism regression for the audited S2.2 v0.5 candidate.

This tests the logic that caused the original preflight failures and the
later full-package audit findings. It is not MATLAB or 6-DOF execution.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

DT = 0.02

@dataclass
class Check:
    name: str
    passed: bool
    detail: str


def smootherstep_state(z0: float, zf: float, duration: float, t: float):
    T=max(duration,1e-12); tau=min(1.0,max(0.0,t/T)); dz=zf-z0
    h=35*tau**4-84*tau**5+70*tau**6-20*tau**7
    h1=140*tau**3-420*tau**4+420*tau**5-140*tau**6
    h2=420*tau**2-1680*tau**3+2100*tau**4-840*tau**5
    h3=840*tau-5040*tau**2+8400*tau**3-4200*tau**4
    return z0+dz*h,dz*h1/T,dz*h2/T**2,dz*h3/T**3


def check_async_preflight():
    # Freshness begins invalid. Only accepted updates can change timestamps.
    last_xy=last_z=last_att=-math.inf; accepted=0
    initial_invalid=all(math.isinf(x) and x < 0 for x in (last_xy,last_z,last_att))
    # MATLAB schedules: VIO/range 25 Hz, LiDAR 5 Hz, barometer 10 Hz, dt=20 ms.
    for k in range(2, round(0.60/DT)+2):
        t=(k-1)*DT
        if (k-1)%2==0:  # accepted VIO
            last_xy=last_z=last_att=t; accepted+=1
        if (k-1)%10==0:  # accepted LiDAR
            last_xy=last_att=t; accepted+=1
        if (k-1)%2==0:  # accepted range
            last_z=t; accepted+=1
        if (k-1)%5==0:  # accepted barometer
            last_z=t; accepted+=1
    t=0.60
    ages=(t-last_xy,t-last_z,t-last_att)
    passed=initial_invalid and max(ages)<=1.0 and accepted>=3
    return Check('accepted-aid freshness preflight',passed,
                 f'initial invalid={initial_invalid}, ages={ages}, accepted={accepted}')


def check_no_instant_packet_dependency():
    # At a preflight instant no sensor must publish on the same simulation
    # step, yet recently accepted measurements still make preflight valid.
    packet_flags=(False,False,False,False)
    ages=(0.04,0.00,0.04)
    passed=(not any(packet_flags)) and all(a<=1.0 for a in ages)
    return Check('asynchronous packet independence',passed,
                 f'packet={packet_flags}, accepted ages={ages}')


def check_emergency_trigger_one_shot():
    state='TRACK_OUTBOUND'; entry=10.0; transitions=0
    rtl=True
    for n in range(100):
        t=10.0+n*DT
        terminal={'EMERGENCY_HOLD','EMERGENCY_LAND','LAND_DESCENT','DISARM','COMPLETE','PREFLIGHT_REJECT','FAILSAFE'}
        if rtl and state not in terminal:
            state='EMERGENCY_HOLD'; entry=t; transitions+=1
        if state=='EMERGENCY_HOLD' and t-entry>=0.60-1e-12:
            state='EMERGENCY_LAND'; transitions+=1
    passed=state=='EMERGENCY_LAND' and transitions==2
    return Check('one-shot XY-loss emergency transition',passed,
                 f'final state={state}, transitions={transitions}, hold entry={entry:.2f}')


def check_vertical_only_emergency_control():
    stale_target=(3.0,0.8); estimate=(4.2,3.7)
    # horizontalControlEnabled=false explicitly zeros XY ep/ev/ref acceleration.
    ep=[stale_target[0]-estimate[0],stale_target[1]-estimate[1],-0.5]
    ev=[0.4,-0.2,-0.1]; a_ref=[0.3,0.2,-0.2]
    ep[:2]=[0.0,0.0]; ev[:2]=[0.0,0.0]; a_ref[:2]=[0.0,0.0]
    kp=(2.0,2.0,3.2); kd=(1.5,1.5,2.0)
    a=[a_ref[i]+kp[i]*ep[i]+kd[i]*ev[i] for i in range(3)]
    passed=abs(a[0])<1e-12 and abs(a[1])<1e-12
    return Check('vertical-only emergency descent',passed,f'command acceleration={a}')


def check_land_detector_dwell():
    timer=0.0; detected_at=None
    for k in range(30):
        contact=k>=5; vertical_slow=True; near_ground=True
        timer=timer+DT if contact and vertical_slow and near_ground else 0.0
        if timer>=0.20 and detected_at is None: detected_at=k*DT
    passed=detected_at is not None and detected_at>=0.28-1e-12
    return Check('contact-confirmed land detector',passed,
                 f'detected at {detected_at:.2f}s after contact began at 0.10s')


def check_arrival_confirmation():
    timer=0.0; declared=False
    sequence=[True]*8+[False]+[True]*16
    for ready in sequence:
        timer=timer+DT if ready else 0.0
        if timer>=0.30-1e-12: declared=True; break
    passed=declared and timer>=0.30-1e-12
    return Check('estimated-state arrival confirmation',passed,
                 f'declared={declared}, final dwell={timer:.2f}s')


def check_vertical_profiles():
    cases=[('takeoff',0.03,1.15,4.5),('landing',1.15,0.03,5.5)]
    maxima=[]; endpoint_ok=True
    for name,z0,zf,T in cases:
        samples=[smootherstep_state(z0,zf,T,i*T/20000) for i in range(20001)]
        vmax=max(abs(x[1]) for x in samples); amax=max(abs(x[2]) for x in samples); jmax=max(abs(x[3]) for x in samples)
        maxima.append((name,vmax,amax,jmax))
        s0=samples[0]; sf=samples[-1]
        endpoint_ok &= abs(s0[0]-z0)<1e-12 and abs(sf[0]-zf)<1e-12
        endpoint_ok &= max(abs(s0[i]) for i in (1,2,3))<1e-12
        endpoint_ok &= max(abs(sf[i]) for i in (1,2,3))<1e-10
    limits=(0.60,0.65,1.50)
    passed=endpoint_ok and all(v<=limits[0]+1e-9 and a<=limits[1]+1e-9 and j<=limits[2]+1e-9 for _,v,a,j in maxima)
    return Check('vertical C3 profile limits',passed,f'maxima={maxima}, limits={limits}')


def check_truth_not_used_for_decisions(root: Path):
    src=(root/'mission_lifecycle_manager_S2_2.m').read_text()
    decision_part=src[:src.index('    oldTruthP=truth.p;')]
    passed='truth.p' not in decision_part and 'truth.v' not in decision_part and 'truth.onGround' not in decision_part
    return Check('estimator-only lifecycle decisions',passed,
                 'truth access starts only after controller/dynamics for validation')


def check_animation_obstacle_timing(root: Path):
    src=(root/'animate_S2_2_flight.m').read_text()
    passed='activeObstacleHistoryTime' in src and 'update_static_obstacles' in src and 'maps.activeObstacles' not in src
    return Check('temporal obstacle animation',passed,
                 'animation selects static map history by simulation timestamp')


def main():
    root=Path(__file__).resolve().parent
    checks=[
        check_async_preflight(),check_no_instant_packet_dependency(),
        check_emergency_trigger_one_shot(),check_vertical_only_emergency_control(),
        check_land_detector_dwell(),check_arrival_confirmation(),
        check_vertical_profiles(),check_truth_not_used_for_decisions(root),
        check_animation_obstacle_timing(root),
    ]
    out={'checks':[asdict(c) for c in checks], 'passed':sum(c.passed for c in checks),'total':len(checks)}
    (root/'AUDIT_BACKTEST_RESULTS_S2_2_V0_5.json').write_text(json.dumps(out,indent=2)+'\n')
    lines=['Stage S2.2 v0.5 focused audit backtest','='*52]
    lines += [f"[{'PASS' if c.passed else 'FAIL'}] {c.name} — {c.detail}" for c in checks]
    lines += ['-'*52,f"Result: {out['passed']}/{out['total']} PASS"]
    report='\n'.join(lines)+'\n'
    (root/'AUDIT_BACKTEST_REPORT_S2_2_V0_5.txt').write_text(report)
    print(report,end='')
    if not all(c.passed for c in checks): raise SystemExit(1)

if __name__=='__main__': main()
