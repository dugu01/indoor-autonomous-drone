#!/usr/bin/env python3
"""Source-faithful yaw-reference backtest for G v1.0.4.

This is not a MATLAB/ESKF substitute. It tests the exact discovered yaw-command
seam with the inherited yaw-axis controller/plant parameters and checks that the
runtime source implements bounded target slewing while preserving the 2 deg gate.
"""
from __future__ import annotations
import math,re,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
M=(ROOT/'coupled'/'mission'/'mission_lifecycle_manager_S2_4.m').read_text()
C=(ROOT/'frozen_parent'/'S2_3_online_mapping_v1_0_0_validated'/'init_S2_2_config.m').read_text()
S3=(ROOT/'frozen_parent'/'S2_3_online_mapping_v1_0_0_validated'/'init_S2_3_config.m').read_text()
D=(ROOT/'frozen_parent'/'S2_3_online_mapping_v1_0_0_validated'/'RUNTIME_DIAGNOSIS_UNKNOWN_ROOM_NOMINAL_SEED0_SECOND_RUN.md').read_text()
errors=[]
def check(name,cond,detail=''):
    print(f'{name:48s} {"PASS" if cond else "FAIL"} {detail}')
    if not cond: errors.append(name)

# Source contract: targets are not injected as angle steps.
check('target state initialized','yawTargetCommand=yawCommand' in M)
check('planned viewpoint updates target only','yawTargetCommand=plannedRequest.yaw' in M and 'yawCommand=plannedRequest.yaw' not in M)
check('reauthorized viewpoint updates target only','yawTargetCommand=recovery.request.yaw' in M and 'yawCommand=recovery.request.yaw' not in M)
check('TRACK_OUTBOUND applies yaw slew',"if strcmp(state,'TRACK_OUTBOUND')" in M and 'slew_yaw_reference_local' in M)
check('slew reuses inherited scan-rate bound','cfg.mapScanYawRate_radps,cfg.dt' in M)
check('revocation freezes stale viewing yaw',M.count('yawCommand=estimated_yaw_S2_3(est);yawTargetCommand=yawCommand')>=3)
check('scan yaw remains inherited continuous law',M.count('scanStartYaw+cfg.mapScanYawRate_radps*(t-stateEntryTime)')>=2)
check('estimator attitude threshold unchanged','cfg.maxEstimatorAttitudeError_deg=2.0;' in C)
check('inherited qualified scan rate still 35 deg/s','cfg.mapScanYawRate_radps=deg2rad(35);' in S3)
check('historical 55->35 scan diagnosis retained','55 deg/s' in D and '35 deg/s' in D)

# Parse exact yaw-axis controller/plant values from inherited config.
def scalar(pattern):
    m=re.search(pattern,C)
    if not m: raise RuntimeError(pattern)
    return float(m.group(1))
Jz=scalar(r'cfg\.inertia_kgm2=diag\(\[0\.030 0\.030 ([0-9.]+)\]\)')
Kp=scalar(r'cfg\.attitudeKp=\[5\.0;5\.0;([0-9.]+)\]')
Kd=scalar(r'cfg\.rateKd=\[0\.55;0\.55;([0-9.]+)\]')
drag=scalar(r'cfg\.angularDrag=([0-9.]+);')
Mz=scalar(r'cfg\.maxMoment_Nm=\[0\.90;0\.90;([0-9.]+)\]')
dt=scalar(r'cfg\.dt=([0-9.]+);')
rate=math.radians(35.0)

def wrap(x): return (x+math.pi)%(2*math.pi)-math.pi

def yaw_axis(target_deg,slew_rate=None,T=8.0):
    psi=0.0;omega=0.0;cmd=0.0;peak=0.0
    target=math.radians(target_deg)
    for _ in range(round(T/dt)):
        if slew_rate is None:
            cmd=target
        else:
            e=wrap(target-cmd);step=max(-slew_rate*dt,min(slew_rate*dt,e));cmd=wrap(cmd+step)
        eR=math.sin(wrap(psi-cmd))
        moment=max(-Mz,min(Mz,-Kp*eR-Kd*omega))
        odot=(moment-drag*omega)/Jz
        omid=omega+0.5*odot*dt
        psi=wrap(psi+omid*dt);omega += odot*dt
        peak=max(peak,abs(omega))
    return math.degrees(peak)

step90=yaw_axis(90,None)
slew90=yaw_axis(90,rate)
slew120=yaw_axis(120,rate)
check('90-deg angle step reproduces aggressive yaw',step90>180,f'peak={step90:.2f} deg/s')
check('35-deg/s slew bounds 90-deg maneuver',slew90<60,f'peak={slew90:.2f} deg/s')
check('35-deg/s slew bounds 120-deg maneuver',slew120<60,f'peak={slew120:.2f} deg/s')
check('slew reduces yaw-rate peak by >70%',slew90/step90<0.30,f'ratio={slew90/step90:.3f}')

# Actual user-run peak diagnostic (evidence values, not simulated claims).
base_err=1.815676;fault_err=2.140175;fault_yaw=2.140151
base_omega=177.691;fault_omega=204.865
check('user failure is yaw-dominated',fault_yaw/fault_err>0.9999,f'{100*fault_yaw/fault_err:.4f}% yaw')
check('user fault peak exceeds inherited yaw policy',fault_omega>5*35,f'{fault_omega:.3f} deg/s vs 35 deg/s')
check('baseline already has narrow estimator margin',2.0-base_err<0.20,f'margin={2.0-base_err:.6f} deg')
check('fault overshoot is modest but real',0<fault_err-2.0<0.20,f'overshoot={fault_err-2.0:.6f} deg')

if errors:
    print('\nS2.4-G YAW-SLEW SOURCE/FIRST-PRINCIPLES BACKTEST: FAIL')
    for e in errors: print(' -',e)
    sys.exit(1)
print('\nS2.4-G YAW-SLEW SOURCE/FIRST-PRINCIPLES BACKTEST: PASS')
print('NOTE: MATLAB coupled estimator qualification remains required; this test does not emulate the ESKF.')
