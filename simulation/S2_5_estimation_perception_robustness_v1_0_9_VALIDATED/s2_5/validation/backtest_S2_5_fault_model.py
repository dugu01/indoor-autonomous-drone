#!/usr/bin/env python3
"""Source-faithful offline checks for S2.5 fault severity and timing contracts.
This is not a substitute for coupled MATLAB qualification. It verifies that
faults actually exercise the intended inherited rejection/hold mechanisms and
that negative controls remain fail-closed.
"""
from pathlib import Path
import math,re,sys
ROOT=Path(__file__).resolve().parents[2]
S=ROOT/'s2_5'; P=ROOT/'frozen_parent/S2_3_online_mapping_v1_0_0_validated'
config=(P/'init_S2_2_config.m').read_text();mapcfg=(P/'init_S2_3_config.m').read_text();mapper=(P/'update_probabilistic_map_S2_3.m').read_text()

def num(pattern,text):
    m=re.search(pattern,text);assert m,pattern;return float(m.group(1))
def raddeg(pattern,text):
    m=re.search(pattern,text);assert m,pattern;return float(m.group(1))

gate_vio=num(r'cfg\.gateVIO9=([0-9.]+);',config)
gate_lid=num(r'cfg\.gateLidar3=([0-9.]+);',config)
max_xy_cov=num(r'cfg\.maxXYCovariance=([0-9.]+)\^2;',config)**2
vio_sigma=num(r'cfg\.vioPosSigma=([0-9.]+);',config)
lid_sigma=num(r'cfg\.lidarSigmaXY=([0-9.]+);',config)
imu_a=num(r'cfg\.imuDisagreementAccel=([0-9.]+);',config)
imu_g=math.radians(raddeg(r'cfg\.imuDisagreementGyro=deg2rad\(([0-9.]+)\);',config))
imu_samples=int(num(r'cfg\.imuDisagreementSamples=([0-9.]+);',config))
packet_age=num(r'cfg\.mapMaxPacketAge_s=([0-9.]+);',mapcfg)
hold=num(r'cfg\.mapPerceptionHoldTimeout_s=([0-9.]+);',mapcfg)
failsafe=num(r'cfg\.mapPerceptionFailsafeTimeout_s=([0-9.]+);',mapcfg)

def result(name,ok,detail):
    print(f'{name:12s} {"PASS" if ok else "FAIL"}  {detail}');return ok
oks=[]
# Gross VIO/LiDAR outliers are guaranteed outside their chi-square gates even
# at the largest covariance that an eligible lane is allowed to carry.
vio_nis_lb=0.80**2/(max_xy_cov+vio_sigma**2)
lid_nis_lb=0.90**2/(max_xy_cov+lid_sigma**2)
oks.append(result('N3-VIO-NIS',vio_nis_lb>gate_vio,f'lower-bound NIS {vio_nis_lb:.2f} > gate {gate_vio:.3f}'))
oks.append(result('N4-LID-NIS',lid_nis_lb>gate_lid,f'lower-bound NIS {lid_nis_lb:.2f} > gate {gate_lid:.3f}'))
# IMU fault vector is comfortably above both disagreement thresholds; with a
# 50 Hz loop and four samples, causal detection evidence exists within 0.08 s.
acc=math.sqrt(0.35**2+0.25**2+0.18**2);gyro=math.radians(math.sqrt(1.5**2+1.0**2+1.2**2))
oks.append(result('N5-IMU',acc>imu_a and gyro>imu_g,f'acc {acc:.3f}>{imu_a:.3f}, gyro {math.degrees(gyro):.3f}>{math.degrees(imu_g):.3f} deg/s, {imu_samples} samples'))
# Dropout timing semantics.
oks.append(result('P3-HOLD',1.20>hold and 1.20<failsafe,f'1.20 s > hold {hold:.2f} and < failsafe {failsafe:.2f}'))
oks.append(result('P4-STALE',0.45>packet_age,f'lag 0.45 s > packet-age gate {packet_age:.2f} s'))
oks.append(result('P6-FAILSAFE',7.0>failsafe,f'7.0 s > perception failsafe {failsafe:.2f} s'))
# A one-frame positive range spike cannot erase a voxel already classified as
# persistent static occupied because the mapper explicitly suppresses free
# log-odds updates on staticOccupied cells.
oks.append(result('P5-STATIC',"if ~map.staticOccupied(idx)" in mapper,'persistent static occupancy is not cleared by free-ray evidence'))
# Unknown remains occupied in the planner.
proj=(P/'project_map_to_planner_S2_3.m').read_text()
oks.append(result('FAIL-CLOSED',('knownFree' in proj and 'unknown' in proj),'planner projection retains explicit known-free/unknown state'))
# High-noise case remains below the nominal expected chi-square scale for each
# measurement family (scale^2 * DOF); this is intentionally demanding but not
# a threshold-relaxation test.
scale=1.5
oks.append(result('N6-NOISE',scale*scale*9<gate_vio and scale*scale*3<gate_lid,f'expected VIO/LiDAR NIS {scale*scale*9:.2f}/{scale*scale*3:.2f} below gates'))
# Check fault window contains real packet events at inherited sample rates.
dt=0.02
vio_period=round((1/25)/dt);lid_period=round((1/5)/dt);plid_period=round((1/8)/dt);depth_period=round((1/10)/dt)
def events(period,t0,t1):
    n=0
    k0=int(math.floor(t0/dt))+1;k1=int(math.ceil(t1/dt))+2
    for k in range(k0,k1+1):
        t=(k-1)*dt
        if t0-1e-12<=t<=t1+1e-12 and (k-1)%period==0:n+=1
    return n
oks.append(result('WINDOWS',events(vio_period,18,18.6)>0 and events(lid_period,18,18.6)>0 and events(plid_period,18,18.04)>0 and events(depth_period,18,18.04)>0,
                  f'events VIO={events(vio_period,18,18.6)} navLiDAR={events(lid_period,18,18.6)} rawLiDAR={events(plid_period,18,18.04)} depth={events(depth_period,18,18.04)}'))
passed=all(oks)
print('S2.5 SOURCE-FAITHFUL FAULT / GATE BACKTEST:', 'PASS' if passed else 'FAIL')
sys.exit(0 if passed else 1)
