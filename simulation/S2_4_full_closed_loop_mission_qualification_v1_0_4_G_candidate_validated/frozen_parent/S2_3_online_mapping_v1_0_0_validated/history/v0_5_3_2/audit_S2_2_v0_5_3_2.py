#!/usr/bin/env python3
from pathlib import Path
import hashlib, subprocess, sys, re
ROOT=Path(__file__).resolve().parent
checks=[]
def add(name,ok,detail=''):
    checks.append((name,bool(ok),detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f": {detail}" if detail else ''))

def sha(name):
    return hashlib.sha256((ROOT/name).read_bytes()).hexdigest()

def norm_core(src):
    marker='activeObstacles=scenario.knownObstacles;'
    if marker not in src:
        return ''
    return src[src.index(marker):].replace('\r\n','\n').strip()+'\n'

required=['run_S2_2_mission_replanning.m','mission_manager_S2_2.m',
'mission_manager_v0_4_core_S2_2.m','mission_manager_v0_5_3_core_S2_2.m',
'mission_lifecycle_manager_S2_2.m','multi_lane_eskf_S2_2.m',
'multi_lane_eskf_lifecycle_S2_2.m','multi_lane_eskf_robust_S2_2.m',
'geometric_controller_S2_2.m','validate_S2_2.m',
'validate_S2_2_v0_5_3_focus.m','validate_S2_2_multiseed_robustness.m',
's2_2_v0_5_3_2_xy_loss_regression.py']
add('required files',all((ROOT/x).exists() for x in required))

cfg=(ROOT/'init_S2_2_config.m').read_text()
life=(ROOT/'mission_lifecycle_manager_S2_2.m').read_text()
ctrl=(ROOT/'geometric_controller_S2_2.m').read_text()
add('v0.5.3.2 version',"cfg.version='v0.5.3.2';" in cfg)
add('frozen braking impulse', 'aBrake=-vReliable/duration;' in life)
add('post-loss damping disabled', 'dampingEnabled=false;' in life)
add('jerk settle used', 'cfg.xyLossBrakeSettleTime_s' in life and 'cfg.xyLossBrakeSettleTime_s=0.16;' in cfg)
add('blind speed-guard bypass', ctrl.count('if ~horizontalBlindMode')>=2)
add('no direct truth in emergency reference', "ref3=struct('p',[truth" not in life)

# Exact frozen v0.4 artifacts from the validated package.
core=(ROOT/'mission_manager_v0_4_core_S2_2.m').read_text()
add('v0.4 core hash preserved',hashlib.sha256(norm_core(core).encode()).hexdigest()=='9ac6336160b3ee68f032b6610fa6684a83854109ecddd3a854bd221a1353a483')
add('v0.4 ESKF hash preserved',sha('multi_lane_eskf_S2_2.m')=='b4cb3f8a18b3936520cf91fd0215c3f0e28c230294b6a9c367319aaf2b742d25')

p=subprocess.run([sys.executable,'matlab_source_sanity.py'],cwd=ROOT,text=True,capture_output=True)
add('MATLAB source sanity',p.returncode==0,(p.stdout+p.stderr).strip().splitlines()[-1])
p=subprocess.run([sys.executable,'s2_2_v0_5_3_2_xy_loss_regression.py'],cwd=ROOT,text=True,capture_output=True)
add('focused XY-loss mechanism regression',p.returncode==0,(p.stdout+p.stderr).strip().splitlines()[-1])

passed=sum(x[1] for x in checks)
print(f'Result: {passed}/{len(checks)} checks passed')
if passed!=len(checks): raise SystemExit(1)
print('CUMULATIVE S2.2 v0.5.3.2 AUDIT: PASS')
