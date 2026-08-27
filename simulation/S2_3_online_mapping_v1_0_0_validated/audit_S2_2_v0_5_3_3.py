#!/usr/bin/env python3
from pathlib import Path
import hashlib, subprocess, sys, re
ROOT=Path(__file__).resolve().parent
checks=[]
def add(n,c): checks.append((n,bool(c)))
def sha(p): return hashlib.sha256((ROOT/p).read_bytes()).hexdigest()
required=['init_S2_2_config.m','mission_lifecycle_manager_S2_2.m','land_detector_S2_2.m',
          'geometric_controller_S2_2.m','quadrotor_dynamics_S2_2.m','run_S2_2_mission_replanning.m',
          'validate_S2_2.m','validate_S2_2_v0_5_3_focus.m','validate_S2_2_multiseed_robustness.m',
          'SEED7_EXACT_TRACE_REPLAY_V0_5_3_3.json','s2_2_v0_5_3_3_touchdown_replay_regression.py']
for x in required:add('required '+x,(ROOT/x).is_file())
cfg=(ROOT/'init_S2_2_config.m').read_text(); life=(ROOT/'mission_lifecycle_manager_S2_2.m').read_text(); land=(ROOT/'land_detector_S2_2.m').read_text()
add('v0.5.3.3 namespace',"cfg.version='v0.5.3.3';" in cfg)
add('two landing gates',life.count('landingDetectionArmed=vz.complete;')==2)
add('per-step gate reset','landingDetectionArmed=false;' in life)
add('detector receives previous latch','landContactTimer,landDetected,landingDetectionArmed' in life)
add('qualified touchdown latch',all(x in land for x in ['previousDetected','qualifiedTouchdown','landingDetectionArmed']))
code='\n'.join(x for x in land.splitlines() if not x.lstrip().startswith('%'))
add('detector has no truth dependency','truth' not in code)
# Frozen baseline fingerprints from validated package.
add('v0.4 core hash',sha('mission_manager_v0_4_core_S2_2.m')=='32b58eea2ed291a71fb81b9ae60c1044e0b024fcd828e57e384598d5b66b54ce')
add('v0.4 ESKF hash',sha('multi_lane_eskf_S2_2.m')=='b4cb3f8a18b3936520cf91fd0215c3f0e28c230294b6a9c367319aaf2b742d25')
for n,c in checks: print(f"[{'PASS' if c else 'FAIL'}] {n}")
if not all(c for _,c in checks): raise SystemExit(1)
for script in ['matlab_source_sanity.py','s2_2_v0_5_3_3_touchdown_replay_regression.py']:
 p=subprocess.run([sys.executable,script],cwd=ROOT,text=True,capture_output=True)
 print(p.stdout,end='')
 if p.returncode: print(p.stderr); raise SystemExit(p.returncode)
print(f'Result: {len(checks)}/{len(checks)} static checks passed')
print('CUMULATIVE S2.2 v0.5.3.3 AUDIT: PASS')
