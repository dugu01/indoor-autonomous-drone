#!/usr/bin/env python3
"""Cumulative source/package audit for Stage S2.2 v0.5.3.

This audit checks preservation, routing, source invariants, field completeness,
portable paths, Python mechanism evidence and MATLAB source sanity. It does not
claim MATLAB runtime validation.
"""
from __future__ import annotations

import hashlib
import re
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT=Path(__file__).resolve().parent
EXPECTED_CORE='9ac6336160b3ee68f032b6610fa6684a83854109ecddd3a854bd221a1353a483'
EXPECTED_ESKF='b4cb3f8a18b3936520cf91fd0215c3f0e28c230294b6a9c367319aaf2b742d25'
checks=[]

def add(name,ok,detail=''):
    checks.append((name,bool(ok),detail))

def text(name):
    return (ROOT/name).read_text(encoding='utf-8')

def norm_core(src):
    marker='activeObstacles=scenario.knownObstacles;'
    if marker not in src:return ''
    return src[src.index(marker):].replace('\r\n','\n').strip()+'\n'

def funcs(src):
    return set(re.findall(r'(?m)^\s*function\s+(?:(?:\[[^\]]*\]|[A-Za-z]\w*)\s*=\s*)?([A-Za-z]\w*)\s*\(',src))

def run(name,*args):
    r=subprocess.run([sys.executable,str(ROOT/name),*args],cwd=ROOT,text=True,capture_output=True)
    tail=(r.stdout+r.stderr).strip().splitlines()
    return r.returncode==0,(tail[-1] if tail else '')

required=[
 'run_S2_2_mission_replanning.m','validate_S2_2.m','validate_S2_2_multiseed_robustness.m',
 'validate_S2_2_v0_5_3_focus.m',
 'mission_manager_S2_2.m','mission_manager_v0_4_core_S2_2.m','mission_manager_v0_5_3_core_S2_2.m',
 'mission_lifecycle_manager_S2_2.m','multi_lane_eskf_S2_2.m','multi_lane_eskf_robust_S2_2.m',
 'multi_lane_eskf_lifecycle_S2_2.m','geometric_controller_S2_2.m','init_S2_2_config.m',
 'scenario_S2_2.m','dstar_lite_S2_2.m','astar_grid_S2_2.m','generate_min_snap_trajectory_S2_2.m',
 'plot_S2_2_dashboard.m','animate_S2_2_flight.m','README.md','LITERATURE_S2_2.md',
 'CHANGELOG_S2_1_TO_S2_2.md','MULTISEED_ROBUSTNESS_REPORT_S2_2_V0_5_3.md',
 's2_2_v0_5_3_multiseed_regression.py','MULTISEED_MECHANISM_REGRESSION_S2_2_V0_5_3.txt']
missing=[x for x in required if not (ROOT/x).is_file()]
add('required files',not missing,'complete' if not missing else ', '.join(missing))

cfg=text('init_S2_2_config.m');runm=text('run_S2_2_mission_replanning.m')
dispatch=text('mission_manager_S2_2.m');core=text('mission_manager_v0_4_core_S2_2.m')
robust=text('mission_manager_v0_5_3_core_S2_2.m');life=text('mission_lifecycle_manager_S2_2.m')
eskf=text('multi_lane_eskf_robust_S2_2.m');geom=text('geometric_controller_S2_2.m')
add('public interface',bool(re.search(r'^function\s+results\s*=\s*run_S2_2_mission_replanning\s*\(seed,scenarioName,makePlots,makeAnimation\)',runm,re.M)))
add('v0.5.3 result namespace',"cfg.version='v0.5.3';" in cfg and "'S2_2_mission_replanning',versionFolder" in runm)
add('frozen v0.4 core preserved',hashlib.sha256(norm_core(core).encode()).hexdigest()==EXPECTED_CORE)
add('frozen v0.4 estimator preserved',hashlib.sha256((ROOT/'multi_lane_eskf_S2_2.m').read_bytes()).hexdigest()==EXPECTED_ESKF)
add('robust derivative routed','mission_manager_v0_5_3_core_S2_2' in dispatch and 'mission_manager_v0_4_core_S2_2' not in dispatch.split('else',1)[-1])
add('robust estimator isolated',"multi_lane_eskf_robust_S2_2('init'" in robust and "multi_lane_eskf_robust_S2_2('step'" in robust)
add('REJOIN liveness watchdog',all(x in robust for x in ['rejoinProgressTimeout_s','progressRecoveryCount','GRID_FALLBACK']))
add('route-before-land fallback',all(x in life for x in ['pendingFallbackPath','gridFallbackRetryLimit','gridFallbackActive']) and 'gridFallbackActive' in robust)
add('fault-aware lane transition',all(x in eskf for x in ['imu_attribution_score','outputBlendTimeFault_s','faultAware','blendDuration_s']))
add('position-loss reliable-velocity capture',all(x in life for x in ['lastReliableXYVelocity','make_xy_loss_brake','xyLossBrakeEndTime']))
add('blind descent ignores drifting velocity',"horizontalVelocityDampingEnabled',false" in life and 'horizontalFeedforwardAccelEnabled' in life)
add('no stale position feedback',"'horizontalControlEnabled',false" in life and 'ep(1:2)=0' in geom)
add('explicit feedforward brake',all(x in geom for x in ['horizontalFeedforwardAccelEnabled','feedforwardEnabled']))
add('multi-seed validator matrix',all(x in text('validate_S2_2_multiseed_robustness.m') for x in [
 'dynamic_crossing_6dof','dynamic_blocker_becomes_static_6dof','primary_imu_fault_vio_outage',
 'rtl_obstacle_replan','alternate_landing_zone','xy_loss_emergency_land']))
focus_validator=text('validate_S2_2_v0_5_3_focus.m')
failed_case_tokens=[
 "'dynamic_crossing_6dof',                 1",
 "'dynamic_crossing_6dof',                 7",
 "'primary_imu_fault_vio_outage',          1",
 "'primary_imu_fault_vio_outage',          2",
 "'primary_imu_fault_vio_outage',          6",
 "'primary_imu_fault_vio_outage',          9",
 "'rtl_obstacle_replan',                   2",
 "'xy_loss_emergency_land',                2",
 "'xy_loss_emergency_land',                3",
 "'xy_loss_emergency_land',                6",
 "'xy_loss_emergency_land',                7",
 "'xy_loss_emergency_land',                9"]
add('focused former-failure matrix',all(x in focus_validator for x in failed_case_tokens))

# MATLAB file/function and call resolution.
files=sorted(ROOT.glob('*.m')); file_funcs={p.stem for p in files}; local={p:funcs(p.read_text()) for p in files}
mismatch=[p.name for p in files if p.stem not in local[p]]
add('MATLAB file/function consistency',not mismatch,f'{len(files)} files' if not mismatch else ', '.join(mismatch))
unresolved=[]
for p in files:
    called=set(re.findall(r'\b([A-Za-z]\w*_S2_2)\s*\(',p.read_text()))
    unresolved.extend(f'{p.name}:{f}' for f in sorted(called-(file_funcs|local[p])))
add('internal calls resolve',not unresolved,'all resolve' if not unresolved else '; '.join(unresolved))

# Field completeness.
cfg_defs=set(re.findall(r'cfg\.([A-Za-z]\w*)\s*=',cfg))|{'seed','initialPosition','inflationRadius'}
scenario=text('scenario_S2_2.m'); sc_defs=set(re.findall(r'scenario\.([A-Za-z]\w*)\s*=',scenario))
cfg_use=set();sc_use=set()
for p in files:
    src=p.read_text();cfg_use.update(re.findall(r'cfg\.([A-Za-z]\w*)',src));sc_use.update(re.findall(r'scenario\.([A-Za-z]\w*)',src))
add('configuration fields complete',not (cfg_use-cfg_defs),', '.join(sorted(cfg_use-cfg_defs)))
add('scenario fields complete',not (sc_use-sc_defs),', '.join(sorted(sc_use-sc_defs)))

# Hygiene.
hits=[]
python_files=[q for q in ROOT.glob('*.py') if not q.name.startswith('audit_')]
for p in files+python_files:
    src=p.read_text(errors='replace')
    if re.search(r'/Users/|Documents/MATLAB|[A-Za-z]:\\\\',src):hits.append(p.name)
add('portable source paths',not hits,', '.join(hits))
bad=[]
for p in files:
    src=p.read_text(errors='replace')
    if '\x00' in src or re.search(r'(?m)^(<<<<<<<|=======|>>>>>>>)',src):bad.append(p.name)
add('source integrity',not bad,', '.join(bad))

# Executed evidence.
with tempfile.TemporaryDirectory(prefix='s2_2_v053_') as tmp:
    life_ok,life_tail=run('s2_2_v0_5_lifecycle_backtest.py','--output',tmp,'--seeds','10')
focus_ok,focus_tail=run('s2_2_v0_5_3_multiseed_regression.py')
v04a,v04a_tail=run('patch_regression_s2_2_v0_4.py')
v04b,v04b_tail=run('patch2_regression_s2_2_v0_4.py')
sanity,sanity_tail=run('matlab_source_sanity.py')
add('Python lifecycle/grid matrix',life_ok,life_tail)
add('focused v0.5.3 mechanisms',focus_ok,focus_tail)
add('preserved v0.4 mechanism tests',v04a and v04b,f'{v04a_tail}; {v04b_tail}')
add('MATLAB source sanity',sanity,sanity_tail)

print('Stage S2.2 v0.5.3 cumulative audit')
print('='*64)
for name,ok,detail in checks:
    print(f"[{'PASS' if ok else 'FAIL'}] {name}"+(f' — {detail}' if detail else ''))
passed=sum(x[1] for x in checks)
print('-'*64)
print(f'Result: {passed}/{len(checks)} checks passed')
if passed!=len(checks):sys.exit(1)
print('CUMULATIVE S2.2 v0.5.3 AUDIT: PASS')
