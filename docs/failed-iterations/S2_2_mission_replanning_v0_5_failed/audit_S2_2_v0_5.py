#!/usr/bin/env python3
"""Cumulative static audit for Stage S2.2 v0.5.

This validates package structure, interfaces, v0.4 core preservation,
preflight/failsafe/lifecycle source invariants, field consistency and Python
regression evidence. It does not execute MATLAB.
"""
from __future__ import annotations

import hashlib
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT=Path(__file__).resolve().parent
EXPECTED_V04_CORE_SHA256='9ac6336160b3ee68f032b6610fa6684a83854109ecddd3a854bd221a1353a483'
EXPECTED_V04_ESKF_SHA256='b4cb3f8a18b3936520cf91fd0215c3f0e28c230294b6a9c367319aaf2b742d25'
V04_CORE_MARKER='activeObstacles=scenario.knownObstacles;'

REQUIRED_M=[
 'run_S2_2_mission_replanning.m','validate_S2_2.m','validate_S2_2_monte_carlo.m',
 'mission_manager_S2_2.m','mission_manager_v0_4_core_S2_2.m','mission_lifecycle_manager_S2_2.m',
 'init_S2_2_config.m','scenario_S2_2.m','preflight_check_S2_2.m','land_detector_S2_2.m',
 'vertical_profile_S2_2.m','landing_zone_clear_S2_2.m','select_safe_landing_zone_S2_2.m',
 'init_quadrotor_state_S2_2.m','quadrotor_dynamics_S2_2.m','geometric_controller_S2_2.m',
 'multi_lane_eskf_S2_2.m','multi_lane_eskf_lifecycle_S2_2.m','simulate_sensor_packet_S2_2.m','dstar_lite_S2_2.m',
 'astar_grid_S2_2.m','build_occupancy_grid_S2_2.m','inflate_obstacles_S2_2.m',
 'segment_occupied_grid_S2_2.m','generate_min_snap_trajectory_S2_2.m',
 'sample_min_snap_trajectory_S2_2.m','sample_min_snap_state_S2_2.m',
 'eval_min_snap_segment_S2_2.m','track_smooth_trajectory_S2_2.m',
 'plot_S2_2_dashboard.m','animate_S2_2_flight.m']
REQUIRED_DOCS=[
 'README.md','LITERATURE_S2_2.md','CHANGELOG_S2_1_TO_S2_2.md','PACKAGE_MANIFEST.md',
 'SOURCE_BASELINE_S2_2_V0_5.md','MATLAB_VALIDATION_PROTOCOL_S2_2_V0_5.md',
 'PYTHON_BACKTEST_REPORT_S2_2_V0_5.md','PYTHON_BACKTEST_RESULTS_S2_2_V0_5.json',
 'AUDIT_BACKTEST_REPORT_S2_2_V0_5.txt','AUDIT_BACKTEST_RESULTS_S2_2_V0_5.json',
 'STATIC_AUDIT_S2_2_V0_5.md','matlab_source_sanity.py','V0_4_CONFIG_BASELINE_S2_2.json',
 'CONSOLIDATED_AUDIT_NOTES_S2_2_V0_5.md']
LEGACY=['nominal_6dof','incremental_static_estimated','dynamic_crossing_6dof',
'dynamic_blocker_becomes_static_6dof','obstacle_sensor_dropout_recover_6dof',
'primary_imu_fault_vio_outage','xy_aid_loss_failsafe']
LIFECYCLE=['full_mission_nominal','rtl_obstacle_replan','alternate_landing_zone',
'preflight_reject_unsafe_home','xy_loss_emergency_land']
STATES=['PREFLIGHT','ARM','TAKEOFF','INITIAL_HOVER','WAIT_FOR_GOAL','PLAN_OUTBOUND',
'TRACK_OUTBOUND','GOAL_HOVER','PLAN_RTL','TRACK_RTL','LAND_HOVER','LAND_DESCENT',
'DISARM','COMPLETE','PREFLIGHT_REJECT','EMERGENCY_HOLD','EMERGENCY_LAND',
'LIFECYCLE_REPLAN_BRAKE','FAILSAFE']
checks=[]
def add(name,ok,detail=''): checks.append((name,bool(ok),detail))
def text(name): return (ROOT/name).read_text(encoding='utf-8')
def function_names(src):
    return set(re.findall(r'(?m)^\s*function\s+(?:(?:\[[^\]]*\]|[A-Za-z]\w*)\s*=\s*)?([A-Za-z]\w*)\s*\(',src))

def strip_matlab_comments(src):
    out=[]
    for line in src.splitlines():
        acc=[]; quoted=False; i=0
        while i<len(line):
            c=line[i]
            if c=="'":
                if quoted and i+1<len(line) and line[i+1]=="'":
                    acc.extend(["'","'"]); i+=2; continue
                quoted=not quoted; acc.append(c); i+=1; continue
            if c=='%' and not quoted: break
            acc.append(c); i+=1
        out.append(''.join(acc))
    return '\n'.join(out)

def matlab_statements(src):
    result=[]; start=0; quoted=False; stack=[]; i=0
    pairs={')':'(',']':'[','}':'{'}
    while i<len(src):
        c=src[i]
        if c=="'":
            if quoted and i+1<len(src) and src[i+1]=="'": i+=2; continue
            quoted=not quoted; i+=1; continue
        if not quoted:
            if c in '([{': stack.append(c)
            elif c in ')]}':
                if stack and stack[-1]==pairs[c]: stack.pop()
            elif c==';' and not stack:
                result.append(src[start:i+1]); start=i+1
        i+=1
    return result

def config_assignments(src):
    out={}
    for statement in matlab_statements(strip_matlab_comments(src)):
        m=re.search(r'cfg\.([A-Za-z]\w*)\s*=\s*(.*)\s*;\s*$',statement,re.S)
        if m: out[m.group(1)]=' '.join(m.group(2).split())
    return out

missing=[f for f in REQUIRED_M+REQUIRED_DOCS if not (ROOT/f).is_file()]
add('required package files',not missing,f'{len(REQUIRED_M)} MATLAB files, {len(REQUIRED_DOCS)} reports' if not missing else 'missing: '+', '.join(missing))
run=text('run_S2_2_mission_replanning.m'); cfg=text('init_S2_2_config.m'); sc=text('scenario_S2_2.m')
val=text('validate_S2_2.m'); life=text('mission_lifecycle_manager_S2_2.m'); eskf=text('multi_lane_eskf_lifecycle_S2_2.m'); legacy_eskf=text('multi_lane_eskf_S2_2.m')
pre=text('preflight_check_S2_2.m'); geom=text('geometric_controller_S2_2.m')
anim=text('animate_S2_2_flight.m'); plot=text('plot_S2_2_dashboard.m')
add('four-argument public interface',bool(re.search(r'^function\s+results\s*=\s*run_S2_2_mission_replanning\s*\(\s*seed\s*,\s*scenarioName\s*,\s*makePlots\s*,\s*makeAnimation\s*\)',run,re.M)))
add('versioned result path',"cfg.version='v0.5';" in cfg and "'S2_2_mission_replanning',versionFolder" in run)
try:
    baseline=json.loads(text('V0_4_CONFIG_BASELINE_S2_2.json'))['assignments']
    current_cfg=config_assignments(cfg)
    cfg_diffs=[k for k,v in baseline.items() if current_cfg.get(k)!=v]
    add('validated v0.4 configuration constants retained',not cfg_diffs,
        f"{len(baseline)} legacy fields unchanged" if not cfg_diffs else 'changed: '+', '.join(cfg_diffs))
except Exception as e:
    add('validated v0.4 configuration constants retained',False,str(e))
miss=[s for s in LEGACY+LIFECYCLE if s not in sc or s not in val]
add('12-scenario matrix',not miss,'7 frozen regressions + 5 lifecycle' if not miss else 'missing '+', '.join(miss))
add('unsafe-home rejection is geometric',"scenario.knownObstacles=[scenario.knownObstacles;scenario.homeBlockRect];" in sc and "scenario.forcePreflightReject=false;" in sc)

core=text('mission_manager_v0_4_core_S2_2.m')
if V04_CORE_MARKER in core:
    norm=core[core.index(V04_CORE_MARKER):].replace('\r\n','\n').strip()+'\n'
    core_hash=hashlib.sha256(norm.encode()).hexdigest()
else: core_hash='marker-not-found'
add('exact validated v0.4 core preserved',core_hash==EXPECTED_V04_CORE_SHA256,core_hash)
add('dispatcher preserves legacy core','mission_lifecycle_manager_S2_2' in text('mission_manager_S2_2.m') and 'mission_manager_v0_4_core_S2_2' in text('mission_manager_S2_2.m'))
legacy_eskf_hash=hashlib.sha256((ROOT/'multi_lane_eskf_S2_2.m').read_bytes()).hexdigest()
add('exact validated v0.4 ESKF preserved',legacy_eskf_hash==EXPECTED_V04_ESKF_SHA256,legacy_eskf_hash)
add('estimator routing is isolated',
    "multi_lane_eskf_lifecycle_S2_2('init'" in life and "multi_lane_eskf_lifecycle_S2_2('step'" in life and
    "multi_lane_eskf_S2_2('init'" in core and "multi_lane_eskf_S2_2('step'" in core)
add('all lifecycle states present',all(s in life for s in STATES),f'{len(STATES)} states')

# Preflight and estimator freshness invariants.
init_section=eskf[eskf.index('function nav=init_nav'):eskf.index('function [nav,out]=step_nav')]
add('aid timestamps start invalid',"'lastHorizontalAidTime',-inf" in init_section and "'lastVerticalAidTime',-inf" in init_section and "'lastAttitudeAidTime',-inf" in init_section)
add('no optimistic aid freshness at construction','lastHorizontalAidTime=t' not in init_section and 'lastVerticalAidTime=t' not in init_section and 'lastAttitudeAidTime=t' not in init_section)
add('freshness updated only after accepted filter update',all(re.search(r'if ok[^\n]*'+field+r'=t',eskf) for field in ['lastHorizontalAidTime','lastVerticalAidTime','lastAttitudeAidTime']))
add('preflight uses accepted-age/eligibility/covariance gates',all(x in pre for x in ['horizontalAidAge_s','verticalAidAge_s','attitudeAidAge_s','activeLaneEligible','covarianceOK','updateCountOK','goalReachable']))
add('preflight does not depend on instant packet flags',all(x not in pre for x in ['packet.hasVio','packet.hasLidar','packet.hasRange','packet.hasBaro']))
add('airborne goal is not treated as landing footprint','landing_zone_clear_S2_2( ...\n    grid,scenario.home' in pre and 'goalCellClear=~segment_occupied_grid_S2_2' in pre)

# Estimator-in-loop decisions and failsafe invariants.
decision=life[:life.index('    oldTruthP=truth.p;')]
add('mission decisions do not read truth state',not any(x in decision for x in ['truth.p','truth.v','truth.onGround']))
add('landing uses contact-confirmed detector','land_detector_S2_2' in life and 'vz.complete&&landDetected' in life and "packet.groundContact" in text('land_detector_S2_2.m'))
add('XY-loss trigger is one-shot',"'EMERGENCY_HOLD','EMERGENCY_LAND','LAND_DESCENT'" in life)
add('XY-loss descent disables horizontal control',"'horizontalControlEnabled',false" in life and "~ref.horizontalControlEnabled" in geom)
add('normal lifecycle requires completion plus truth validation',all(x in life for x in ['completionPass','truthGoalPass','truthTakeoffPass','truthLandingPass']))
add('vertical reference and executed gates present',all(x in life for x in ['maxVerticalReferenceSpeed','maxExecutedVerticalSpeed','maxExecutedVerticalAccel','maxExecutedVerticalJerk']) and all(x in cfg for x in ['maxExecutedVerticalSpeed_mps','maxExecutedVerticalAccel_mps2','maxExecutedVerticalJerk_mps3']))
add('RTL obstacle count tied to actual repair','rtlObstacleReplanRecorded' in life and 'rtlMidcourseReplanCount=rtlMidcourseReplanCount+1' in life)

# Plot, animation, paths.
add('docked tabbed plot convention',"'WindowStyle','docked'" in plot or "set(fig,'WindowStyle','docked')" in plot)
add('versioned v0.5 output conventions','v0_5' in run or 'versionFolder' in run)
add('animation replays obstacle insertion time','activeObstacleHistoryTime' in anim and 'update_static_obstacles' in anim and 'maps.activeObstacles' not in anim)
add('vertical metrics exposed in console/dashboard','Executed Z v/a/j' in run and '|Z speed|' in plot)
add('legacy airborne extensions are conditional',
    "nargin<3||isempty(startAltitude)" in text('init_quadrotor_state_S2_2.m') and
    "isfield(cfg,'groundHeight_m')" in text('quadrotor_dynamics_S2_2.m') and
    "isfield(ref,'horizontalControlEnabled')" in geom)
add('landing confirmation cuts motor command immediately',
    "'DISARM','COMPLETE','PREFLIGHT_REJECT','FAILSAFE'" in life and
    'motorsActive=motorsActive&&~any' in life)
mc=text('validate_S2_2_monte_carlo.m')
add('Monte Carlo covers all five lifecycle scenarios',all(x in mc for x in LIFECYCLE))

# Function resolution and field consistency.
all_m=sorted(ROOT.glob('*.m')); file_funcs={p.stem for p in all_m}; locals_={p:function_names(p.read_text()) for p in all_m}
mismatch=[p.name for p in all_m if p.stem not in locals_[p]]
add('MATLAB file/function-name consistency',not mismatch,f'{len(all_m)} .m files' if not mismatch else ', '.join(mismatch))
unresolved=[]
for p in all_m:
    src=p.read_text(); called=set(re.findall(r'\b([A-Za-z]\w*_S2_2)\s*\(',src)); available=file_funcs|locals_[p]
    unresolved.extend(f'{p.name}:{fn}' for fn in sorted(called-available))
add('internal _S2_2 calls resolve',not unresolved,'all calls resolve' if not unresolved else '; '.join(unresolved))
cfg_def=set(re.findall(r'cfg\.([A-Za-z]\w*)\s*=',cfg))|{'seed','initialPosition','inflationRadius'}
sc_def=set(re.findall(r'scenario\.([A-Za-z]\w*)\s*=',sc))
cfg_use=set(); sc_use=set()
for p in all_m:
    src=p.read_text(); cfg_use.update(re.findall(r'cfg\.([A-Za-z]\w*)',src)); sc_use.update(re.findall(r'scenario\.([A-Za-z]\w*)',src))
add('configuration fields complete',not sorted(cfg_use-cfg_def),'missing '+', '.join(sorted(cfg_use-cfg_def)) if cfg_use-cfg_def else f'{len(cfg_use)} fields')
add('scenario fields complete',not sorted(sc_use-sc_def),'missing '+', '.join(sorted(sc_use-sc_def)) if sc_use-sc_def else f'{len(sc_use)} fields')

# Source hygiene and simple integrity.
hits=[]
patterns=[r'/Users/',r'Documents/MATLAB',r'[A-Za-z]:\\\\']
source_python=[p for p in ROOT.glob('*.py') if p.name not in {'audit_S2_2_v0_5.py'}]
for p in list(ROOT.glob('*.m'))+source_python:
    src=p.read_text(errors='replace')
    hits.extend(f'{p.name}:{pat}' for pat in patterns if re.search(pat,src))
add('no absolute developer paths',not hits,'portable paths' if not hits else '; '.join(hits))
bad=[]
for p in all_m:
    raw=p.read_bytes(); src=raw.decode(errors='replace')
    if b'\x00' in raw or re.search(r'(?m)^(<<<<<<<|=======|>>>>>>>)\s*$',src): bad.append(p.name)
add('source integrity',not bad,'no NUL/conflict markers' if not bad else ', '.join(bad))

# Python evidence. Run scripts again so stale reports cannot satisfy the audit.
def run_script(name,*args):
    r=subprocess.run([sys.executable,str(ROOT/name),*args],cwd=ROOT,capture_output=True,text=True)
    return r.returncode==0,(r.stdout+r.stderr).strip().splitlines()[-1] if (r.stdout+r.stderr).strip() else ''
with tempfile.TemporaryDirectory(prefix='s2_2_v05_audit_') as tmp:
    life_ok,life_tail=run_script('s2_2_v0_5_lifecycle_backtest.py','--output',tmp,'--seeds','10')
audit_ok,audit_tail=run_script('s2_2_v0_5_audit_backtest.py')
v04a_ok,v04a_tail=run_script('patch_regression_s2_2_v0_4.py')
v04b_ok,v04b_tail=run_script('patch2_regression_s2_2_v0_4.py')
sanity_ok,sanity_tail=run_script('matlab_source_sanity.py')
add('Python lifecycle matrix',life_ok,'50/50 expected; '+life_tail)
add('focused lifecycle mechanism audit',audit_ok,audit_tail)
add('preserved v0.4 mechanism regressions',v04a_ok and v04b_ok,f'patch1={v04a_ok}, patch2={v04b_ok}')
add('MATLAB source sanity scan',sanity_ok,sanity_tail)
try:
    d=json.loads(text('AUDIT_BACKTEST_RESULTS_S2_2_V0_5.json'))
    add('audit evidence report complete',d['passed']==d['total']==9,'9/9 PASS')
except Exception as e: add('audit evidence report complete',False,str(e))

passed=sum(ok for _,ok,_ in checks)
print('Stage S2.2 v0.5 cumulative static audit')
print('='*62)
for name,ok,detail in checks:
    print(f"[{'PASS' if ok else 'FAIL'}] {name}"+(f' — {detail}' if detail else ''))
print('-'*62); print(f'Result: {passed}/{len(checks)} checks passed')
if passed!=len(checks): sys.exit(1)
print('CUMULATIVE STATIC AUDIT: PASS')
