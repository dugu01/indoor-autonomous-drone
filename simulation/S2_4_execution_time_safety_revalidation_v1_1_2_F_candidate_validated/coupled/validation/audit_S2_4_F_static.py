#!/usr/bin/env python3
from __future__ import annotations
import hashlib, os, pathlib, re, subprocess, sys
ROOT=pathlib.Path(__file__).resolve().parents[2]
ENV=os.environ.copy();ENV['PYTHONDONTWRITEBYTECODE']='1'
C=ROOT/'coupled'; P=ROOT/'frozen_parent'/'S2_3_online_mapping_v1_0_0_validated'
required=[
 C/'mission'/'init_S2_4_F_config.m',
 C/'mission'/'mission_lifecycle_manager_S2_4.m',
 C/'mission'/'run_S2_4_F_fault_case.m',
 C/'execution'/'revalidate_active_exploration_request_S2_4_F.m',
 C/'execution'/'apply_validation_fault_S2_4_F.m',
 C/'execution'/'validate_exploration_request_S2_4.m',
 C/'validation'/'test_S2_4_F_revalidation_contracts.m',
 C/'validation'/'validate_S2_4_F_all.m',
 ROOT/'setup_S2_4_F_path.m',
 ROOT/'run_validate_S2_4_F_all.m',
 C/'validation'/'run_all_checks_S2_4_F.py',
 C/'docs'/'S2_4_F_EXECUTION_SAFETY.md',
]
errors=[]
for p in required:
    if not p.is_file(): errors.append(f'missing {p.relative_to(ROOT)}')
for script in ('audit_parent_immutability.py','audit_final_parent_manifest.py'):
    q=subprocess.run([sys.executable,'-B',str(ROOT/'tools'/script)],cwd=ROOT,text=True,capture_output=True,env=ENV)
    print(q.stdout,end='')
    if q.returncode: errors.append(f'parent audit failed: {script}')
# E entry/config files intentionally unchanged from the validated uploaded baseline.
expected={
 C/'mission'/'init_S2_4_E_config.m':'f6baeef2cbb671aceb7a8754302c4ce88fb63da88234995102eeaaac6d9ca8d3',
 C/'mission'/'run_S2_4_coupled.m':'8574817db72147a1145ee7c8f35685990298347a14c3ae976e13d62493bcc6b2',
}
for p,h in expected.items():
    got=hashlib.sha256(p.read_bytes()).hexdigest()
    if got!=h: errors.append(f'validated E entry changed unexpectedly: {p.relative_to(ROOT)}')
# New decision/safety helpers must not read truth or emit low-level commands.
for p in (C/'execution'/'revalidate_active_exploration_request_S2_4_F.m', C/'execution'/'apply_validation_fault_S2_4_F.m'):
    if not p.is_file(): continue
    text=p.read_text()
    for pat in (r'scenario\.truth',r'truthWorld',r'truthStatic',r'truthDynamic',r'validationGeometry',r'geometric_controller',r'thrust_N\s*=',r'moment_Nm\s*='):
        if re.search(pat,text,re.I): errors.append(f'forbidden dependency {pat}: {p.relative_to(ROOT)}')
manager=(C/'mission'/'mission_lifecycle_manager_S2_4.m').read_text()
for token in (
 'revalidate_active_exploration_request_S2_4_F',
 'apply_validation_fault_S2_4_F',
 "state='LIFECYCLE_REPLAN_BRAKE'",
 'pendingExplorationReplan',
 'staleCommandContinuationCount',
 'executionAuthorityInvalidationCount',
 'route_progress_fraction',
 'faultInjectedAuthorityGenerations',
 'executionLeaseRenewalCount',
 "'executionSafetyHistory'",
):
    if token not in manager: errors.append(f'manager F token missing: {token}')
# Ensure F code is disabled for the E config path.
if "executionSafetyEnabled=isfield(cfg,'executionSafety')&&cfg.executionSafety.enabled" not in manager:
    errors.append('F enable gate missing')
fcfg=(C/'mission'/'init_S2_4_F_config.m').read_text()
for token in ("cfg.stage='S2.4-F'",'maxAuthorityInvalidations','dynamicPredictionLiveSupported',"'enabled',true"):
    if token not in fcfg: errors.append(f'F config token missing: {token}')
if "'dynamicPredictionLiveSupported',false" not in fcfg:
    errors.append('F15 must remain explicitly unsupported until live predictor is connected')
if 'cfg.activeExploration.requestValidity_s' in fcfg:
    errors.append('F must not extend/override the validated E request TTL; use rolling current revalidation instead')
for token in ('renewLeaseOnValidRevalidation','triggerProgress','repeatPerAuthority'):
    if token not in fcfg: errors.append(f'F authority-relative config token missing: {token}')

revalidator=(C/'execution'/'revalidate_active_exploration_request_S2_4_F.m').read_text()
if 'validate_known_free_stop_S2_3' in revalidator:
    errors.append('runtime F revalidator must not re-run the frozen S2.3 planning-time terminal stop gate')
for token in ('routeStopReserveSafe','terminalOverrunReserveSafe','stoppingDistance_m','remainingRouteLength_m'):
    if token not in revalidator: errors.append(f'runtime stop-reserve token missing: {token}')
validator=(C/'execution'/'validate_exploration_request_S2_4.m').read_text()
if 'metricRouteExecutable(grid,request.retreatRouteXY)' not in validator:
    errors.append('pre-acceptance retreat geometry is still not validated')
wrapper=(C/'validation'/'validate_S2_4_F_all.m').read_text()
for token in ('validate_S2_4_E_all(true)','referenceParityCheck',"'F14'",'NOT_APPLICABLE_LIVE_PREDICTOR_NOT_CONNECTED'):
    if token not in wrapper: errors.append(f'F wrapper token missing: {token}')
# Lightweight MATLAB source hygiene for all new/modified F-facing files.
def strip_comments(text:str)->str:
    # MATLAB apostrophe is overloaded: it starts a character vector *or* is a
    # transpose operator. Treat an apostrophe following an expression token as
    # transpose; otherwise treat it as a string delimiter. This avoids the
    # common false positive on expressions such as est.p(1:2).' .
    out=[]
    for line in text.splitlines():
        in_str=False;buf=[];i=0;prev_sig=''
        while i<len(line):
            ch=line[i]
            if ch=="'":
                if in_str:
                    if i+1<len(line) and line[i+1]=="'":
                        i+=2; continue
                    in_str=False;i+=1;continue
                # MATLAB transpose when apostrophe immediately follows a token
                # that can terminate an expression (including non-conjugate .').
                prev=line[i-1] if i>0 else ''
                transpose=(prev=='.' or prev==')' or prev==']' or prev=='}' or
                           prev.isalnum() or prev=='_')
                if transpose:
                    i+=1;continue
                in_str=True;i+=1;continue
            if ch=='%' and not in_str: break
            if not in_str:
                buf.append(ch)
                if not ch.isspace(): prev_sig=ch
            i+=1
        out.append(''.join(buf))
    return '\n'.join(out)
def balanced(code,l,r):
    n=0
    for ch in code:
        if ch==l:n+=1
        elif ch==r:
            n-=1
            if n<0:return False
    return n==0
for p in [x for x in required if x.suffix=='.m']:
    if not p.is_file(): continue
    raw=p.read_text(); code=strip_comments(raw)
    first=next((ln.strip() for ln in raw.splitlines() if ln.strip()),'')
    if not first.startswith('function'): errors.append(f'primary function missing: {p.relative_to(ROOT)}')
    for l,r,label in [('(',')','parentheses'),('[',']','brackets'),('{','}','braces')]:
        if not balanced(code,l,r): errors.append(f'unbalanced {label}: {p.relative_to(ROOT)}')
    if re.search(r'\b(?:TODO|FIXME|PLACEHOLDER)\b',raw,re.I): errors.append(f'unfinished marker: {p.relative_to(ROOT)}')
print(f'S2.4-F required files checked: {len(required)}')
if errors:
    print('S2.4-F STATIC GATE: FAIL')
    for e in errors: print(' -',e)
    raise SystemExit(1)
print('Frozen S2.3 immutability: PASS')
print('E entry/config byte identity: PASS')
print('F truth/command isolation: PASS')
print('F15 live predictive interface qualification: NOT APPLICABLE (explicitly unsupported)')
print('S2.4-F STATIC GATE: PASS')
