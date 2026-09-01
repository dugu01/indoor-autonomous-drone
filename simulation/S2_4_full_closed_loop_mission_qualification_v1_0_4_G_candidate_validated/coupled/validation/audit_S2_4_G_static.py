#!/usr/bin/env python3
from __future__ import annotations
import os,re,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
C=ROOT/'coupled'; ENV=os.environ.copy();ENV['PYTHONDONTWRITEBYTECODE']='1'
required=[
 ROOT/'run_validate_S2_4_G_all.m',ROOT/'setup_S2_4_G_path.m',
 C/'mission'/'run_S2_4_G_qualification_case.m',
 C/'validation'/'validate_S2_4_G_all.m',
 C/'validation'/'backtest_S2_4_G_matrix.py',
 C/'validation'/'backtest_S2_4_G_recovery_patch.py',
 C/'validation'/'backtest_S2_4_G_v102_user_log.py',
 C/'validation'/'audit_S2_4_G_qualification_semantics.py',
 C/'docs'/'S2_4_G_FULL_CLOSED_LOOP_QUALIFICATION.md',
 ROOT/'tools'/'audit_S2_4_G_f_baseline_delta.py',
 ROOT/'tools'/'audit_S2_4_G_v103_runtime_delta.py',
 ROOT/'evidence'/'S2_4_G_V1_0_2_RUNTIME_BASELINE_SHA256.txt',
 ROOT/'evidence'/'S2_4_G_V1_0_3_ESTIMATOR_PEAK_DIAGNOSIS.txt',
 C/'validation'/'backtest_S2_4_G_yaw_slew.py',
 ROOT/'evidence'/'S2_4_F_VALIDATED_BASELINE_SHA256.txt',
 ROOT/'evidence'/'S2_4_G_REVIEWED_F_RUNTIME_DELTA.txt',
]
errors=[]
for p in required:
    if not p.is_file(): errors.append(f'missing {p.relative_to(ROOT)}')
q=subprocess.run([sys.executable,'-B',str(ROOT/'tools'/'audit_S2_4_G_f_baseline_delta.py')],cwd=ROOT,env=ENV,text=True,capture_output=True)
print(q.stdout,end='')
if q.returncode: errors.append('reviewed F runtime delta audit failed')
wrapper=(C/'validation'/'validate_S2_4_G_all.m').read_text()
for token in ("faults={'F2','F3','F6','F9','F10'}","timings={'early','mid','late'}",'seeds=0:4','expectedRuns==75',
              'validate_S2_4_F_all()','NO-FAULT SEED BASELINES','actualTruthIsolation','unknownCommitmentCount==0',
              'missionCompletion','closedLoopIntegrity','nominalExplorationPass','staleCommandContinuationCount==0'):
    if token not in wrapper: errors.append(f'G wrapper token missing: {token}')
entry=(ROOT/'run_validate_S2_4_G_all.m').read_text()
for token in ('run_validate_S2_4_AD_all','setup_S2_4_G_path','validate_S2_4_G_all','S2_4:GValidationFailed'):
    if token not in entry: errors.append(f'G entry token missing: {token}')
case=(C/'mission'/'run_S2_4_G_qualification_case.m').read_text()
for token in ("cfg=init_S2_4_F_config()","case 'early',progress=0.20","case 'mid',  progress=0.50","case 'late', progress=0.75"):
    if token not in case: errors.append(f'G case token missing: {token}')
# G wrapper remains qualification-only. The one reviewed runtime delta is
# audited separately and may not introduce planner/controller/plant logic.
new_matlab=[ROOT/'run_validate_S2_4_G_all.m',ROOT/'setup_S2_4_G_path.m',C/'mission'/'run_S2_4_G_qualification_case.m',C/'validation'/'validate_S2_4_G_all.m']
for p in new_matlab:
    text=p.read_text()
    for pat in (r'scenario\.truth',r'truthWorld',r'truthStatic',r'truthDynamic',r'geometric_controller',r'thrust_N\s*=',r'moment_Nm\s*=',r'dstar_',r'astar_grid'):
        if re.search(pat,text,re.I): errors.append(f'forbidden G wrapper dependency {pat}: {p.relative_to(ROOT)}')
mission=(C/'mission'/'mission_lifecycle_manager_S2_4.m').read_text()
for token in ('CURRENT_REFERENCE_NOT_KNOWN_FREE','reference_known_free_local','executionReferenceGuardCount','executionReferenceGuardRTLCount','try_reauthorize_suspended_request_local','REAUTHORIZED_AFTER_REVOCATION','yawTargetCommand','slew_yaw_reference_local'):
    if token not in mission: errors.append(f'reviewed F runtime patch token missing: {token}')
# Ensure the runtime delta does not implement a new planner/controller/plant.
patch_region=mission[mission.index('% G-discovered execution seam:'):mission.index('% Terminal/disarm states cut motor command immediately')]
for pat in (r'dstar_',r'astar_grid',r'generate_.*trajectory',r'geometric_controller',r'quadrotor_dynamics'):
    if re.search(pat,patch_region,re.I): errors.append(f'forbidden implementation in F runtime guard: {pat}')

def strip_comments(text:str)->str:
    out=[]
    for line in text.splitlines():
        in_str=False;buf=[];i=0
        while i<len(line):
            ch=line[i]
            if ch=="'":
                if in_str:
                    if i+1<len(line) and line[i+1]=="'": i+=2; continue
                    in_str=False;i+=1;continue
                prev=line[i-1] if i>0 else ''
                transpose=(prev in '.)]}' or prev.isalnum() or prev=='_')
                if transpose: i+=1;continue
                in_str=True;i+=1;continue
            if ch=='%' and not in_str: break
            if not in_str: buf.append(ch)
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
for p in new_matlab+[C/'mission'/'mission_lifecycle_manager_S2_4.m']:
    raw=p.read_text();code=strip_comments(raw)
    first=next((x.strip() for x in raw.splitlines() if x.strip()),'')
    if not first.startswith('function'): errors.append(f'primary function missing: {p.relative_to(ROOT)}')
    for l,r,label in [('(',')','parentheses'),('[',']','brackets'),('{','}','braces')]:
        if not balanced(code,l,r): errors.append(f'unbalanced {label}: {p.relative_to(ROOT)}')
    if re.search(r'\b(?:TODO|FIXME|PLACEHOLDER)\b',raw,re.I): errors.append(f'unfinished marker: {p.relative_to(ROOT)}')
if errors:
    print('S2.4-G STATIC GATE: FAIL')
    for e in errors: print(' -',e)
    raise SystemExit(1)
print('S2.4-G reviewed F-v1.1.2 delta integrity: PASS')
print('S2.4-G wrapper isolation: PASS')
print('S2.4-G 5 no-fault baselines + 75-run targeted matrix contract: PASS')
print('S2.4-G MATLAB source hygiene: PASS')
print('S2.4-G STATIC GATE: PASS')
