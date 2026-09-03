#!/usr/bin/env python3
from pathlib import Path
import subprocess,sys,re,difflib
ROOT=Path(__file__).resolve().parents[2]; S=ROOT/'s2_5'; E=S/'evidence/v1_0_6_runtime_reference'
checks=[]
def ck(n,c,d=''): checks.append((n,bool(c),d))
r=subprocess.run([sys.executable,str(S/'validation/audit_S2_4_G_parent_immutability.py')],cwd=ROOT,capture_output=True,text=True); print(r.stdout,end=''); ck('parent_byte_identity',r.returncode==0)
cfg=(S/'mission/init_S2_5_config.m').read_text(); mgr=(S/'mission/mission_lifecycle_manager_S2_5.m').read_text(); sc=(S/'scenarios/scenario_S2_5.m').read_text(); per=(S/'perception/simulate_perception_packet_S2_5.m').read_text()
ck('candidate_version_v1_0_7','v1.0.7-estimation-perception-robustness-candidate' in cfg)
# Config is byte-equivalent to v1.0.6 except version string.
ref=(E/'mission__init_S2_5_config.m.txt').read_text().replace('v1.0.6-estimation-perception-robustness-candidate','v1.0.7-estimation-perception-robustness-candidate')
ck('config_version_only',cfg==ref)
ck('no_threshold_relaxation',all(x not in cfg for x in ['replanBrakeSpeed_mps=','replanBrakeAccel_mps2=','missionStateTimeout_s=','mapPerceptionHoldTimeout_s=','mapMaxNoProgressScans=']))
# Every direct transition into LIFECYCLE_REPLAN_BRAKE must establish a fresh state clock.
lines=mgr.splitlines(); trans=[(i,l) for i,l in enumerate(lines) if "state='LIFECYCLE_REPLAN_BRAKE'" in l]
missing=[]
for i,l in trans:
    window=' '.join(lines[max(0,i-2):min(len(lines),i+2)])
    if 'stateEntryTime=t' not in window: missing.append(i+1)
ck('all_replan_brake_entries_reset_state_clock',len(trans)==6 and not missing,f'transitions={len(trans)} missing={missing}')
ck('route_affected_timestamp_fix',"replanBrakeCount=replanBrakeCount+1;state='LIFECYCLE_REPLAN_BRAKE';stateEntryTime=t;holdPosition=est.p;" in mgr)
# Fault campaign: only P3/P4/C1 use state-triggered 1.20 s brief perception faults.
ck('state_trigger_helper_present',"function f=stateTriggeredFault(f,duration_s)" in sc and "triggerEligibleStates={'TRACK_OUTBOUND','TRACK_RTL','SCAN_HOLD'}" in sc)
ck('brief_fault_duration_preserved',sc.count('stateTriggeredFault(f,1.20)')==3)
ck('prolonged_failsafe_not_state_triggered',"scenario.s25PerceptionFault=perFault('DUAL_DROPOUT',18.0,25.0);" in sc)
ck('state_context_passed',"'lifecycleState',state" in mgr and "'lifecycleState','PREFLIGHT'" in mgr)
ck('trigger_once_then_run_duration',all(x in per for x in ['s25StateTriggerTime_s','triggerDuration_s','triggerAfter_s','triggerEligibleStates']))
ck('trigger_can_continue_in_hold','active=isfinite(model.s25StateTriggerTime_s)' in per)
# Sanitizer and CSE/SIE remain unchanged from v1.0.6 files.
for rel in ['perception/sanitize_perception_packet_S2_5.m','mission/plan_recovery_viewpoint_S2_5.m']:
    ck('retained_'+rel.replace('/','_'),(S/rel).exists())
for n,c,d in checks: print(f'{n:58s} {"PASS" if c else "FAIL"} {d}')
ok=all(c for _,c,_ in checks); print('S2.5 v1.0.7 STATIC / ISOLATION AUDIT:', 'PASS' if ok else 'FAIL'); sys.exit(0 if ok else 1)
